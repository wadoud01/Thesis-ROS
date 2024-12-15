#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Bool, String

# Global variables for state management
is_processing = False
last_coordinates = None
cooldown_time = 1.0  # Minimum time between operations (in seconds)
last_motion_time = None  # Initialize after ROS node setup

# Initial settings for finger joint values
finger_state = False
open_finger_value = 0.035
close_finger_value = 0.015

def move_to_cartesian_goal(x, y, z):
    """
    Moves the robot arm to the specified Cartesian coordinates using MoveIt.
    """
    global is_processing, last_motion_time

    # Prevent retriggering during cooldown
    if (rospy.Time.now() - last_motion_time).to_sec() < cooldown_time:
        rospy.loginfo("Cooldown period active, ignoring command.")
        return

    is_processing = True
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    group_name = "panda_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # Define target pose with downward orientation
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = z

    quaternion = quaternion_from_euler(3.14159, 0, 0)
    pose_goal.orientation.x = quaternion[0]
    pose_goal.orientation.y = quaternion[1]
    pose_goal.orientation.z = quaternion[2]
    pose_goal.orientation.w = quaternion[3]

    move_group.set_pose_target(pose_goal)

    rospy.loginfo("IK solving...")
    feedback_pub.publish(String(data="IK solving..."))

    result = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    if result:
        rospy.loginfo("Goal reached successfully.")
        feedback_pub.publish(String(data="Goal reached successfully."))
    else:
        rospy.logwarn("Motion failed or timed out.")
        feedback_pub.publish(String(data="Motion failed or timed out."))

    last_motion_time = rospy.Time.now()
    is_processing = False  # Reset processing flag after completion

def move_fingers(position, feedback_msg):
    """
    Move the robot fingers to the specified position and send feedback.
    """
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    group_name = "hand"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    current_joint_values = move_group.get_current_joint_values()
    current_joint_values[0] = position
    current_joint_values[1] = position

    move_group.set_joint_value_target(current_joint_values)
    move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    feedback_pub.publish(String(data=feedback_msg))

def grasp_signal_callback(msg):
    """
    Callback function to handle the grasp signal from Unity.
    """
    global finger_state
    finger_state = msg.data

    if finger_state:
        move_fingers(close_finger_value, "Gripper closed.")
    else:
        move_fingers(open_finger_value, "Gripper opened.")

def unity_coordinates_callback(data):
    """
    Callback function for handling incoming coordinates from Unity.
    """
    global last_coordinates

    # Compare received coordinates with the last processed ones
    new_coordinates = (round(data.x, 3), round(data.y, 3), round(data.z, 3))  # Round for float precision
    if last_coordinates == new_coordinates:
        rospy.loginfo("Duplicate coordinates received, ignoring.")
        return

    if is_processing:
        rospy.loginfo("Already processing a command, ignoring.")
        return

    last_coordinates = new_coordinates
    rospy.loginfo("Coordinates received: ({:.3f}, {:.3f}, {:.3f})".format(data.x, data.y, data.z))
    feedback_pub.publish(String(data="Coordinates received..."))

    move_to_cartesian_goal(data.x, data.y, data.z)

def listener():
    """
    Initializes ROS, subscribes to the necessary topics, and processes incoming messages.
    """
    global feedback_pub, last_motion_time
    rospy.init_node('coordinate_and_grasp_listener', anonymous=True)

    # Initialize the last motion time after ROS initialization
    last_motion_time = rospy.Time.now()

    feedback_pub = rospy.Publisher("unity_feedback", String, queue_size=10)

    rospy.Subscriber("unity_coordinates", geometry_msgs.msg.Point, unity_coordinates_callback)
    rospy.Subscriber("/grasp_signal", Bool, grasp_signal_callback)

    rospy.loginfo("Listening for Unity commands and grasp signals...")
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"An error occurred: {e}")

