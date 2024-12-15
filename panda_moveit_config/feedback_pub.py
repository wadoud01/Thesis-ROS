#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Point
from moveit_commander import MoveGroupCommander, MoveItCommanderException

# Initialize MoveIt! groups
arm_group = MoveGroupCommander("panda_arm")
hand_group = MoveGroupCommander("hand")

# Gripper values
open_finger_value = 0.035  # Open gripper position
close_finger_value = 0.015  # Close gripper position

# Feedback publisher
feedback_pub = rospy.Publisher('/unity_feedback', String, queue_size=10)

def send_feedback(message):
    """
    Publish feedback to Unity.
    """
    rospy.loginfo(f"Feedback: {message}")
    feedback_pub.publish(message)

def move_to_cartesian_point(target_point):
    """
    Moves the robot to a target Cartesian point.
    """
    try:
        send_feedback("IK solving...")  # Notify IK solving
        rospy.loginfo("IK solving...")  # Log IK solving

        arm_group.set_position_target([target_point.x, target_point.y, target_point.z])

        result = arm_group.go(wait=True)
        arm_group.stop()

        if result:
            send_feedback("Goal reached successfully.")
            rospy.loginfo("Goal reached successfully.")  # Log success
        # No action or log if result is False (goal fails)
    except Exception as e:
        rospy.logerr(f"Error moving to point: {e}")
        send_feedback("Failed to move to target point.")

def move_fingers(position):
    """
    Moves the gripper fingers to a specific position.
    """
    try:
        current_joint_values = hand_group.get_current_joint_values()
        current_joint_values[0] = position
        current_joint_values[1] = position

        hand_group.set_joint_value_target(current_joint_values)
        hand_group.go(wait=True)
        hand_group.stop()

        if position > 0.03:
            send_feedback("Gripper opened")
        else:
            send_feedback("Grasping")
    except MoveItCommanderException as e:
        rospy.logerr(f"Gripper movement error: {e}")
        send_feedback("Failed grasping")

def execute_real_robot_movement():
    """
    Command the real robot to execute planned motion.
    """
    try:
        send_feedback("Moving real robot")
        arm_group.execute()
    except Exception as e:
        rospy.logerr(f"Execution error: {e}")
        send_feedback("Execution failed")

# Callback functions
def coordinate_callback(data):
    """
    Handles incoming coordinate commands.
    """
    rospy.loginfo(f"Received coordinates: x={data.x}, y={data.y}, z={data.z}")
    send_feedback("Coordination (X Y Z) sent!")
    move_to_cartesian_point(data)

def grasp_signal_callback(msg):
    """
    Handles grasping signals from Unity.
    """
    try:
        if msg.data:
            move_fingers(close_finger_value)  # Close gripper
        else:
            move_fingers(open_finger_value)  # Open gripper
    except Exception as e:
        rospy.logerr(f"Grasp signal error: {e}")
        send_feedback("Failed grasping")

def execute_callback(msg):
    """
    Handles execute signal for real robot.
    """
    if msg.data:
        send_feedback("Executed")
        execute_real_robot_movement()

def main():
    rospy.init_node("unity_feedback_publisher", anonymous=True)

    # Subscribers for Unity commands
    rospy.Subscriber("/unity_coordinates", Point, coordinate_callback)
    rospy.Subscriber("/unity_grasp_signal", Bool, grasp_signal_callback)
    rospy.Subscriber("/unity_execute", Bool, execute_callback)

    rospy.loginfo("Listening for Unity commands and feedback signals...")
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

