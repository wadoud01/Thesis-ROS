#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_commander.conversions import pose_to_list
from std_msgs.msg import String

def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    """
    all_equal = True
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    return all_equal

def move_to_joint_goal(joint_values):
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "panda_arm"  # Use your planning group name
    move_group = moveit_commander.MoveGroupCommander(group_name)

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    print("Printing robot state")
    print(robot.get_current_state())
    print("")

    # Set initial joint state to a known safe position
    initial_joint_positions = [0.0] * 7  # Adjust if needed
    move_group.set_joint_value_target(initial_joint_positions)
    move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    # Set joint target
    move_group.set_joint_value_target(joint_values)

    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    current_joint_values = move_group.get_current_joint_values()
    return all_close(joint_values, current_joint_values, 0.01)

if __name__ == '__main__':
    try:
        print("Enter the joint values for Panda robot (in radians):")
        joint_values = []
        for i in range(1, 8):  # Joints 1 to 7
            value = float(input(f"Joint {i}: "))
            joint_values.append(value)

        # Optional: Add any additional checks if needed
        print(f"Setting joints to: {joint_values}")

        if move_to_joint_goal(joint_values):
            print("Successfully moved to the target joint configuration.")
        else:
            print("Failed to move to the target joint configuration.")
        
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        print(f"An error occurred: {e}")
