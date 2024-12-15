#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_commander.conversions import pose_to_list
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False
    elif type(goal) in [geometry_msgs.msg.PoseStamped, geometry_msgs.msg.Pose]:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)
    return True


def move_to_cartesian_goal(x, y, z):
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "panda_arm"  # Use your planning group name
    move_group = moveit_commander.MoveGroupCommander(group_name)
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    planning_frame = move_group.get_planning_frame()
    print("Planning frame: %s" % planning_frame)

    eef_link = move_group.get_end_effector_link()
    print("End effector link: %s" % eef_link)

    group_names = robot.get_group_names()
    print("Available Planning Groups:", robot.get_group_names())

    print("Printing robot state")
    print(robot.get_current_state())
    print("")

    # Define target pose with downward orientation
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = z

    # Quaternion for downward orientation (ensures the Z-axis points down)
    quaternion = quaternion_from_euler(3.14159, 0, 0)  # Roll, Pitch, Yaw (in radians)
    pose_goal.orientation.x = quaternion[0]
    pose_goal.orientation.y = quaternion[1]
    pose_goal.orientation.z = quaternion[2]
    pose_goal.orientation.w = quaternion[3]

    # Apply orientation constraint to keep EEF pointing downward
    constraints = moveit_msgs.msg.Constraints()
    orientation_constraint = moveit_msgs.msg.OrientationConstraint()
    orientation_constraint.link_name = eef_link
    orientation_constraint.header.frame_id = planning_frame
    orientation_constraint.orientation.x = quaternion[0]
    orientation_constraint.orientation.y = quaternion[1]
    orientation_constraint.orientation.z = quaternion[2]
    orientation_constraint.orientation.w = quaternion[3]
    orientation_constraint.absolute_x_axis_tolerance = 0.01
    orientation_constraint.absolute_y_axis_tolerance = 0.01
    orientation_constraint.absolute_z_axis_tolerance = 0.01
    orientation_constraint.weight = 1.0

    constraints.orientation_constraints.append(orientation_constraint)
    move_group.set_path_constraints(constraints)

    # Plan and execute the motion
    move_group.set_pose_target(pose_goal)

    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    # Clear path constraints
    move_group.clear_path_constraints()

    current_pose = move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)


if __name__ == '__main__':
    try:
        print("Enter the target Cartesian coordinates (x y z):")
        x, y, z = [float(coord) for coord in input().split()]
        if move_to_cartesian_goal(x, y, z):
            print("Successfully moved to the target Cartesian coordinates.")
        else:
            print("Failed to move to the target Cartesian coordinates.")
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        print("An error occurred: %s" % e)

