#!/usr/bin/env python

import sys
import copy
import rospy, tf
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import *
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


moveit_commander.roscpp_initialize(sys.argv) 
rospy.init_node('move_group_python_interface_tutorial',anonymous=True)
robot = moveit_commander.RobotCommander()

group_name1 = "arm"
group_name2 = "gripper"
group1 = moveit_commander.MoveGroupCommander(group_name1)
group2 = moveit_commander.MoveGroupCommander(group_name2)

display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory)

# print "============ Waiting for RVIZ..."
# rospy.sleep(10)
# print "============ Starting tutorial"

print "============ Reference frame: %s" % group1.get_planning_frame()

print "============ Reference frame: %s" % group2.get_end_effector_link()

print "============ Robot Groups:"
print robot.get_group_names()

print "============ Printing robot state"
print robot.get_current_state()
print "============"

quat=tf.transformations.quaternion_from_euler(3.14,0,-1.57)

print "============ Generating plan 1"
# pose_target = geometry_msgs.msg.Pose()

# pose_target.position.x = 0.03
# pose_target.position.y = -0.012
# pose_target.position.z = 1.010
# pose_target.orientation.x = quat[0]
# pose_target.orientation.y = quat[1]
# pose_target.orientation.z = quat[2]
# pose_target.orientation.w = quat[3]
# group1.set_pose_target(pose_target)

group1.clear_pose_targets()
joint_goal = group1.get_current_joint_values()
print(joint_goal)
joint_goal[5] = 1.57
group1.set_pose_target(joint_goal)
plan1 = group1.plan()
group1.execute(plan1, wait= True)
print "============ Waiting while RVIZ displays plan1..."
rospy.sleep(1)
group1.clear_pose_targets()

# group_variable_values = group1.get_current_joint_values()
# print "============ Joint values: ", group_variable_values
# group_variable_values[0] = 0
# group1.set_joint_value_target(group_variable_values)
# plan1 = group1.plan()
# group1.execute(plan1,wait=True)
# print "============ Waiting while RVIZ displays plan2..."
# rospy.sleep(1)

# group2.clear_pose_targets()
# group_variable_values = group2.get_current_joint_values()
# # close : 0.02, open : 0.00
# print "============ Joint values: ", group_variable_values
# group_variable_values[0] = 0.00
# group_variable_values[1] = 0.00
# group2.set_joint_value_target(group_variable_values)
# plan2 = group2.go(wait = True)
# print "============ Waiting while RVIZ displays plan2..."
# rospy.sleep(1)


# group_variable_values[0] = 0
# group_variable_values[1] = 0
# group2.set_joint_value_target(group_variable_values)
# plan2 = group2.go(wait = True)
# print "============ Waiting while RVIZ displays plan2..."
# rospy.sleep(5)