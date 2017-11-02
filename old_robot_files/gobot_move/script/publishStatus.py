#!/usr/bin/env python  
#refernence: http://answers.ros.org/question/29706/twist-message-example-and-cmd_vel/  
import roslib; roslib.load_manifest('move_base')
import rospy
import tf.transformations
import os
import time
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from move_base_msgs.msg import MoveBaseGoal
from gobot_move.msg import robotStatus
from math import sqrt, atan2, pow
pub = rospy.Publisher("/robotStatus", robotStatus, queue_size = 10 )
goal_info = MoveBaseGoal()

def callback(msg):
	global goal_info
	robot_rotation_z = msg.pose.pose.orientation.z
	robot_rotation_w = msg.pose.pose.orientation.w
	robot_x  = msg.pose.pose.position.x
	robot_y =  msg.pose.pose.position.y
	x = pow(robot_x-goal_info.target_pose.pose.position.x,2)
	y = pow(robot_y-goal_info.target_pose.pose.position.y,2)
	distance = sqrt(x+y)
	print "robot_pose, x,y,z,w", robot_x, robot_y, robot_rotation_z, robot_rotation_w
	print "goal", goal_info
	print "distance", distance
	status = robotStatus()
	if (distance<0.4):
		status.status = 1
	else:
		status.status = 0
	pub.publish(status)	

def goalback(msg):
        global goal_info
        goal_info.target_pose.header.frame_id = "map"
        goal_info.target_pose.pose.position.x = msg.pose.position.x
        goal_info.target_pose.pose.position.y = msg.pose.position.y
        goal_info.target_pose.pose.orientation.w = msg.pose.orientation.w

def listener():
	rospy.init_node('robot_pose_listener')
	rospy.Subscriber("/move_base_simple/goal", PoseStamped, goalback )
	rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, callback)#/cmd_vel  
	rospy.spin()

if __name__ == '__main__':
    listener()

