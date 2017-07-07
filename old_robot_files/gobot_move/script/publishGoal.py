#!/usr/bin/env python

import serial
import time
import struct
import rospy
import math

from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseGoal
from actionlib_msgs.msg import GoalStatusArray

pub = rospy.Publisher("/move_base/goal", MoveBaseGoal, queue_size = 10)
goal_info = MoveBaseGoal()

def statusback(msg):
	global goal_info
	status = msg.status_list[0].status
	if status == 4:
		pub.publish(goal_info)

def goalback(msg):
	global goal_info
	goal_info.target_pose.header.frame_id = "map"
	goal_info.target_pose.pose.position.x = msg.pose.position.x
	goal_info.target_pose.pose.position.y = msg.pose.position.y
	goal_info.target_pose.pose.orientation.w = msg.pose.orientation.w

if __name__ == '__main__':
	rospy.init_node('goal_listener_node')
	rospy.Subscriber("/move_base/status", GoalStatusArray, statusback )
	rospy.Subscriber("/move_base_simple/goal", PoseStamped, goalback )

	init()
	rospy.spin()
