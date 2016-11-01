#!/usr/bin/env python
# license removed for brevity
import rospy
import math
from nav_msgs.msg import Path
from gobot_move.msg import moveInfo

x = 0.0
y = 0.0
distance = 0.0

pub = rospy.Publisher("gobot_move_info", moveInfo, queue_size = 10 )

def callback(msg):
	global x,y,distance
	move_info = moveInfo()
	#print len(msg.poses)
	for pose in msg.poses:
		x1 = math.pow( pose.pose.position.x - x, 2 )
		y1 = math.pow( pose.pose.position.y - y, 2 )
		z = x1 + y1
		x = pose.pose.position.x
		y = pose.pose.position.y
		#print pose.pose.position.x
		#print pose.pose.position.y
		#print pose.pose.position.z
		distance = distance + math.sqrt(z)
	#print distance
	move_time = int(distance/0.1) + 1
	#print move_time
	move_info.distance = distance
	move_info.time = move_time
	distance = 0.0
	x = 0.0
	y = 0.0
	pub.publish(move_info)

def listener():
	rospy.Subscriber("/move_base/NavfnROS/plan", Path, callback)#/cmd_vel  

if __name__ == '__main__':
	rospy.init_node('get_global_path_node')
	listener()
	rospy.spin()
