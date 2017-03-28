#!/usr/bin/env python  
#refernence: http://answers.ros.org/question/29706/twist-message-example-and-cmd_vel/  
import roslib
import rospy
import std_srvs.srv
import time
from geometry_msgs.msg import Twist,PoseStamped
from sensor_msgs.msg import LaserScan
from wheel.srv import *


WHEELSPEED = 0.0,0.0,0.0
COUNTSTEP = 0
setSpeedsProxy = rospy.ServiceProxy( "setSpeeds", SetSpeeds )
locationProxy = rospy.ServiceProxy( "global_localization", std_srvs.srv.Empty )

#2 is right wheel, 1 is left wheel
def setSpeed(rDirection, rSpeed, lDirection, lSpeed):
	print rDirection, rSpeed, lDirection, lSpeed
	setSpeedsProxy( str(lDirection), lSpeed, str(rDirection), rSpeed )

def callback(msg):
	global WHEELSPEED
	global COUNTSTEP

	WHEELSPEED = msg.linear.x, msg.linear.y, msg.angular.z
	if (WHEELSPEED[0] == 0.0 ) and (WHEELSPEED[1] == 0.0) and (WHEELSPEED[2] == 0.0):
		COUNTSTEP = COUNTSTEP + 1
	else:
		COUNTSTEP = 0
	print COUNTSTEP
    	if 6>COUNTSTEP >= 2:
		setSpeedsProxy( "F", 6, "B", 6 )
		locationProxy()
		print "move"
		time.sleep(60)
        	rarm_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped)
        	goal = PoseStamped()
        	goal.pose.position.x = LAST_DISTANCE[0]
        	goal.pose.position.y = LAST_DISTANCE[1]
        	goal.pose.position.z = 0.0
        	goal.pose.orientation.z = LAST_DISTANCE[2]
        	goal.pose.orientation.w = LAST_DISTANCE[3]
        	goal.header.frame_id = 'map'
        	goal.header.stamp = rospy.Time.now()
        	rarm_pub.publish(goal)
        
def location(msg):
        global LAST_DISTANCE
        LAST_DISTANCE = msg.pose.position.x, msg.pose.position.y, msg.pose.orientation.z, msg.pose.orientation.w

def scan(msg):
	scan_pub = rospy.Publisher("/scan1", LaserScan, queue_size=10 )
	scan = LaserScan()
	scan.header.frame_id = "base_laser1"
	scan.header.stamp = msg.header.stamp
	scan.angle_min = msg.angle_min
	scan.angle_max = msg.angle_max
	scan.angle_increment = msg.angle_increment
	scan.time_increment = msg.time_increment
	scan.scan_time = msg.scan_time
	scan.range_min = msg.range_min
	scan.range_max = msg.range_max
	scan.ranges = msg.ranges
	scan.intensities = msg.intensities
	scan_pub.publish(scan)

def listener():
	rospy.init_node('cmd_vel_listener')
	#rospy.Subscriber("/cmd_vel", Twist, callback)#/cmd_vel  
	rospy.Subscriber("/move_base_simple/goal", PoseStamped, location )
	rospy.Subscriber("/scan", LaserScan, scan )
	rospy.spin()

if __name__ == '__main__':
    listener()

