#!/usr/bin/env python
# license removed for brevity
import rospy
import os
import rospkg
import roslaunch
import std_srvs.srv
import time
import numpy as np
from std_msgs.msg import String
from actionlib_msgs.msg import GoalStatusArray
from sensor_msgs.msg import LaserScan
from wheel.srv import *

setSpeedsProxy = rospy.ServiceProxy( "setSpeeds", SetSpeeds )
locationProxy = rospy.ServiceProxy( "global_localization", std_srvs.srv.Empty )
angle_increment = 0.00613592332229
direct_flag = 0 # 1 is mean turn right, 2 is mean turn left
run_flag = True
first_flag = True

def robotMove(scan):
	global direct_flag, run_flag, first_flag
	fr_ranges = 0
	fr_count = 0
	inf_count = 0
	nan_count = 0
	mid_count = 0
	left_ranges = 0
	left_count = 0
	right_ranges = 0
	right_count = 0
	min_flag = False
	right_min_flag = False
	left_min_flag = False
	if (not run_flag):
		return
	if ( first_flag ):
		first_flag = False
		locationProxy()
	for i in range(146,458):
		if (np.isinf(scan.ranges[i])):
    			inf_count = inf_count + 1
    		elif (np.isnan(scan.ranges[i])):
			nan_count = nan_count + 1
		else:
			if ( scan.ranges[i]<0.3):
				min_flag = True
			fr_ranges = fr_ranges + scan.ranges[i]
			fr_count = fr_count+ 1
			if (scan.ranges[i]<1):
				mid_count = mid_count + 1
	for i in range(0, 146):
		if ((not np.isinf(scan.ranges[i])) and (not np.isnan(scan.ranges[i]))):
			right_ranges = right_ranges + scan.ranges[i]
			right_count = right_count + 1
			if (scan.ranges[i]<0.3):
				right_min_flag = True
	for i in range(458,598):
		if ((not np.isinf(scan.ranges[i])) and (not np.isnan(scan.ranges[i]))):
			left_ranges = left_ranges + scan.ranges[i]
			left_count = left_count + 1
			if (scan.ranges[i]<0.3):
				left_min_flag = True
	if ( min_flag ):
		fr_dis = 0.2
	else:
		fr_dis = fr_ranges / fr_count		
	if ( left_min_flag ):
		left_dis = 0.2
	else:
		left_dis = left_ranges / left_count
	if ( right_min_flag ):
		right_dis = 0.2
	else:
        	right_dis = right_ranges / right_count
	print "fr_dis is ", fr_dis, " left_dis is ", left_dis, " right_dis is ", right_dis
	
	if (inf_count>20):
		print "inf select: fr_dis is ", fr_dis, " left_dis is ", left_dis, " right_dis is ", right_dis
		if ( fr_dis > 1 ) and (not right_min_flag) and (not left_min_flag):
			setSpeedsProxy("F", 15, "F", 15)
		elif (fr_dis<0.3):
			if (left_dis > right_dis):
				setSpeedsProxy("B", 30, "B", 5)
			else:
				setSpeedsProxy("B", 5, "B", 30)
		else:
			if ( left_dis - right_dis > 1.0 ):
				setSpeedsProxy("B", 15, "F", 15)
				direct_flag = 2
			else:
				if ( right_dis - left_dis > 1.0 ):
					setSpeedsProx("F", 15, "B", 15)
					direct_flag = 1
				else:
					if ( left_dis < 0.5 or right_dis < 0.5):
						setSpeedsProxy( "B", 15, "B", 15)
					else:
						if (direct_flag == 2):
                                			setSpeedsProxy("B", 15, "F", 15)
						else:
							setSpeedsProxy("F", 15, "B", 15)
	else:
		print "mid select: fr_dis is ", fr_dis, " left_dis is ", left_dis, " right_dis is ", right_dis
		if ( mid_count == 0 ) and (not right_min_flag) and (not left_min_flag):
			if (fr_dis>1):
				setSpeedsProxy("F", 15, "F", 15)
			elif (fr_dis<0.5):
				setSpeedsProxy("B", 15, "B", 15)
		else:
			if ( left_dis - right_dis > 1.0 ):
                                setSpeedsProxy("B", 15, "F", 15)
				direct_flag = 2
                        else:
                                if ( right_dis - left_dis > 1.0):
                                        setSpeedsProx15("F", 15, "B", 16)
					direct_flag = 1
                                else:
                                        if ( left_dis < 0.5 or right_dis < 0.5):
                                                setSpeedsProxy( "B", 15, "B", 15 )
                                        else:
						if ( direct_flag == 2 ):
                                                	setSpeedsProxy( "B", 15, "F", 15 )
						else:
							setSpeedsProxy("F", 15, "B", 15)
	

def robotStop(msg):
	global run_flag
	setSpeedsProxy("F", 0, "F", 0)
	run_flag = False
	uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid,["/home/gtdollar/catkin_ws/src/gobot_move/launch/make_move_base.launch"])
        launch.start()

def robotStatus(msg):
        global ROBOT_STATUS, run_flag, first_flag
        if (ROBOT_STATUS == 4):
		run_flag = True
		first_flag = True

if __name__ == '__main__':
	rospy.init_node('gobot_move_node')
	rospy.Subscriber('/scan', LaserScan, robotMove )
	rospy.Subscriber('/position_find', String, robotStop)
	rospy.Subscriber('/move_base/status', GoalStatusArray, robotStatus )
	rospy.spin()
