#!/usr/bin/env python
# license removed for brevity
import rospy
import os
import rospkg
import roslaunch
import std_srvs.srv
import time
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import MoveBaseActionGoal
from std_msgs.msg import String
from wheel.srv import *

GOAL_POSITION = 0.0, 0.0, 0.0, 0.0
ROBOT_SPEED = 0.0, 0.0, 0.0
ROBOT_STATUS = 0
setSpeedsProxy = rospy.ServiceProxy( "setSpeeds", SetSpeeds )
locationProxy = rospy.ServiceProxy( "global_localization", std_srvs.srv.Empty )

def startUpSmallMap():
    	#os.system('roslaunch gobot_move gobot_twoFrame.launch')
	uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid,["/home/gtdollar/catkin_ws/src/gobot_move/launch/gobot_twoFrame.launch"])
        launch.start()
    	print "turning on :"
    	print "- hokuyo_node "
    	print "- wheel "
    	print "- amcl_adjust.py "
    	print "- static_transform_publisher "
	print "- odom_twoFrame.py "
	print "- twist.py "
	print "- hector_mapping "
	print "- move_base "
	print "- map_server "
	print "- amcl "
	time.sleep(4)
	locationProxy()
        setSpeedsProxy( "F", 6, "B", 6 )
        time.sleep(6)
        setSpeedsProxy( "F", 0, "F", 0 )

def stopSmallMap(msg):
        print "start stopSmallMap"
        os.system("rosnode kill hector_mapping")
        os.system("rosnode kill move_base_new")
        os.system("rosnode kill adjust")
        setSpeedsProxy( "F", 0, "F", 0 )
        os.system("rosnode kill isLocalised" )
        #os.system("roslaunch gobot_move my_move_base.launch")
	uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid,["/home/gtdollar/catkin_ws/src/gobot_move/launch/my_move_base.launch"])
        launch.start()

def restartSmallMap(msg):
        print "restart smallmap"
        os.system("rosnode kill hector_mapping")
        os.system("rosnode kill move_base_new")
	os.system("rosnode kill move_base")
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid,["/home/gtdollar/catkin_ws/src/gobot_move/launch/small_map.launch"])
        launch.start()
	time.sleep(4)
        setSpeedsProxy( "F", 6, "F", 6 )
        time.sleep(6)
        setSpeedsProxy( "F", 0, "F", 0 )

def location(msg):
        global GOAL_POSITION
        GOAL_POSITION = msg.goal.target_pose.pose.position.x, msg.goal.target_pose.pose.position.y, msg.goal.target_pose.pose.orientation.z, msg.goal.target_pose.pose.orientation.w
	print GOAL_POSITION

def getRobotSpeed(msg):
	global ROBOT_SPEED
	ROBOT_SPEED = msg.linear.x, msg.linear.y, msg.angular.z
	#print ROBOT_SPEED

def robotStatus(msg):
	global ROBOT_STATUS
	if len(msg.status_list)>0:
		i = len(msg.status_list)
		ROBOT_STATUS = msg.status_list[i-1].status
	if (ROBOT_STATUS == 4):
		os.system("rosnode kill hector_mapping")
        	os.system("rosnode kill move_base_new")
        	os.system("rosnode kill move_base")
        	uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        	roslaunch.configure_logging(uuid)
        	launch = roslaunch.parent.ROSLaunchParent(uuid,["/home/gtdollar/catkin_ws/src/gobot_move/launch/small_map.launch"])
        	launch.start()
        	time.sleep(4)
        	setSpeedsProxy( "F", 6, "F", 6 )
        	time.sleep(6)
        	setSpeedsProxy( "F", 0, "F", 0 )

if __name__ == '__main__':
	rospy.init_node('gobot_move_node')
	startUpSmallMap()
	rospy.Subscriber("/position_found", String, stopSmallMap)
	rospy.Subscriber("/restartSmallMap", String, restartSmallMap)
	rospy.Subscriber("/move_base/goal", MoveBaseActionGoal, location )
	rospy.Subscriber("/cmd_vel", Twist, getRobotSpeed )
	rospy.Subscriber('/move_base/status', GoalStatusArray, robotStatus )
	rospy.spin()
