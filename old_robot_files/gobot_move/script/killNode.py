#!/usr/bin/env python
# license removed for brevity
import rospy
import os
import rospkg
import time
import std_srvs.srv
from std_msgs.msg import String
from wheel.srv import *

setSpeedsProxy = rospy.ServiceProxy( "setSpeeds", SetSpeeds )
locationProxy = rospy.ServiceProxy( "global_localization", std_srvs.srv.Empty )

def setGlobalLocalization():
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
        os.system("roslaunch gobot_move my_move_base.launch")

def restartSmallMap(msg):
	print "restart smallmap"
	os.system("rosnode kill hector_mapping")
        os.system("rosnode kill move_base_new")
	uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid,["/home/gtdollar/catkin_ws/src/gobot_move/launch/small_map.launch"])
        launch.start()
	setSpeedsProxy( "F", 6, "F", 6 )
        time.sleep(6)
        setSpeedsProxy( "F", 0, "F", 0 )
	return True

if __name__ == '__main__':
	rospy.init_node('kill_node')
	setGlobalLocalization()
	rospy.Subscriber("/position_found", String, stopSmallMap)
	rospy.Subscriber("/restartSmallMap", String restartSmallMap)
	rospy.spin()
