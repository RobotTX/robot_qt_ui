#!/usr/bin/env python

import sys
import rospy
import time
import std_srvs.srv
from wheel.srv import *
setSpeedsProxy = rospy.ServiceProxy( "setSpeeds", SetSpeeds )
getSpeedProxy = rospy.ServiceProxy( "getSpeed", GetSpeed )
setSpeedFProxy = rospy.ServiceProxy( "setSpeedF", SetSpeedDirection )
setSpeedBProxy = rospy.ServiceProxy( "setSpeedB", SetSpeedDirection )
getEncodersProxy = rospy.ServiceProxy( "getEncoders", GetEncoders )
getEncoderProxy = rospy.ServiceProxy("getEncoder", GetEncoder )
resetEncodesProxy = rospy.ServiceProxy( "resetEncoders", std_srvs.srv.Empty ) 

def setSpeed_client():
    try:
        #resetEncodesProxy()
        setSpeedsProxy( "F", 15, "B", 15 )
        time.sleep(8.6)
        setSpeedsProxy( "F", 15, "F", 15 )
	time.sleep(19)
        setSpeedsProxy( "F", 15, "B", 15 )
	time.sleep(4.4)
	setSpeedsProxy( "F", 15, "F", 15)
	time.sleep(18.5)
	setSpeedsProxy( "F", 15, "B", 15)
	time.sleep(4.5)
	setSpeedsProxy( "F", 0, "F", 0)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [left_Direction, left_Speed, right_Direction, right_Speed, exec_time]"%sys.argv[0]

if __name__ == "__main__":
        setSpeed_client()
