#!/usr/bin/env python

import sys
import rospy
import time
import std_srvs.srv
from sonar.srv import *
getBumpersProxy = rospy.ServiceProxy( "getBumpers", GetBumpers )
getSonarsProxy = rospy.ServiceProxy( "getSonars", GetSonars )

def getBumpers():
    	try:
		while True:
        		bumpers = getBumpersProxy().values
        		#print bumpers[0],bumpers[1],bumpers[2],bumpers[3],bumpers[4],bumpers[5],bumpers[6],bumpers[7]
			sonars = getSonarsProxy().values
			print "0:", sonars[0], " 1:", sonars[1], " 2:", sonars[2], " 3:", sonars[3], " 4:", sonars[4], " 5:", sonars[5], " 6:", sonars[6]
    	except rospy.ServiceException, e:
        	print "Service call failed: %s"%e

if __name__ == "__main__":
    if len(sys.argv) == 1:
	rospy.init_node('cmd_vel_listener')
	getBumpers()
    	rospy.spin()
    else:
        sys.exit(1)
