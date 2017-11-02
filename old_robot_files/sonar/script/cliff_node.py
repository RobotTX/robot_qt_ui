#!/usr/bin/env python

import rospy
import rospy
import actionlib
from sonar.srv import *

getCliffProxy = rospy.ServiceProxy( "getCliffData", GetCliffData)
setSpeedsProxy = rospy.ServiceProxy( "setSpeeds", SetSpeeds )

def init():
	move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
	rospy.loginfo("Waiting for move_base action server...")
	cliffData = getCliffProxy().CliffData
	if (cliffData[0]>15) or (cliffData[1]>15) or (cliffData[2])>15 or (cliffData[3]>15):
		move_base.cancel_goal()
		setSpeed("F", 0, "F", 0)
		

if __name__ == "__main__":
	rospy.init_node('cliff_sensor_node', anonymous=False)
	rate = rospy.Rate(20)
	init()
	rospy.spin()

