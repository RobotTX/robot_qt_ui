#!/usr/bin/env python
# license removed for brevity
import rospy
import os
import rospkg
import time

def stopSmallMap():
        print "start stopSmallMap"
        os.system("rosnode kill /cmd_vel_listener")
	os.system("rosrun gobot_move twist.py")

if __name__ == '__main__':
	stopSmallMap()
