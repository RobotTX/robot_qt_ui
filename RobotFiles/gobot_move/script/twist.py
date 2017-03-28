#!/usr/bin/env python  
#refernence: http://answers.ros.org/question/29706/twist-message-example-and-cmd_vel/  
import roslib; roslib.load_manifest('move_base')
import rospy
import tf.transformations
import os
import time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import sqrt, atan2, pow
from wheel.srv import *

WHEELSPEED = 0.0,0.0,0.0
LAST_TIME = time.time()
fileHandle = open ( 'twistlog.txt', 'w' )
setSpeedsProxy = rospy.ServiceProxy( "setSpeeds", SetSpeeds )

#2 is right wheel, 1 is left wheel
def setSpeed(rDirection, rSpeed, lDirection, lSpeed):
    print rDirection, rSpeed, lDirection, lSpeed
    setSpeedsProxy( str(lDirection), lSpeed, str(rDirection), rSpeed )

def callback(msg):
    global WHEELSPEED
    cmd_twist_rotation =  msg.angular.z
    cmd_twist_x  = msg.linear.x
    cmd_twist_y =  msg.linear.y
    WHEELSPEED = msg.linear.x, msg.linear.y, msg.angular.z
    #change msg.linear.x to wheelspeed
    if ( WHEELSPEED[0] > 0.0 ):
        wheelSpeed = 101.9 * abs(WHEELSPEED[0]) + 0.475
    elif ( WHEELSPEED[0] < 0.0 ):
       	wheelSpeed = 0.0 - (91.67 * abs(WHEELSPEED[0]) + 0.574)
    else:
       	wheelSpeed = 0.0
    #change msg.linear.z to anglespeed
    if ( WHEELSPEED[2] != 0.0 ):
       	angleSpeed = 16.86 * abs(WHEELSPEED[2]) + 0.402
    else:
       	angleSpeed = 0.0
    if ( WHEELSPEED[2] > 0.0 ):
       	angleSpeed = angleSpeed
    else :
       	angleSpeed = 0.0 - angleSpeed
    #calculate rightwheelspeed and leftwheelspeed
    rightSpeed = wheelSpeed + angleSpeed /2.0
    leftSpeed = wheelSpeed - angleSpeed /2.0

    if rightSpeed > 0.0 :
       	rDir = 'F'
    else:
       	rDir = 'B'
       	rightSpeed = round(abs(rightSpeed),2)
    if leftSpeed > 0.0 :
       	lDir = "F"
    else:
       	lDir = "B"
       	leftSpeed = round(abs(leftSpeed),2)

    if rightSpeed < 6.0 and rightSpeed != 0.0:
       	rightSpeed = rightSpeed + 6.0
    if leftSpeed < 6.0 and leftSpeed != 0.0:
       	leftSpeed = leftSpeed + 6.0
    if 10 < rightSpeed < 11:
       	rightSpeed = rightSpeed + 1
    if 10 < leftSpeed < 11 :
       	leftSpeed = leftSpeed + 1

    logging = "\n rightSpeed is :" + str(rightSpeed) + "rightDirection is: " + rDir + ", leftSpeed is :" + str(leftSpeed) + ", leftDirection is :" + lDir
    fileHandle.write( logging )

    setSpeed(rDir, rightSpeed, lDir, leftSpeed)
    setSpeed(rDir, rightSpeed, lDir, leftSpeed)

def listener():
    rospy.init_node('cmd_vel_listener')
    rospy.Subscriber("/cmd_vel", Twist, callback)#/cmd_vel  
    rospy.spin()

if __name__ == '__main__':
    listener()

