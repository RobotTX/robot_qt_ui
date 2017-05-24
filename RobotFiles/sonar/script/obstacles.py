#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32 , Empty
import serial
import time
import struct
from datetime import datetime 
import threading,thread
from sonar.msg import DistanceMsg 



def get_sonar_value(Serialport,side):


	sonar = serial.Serial(port = Serialport, baudrate=9600,timeout=1)
	sonar.close()
	sonar.open()
        pub = rospy.Publisher('sonar_topic', DistanceMsg, queue_size=10)

	########################
	while True :
		sonar.write('\x00\x54')
		res = sonar.read(2)
                if len(res)== 2:
			value = ord(res[0])*256+ord(res[1])
			pubDist = DistanceMsg(side,distance)		
			pub.publish(pubDist)

if __name__ == "__main__":
    rospy.init_node('sonar_listener')
    Serialport = rospy.get_param('~serialPort','/dev/ttyO1')
    side = rospy.get_param('~side',"R")

    get_sonar_value(Serialport,side)
    rospy.spin()

