#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32 , Empty
import serial
import time
import struct
import commands
from datetime import datetime 
import threading,thread
from sonar.msg import DistanceMsg 


def get_sonar_value():
	pub = rospy.Publisher("sonar_topic", DistanceMsg, queue_size = 10 )
	rate = rospy.Rate(20)
	while True:
		sonar.write("\xEE\x51\x07\xE0\xE2\xE4\xE6\xE8\xEA\xEC\xB1")
		res = sonar.read(18)
		if len(res) == 18:
			value0 = ord(res[3])*256 + ord(res[4])
			value1 = ord(res[5])*256 + ord(res[6])
			value2 = ord(res[7])*256 + ord(res[8])
			value3 = ord(res[9])*256 + ord(res[10])
			value4 = ord(res[11])*256 + ord(res[12])
			value5 = ord(res[13])*256 + ord(res[14])
			value6 = ord(res[15])*256 + ord(res[16])
			pubDist = DistanceMsg("sonar", value0, value1, value2, value3, value4, value5, value6)
			pub.publish(pubDist)
def init():
	global sonar
	deviceNode = "1-8:1.0"
	(status, output) = commands.getstatusoutput("ls -l /sys/class/tty/ttyUSB*")
	devPort = "/dev"+output[output.index(deviceNode)+7:output.index(deviceNode)+15]
	print devPort
	sonar = serial.Serial(
        	port = devPort,
        	baudrate = 115200,
        	stopbits = 1,
        	bytesize = serial.EIGHTBITS,
        	timeout = 0.5
	)
	sonar.close()
	sonar.open()

if __name__ == "__main__":
    	rospy.init_node('sonar_pub', anonymous=False)
	init()
	get_sonar_value()
    	rospy.spin()

