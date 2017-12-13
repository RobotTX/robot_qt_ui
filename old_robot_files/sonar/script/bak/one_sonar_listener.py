#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32 , Empty
import serial
import time
import struct
from datetime import datetime 
import threading,thread
from sonar.msg import DistanceMsg 

class filterer(threading.Thread):
	def __init__(self,side):
		self.before = time.time()
		self.remindValue=[]
		self.remindTime=[]
		self.tobeFilled = 3
		self.side =side
	def check_validity(self,distance, nowTime,ID):

		if self.tobeFilled > 1:
			self.remindValue.append(distance)
			self.remindTime.append(nowTime)
			self.tobeFilled = self.tobeFilled -1
		else: 
			add = False
			for i in range(0,(3-self.tobeFilled)) :
				# if the velocity is more than 1.5m/s value is impossible
				if ( abs(self.remindValue[i] - distance) < (nowTime - self.remindTime[i] )*150 ):
					add= True
			if add :
				pubDist = DistanceMsg(self.side,distance,ID)
				pub = rospy.Publisher('sonar_topic', DistanceMsg, queue_size=10)
				pub.publish(pubDist)
				self.remindValue.append(distance)
				self.remindTime.append(nowTime)
				if self.tobeFilled ==1 : 
					self.tobeFilled = self.tobeFilled -1
				else :

					self.remindValue= self.remindValue[1:]
					self.remindTime=self.remindTime[1:]


def get_sonar_value(SerialportR,SerialportL):


	sonarR = serial.Serial(port = SerialportR, baudrate=9600,timeout=1)
	sonarL = serial.Serial(port = SerialportL, baudrate=9600,timeout=1)
	sonarR.close()
	sonarR.open()
	sonarL.close()
	sonarL.open()

	myFiltererR = filterer("R")
	myFiltererL = filterer("L")
	########################
	# BLE 
	########################
	ble = serial.Serial(port = "/dev/ttyO2", baudrate=115200)	 
	ble.close()
	ble.open()
	if ble.isOpen():
		ble.write('\x31\x31\x31\x31')
		print 'send BLE'
	########################
	i=0
	while True :
		time.sleep(0.07)
		#print "send burst"
		sonarR.write('\x00\x5C')
		time.sleep(0.07)
		#thread.start_new_thread(getValue,(sonarR,myFiltererR))
		#thread.start_new_thread(getValue,(sonarL,myFiltererL ,30,i))
		getValue(sonarR,myFiltererR,80,i )
		i=i+1

def getValue(sonar,myFilterer,calibration,ID):
	sonar.write('\x00\x5A')
	if sonar.isOpen():
		res = sonar.read(2)
		if len(res) == 2 :
			value = ord(res[0])*256+ord(res[1])
			if value !=0:
				#print value
				value = value-calibration
				#print value
				if value < 1000 :
					if value <0:					
						#continue
						myFilterer.check_validity(0, time.time(),ID)
						#print 0
					else :
						#continue
						#print value
						myFilterer.check_validity(value,time.time(),ID)

if __name__ == "__main__":
    rospy.init_node('sonar_listener')
    #Serialport = rospy.get_param('/sonar_listener/serialPort')
    SerialportR = rospy.get_param('~serialPortR')
    SerialportL = rospy.get_param('~serialPortL')

    get_sonar_value(SerialportR,SerialportL)
    rospy.spin()
