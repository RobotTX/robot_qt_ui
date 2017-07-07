#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32 , Empty
import serial
import time
import struct
from datetime import datetime 
import threading,thread
from sonar.msg import DistanceMsg 
from bluetooth.srv import *

class filterer(threading.Thread):
	def __init__(self,side):
		self.before = time.time()
		self.remindValue=[]
		self.remindTime=[]
		self.tobeFilled = 3
		self.side =side
	def check_validity(self,distance, nowTime,ID):

		if self.tobeFilled > 0:
			self.remindValue.append(distance)
			self.remindTime.append(nowTime)
			self.tobeFilled = self.tobeFilled -1
		else: 
			add = False
			
			for i in range(0,(3-self.tobeFilled)) :
				# if the velocity is more than 1.5m/s value is impossible
				#print "distance =", distance
				#print "remind=",self.remindValue[i]
				#print ((abs(self.remindValue[i] - distance)) / (nowTime - self.remindTime[i] )) 
				if ( ((abs(self.remindValue[i] - distance)) / (nowTime - self.remindTime[i] )) < 150 ):
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
	time.sleep(1)
	bleProxy = rospy.ServiceProxy("send_BLE", Send)
	bleProxy("1111")
	print 'send BLE'
	########################
	i=0
	while True :
		time.sleep(0.07)
		#print "send burst"
		sonarR.write('\x00\x5C')
		time.sleep(0.07)
		thread.start_new_thread(getValue,(sonarL,myFiltererL ,-20,i))
		getValue(sonarR,myFiltererR,80,i )
		i=i+1

def getValue(sonar,myFilterer,calibration,ID):
	sonar.write('\x00\x5A')
	if sonar.isOpen():
		res = sonar.read(2)
		saveTime= time.time()
		if len(res) == 2 :
			value = ord(res[0])*256+ord(res[1])
			if value !=0:
				#print value
				value = value-calibration
				#print value
				if value < 1000 :
					if value <0:					
						#continue
						myFilterer.check_validity(0, saveTime,ID)
						#print 0
					else :
						#continue
						#print value
						myFilterer.check_validity(value,saveTime,ID)

if __name__ == "__main__":
    rospy.init_node('sonar_listener')
    #Serialport = rospy.get_param('/sonar_listener/serialPort')
    SerialportR = rospy.get_param('~serialPortR','/dev/ttyO1')
    SerialportL = rospy.get_param('~serialPortL','/dev/ttyO4')

    get_sonar_value(SerialportR,SerialportL)
    rospy.spin()

