#!/usr/bin/env python

import rospy
import errno
import socket
import sys 
import os
from socket import error as socket_error
import time 
from std_msgs.msg import String
import subprocess

file_IPs = rospy.get_param("server_file")
# to retrieve all scanned IPs
#file_IPs = computer_software + "IP/isAlive.txt"
pingFile = rospy.get_param("ping_file")
ping_script = "sudo sh " + pingFile


def isServer(IP) :
	s = socket.socket()
	host= IP
	port = 6000
	find = False
	try : 
		s.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
		# blocking operations of the socket time out after 5 seconds
		s.settimeout(5)
		s.connect((host,port))
		res = s.recv(1024)
		if res == "OK" :
			find = True
			# Get the hostname of the robot
			nameFile = rospy.get_param("robot_name_file")
			with open(nameFile, 'r') as file_name:
				hostname = file_name.readline()
				file_name.close()

			hostname = hostname.split('\n')[0]

			if hostname == "" :
				hostname = "Default Name"

			# Get the battery level
			battery = 50
			batteryFile = rospy.get_param("battery_file")
			print 'batteryfile', batteryFile
			with open(batteryFile, 'r') as file_name:
				battery = file_name.readline()
				file_name.close()

			# Get the SSID of the robot
			ssid = subprocess.Popen(["iwgetid", "-r"], stdout = subprocess.PIPE).communicate()[0]

			# Get the path stage
			stage = 0
			pathStageFile = rospy.get_param("path_stage_file")
			with open(pathStageFile, 'r') as file_path:
				stage = file_path.readline()
				#print "stage ", stage
				file_path.close()

			# Send everything to the application

			toSend = "%s\"%s\"%s\"%s" % (hostname, ssid, stage, battery)
			print "ping_server sending :",toSend
			s.send(toSend)

	except : 
		find = False
	return find
	#s.close



def checkIPs ():
	print "checkIPs"
	with open(file_IPs) as f:
		for line in f:
			print line
			found = isServer(line)
			if found:
				#print "my server is ",line
				serverFile = rospy.get_param("server_file")
				file = open(serverFile,"w")
				file.write(line)
				return True
	return False



def client (pub, checkOldIP):
	found = False

	if checkOldIP :
		#print "check old"
		found = False
		serverFile = rospy.get_param("server_file")
		if os.path.exists(serverFile):
			file = open(serverFile)
			IP = file.readline()
			found = isServer(IP)
		if not found and not rospy.is_shutdown(): 
			print "scanning for a new server"
			rospy.loginfo("disconnected")
			pub.publish("disconnected")
			os.system(ping_script)
			client(pub, False)

	#if found :
		#print "server is ", IP 
	elif not checkOldIP :
			found = checkIPs()
			if not found : 
				print "server not found"

if __name__ == "__main__":
    
    rospy.init_node('ping_server', anonymous = True)
    rate = rospy.Rate(10)
    pub = rospy.Publisher('server_disconnected', String, queue_size = 10)
      
    while not rospy.is_shutdown(): 
    	rate.sleep()
    	client(pub, True)
    	time.sleep(2)
