#!/usr/bin/env python

import rospy
import errno
import socket
import sys 
import os
from socket import error as socket_error
import time 
from std_msgs.msg import String

file_server="/home/ubuntu/computer_software/IP/serverIP.txt"
file_IPs="/home/ubuntu/computer_software/IP/isAlive.txt"
ping_script = "sh /home/ubuntu/computer_software/IP/ping.sh"
file_hostname = "/home/ubuntu/computer_software/Robot_Infos/name.txt"

def isServer(IP) :
	s=socket.socket()
	host= IP
	port = 6000
	find = False
	try : 
		s.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
		s.settimeout(1)
		s.connect((host,port))
		res= s.recv(1024)
		if res == "OK" :
			find = True
			file=open(file_hostname)
			hostname = file.readline()
			hostname = hostname.split('\n')[0]
			s.send(hostname)
	except : 
		find=False
	return find
	s.close



def checkIPs ():
	print "checkIPs"
        with open(file_IPs) as f:
                for line in f:
			print line
                        found= isServer(line)
                        if found:
                                print "my server is ",line
                                file= open(file_server,"w")
                                file.write(line)
                                return True
	return False



def client (pub, checkOldIP):
	found = False

	if checkOldIP :
		print "check old"
		found=False
		if os.path.exists(file_server):
			file = open(file_server)
			IP= file.readline()
			found = isServer(IP)
		if not found and not rospy.is_shutdown(): 
			print "scanning for a new server"
			rospy.loginfo("disconnected")
			pub.publish("disconnected")
			os.system(ping_script)
			client(pub, False)

	if found :
		print "server is ", IP 
	elif not checkOldIP :
			found = checkIPs()
			if not found : 
				print "server not found"


rospy.init_node('ping_server', anonymous=True)
rate = rospy.Rate(10)
pub = rospy.Publisher('server_disconnected', String, queue_size=10)
  
while not rospy.is_shutdown(): 
	rate.sleep()
	client(pub, True)
	time.sleep(2)
