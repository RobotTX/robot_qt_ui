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

computer_software = "/home/gtdollar/computer_software/"
file_server = computer_software + "IP/serverIP.txt"
file_IPs = computer_software + "IP/serverIP.txt"
#file_IPs = computer_software + "IP/isAlive.txt"
ping_script = "sudo sh " + computer_software + "IP/ping.sh"
file_hostname = computer_software + "Robot_Infos/name.txt"
file_map_id = computer_software + "Robot_Infos/mapId.txt"
file_path_stage = computer_software + "Robot_Infos/path_stage.txt"
file_home = "/home/gtdollar/computer_software/Robot_Infos/home.txt"

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
            file = open(file_hostname)
            hostname = file.readline()
            hostname = hostname.split('\n')[0]
            if hostname == "" :
                hostname = "Default Name"

            # Get the map id of the map we use
            file_id = open(file_map_id)
            map_id = file_id.readline()
            map_id = map_id.split('\n')[0]
            map_date = file_id.readline()
            map_date = map_date.split('\n')[0]
            if map_id == "" :
                map_id = "0"
            if map_date == "" :
                map_date = "1970-01-01"

            # Get the SSID of the robot
            ssid = subprocess.Popen(["iwgetid", "-r"], stdout = subprocess.PIPE).communicate()[0]

            # sends the path stage
            stage = 0
            with open(file_path_stage, 'r') as file_path:
                stage = file_path.readline()
                print "stage ", stage
                file_path.close()
            
            # Send everything to the application
            toSend = "%s\"%s\"%s\"%s\"%s" % (hostname, map_id, map_date, ssid, stage)
            #print "ping_server sending :",toSend
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
				print "my server is ",line
				file= open(file_server,"w")
				file.write(line)
				return True
	return False



def client (pub, checkOldIP):
	found = False

	if checkOldIP :
		print "check old"
		found = False
		if os.path.exists(file_server):
			file = open(file_server)
			IP = file.readline()
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

if __name__ == "__main__":
    
    rospy.init_node('ping_server', anonymous = True)
    rate = rospy.Rate(10)
    pub = rospy.Publisher('server_disconnected', String, queue_size = 10)
      
    while not rospy.is_shutdown(): 
    	rate.sleep()
    	client(pub, True)
    	time.sleep(2)
