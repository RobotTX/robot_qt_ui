#!/usr/bin/env python
import rospy
import time
import os
import commands
from math import sqrt, pow
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseGoal
from wheel.srv import *
from sonar.msg import BumperMsg, IrSignalMsg, ShortSignalMsg, BatteryInfo
from sonar.srv import *

setSpeedsProxy = rospy.ServiceProxy( "setSpeeds", SetSpeeds )
getIrSignalProxy = rospy.ServiceProxy( "getIrSignal", GetIrSignal)
getShortSignalProxy = rospy.ServiceProxy( "getShortSignal", GetShortSignal)
getBumpersProxy = rospy.ServiceProxy( "getBumpers", GetBumpers )
getBatteryProxy = rospy.ServiceProxy("getBatteryInfo", GetBatteryInfo)

goal_info = MoveBaseGoal()
setChargingGoal = False
chargingFlag = False
twistFlag = False
distance = 10
rearFlag = True
leftFlag = False
rightFlag = False
bumperCrashCount = 0
beginTime = time.time()

def batteryCallback(batteryInfo):
	global setChargingGoal,chargingFlag
	if (batteryInfo.RemainCapacity< 50) and (not setChargingGoal):
		setChargingGoal = True
		nav_goal = PoseStamped()
		nav_goal.header.frame_id = "map"
		nav_goal.pose.position.x = 1.40
		nav_goal.pose.position.y = 5.10
		nav_goal.pose.position.z = 0.0
		nav_goal.pose.orientation.x = 0.0
		nav_goal.pose.orientation.y = 0.0
		nav_goal.pose.orientation.z = 0.9
		nav_goal.pose.orientation.w = 0.8
		pub.publish(nav_goal) #publish gobot to move goal location
		#this goal is charging station position
	if (chargingFlag) :
		auto_charging()
	else :
		if (distance<0.4) and (not twistFlag):
			#Turn off move_base before starting the auto charging function
			os.system("rosnode kill /cmd_vel_listener")
			os.system("rosnode kill /move_base")
			chargingFlag = True
			#start auto charging
			auto_charging()

def twistCallback(twistMsg):
	global twistFlag
	#The robot reaches the specified position
	if (twistMsg.linear.x==0) and (twistMsg.linear.y == 0) and (twistMsg.linear.z == 0):
		twistFlag = False
	else:
		twistFlag = True

def poseCallback(msg):
	global twistFlag, distance
	goal_info.target_pose.header.frame_id = "map"
	goal_info.target_pose.pose.position.x = 1.40
	goal_info.target_pose.pose.position.y = 5.10
	
	robot_x = msg.pose.pose.position.x
	robot_y = msg.pose.pose.position.y
	x = pow(robot_x - goal_info.target_pose.pose.position.x, 2)
	y = pow(robot_y - goal_info.target_pose.pose.position.y, 2)
	distance = sqrt(x+y)
	#Check the distance between the robot and the specified position
	#print distance

def auto_charging():
	global rearFlag, leftFlag,rightFlag,beginTime, bumperCrashCount
	batteryFlag = getBatteryProxy().ChargingFlag
	bumpers = getBumpersProxy() #read bumpers data
	irSignal = getIrSignalProxy() #read ir data
	shortSignal = getShortSignalProxy() #read short distance data
	rearSignal = irSignal.rearSignal   #rear ir data
	leftSignal = irSignal.leftSignal   #left ir data
	rightSignal = irSignal.rightSignal #right ir data
	bumperValue = bumpers.values[4] + bumpers.values[5] + bumpers.values[6] + bumpers.values[7]
	#rear bumpers data: if data is 1, means bumpers isn't collision,if data is 0, means bumpers is collision
	if (bumperValue<4):
		bumperCrashCount = bumperCrashCount + 1 #Calculate the number of collisions
	if (bumperCrashCount == 0):
		beginTime = time.time()  #start the timer
		timeInterval = 0 
	else:
		timeInterval = time.time() - beginTime #Record the time interval
	
	print batteryFlag, bumperCrashCount, rearSignal, leftSignal, rightSignal, rearFlag, leftFlag, rightFlag
	if (batteryFlag):
		setSpeedsProxy("F", 0, "F", 0)  #The charging is successful and the robot is stopped
	else:
		if (bumperCrashCount==0):  ##no collision
			if (rearSignal == 0) and (leftSignal == 0) and (rightSignal == 0):
			#rear ir, left ir, right ir all didn't received the signal
				if (not leftFlag ) and ( not rightFlag):
					if (rearFlag):
						setSpeedsProxy("B", 3, "F", 3)
					else:
						setSpeedsProxy("B", 3, "B", 3)
			else:
			#received ir signal
				if (rearSignal != 0):
				#received rear signal
					leftFlag = False
					rightFlag = False
					if (rearFlag):
						if (rearSignal==3):
							setSpeedsProxy("B", 3, "B", 3)
							#rear ir received 1 and 2 signal, so robot back
						elif (rearSignal==2):
							setSpeedsProxy("F", 3, "B", 3)
							#rear ir received 2 signal, so robot turn right
						elif (rearSignal==1):
							setSpeedsProxy("B", 3, "F", 3)
							##rear ir received 1 signal, so robot turn left
					else:
						setSpeedsProxy("B",4, "B", 4)
						#Because the robot is too close to the charging station, the robot is back 
				if (leftSignal!=0):
				#received left signal
					leftFlag = True
					if (leftSignal==1):
						setSpeedsProxy("F", 3, "B", 5)
					elif (leftSignal==2):
						setSpeedsProxy("F", 5, "B", 3)
					else:
						setSpeedsProxy("F", 3, "B", 3)
				if (rightSignal!=0):
				#received right signal
					rightFlag = True
					if (rightSignal==1):
						setSpeedsProxy("B", 5, "F", 3)
					elif (rightSignal==2):
						setSpeedsProxy("B", 3, "F", 5)
					else:
						setSpeedsProxy("B", 3, "F", 3)
		else:
		#the robot is collision,so robot is stopped
			setSpeedsProxy("F", 0, "F", 0)
			if (timeInterval>20):
			#Time interval of more than 20 seconds, check the battery status. The battery is not charging, the robot moves forward, re-looking for the correct location of the charging station
				setSpeedsProxy( "F", 10, "F", 10)
				time.sleep(10)
				bumperCrashCount = 0
def setAutoCharging(req):
	global chargingFlag
	batteryFlag = getBatteryProxy().ChargingFlag
	if (not batteryFlag):
		pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size = 10)
		nav_goal = PoseStamped()
		nav_goal.header.frame_id = "map"
		nav_goal.pose.position.x = 1.40
		nav_goal.pose.position.y = 5.10
		nav_goal.pose.position.z = 0.0
		nav_goal.pose.orientation.x = 0.0
		nav_goal.pose.orientation.y = 0.0
		nav_goal.pose.orientation.z = 0.9
		nav_goal.pose.orientation.w = 0.8
		pub.publish(nav_goal)
	return SetAutoChargingResponse()

if __name__ == "__main__":
    	rospy.init_node('auto_charging_node', anonymous=False)
	rospy.Subscriber("/battery_topic", BatteryInfo, batteryCallback)
	rospy.Subscriber("/cmd_vel", Twist, twistCallback)
	rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, poseCallback)
	auto_charging_service = rospy.Service("setAutoCharging", SetAutoCharging, setAutoCharging)
    	rospy.spin()


