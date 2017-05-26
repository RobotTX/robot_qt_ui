#!/usr/bin/env python
import rospy
import time
import os
import commands
from math import sqrt, pow
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseActionGoal
from wheel.srv import *
from sonar.msg import BumperMsg, IrSignalMsg, ShortSignalMsg, BatteryInfo
from sonar.srv import *
from std_srvs.srv import Empty, EmptyResponse

homeFile = rospy.get_param("home_file")

setSpeedsProxy = rospy.ServiceProxy( "setSpeeds", SetSpeeds )
getIrSignalProxy = rospy.ServiceProxy( "getIrSignal", GetIrSignal)
getShortSignalProxy = rospy.ServiceProxy( "getShortSignal", GetShortSignal)
getBumpersProxy = rospy.ServiceProxy( "getBumpers", GetBumpers )
getBatteryProxy = rospy.ServiceProxy("getBatteryInfo", GetBatteryInfo)

goal_info = MoveBaseActionGoal()
setChargingGoal = False
chargingFlag = False
# true if the robot has a non-null linear velocity
twistFlag = False
# square of the distance between the robot and the charging station
distance = 10
rearFlag = True
leftFlag = False
rightFlag = False
bumperCrashCount = 0
beginTime = time.time()

def batteryCallback(batteryInfo):
	global setChargingGoal,chargingFlag
	# not sure we should do that, send the robot to charging station as soon as its battery level drops below 50 % ?
	if (batteryInfo.RemainCapacity< 50) and (not setChargingGoal):
		setChargingGoal = True
		nav_goal = MoveBaseActionGoal()
		nav_goal.header.frame_id = "map"
		nav_goal.header.stamp = rospy.Time.now()

		nav_goal.goal.target_pose.pose.position.x = 1.40
		nav_goal.goal.target_pose.pose.position.y = 5.10
		nav_goal.pose.position.z = 0.0
		nav_goal.pose.orientation.x = 0.0
		nav_goal.pose.orientation.y = 0.0
		nav_goal.goal.target_pose.pose.orientation.z = 0.9
		nav_goal.goal.target_pose.pose.orientation.w = 0.8
		pub.publish(nav_goal) #publish gobot to move goal location

	if (chargingFlag) :
		auto_charging()
	else :
		if (distance < 0.16) and (not twistFlag):
			os.system("rosnode kill /cmd_vel_listener")
			os.system("rosnode kill /move_base")
			chargingFlag = True
			auto_charging()

def twistCallback(twistMsg):
	global twistFlag
	if (twistMsg.linear.x==0) and (twistMsg.linear.y == 0) and (twistMsg.linear.z == 0):
		twistFlag = False
	else:
		twistFlag = True

def poseCallback(msg):
	# computes the distance between the robot and the charging station (whose coordinates are retrieved through the home.txt file)
	# this is used in the battery callback to determine if the charging flag should be set to true or not
	global twistFlag, distance
	file = open(homeFile, "r")
	if file.closed:
		print "could not open home.txt"
	home_parameters = file.readline().split()
	print len(home_parameters)
	if(len(home_parameters) == 6):
		goal_info.goal.target_pose.pose.position.x = float(home_parameters[0])
		goal_info.goal.target_pose.pose.position.y = float(home_parameters[1])
	
	robot_x = msg.pose.pose.position.x
	robot_y = msg.pose.pose.position.y
	distance = pow(robot_x - goal_info.goal.target_pose.pose.position.x, 2) + pow(robot_y - goal_info.goal.target_pose.pose.position.y, 2)
	print "square distance to charging station: ", distance

def auto_charging():
	global rearFlag, leftFlag,rightFlag,beginTime, bumperCrashCount
	batteryFlag = getBatteryProxy().ChargingFlag
	bumpers = getBumpersProxy()
	irSignal = getIrSignalProxy()
	shortSignal = getShortSignalProxy()
	rearSignal = irSignal.rearSignal
	leftSignal = irSignal.leftSignal
	rightSignal = irSignal.rightSignal
	bumperValue = bumpers.values[4] + bumpers.values[5] + bumpers.values[6] + bumpers.values[7]
	if (bumperValue<4):
		bumperCrashCount = bumperCrashCount + 1
	if (bumperCrashCount == 0):
		beginTime = time.time()
		timeInterval = 0
	else:
		timeInterval = time.time() - beginTime
	
	print batteryFlag, bumperCrashCount, rearSignal, leftSignal, rightSignal, rearFlag, leftFlag, rightFlag
	if (batteryFlag):
		setSpeedsProxy("F", 0, "F", 0)
	else:
		if (bumperCrashCount==0):
			if (rearSignal == 0) and (leftSignal == 0) and (rightSignal == 0):
				if (not leftFlag ) and ( not rightFlag):
					if (rearFlag):
						setSpeedsProxy("B", 3, "F", 3)
					else:
						setSpeedsProxy("B", 3, "B", 3)
			else:
				if (rearSignal != 0):
					leftFlag = False
					rightFlag = False
					if (rearFlag):
						if (rearSignal==3):
							setSpeedsProxy("B", 3, "B", 3)
						elif (rearSignal==2):
							setSpeedsProxy("F", 3, "B", 3)
						elif (rearSignal==1):
							setSpeedsProxy("B", 3, "F", 3)
					else:
						setSpeedsProxy("B",4, "B", 4)
				if (leftSignal!=0):
					leftFlag = True
					if (leftSignal==1):
						setSpeedsProxy("F", 3, "B", 5)
					elif (leftSignal==2):
						setSpeedsProxy("F", 5, "B", 3)
					else:
						setSpeedsProxy("F", 3, "B", 3)
				if (rightSignal!=0):
					rightFlag = True
					if (rightSignal==1):
						setSpeedsProxy("B", 5, "F", 3)
					elif (rightSignal==2):
						setSpeedsProxy("B", 3, "F", 5)
					else:
						setSpeedsProxy("B", 3, "F", 3)
		else:
			setSpeedsProxy("F", 0, "F", 0)
			if (timeInterval>20):
				setSpeedsProxy( "F", 10, "F", 10)
				time.sleep(10)
				bumperCrashCount = 0

def setAutoCharging(req):

	print "setAutoCharging called"
	global chargingFlag
	# TODO put this guy back when testing phase is over
	batteryFlag = getBatteryProxy().ChargingFlag
	batteryFlag = False
	# if we have not already arrived at the charging station we publish the goal
	if(not batteryFlag):
		pub = rospy.Publisher("/move_base/goal", MoveBaseActionGoal, queue_size = 10)
		nav_goal = MoveBaseActionGoal()
		nav_goal.header.frame_id = "map"
		nav_goal.header.stamp = rospy.Time.now()
		print "home file is ", homeFile
		file = open(homeFile, "r")
		if file.closed:
			print "could not open home.txt"
		home_parameters = file.readline().split()
		print len(home_parameters)
		if(len(home_parameters) == 6):
			print "home position and orientation", home_parameters[0], home_parameters[1], home_parameters[4], home_parameters[5]
			nav_goal.goal.target_pose.header.frame_id = "map"
			nav_goal.goal.target_pose.header.stamp = rospy.Time.now()
			nav_goal.goal.target_pose.pose.position.x = float(home_parameters[0])
			nav_goal.goal.target_pose.pose.position.y = float(home_parameters[1])
			nav_goal.goal.target_pose.pose.position.z = 0.0
			nav_goal.goal.target_pose.pose.orientation.x = 0.0
			nav_goal.goal.target_pose.pose.orientation.y = 0.0
			nav_goal.goal.target_pose.pose.orientation.z = float(home_parameters[4])
			nav_goal.goal.target_pose.pose.orientation.w = float(home_parameters[5])
			pub.publish(nav_goal)
		else:
			print "not enough values found in home.txt"
	return EmptyResponse()

if __name__ == "__main__":
	rospy.init_node('auto_charging_node', anonymous=False)
	rospy.Subscriber("/battery_topic", BatteryInfo, batteryCallback)
	rospy.Subscriber("/cmd_vel", Twist, twistCallback)
	rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, poseCallback)
	auto_charging_service = rospy.Service("setAutoCharging", Empty, setAutoCharging)
	rospy.spin()


