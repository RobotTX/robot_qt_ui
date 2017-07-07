#!/usr/bin/env python  
#refernence: http://answers.ros.org/question/29706/twist-message-example-and-cmd_vel/  
import roslib; roslib.load_manifest('move_base')
import rospy
from geometry_msgs.msg import Twist

#fileHandle = open ( 'twistlog.txt', 'w' )
def turnEnd():
	pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
	pub.publish(twist)

def listener():
	rospy.init_node('turn_node')
	pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
	twist = Twist()
	twist.linear.x = 0
	twist.linear.y = 0
	twist.linear.z = 0
	twist.angular.x = 0
	twist.angular.y = 0
	twist.angular.z = 0.2
	pub.publish(twist)
	rospy.spin()

if __name__ == '__main__':
    listener()

