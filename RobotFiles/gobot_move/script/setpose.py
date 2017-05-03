#!/usr/bin/env python
# license removed for brevity
import rospy
import time
from geometry_msgs.msg import PoseWithCovarianceStamped


if __name__ == '__main__':
	try:
		rospy.init_node('initial_pose_pub', anonymous=False)
		pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size = 10 )
		nav_pose = PoseWithCovarianceStamped()
		i=0
		while i<100000:
			i=i+1
			nav_pose.header.frame_id='map'
			nav_pose.pose.pose.position.x = -1.32290780544
			nav_pose.pose.pose.position.y = 6.986808472
			nav_pose.pose.pose.position.z = 0.0
			nav_pose.pose.pose.orientation.x = 0.0
			nav_pose.pose.pose.orientation.y = 0.0
			nav_pose.pose.pose.orientation.z = -0.715217051432
			nav_pose.pose.pose.orientation.w = 0.698902403302
			pub.publish(nav_pose)
		#rospy.spin()
	except rospy.ROSInterruptException:
        	pass
