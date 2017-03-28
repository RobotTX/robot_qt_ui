#!/usr/bin/env python
import rospy,time
from sensor_msgs.msg import Range
from sonar.msg import DistanceMsg
from math import pow

range_msg1 = Range()
range_msg2 = Range()
range_msg3 = Range()
range_msg4 = Range()
range_timer = time.time()

range_msg1.radiation_type = Range.INFRARED
range_msg1.header.frame_id = "sonar_zero_range"
range_msg1.field_of_view = 0.1
range_msg1.min_range = 0.15
range_msg1.max_range = 3.0

range_msg2.radiation_type = Range.INFRARED
range_msg2.header.frame_id = "sonar_two_range"
range_msg2.field_of_view = 0.1
range_msg2.min_range = 0.15
range_msg2.max_range = 3.0

range_msg3.radiation_type = Range.INFRARED
range_msg3.header.frame_id = "sonar_four_range"
range_msg3.field_of_view = 0.1
range_msg3.min_range = 0.15
range_msg3.max_range = 3.0

range_msg4.radiation_type = Range.INFRARED
range_msg4.header.frame_id = "sonar_six_range"
range_msg4.field_of_view = 0.1
range_msg4.min_range = 0.15
range_msg4.max_range = 3.0

range_msg4.radiation_type = Range.INFRARED
range_msg4.header.frame_id = "sonar_eight_range"
range_msg4.field_of_view = 0.1
range_msg4.min_range = 0.15
range_msg4.max_range = 3.0

range_msg4.radiation_type = Range.INFRARED
range_msg4.header.frame_id = "sonar_A_range"
range_msg4.field_of_view = 0.1
range_msg4.min_range = 0.15
range_msg4.max_range = 3.0

range_msg4.radiation_type = Range.INFRARED
range_msg4.header.frame_id = "sonar_C_range"
range_msg4.field_of_view = 0.1
range_msg4.min_range = 0.15
range_msg4.max_range = 3.0

def callback(msg):
    pub1 = rospy.Publisher("/sensors/sonar_sensor/sonar_zero_range", Range, queue_size=10)
    pub2 = rospy.Publisher("/sensors/sonar_sensor/sonar_two_range", Range, queue_size=10)
    pub3 = rospy.Publisher("/sensors/sonar_sensor/sonar_four_range", Range, queue_size=10)
    pub4 = rospy.Publisher("/sensors/sonar_sensor/sonar_six_range", Range, queue_size=10)
    pub5 = rospy.Publisher("/sensors/sonar_sensor/sonar_eight_range", Range, queue_size=10)
    pub6 = rospy.Publisher("/sensors/sonar_sensor/sonar_A_range", Range, queue_size=10)
    pub7 = rospy.Publisher("/sensors/sonar_sensor/sonar_C_range", Range, queue_size=10)
    loop(msg,pub1,pub2,pub3,pub4)
    
def loop(msg,pub_range1,pub_range2,pub_range3,pub_range4):
    global range_timer
    millis = time.time()
    if ( (millis-range_timer) * 1000 > 50):
	stamp = rospy.Time.now()
        distance1 = msg.distance1/100.0
	distance2 = msg.distance2/100.0
	distance3 = msg.distance3/100.0
	distance4 = msg.distance4/100.0
        distance5 = msg.distance5/100.0
        distance6 = msg.distance6/100.0
        distance7 = msg.distance7/100.0
        range_msg1.range = distance1
        range_msg1.header.stamp = stamp
	range_msg2.range = distance2
	range_msg2.header.stamp = stamp
	range_msg3.range = distance3
	range_msg3.header.stamp = stamp
	range_msg4.range = distance4
	range_msg4.header.stamp = stamp
	range_msg5.range = distance5
        range_msg5.header.stamp = stamp
	range_msg6.range = distance6
        range_msg6.header.stamp = stamp
	range_msg7.range = distance7
        range_msg7.header.stamp = stamp
        pub_range1.publish(range_msg1)
	pub_range2.publish(range_msg2)
	pub_range3.publish(range_msg3)
	pub_range4.publish(range_msg4)
        pub_range5.publish(range_msg5)
        pub_range6.publish(range_msg6)
        pub_range7.publish(range_msg7)
        range_timer = time.time()

def listener():
    rospy.init_node('get_sonar_listener')
    rospy.Subscriber("/sonar_topic", DistanceMsg, callback)#/cmd_vel
    rospy.spin()

if __name__ == '__main__':
    listener()
