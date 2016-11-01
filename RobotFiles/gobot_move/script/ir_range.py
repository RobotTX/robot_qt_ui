#!/usr/bin/env python
import rospy,time
from sensor_msgs.msg import Range
from IR.msg import IRMsg
from math import pow
from wheel.srv import *

range_msg = Range()
range_timer = time.time()
frameid = "_ir_range"
A_CONST = 3619
B_CONST = -1.36
setSpeedsProxy = rospy.ServiceProxy( "setSpeeds", SetSpeeds )
getSpeedProxy = rospy.ServiceProxy( "getSpeed", GetSpeed)

def setSpeed(rDirection, rSpeed, lDirection, lSpeed):
    print rDirection, rSpeed, lDirection, lSpeed
    setSpeedsProxy( str(lDirection), lSpeed, str(rDirection), rSpeed )

def callback(msg):
    range_msg.radiation_type = Range.INFRARED
#    range_msg.header.frame_id =  msg.side + frameid
    range_msg.header.frame_id = "ir_range"
    range_msg.field_of_view = 0.05
    range_msg.min_range = 0.05
    range_msg.max_range = 0.5
#    if ( msg.side == "rear" ):
#        pub = rospy.Publisher("/sensors/ir_sensor/rear_ir_range_topic", Range, queue_size=10)
#    elif ( msg.side == "left" ):
#        pub = rospy.Publisher("/sensors/ir_sensor/left_ir_range_topic", Range, queue_size=10)
#    else:
#        pub = rospy.Publisher("/sensors/ir_sensor/right_ir_range_topic", Range, queue_size=10)
    pub = rospy.Publisher("/sensors/ir_sensor/ir_range_topic", Range, queue_size=10)
    loop(msg,pub)
    
def loop(msg,pub_range):
    global range_timer
    millis = time.time()
    if ( (millis-range_timer) * 1000 > 50):
        power = msg.value
        side = msg.side
        if ( power <= 600 ):
            range = 0.5
        else:
            c = pow( power, B_CONST )
            range = A_CONST * c
        range_msg.range = range
        range_msg.header.stamp = rospy.Time.now()
        pub_range.publish(range_msg)
        range_timer = time.time()
        if ( range < 0.25 ):
            rightspeed = getSpeedProxy(2).speed
            leftspeed = getSpeedProxy(1).speed
            if ( side == "rear" and rightspeed < 0 and leftspeed < 0 ) or (( side =="right" or side=="left") and (rightspeed > 0 and leftspeed > 0 )):
                setSpeed("F", 0, "F", 0)

def listener():
    rospy.init_node('get_ir_listener')
    rospy.Subscriber("/IR_topic", IRMsg, callback)#/cmd_vel
    rospy.spin()

if __name__ == '__main__':
    listener()
