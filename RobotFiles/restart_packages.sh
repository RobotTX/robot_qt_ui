#!/bin/bash
export ROS_PACKAGE_PATH=/home/gtdollar/catkin_ws/src:/opt/ros/jade/share:/opt/ros/jade/stacks
. /opt/ros/jade/setup.sh
. /home/gtdollar/catkin_ws/devel/setup.sh

#roslaunch gobot_move slam.launch &
#roskill "/robot_pos_transfer"

#sleep(5) 
robot_pos_transfer_pid=`ps -ef | awk '$NF~"robot_pos_transfer" {print $2}'`
kill $robot_pos_transfer_pid & roslaunch gobot_software slam.launch &


#roslaunch gobot_software slam.launch &
