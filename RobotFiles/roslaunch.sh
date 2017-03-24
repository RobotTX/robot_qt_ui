#!/bin/bash

export ROS_PACKAGE_PATH=/home/gtdollar/catkin_ws/src:/opt/ros/jade/share:/opt/ros/jade/stacks
. /opt/ros/jade/setup.sh
. /home/gtdollar/catkin_ws/devel/setup.sh

roslaunch gobot_software slam.launch &
