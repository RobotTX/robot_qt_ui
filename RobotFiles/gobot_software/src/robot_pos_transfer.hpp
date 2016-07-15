#ifndef ROBOT_POSE_TRANSFER
#define ROBOT_POSE_TRANSFER

#include "ros/ros.h"
#include "gobot_software/Port.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/String.h"
#include <iostream>
#include <string>
#include <unistd.h>
#include <cstdint>
#include <vector>
#include <cstdlib>
#include <signal.h>
#include <boost/bind.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>

using boost::asio::ip::tcp;

void getRobotPos(const geometry_msgs::Pose::ConstPtr& msg);
void sendRobotPos(const std::string& robot_string);
bool startRobotPos(gobot_software::Port::Request &req, gobot_software::Port::Response &res);
bool stopRobotPos(gobot_software::Port::Request &req, gobot_software::Port::Response &res);

#endif
