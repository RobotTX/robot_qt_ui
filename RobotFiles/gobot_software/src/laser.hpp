#ifndef LASER_TRANSFER
#define LASER_TRANSFER

#include "ros/ros.h"
#include "gobot_software/Port.h"
#include "sensor_msgs/LaserScan.h"
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
#include <chrono>
#include <thread>

using boost::asio::ip::tcp;

bool startLaser(gobot_software::Port::Request &req, gobot_software::Port::Response &res);

void getLaserData(const sensor_msgs::LaserScan::ConstPtr& msg);

void sendLaserData(const std::vector<float>& scan);

bool stopSendingLaserData(gobot_software::Port::Request &req, gobot_software::Port::Response &res);

#endif