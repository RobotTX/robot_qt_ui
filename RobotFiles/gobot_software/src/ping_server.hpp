#ifndef PING_SERVER_HPP
#define PING_SERVER_HPP

#include <iostream>
#include <string>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <boost/asio.hpp>
#include <boost/smart_ptr.hpp>
#include <fstream>
#include <stdio.h>
#include <gobot_software/GetDockStatus.h>
#include <sonar/GetBatteryInfo.h>

using boost::asio::ip::tcp;

bool isServer(const std::string IP, const std::string ssid);

bool checkIPList(const std::string ssid);

void client(ros::Publisher& publisher, const bool checkOldServer, std::string ssid, const std::string serverFile, const std::string pingFile);

#endif
