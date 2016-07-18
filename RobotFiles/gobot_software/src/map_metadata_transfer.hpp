#ifndef MAP_METADATA_TRANSFER
#define MAP_METADATA_TRANSFER

#include "ros/ros.h"
#include "gobot_software/Port.h"
#include "nav_msgs/MapMetaData.h"
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

void sendMetaData(const std::string& metadata_string);
void getMetaData(const nav_msgs::MapMetaData::ConstPtr& msg);
bool startMeta(gobot_software::Port::Request &req, gobot_software::Port::Response &res);
bool stopMeta(gobot_software::Port::Request &req, gobot_software::Port::Response &res);

#endif
