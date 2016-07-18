#ifndef MAP_TRANSFER
#define MAP_TRANSFER

#include "ros/ros.h"
#include "gobot_software/Port.h"
#include "nav_msgs/OccupancyGrid.h"
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

void sendMap(const std::vector<uint8_t>& my_map);
void getMap(const nav_msgs::OccupancyGrid::ConstPtr& msg);
bool startMap(gobot_software::Port::Request &req, gobot_software::Port::Response &res);
bool stopMap(gobot_software::Port::Request &req, gobot_software::Port::Response &res);


#endif
