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
#include <chrono>
#include <thread>

using boost::asio::ip::tcp;

/**
 * Send the map to the software as pixels (0 to 255)
 */
void sendMap(const std::vector<uint8_t>& my_map);

/**
 * Send the real map to the software as percentage of chance of a wall (-1 to 100)
 */
void sendMap(const std::vector<int8_t>& my_map);

/**
 * Called when map is published
 */
void getMap(const nav_msgs::OccupancyGrid::ConstPtr& msg);

/**
 * Service called to start the listening to the map topic and transfer to the software
 */
bool startMap(gobot_software::Port::Request &req, gobot_software::Port::Response &res);

/**
 * Service called to stop the listening to the map topic and transfer to the software
 */
bool stopMap(gobot_software::Port::Request &req, gobot_software::Port::Response &res);


#endif
