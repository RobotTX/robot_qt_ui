#ifndef READ_NEW_MAP
#define READ_NEW_MAP

#include "ros/ros.h"
#include <iostream>
#include <cstdint>
#include <string>
#include <boost/bind.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include "std_msgs/String.h"
#include <fstream>
#include "nav_msgs/OccupancyGrid.h"

#define PORT 5601

using boost::asio::ip::tcp;


#endif
