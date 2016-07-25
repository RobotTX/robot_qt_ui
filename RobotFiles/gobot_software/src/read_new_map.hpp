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

/**
 * Main function that read the new map with its id and metadata, and save it
 */
void session(boost::shared_ptr<tcp::socket> sock, ros::NodeHandle n);

/**
 * Accept the connection asynchronously
 */
void asyncAccept(boost::shared_ptr<boost::asio::io_service> io_service, boost::shared_ptr<tcp::acceptor> m_acceptor, ros::NodeHandle n);

/**
 * Called when we have been disconnected to the server
 */
void serverDisconnected(const std_msgs::String::ConstPtr& msg);

#endif
