#ifndef COMMAND_SYSTEM
#define COMMAND_SYSTEM

#include "ros/ros.h"
#include "gobot_software/Port.h"
#include "gobot_software/SendMessageToPc.h"
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
#include <boost/regex.hpp>
#include <list>
#include <fstream>
#include "actionlib_msgs/GoalStatusArray.h"

using boost::asio::ip::tcp;

/**
 * Execute the received command and return whether or not it was a success
 */
bool execCommand(ros::NodeHandle n, std::vector<std::string> command);

/**
 * Main fct which receive the commands and execute them
 */
void session(boost::shared_ptr<tcp::socket> sock, ros::NodeHandle n);

/**
 * Asynchronous accept of the socket connection
 */
void asyncAccept(boost::shared_ptr<boost::asio::io_service> io_service, 
	boost::shared_ptr<tcp::acceptor> m_acceptor, 
	ros::NodeHandle n);

/**
 * Start the service to get the robot position
 */
void startRobotPos(ros::NodeHandle n);

/**
 * Stop the servcie to get the robot position
 */
void stopRobotPos();

/**
 * Start the service to get the map metadata
 */
void startMetadata(ros::NodeHandle n);

/**
 * Stop the service to get the map metadata
 */
void stopMetadata();

/**
 * Start the service to get the map metadata
 */
void startMap(ros::NodeHandle n);

/**
 * Stop the service to get the map metadata
 */
void stopMap();

/**
 * Initialize the socket & acceptor and launch the session thread
 */
void server(unsigned short port, ros::NodeHandle n);

/**
 * Get the ports used for the services
 */
void getPorts(boost::shared_ptr<tcp::socket> sock, ros::NodeHandle n);

/**
 * Send a message to the PC to tell when we are connected or if a command succeed
 */
bool sendMessageToPc(boost::shared_ptr<tcp::socket> sock, std::string message);

/**
 * Called when there is a deconnection
 */
void serverDisconnected(const std_msgs::String::ConstPtr& msg);

/**
 * Read a command from the socket
 */
std::vector<std::string> readCommand(boost::shared_ptr<tcp::socket> sock);

#endif
