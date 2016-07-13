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

#define CMD_PORT 5600

using boost::asio::ip::tcp;

bool execCommand(ros::NodeHandle n, std::vector<std::string> command);
void session(boost::shared_ptr<tcp::socket> sock, ros::NodeHandle n);
void asyncAccept(boost::shared_ptr<boost::asio::io_service> io_service, 
	boost::shared_ptr<tcp::acceptor> m_acceptor, 
	ros::NodeHandle n);
void startRobotPos(ros::NodeHandle n);
void stopRobotPos();
void startMetadata(ros::NodeHandle n);
void stopMetadata();
void startMap(ros::NodeHandle n);
void stopMap();
void server(unsigned short port, ros::NodeHandle n);
void getPorts(boost::shared_ptr<tcp::socket> sock, ros::NodeHandle n);
bool sendMessageToPc(boost::shared_ptr<tcp::socket> sock, std::string message);

#endif
