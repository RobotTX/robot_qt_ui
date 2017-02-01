#ifndef PLAY_PATH
#define PLAY_PATH

#include "ros/ros.h"
#include <fstream>
#include <vector>
#include <iostream>
#include "geometry_msgs/PoseStamped.h"
#include <boost/smart_ptr.hpp>
#include <boost/thread.hpp>
#include "actionlib_msgs/GoalStatusArray.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "std_srvs/Empty.h"
#include "std_msgs/String.h"
#include <boost/bind.hpp>
#include <string>
#include <actionlib/client/simple_action_client.h>
#include <memory>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

struct PathPoint {
	double x;
	double y;
	double waitingTime;
};

bool startPathService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

void startPath(ros::NodeHandle n, std::shared_ptr<MoveBaseClient> ac);

void getStatus(const actionlib_msgs::GoalStatusArray::ConstPtr& _status);

bool pausePathService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

void pausePath(ros::NodeHandle n, std::shared_ptr<MoveBaseClient> ac);

bool stopPathService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

bool stopPath(ros::NodeHandle n, std::shared_ptr<MoveBaseClient> ac);

bool goHomeService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

void goHome(ros::NodeHandle n, std::shared_ptr<MoveBaseClient> ac, const PathPoint& home);

bool stopGoingHomeService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

void stopGoingHome(ros::NodeHandle n, std::shared_ptr<MoveBaseClient> ac);

#endif