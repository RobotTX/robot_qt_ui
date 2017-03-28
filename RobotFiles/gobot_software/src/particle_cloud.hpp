#ifndef PARTICLE_CLOUD
#define PARTICLE_CLOUD

#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "gobot_software/Port.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>

#include <boost/bind.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>

using boost::asio::ip::tcp;

struct Position {
    double x;
    double y;
};

// Connects this node to the application opening a socket on a dedicated port
bool connectParticleCloud(gobot_software::Port::Request &req, gobot_software::Port::Response &res);

// Send the particle cloud to the application as an array of Point (we don't send orientations)
void sendParticleCloud(const std::vector<Position>& particle_cloud);

// Gets the particle cloud from the topic /particle_cloud, the different with what we send is that it also contains a timestamp 
// which we don't bother sending, we
void getParticleCloud(const geometry_msgs::PoseArray& particle_cloud);

// the service to subscribe to the /particle_cloud topic in order to start sending the data
bool sendParticleCloudService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

// the service to unsubscribe to the /particle_cloud topic in order to stop sending the data
bool stopParticleCloudService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

#endif