#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <math.h>
#include "std_srvs/Empty.h"
#include <gobot_move/GetEncoders.h>
#include <tf2/LinearMath/Quaternion.h>

#define DIAMETER 0.15
#define WHEELBASE 0.345
#define PI 3.14159

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;
  ros::service::waitForService("resetEncoders");
  ros::service::waitForService("getEncoders");
  ros::ServiceClient resetEncodersClient = n.serviceClient<std_srvs::Empty>("resetEncoders", false);
  std_srvs::Empty arg;
  resetEncodersClient.call(arg);

  ros::ServiceClient client = n.serviceClient<gobot_move::GetEncoders>("getEncoders", true);
  gobot_move::GetEncoders srv;

  double currentLSpeed = 0.0;
  double currentRSpeed = 0.0;

  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 20);

  ros::Time current_time = ros::Time::now(), last_time = ros::Time::now();

  ros::Rate r(40);

  static tf2_ros::TransformBroadcaster broadcaster;

  double last_rencoder = 0.0, last_lencoder = 0.0;
  double th_th = 0.0;

  double x = 0.0, y = 0.0;

  while(n.ok()){

    current_time = ros::Time::now();

    if(client.call(srv)) {
      currentLSpeed = static_cast<double> (srv.response.values[0]);
      currentRSpeed = static_cast<double> (srv.response.values[1]);
    } else
      ROS_ERROR("Failed to call service getEncoders");

    double timeDelta = (current_time - last_time).toSec();

    double erDis = (currentRSpeed-last_rencoder) / 979.4;
    double elDis = (currentLSpeed-last_lencoder) / 979.4;

    double rDis = erDis * DIAMETER * PI;
    double lDis = elDis * DIAMETER * PI;

    double encoderDt = (rDis + lDis) / 2;
    double encoderth = (rDis - lDis) / WHEELBASE;

    double encoder_x = encoderDt * cos(encoderth/2 + th_th);
    double encoder_y = encoderDt * sin(encoderth/2 + th_th);

    double vx = encoderDt / timeDelta;
    double vy = 0.0;
    double vTh = encoderth / timeDelta;

    last_lencoder = currentLSpeed;
    last_rencoder = currentRSpeed;

    x += encoder_x;
    y += encoder_y;

    th_th += encoderth;

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.frame_id = "/odom"; 
    odom_trans.child_frame_id = "/base_link";
    
    tf2::Quaternion odom_quat;
    odom_quat.setRPY(0, 0, th_th); 

    odom_trans.header.stamp = current_time;

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;

    odom_trans.transform.rotation.x = odom_quat.x();
    odom_trans.transform.rotation.y = odom_quat.y();
    odom_trans.transform.rotation.z = odom_quat.z();
    odom_trans.transform.rotation.w = odom_quat.w();
    broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "/odom";
    odom.child_frame_id = "/base_link";

    // set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation.x = odom_quat.x();
    odom.pose.pose.orientation.y = odom_quat.y();
    odom.pose.pose.orientation.z = odom_quat.z();
    odom.pose.pose.orientation.w = odom_quat.w();

    // set the velocity
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vTh;

    odom_pub.publish(odom);

    last_time = current_time;
    
    r.sleep();

  }

  return 0;
}
