#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <gobot_move/GetEncoders.h>
#include "std_srvs/Empty.h"

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
  
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 10);
  tf::TransformBroadcaster odom_broadcaster;

  double x = 0.0;
  double y = 0.0;
  double th = 0.0;

  double vx = 0.0;
  double vy = 0.0;
  double vth = 0.0;

  double Dr = 0.0;
  double Dl = 0.0;
  double speed = 0.0;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();
  
  tf::TransformBroadcaster broadcaster;

  ros::Rate r(20);

  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.frame_id = "/small_robot/odom"; 
  odom_trans.child_frame_id = "/small_robot/base_link";

  while(n.ok()){

    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();

    if ( client.call(srv))
    {
      
      currentLSpeed = static_cast<double> (srv.response.values[0]);
      currentRSpeed = static_cast<double> (srv.response.values[1]);
      //ROS_INFO("Got speed  : %f / %f", currentLSpeed, currentRSpeed);
    }
    else
    {
      ROS_ERROR("Failed to call service getSpeed");
    }

    if ( abs(currentRSpeed) < 50  && abs(currentLSpeed) < 50 )
    {
      if (currentRSpeed > 0 )
      {
        Dr = (currentRSpeed * 0.010 + 0.005);
      }
      else if (currentRSpeed < 0)
      {
        Dr = -(abs(currentRSpeed) * 0.010 + 0.005);
      }
      else
      {
        Dr = 0.0;
      }
      if (currentLSpeed > 0 )
      {
        Dl = (currentLSpeed * 0.010 + 0.005);
      }
      else if (currentLSpeed < 0)
      {
        Dl = -(abs(currentLSpeed) * 0.010 + 0.005);
      }
      else
      {
        Dl = 0.0;
      }
      double currentSpeed = (currentRSpeed - currentLSpeed)/2.0;
      if ( currentSpeed > 0.0 ){
        vth = currentSpeed * 0.063 + 0.0025;
      }
      else if ( currentSpeed < 0.0 ){
        vth = 0.0 - (abs(currentSpeed) * 0.063 + 0.0025);
      }
      else{
        vth = 0.0;
      }
    }
    else
      Dr = 0.0;
      Dl = 0.0;
      vth = 0.0;

    speed = (Dr + Dl)/2;
    vx = speed;
    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat;
    odom_quat = tf::createQuaternionMsgFromRollPitchYaw(0, 0, th); 

    //first, we'll publish the transform over tf
    odom_trans.header.stamp = current_time;

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    //odom_trans.transform.rotation = odom_quat;
    odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(th);

    //send the transform
    //odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "/small_robot/odom";
    odom.child_frame_id = "/small_robot/base_link";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = vth;

    //publish the message
    broadcaster.sendTransform(odom_trans);
    odom_pub.publish(odom);

    last_time = current_time;
    r.sleep();
  }
  return 0;
}
