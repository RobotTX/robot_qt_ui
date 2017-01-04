#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
<<<<<<< HEAD
#include <tf2_ros/transform_broadcaster.h>
#include <math.h>
#include "std_srvs/Empty.h"
#include <gobot_move/GetEncoders.h>
#include <tf2/LinearMath/Quaternion.h>

#define DIAMETER 0.15
#define WHEELBASE 0.345
#define PI 3.14159
=======
#include <gobot_move/GetEncoders.h>
#include "std_srvs/Empty.h"
>>>>>>> 404f128949db180d9a89974861b5a14107c0f937

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;
  ros::service::waitForService("resetEncoders");
  ros::service::waitForService("getEncoders");
  ros::ServiceClient resetEncodersClient = n.serviceClient<std_srvs::Empty>("resetEncoders", false);
  std_srvs::Empty arg;
  resetEncodersClient.call(arg);
<<<<<<< HEAD
  ros::ServiceClient client = n.serviceClient<gobot_move::GetEncoders>("getEncoders");
  gobot_move::GetEncoders srv;

=======
  ros::ServiceClient client = n.serviceClient<gobot_move::GetEncoders>("getEncoders", true);
  gobot_move::GetEncoders srv;
  
>>>>>>> 404f128949db180d9a89974861b5a14107c0f937
  double currentLSpeed = 0.0;
  double currentRSpeed = 0.0;

  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 20);

  ros::Time current_time = ros::Time::now(), last_time = ros::Time::now();

  ros::Rate r(40);

  static tf2_ros::TransformBroadcaster broadcaster;

<<<<<<< HEAD
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

=======
  ros::Rate r(20);

  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.frame_id = "/odom"; 
  odom_trans.child_frame_id = "/base_link";

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
>>>>>>> 404f128949db180d9a89974861b5a14107c0f937
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
