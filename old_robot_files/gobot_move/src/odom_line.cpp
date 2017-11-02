#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <ros/ros.h>
#include "geometry_msgs/Twist.h"

void timerStopTurn(const ros::TimerEvent&);


ros::Publisher teleop_pub;

ros::Timer timerTurn;


void timerStopTurn(const ros::TimerEvent&){
    ros::NodeHandle n;

    std::cout << "FINISHED" << std::endl;
    geometry_msgs::Twist twist;
    twist.linear.x = 0;
    twist.linear.y = 0;
    twist.linear.z = 0;
    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = 0;

    teleop_pub.publish(twist);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "command_system");
    ros::NodeHandle n;

    teleop_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    ros::spinOnce();

    sleep(1);
    std::cout << "STARTED" << std::endl;

    geometry_msgs::Twist twist;
    twist.linear.x = 0.2;
    twist.linear.y = 0;
    twist.linear.z = 0;
    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = 0;

    teleop_pub.publish(twist);

    timerTurn = n.createTimer(ros::Duration(15), timerStopTurn, true);

    ros::spin();

    return 0;
}