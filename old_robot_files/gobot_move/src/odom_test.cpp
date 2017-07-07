#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <ros/ros.h>
#include "geometry_msgs/Twist.h"

void timerStopTurn(const ros::TimerEvent&);
void timerStopTranslation(const ros::TimerEvent&);


ros::Publisher teleop_pub;

int counter(0);
ros::Timer timerTranslation;
ros::Timer timerTurn;


void timerStopTurn(const ros::TimerEvent&){
    ros::NodeHandle n;
    std::cout << "Stop turning and start moving" << std::endl;

    if(counter < 4){
        geometry_msgs::Twist twist;
        twist.linear.x = 0.2;
        twist.linear.y = 0;
        twist.linear.z = 0;
        twist.angular.x = 0;
        twist.angular.y = 0;
        twist.angular.z = 0;

        teleop_pub.publish(twist);

        timerTranslation = n.createTimer(ros::Duration(15.0), timerStopTranslation, true);
    } else {

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

}

void timerStopTranslation(const ros::TimerEvent&){
    ros::NodeHandle n;
    std::cout << "Stop moving and start turning" << std::endl;

    geometry_msgs::Twist twist;
    twist.linear.x = 0;
    twist.linear.y = 0;
    twist.linear.z = 0;
    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = 0.2;

    teleop_pub.publish(twist);
    counter++;

    timerTurn = n.createTimer(ros::Duration(7.853975), timerStopTurn, true);

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

    ros::spinOnce();

    timerTranslation = n.createTimer(ros::Duration(15.0), timerStopTranslation, true);

    ros::spin();

    return 0;
}