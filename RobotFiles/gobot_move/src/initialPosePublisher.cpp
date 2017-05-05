#include <iostream>
#include <fstream>
#include <stdio.h>
#include "ros/ros.h"
#include <time.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"

int main(int argc, char* argv[] ){

    try { 

        ros::init(argc, argv, "initial_pose_publisher");
        ros::NodeHandle n;

        float position_x(0.0), position_y(0.0);
        float angle_x(0.0), angle_y(0.0), angle_z(0.0), angle_w(0.0);
        
        // if we can read the file that contains the initial position then we update <initialPose>
        std::ifstream file("/home/gtdollar/computer_software/Robot_Infos/initialPose.txt", std::ios::in);
        
        if(file){
            file >> position_x >> position_y >> angle_x >> angle_y >> angle_z >> angle_w;
            file.close();
        }

        geometry_msgs::PoseWithCovarianceStamped initialPose;
        initialPose.header.frame_id = "map";
        initialPose.header.stamp = ros::Time::now();
        initialPose.pose.pose.position.x = position_x;
        initialPose.pose.pose.position.y = position_y;
        initialPose.pose.pose.orientation.x = angle_x;
        initialPose.pose.pose.orientation.y = angle_y;
        initialPose.pose.pose.orientation.z = angle_z;
        initialPose.pose.pose.orientation.w = angle_w;

        ros::Publisher initial_pose_publisher = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);
        // TO DO, test if ros::spinOnce() has the same effect as sleep
        // we wait for the topic to be created so we can publish on it
        sleep(3);
        initial_pose_publisher.publish(initialPose);
        ros::spin();

    } catch (std::exception& e) {
        std::cerr << "(initial_pose_publisher) Exception: " << e.what() << std::endl;
    }

    return 0;
}