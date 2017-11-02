#include <iostream>
#include <fstream>
#include <stdio.h>
#include "ros/ros.h"
#include <time.h>
#include <tf/transform_broadcaster.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"

int main(int argc, char* argv[] ){

    try { 

        ros::init(argc, argv, "initial_pose_publisher");
        ros::NodeHandle n; 

        double position_x(0.0), position_y(0.0);
        double angle_x(0.0), angle_y(0.0), angle_z(0.0), angle_w(1.0);

        std::string robotPos;

        // from this file we retrieve an int that tells us if the robot should start at the charging station
        // or at the last known position
        if(n.hasParam("robot_position_file")){
            std::string robotPositionFile;
            n.getParam("robot_position_file", robotPositionFile);

            std::fstream file(robotPositionFile, std::ios::in);
            file >> robotPos;
            file.close();

            file.open(robotPositionFile, std::fstream::out | std::fstream::trunc);
            file << "1";
            file.close();

        } else 
            std::cout << "initialPosePublisher could not find the parameter </initialPosePublisher/robot_position_file>" << std::endl;

        std::cout << "RobotPos : " << robotPos << std::endl;

        if(std::stoi(robotPos) == 0){
            // if we can read the file that contains the initial position then we update <initialPose>
            if(n.hasParam("last_known_position_file")){
                std::string fileName;
                n.getParam("last_known_position_file", fileName);
                std::ifstream file(fileName, std::ios::in);
                if(file){
                    file >> position_x >> position_y >> angle_x >> angle_y >> angle_z >> angle_w;
                    std::cout << "values I got from lastKnownPosition.txt " << std::to_string(position_x) << " " << std::to_string(position_y) << " " << std::to_string(angle_x) << " " << std::to_string(angle_y) << " " << std::to_string(angle_z) << " " << std::to_string(angle_w) << std::endl;
                    file.close();
                } 
            } else 
                std::cout << "initialPosePublisher could not find the parameter <last_known_position_file>" << std::endl;
                
        } else if(std::stoi(robotPos) == 1){
            if(n.hasParam("home_file")){
                std::string fileName;
                n.getParam("home_file", fileName);
                std::ifstream file(fileName, std::ios::in);
                if(file){
                    file >> position_x >> position_y >> angle_x >> angle_y >> angle_z >> angle_w;
                    std::cout << "values I got from home.txt " << std::to_string(position_x) << " " << std::to_string(position_y) << " " << std::to_string(angle_x) << " " << std::to_string(angle_y) << " " << std::to_string(angle_w) << std::endl;
                    file.close();
                } 
            } else 
                std::cout << "initialPosePublisher could not find the parameter <home_file>" << std::endl;
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
        
        // we wait for the topic to be created so we can publish on it
        sleep(3);

        initial_pose_publisher.publish(initialPose);

        ros::spin();

    } catch (std::exception& e) {

        std::cerr << "(initial_pose_publisher) Exception: " << e.what() << std::endl;

    }

    return 0;
}
