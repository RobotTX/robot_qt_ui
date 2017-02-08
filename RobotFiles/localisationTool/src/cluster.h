#ifndef CLUSTER_H
#define CLUSTER_H

#include "ros/ros.h"
#include <geometry_msgs/Pose.h> 
#include <vector>

using namespace std;

class Cluster
{
	public:
		Cluster(std::vector<geometry_msgs::Pose>  poses);
		Cluster(){};

		//float calculateDiameter();
		std::pair<int,float> calculateMaxDistPoint ();
		geometry_msgs::Pose calculateCentroid();
		std::vector<geometry_msgs::Pose>  getPoses(void){return poses;}
		void remove(int i);
		int getSize(){return poses.size();}
		int setPoses(std::vector<geometry_msgs::Pose> posesV){poses.swap(posesV); }
  		geometry_msgs::Pose getCentroid(){return centroid;}
  		float getDiameter(){return diameter;}
		std::pair<int,float> getMaxDistPoint () {return maxPoint;}


 
	private:  
		std::vector<geometry_msgs::Pose>  poses;
		geometry_msgs::Pose centroid ;
		float diameter;	
		float distancePoints (geometry_msgs::Pose A,geometry_msgs::Pose B);
		std::pair<int,float> maxPoint ;
};

#endif