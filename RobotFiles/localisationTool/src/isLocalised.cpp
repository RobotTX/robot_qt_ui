
#include "cluster.h"

#include "ros/ros.h"
#include <geometry_msgs/Pose.h> 
#include <geometry_msgs/PoseArray.h> 
#include <geometry_msgs/PoseStamped.h> 
#include <std_msgs/String.h> 
#include <std_msgs/Header.h> 
#include <typeinfo>
#include <sstream>

  ros::Publisher pub ; 

int test (Cluster* dataSet,int numberData, int seq)
{
	/* 
	std_msgs::Header header = std_msgs::Header();
	header.seq = seq;
	header.stamp = ros::Time::now();
	header.frame_id = "/map";
	std::vector<geometry_msgs::Pose>  msg_poses = dataSet->getPoses();
	geometry_msgs::PoseArray msg_array ;
	msg_array.header = header;
	msg_array.poses= msg_poses;

	pub.publish(msg_array);	
	*/
	
	cout << dataSet->getMaxDistPoint().second << " ";
	// if the diameter of the luster is < 2 then all the cloud is small enough and the position is valid.
	if ( dataSet->getMaxDistPoint().second <= 2)
		return 1;
	// if the size of the cluster becomes less than 90 % of all points but the diameter is still to big then the we didn't find the robot position

	else if(dataSet->getSize() < (90*numberData)/100) 
		return -1;
	else 
		return 0;
}


void checkLocalisation (const geometry_msgs::PoseArray& data)
{
	cout <<  "__________________________________"<< std::endl;
	cout <<  "new position set"<< std::endl;
	 
	/*
	ros::Publisher pub = n.advertise<geometry_msgs::PoseArray>("biggestCluster", 1000);
	ros::Publisher pubCentroid = n.advertise<geometry_msgs::PoseStamped>("centroid", 1000);
	ros::Publisher pubRemove = n.advertise<geometry_msgs::PoseStamped>("remove", 1000);
	*/

	Cluster dataSet= Cluster(data.poses);
	int numberData = data.poses.size();
	int testVal = 0;
	int seq=0;
	int maxDist;
	int maxID ;
	int i=0;
	float dist;
	dataSet.calculateCentroid();
	dataSet.calculateMaxDistPoint();

	while( (testVal=test(&dataSet,numberData,seq)) == 0 and dataSet.getSize()>1 && ros::ok() )
	{
			/* DISPLAY centroid cluster
		
			std_msgs::Header header = std_msgs::Header();
			header.seq = seq;
			header.stamp = ros::Time::now();
			header.frame_id = "/map";
			geometry_msgs::PoseStamped pose_msg;
			pose_msg.header = header;
			pose_msg.pose = dataSet.getCentroid();
	 		pubCentroid.publish(pose_msg);
			*/

	 		/* DISPLAY removed point
			std_msgs::Header header2 = std_msgs::Header();
			header2.seq = seq;
			header2.stamp = ros::Time::now();
			header2.frame_id = "/map";
			geometry_msgs::PoseStamped remove_pose_msg;
			remove_pose_msg.header = header2;
			remove_pose_msg.pose = dataSet.getPoses()[dataSet.getMaxDistPoint().first];
	 		pubRemove.publish(remove_pose_msg);

			*/
			dataSet.remove(dataSet.getMaxDistPoint().first);
			dataSet.calculateCentroid();
			dataSet.calculateMaxDistPoint();
		seq++;
	}
	if(testVal == 1)
	{

		cout << "ok"<<endl;
 		std_msgs::String msg;
 		
	    std::stringstream ss;

	    ss << "position found" ;
	    
	    cout << ss.str() <<endl;

	    msg.data = ss.str();
	   

		pub.publish(msg);


	}
	else
		cout << "nok"<<endl;
		
		
}	
	


int main(int argc, char **argv)
{

  ros::init(argc, argv, "isLocalised");

  ros::NodeHandle n;
  	pub = n.advertise<std_msgs::String>("position_found", 1000);

  ros::Subscriber sub = n.subscribe("particlecloud", 1000, checkLocalisation);

  ros::spin();

  return 0;
}
