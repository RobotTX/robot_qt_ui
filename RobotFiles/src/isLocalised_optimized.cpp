#include "cluster_optimized.h"
#include "ros/ros.h"
#include <geometry_msgs/Pose.h> 
#include <geometry_msgs/PoseArray.h> 
#include <geometry_msgs/PoseStamped.h> 
#include <std_msgs/String.h> 
#include <std_msgs/Header.h> 
#include <typeinfo>
#include <sstream>

ros::Publisher pub ; 

int test (Cluster& dataSet, int numberData, int seq)
{

	std::cout << dataSet.getMaxDistPoint().second << " ";
	// if the diameter of the cluster is < 2 then all the cloud is small enough and the position is valid.
	if(dataSet.getMaxDistPoint().second <= 2)
		return 1;

	// if the size of the cluster becomes less than 90 % of all points but the diameter is still to big then the we didn't find the robot position
	else if(dataSet.getSize() < (90*numberData) / 100) 
		return -1;
	else 
		return 0;
}


void checkLocalisation (const geometry_msgs::PoseArray& data)
{
	std::cout << "__________________________________" << std::endl;
	std::cout << "new position set" << std::endl;
	
	Cluster dataSet = Cluster(data.poses);
	int numberData = data.poses.size();
	int testVal(0);
	int seq(0);
	int i(0);
	dataSet.calculateCentroid();
	dataSet.calculateMaxDistPoint();

	while((testVal = test(dataSet, numberData, seq)) == 0 and dataSet.getSize() > 1 && ros::ok()){
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
	if(testVal == 1){

 		std_msgs::String msg;
 		
	    std::stringstream ss;

	    ss << "position found" ;
	    
	    std::cout << ss.str() << std::endl;

	    msg.data = ss.str();

		pub.publish(msg);

	} else
		std::cout << "nok" << std::endl;
}	
	
int main(int argc, char **argv){

  ros::init(argc, argv, "isLocalised");

  ros::NodeHandle n;
  	
  pub = n.advertise<std_msgs::String>("position_found", 1000);

  ros::Subscriber sub = n.subscribe("particlecloud", 1000, checkLocalisation);

  ros::spin();

  return 0;
}
