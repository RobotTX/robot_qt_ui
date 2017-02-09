#include "cluster_optimized.h"
#include "ros/ros.h"
#include <geometry_msgs/Pose.h> 
#include <geometry_msgs/PoseArray.h> 
#include <geometry_msgs/PoseStamped.h> 
#include <std_msgs/String.h> 
#include <std_msgs/Header.h> 
#include <typeinfo>
#include <sstream>

ros::Publisher pub; 
ros::Subscriber sub;

int test (const Cluster& dataSet, const int numberData, const int seq){
	std::cout << dataSet.getMaxDistPoint().second << " ";

	// if the diameter of the cluster is < 2 then all the cloud is small enough and the position is valid.
	if(dataSet.getMaxDistPoint().second <= 2)
		return 1;

	// if the size of the cluster becomes less than 90 % of all points but the diameter is still too big then we didn't find the robot position
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

	while((testVal = test(dataSet, numberData, seq)) == 0 && dataSet.getSize() > 1 && ros::ok()){
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

		// if the position has been found there is no need to process the messages anymore
		sub.shutdown();

	} else
		std::cout << "nok" << std::endl;
}	
	
int main(int argc, char **argv){

  ros::init(argc, argv, "isLocalised");

  ros::NodeHandle n;
  	
  pub = n.advertise<std_msgs::String>("position_found", 1000);

  sub = n.subscribe("particlecloud", 1000, checkLocalisation);

  ros::spin();

  return 0;
}
