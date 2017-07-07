#include "gobot_move/FindNextPoint.h"
#include "ros/ros.h"



int main(int argc, char **argv)
{
	  ros::init(argc, argv, "add_two_ints_server");
	  ros::NodeHandle n;

	  ros::ServiceClient client = n.serviceClient<gobot_move::FindNextPoint>("Find_next_point");

	  while(ros::ok()){


			std::cout << "apres" << std::endl;

			gobot_move::FindNextPoint srv;


		  if (client.call(srv))
		  {
		  	 //ROS_INFO("Sum: %ld", srv.request.a);
		  	std::cout << "yo 2" << std::endl;
		  }
		  else
		  {
		    ROS_ERROR("Failed to get pos");
		    //return 1;
		  }
		  std::cout << "after call" << std::endl;

	  	ros::spin();
	}
}


/*
	ros::Rate loop_rate(10);

	while(ros::ok()){

		if(metadata.height != 0 && metadata.width != 0 && my_map.size() == metadata.width * metadata.height){

			ReducedMap reducedMap = reduceMap(my_map, metadata.width, metadata.height);

			reducedGraph_t graph;

			graphFromReducedMap(reducedMap, graph, 10);

			std::pair<ReducedVertex, int> furthestPoint = findFurthestPointInReducedGraph(robotOrigin, metadata, graph, reducedMap);
	
			if(furthestPoint.first.row != -1){

				geometry_msgs::PoseStamped msg;

				msg.header.seq = 0;
				msg.header.stamp = ros::Time::now();
				msg.header.frame_id = "/map";

				msg.pose.position.x = (furthestPoint.first.end) * metadata.resolution + metadata.x;
				msg.pose.position.y = (furthestPoint.first.row) * metadata.resolution + metadata.y;

				target_pub.publish(msg);

     			//ros::spinOnce();
				
						
				std::cout << "go there " << furthestPoint.first.start << " " << furthestPoint.first.end << " " << metadata.height-1-furthestPoint.first.row  << 
				" distance : " << furthestPoint.second << std::endl;
				

				//return 0;
			} else 
				std::cout << "";
		}

		ros::spinOnce();
	}*/