#ifndef FIND_FURTHEST_POINT
#define FIND_FURTHEST_POINT

#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <set>
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/OccupancyGrid.h"
#include <cstdint>
#include <queue>
#include <limits>
#include <unistd.h>
#include <assert.h>

/* a vertex is a pair of coordinates on the map */
typedef int Vertex;

typedef std::map<Vertex, std::set<Vertex> > graph_t;

void addEdge(const Vertex& head, const Vertex& tail, graph_t & graph);

void graphFromMap(int8_t const _map[], const uint32_t width, const uint32_t height, const graph_t & graph);

coordinates findFurthestPoint(const Vertex origin, const uint32_t width, const uint32_t height, const graph_t & graph);

void addEdge(const Vertex& head, const Vertex& tail, graph_t & graph){
	graph[head].insert(tail);
	graph[tail].insert(head);
}

coordinates findFurthestPoint(const Vertex origin, const uint32_t width, const uint32_t height, const graph_t & graph){
	std::pair<coordinates, int> coords(coordinates(-1, -1), 0);
	std::queue<Vertex> _queue;
	std::map<Vertex, int> distances;
	for(auto it = graph.cbegin(); it != graph.cend(); ++it)
		distances[(*it).first] = width + height; /* the maximum */

	distances[origin] = 0;
	_queue.push(origin);
	while(!_queue.empty()){
		Vertex current = _queue.front();
		_queue.pop();
		for(std::set<Vertex>::const_iterator it = graph.at(current).cbegin(); it != graph.at(current).cend(); ++it){
			const Vertex next = *it;
			if(distances[next] == width + height){
				distances[next] = distances[current] + 1;
				_queue.push(next);
			}
		}
	}
	for(auto it = distances.cbegin(); it != distances.cend(); ++it){
		/* if this point is further than the furthest point we have found so far we update the furthest point */
		if((*it).second > coords.second){
			coords.second = (*it).second;
			coords.first.first = (*it).first / width;
			coords.first.second = (*it).first % width;
		}
	}

	return coords.first;
}

void graphFromMap(int8_t const _map[], const uint32_t width, const uint32_t height, graph_t & graph){
	for(size_t i = 0; i < height; i++){
		for(size_t j = 0; j < width; j++){
			/* this point is not a wall */
			if(_map[i*width + j] < 30 && _map[i*width + j] != -1){
				// we look at the point immediately above this one if such point exists, if it does and it's not an obstacle we add it to our graph
				if(i > 0 && _map[(i-1)*width + j] < 30 && _map[(i-1)*width + j] != -1)
					addEdge(i*width+j, (i-1)*width+j, graph);
				
				// we look at the point immediately to the right of this one if such point exists
				if(j < width-1 && _map[i*width+j+1] < 30 && _map[i*width+j+1] != -1)
					addEdge(i*width+j, i*width+j+1, graph);

				// we look at the point immediately below this one if such point exists
				if(i < height-1 && _map[(i+1)*width+j] < 30 && _map[(i+1)*width+j] != -1)
					addEdge(i*width+j, (i+1)*width+j, graph);

				// we look at the point immediately to the left of this one if such point exists
				if(j > 0 && _map[i*width+j-1] < 30 && _map[i*width+j-1] != -1)
					addEdge(i*width+j, i*width+j-1, graph);
			}
		}
	}
}

#endif

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
		    ROS_ERROR("Failed to call service add_two_ints");
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