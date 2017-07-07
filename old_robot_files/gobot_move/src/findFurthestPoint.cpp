#include "std_msgs/String.h"
#include "findFurthestPoint.hpp"
#include "gobot_move/FindNextPoint.h"

std::vector<int8_t> my_map;
Metadata metadata;
RobotPos robotOrigin;

ros::Publisher target_pub;
ros::Publisher fail_pub;

void getMap(const nav_msgs::OccupancyGrid::ConstPtr& msg){
	my_map.clear();
	for(size_t i = 0; i < msg->info.width * msg->info.height; i++)
		my_map.push_back(msg->data[i]);
}

void getMetaData(const nav_msgs::MapMetaData::ConstPtr& msg){
	metadata.resolution = msg->resolution;
	metadata.width = msg->width;
	metadata.height = msg->height;
	metadata.x = msg->origin.position.x;
	metadata.y = msg->origin.position.y;
}

void getRobotPos(const geometry_msgs::PoseStamped::ConstPtr& msg){
	robotOrigin.x = msg->pose.position.x;
	robotOrigin.y = msg->pose.position.y;
}

ReducedMap reduceMap(const std::vector<int8_t>& _map, const uint32_t width, const uint32_t height){

	ReducedMap reducedMap;

	for(size_t i = 0; i < height; i++){
		std::vector<Sequence> row;
		Sequence sequence = (_map[i*width] < 30 && _map[i*width] != -1) ? Sequence ({ Color::White, 0, 0 }) : Sequence ({ Color::Black, 0, 0 }); 
		for(size_t j = 1; j < width; j++){
			// this is a white point
			if(_map[i*width + j] < 30 && _map[i*width +j] != -1){
				// this is a sequence of white points so we update the end of the sequence
				if(sequence.color == Color::White)
					sequence.end++;
				// this is a sequence of unknown points so we push the current sequence and start a new sequence of white points
				else {
					row.push_back(sequence);
					sequence.color = Color::White;
					sequence.start = j;
					sequence.end = j;
				}
				
			} else {
				// this is a sequence of white points so we push the current sequence and start a new sequence of black points
				if(sequence.color == Color::White){
					row.push_back(sequence);
					sequence.color = Color::Black;
					sequence.start = j;
					sequence.end = j;
				} 
				// this is a sequence of black points so we update the end of the sequence
				else 
					sequence.end++;
			}
		}
		row.push_back(sequence);
		reducedMap.push_back(row);
	}

	return reducedMap;
}

void addEdgeToReducedGraph(const ReducedVertex head, const ReducedVertex tail, reducedGraph_t & graph){
	graph[head].insert(tail);
	graph[tail].insert(head); 
}

void graphFromReducedMap(const ReducedMap& _map, reducedGraph_t & graph, const int minSpaceRequired){
	// there is no need to look for possible edges at the last row since there is no potential "neighbors" below it, hence the "-1" in _map.size()-1
	for(int i = 0; i < _map.size()-1; i++){
		for(size_t j = 0; j < _map[i].size(); j++){
			Sequence currSequence = _map[i][j];
			// if the current is large enough ( for the robot to move there )
			if(currSequence.color == Color::White && (currSequence.end - currSequence.start + 1) >= minSpaceRequired){
				// we check if there is another sequence that is large enough below us, because the graph is undirected it is not necessary
				// to check whether or not there is such a sequence above ( basically we are constructing the graph from top to bottom )
				for(size_t k = 0; k < _map[i+1].size(); k++){
					Sequence currSeqBelow = _map[i+1][k];
					// if the sequence is a sequence of white points this is a potential "neighbor"
					if(currSeqBelow.color == Color::White){
						if(currSeqBelow.start <= currSequence.end && currSeqBelow.end >= currSequence.start // this is checking if the sequences are somewhat aligned 
						&& std::min(currSeqBelow.end, currSequence.end) - std::max(currSeqBelow.start, currSequence.start) + 1 >= minSpaceRequired){ // this is checking if there is enough space for the robot to pass 
							addEdgeToReducedGraph(ReducedVertex({currSequence.start, currSequence.end, i}), ReducedVertex({currSeqBelow.start, currSeqBelow.end, i+1}), graph);
						}
					}
				}
			}
		}
	}
}

/* this function assumes that the position given for the robot allows it to move at all */
std::pair<ReducedVertex, int> findFurthestPointInReducedGraph(const RobotPos& robotOrigin, const Metadata& _map, 
	const reducedGraph_t& graph, const ReducedMap& _reducedMap){
	coordinates originInPixels = std::make_pair(
						(-_map.x + robotOrigin.x) / _map.resolution + ROBOT_WIDTH,
						_map.height-(-_map.y + robotOrigin.y) / _map.resolution - ROBOT_WIDTH/2
	);

	ReducedVertex origin = reducedVertexFromPixels(originInPixels, _reducedMap);	

	// if the robot cannot move from where it is
	if(graph.find(origin) == graph.end()){
		std::cout << "The origin of the robot does not exist in my map : " << "his position is " << originInPixels.first << " " << originInPixels.second << std::endl;
		std::cout << "Its origin in the internal map is " << origin.row <<  " " << origin.start << std::endl;
	//	for(int i = 0; i < _reducedMap.size(); i++){
	//		for(int j = 0; j < _reducedMap[i].size(); j++){
//std::cout << _reducedMap[i][j].color << " " << _reducedMap[i][j].start << " " << _reducedMap[i][j].end << "|";
//}
//std::cout << std::endl;

//		}			
		return std::make_pair(ReducedVertex({-1, -1, -1}), -1);
	}

	std::pair<ReducedVertex, int> furthestPoint({-1, -1, -1}, 0);
	std::queue<ReducedVertex> _queue;
	std::map<ReducedVertex, int, compReducedVertices> distances;
	for(auto it = graph.cbegin(); it != graph.cend(); ++it)
		distances[(*it).first] = std::numeric_limits<int>::max();

	distances[origin] = 0;
	_queue.push(origin);

	while(!_queue.empty()){
		ReducedVertex current = _queue.front();
		_queue.pop();
		for(auto it = graph.at(current).cbegin(); it != graph.at(current).cend(); ++it){
			const ReducedVertex next = *it;
			
			if(distances[current] + abs(next.end-current.start) + 1 < distances[next]){
				distances[next] = distances[current] + abs(next.end-current.start) + 1;
				_queue.push(next); 
			}
			
		}
	}
	for(auto it = distances.cbegin(); it != distances.cend(); ++it){
		// if this point is further than the furthest point we have found so far we update the furthest point 
		if((*it).second > furthestPoint.second && (*it).second != std::numeric_limits<int>::max()){
			furthestPoint.second = (*it).second;
			furthestPoint.first.row = (*it).first.row;
			furthestPoint.first.start = (*it).first.start;
			furthestPoint.first.end = (*it).first.end;
		}
	}
	return furthestPoint;
}

ReducedVertex reducedVertexFromPixels(const coordinates pixelsCoordinates, const ReducedMap& _map){
	for(size_t i = 0; i < _map[pixelsCoordinates.second].size(); i++){
		if(_map[pixelsCoordinates.second][i].start <= pixelsCoordinates.first && _map[pixelsCoordinates.second][i].end >= pixelsCoordinates.first)
			return ReducedVertex({_map[pixelsCoordinates.second][i].start, _map[pixelsCoordinates.second][i].end, pixelsCoordinates.second});
	}
}

bool findNextPoint(gobot_move::FindNextPoint::Request  &req, gobot_move::FindNextPoint::Response &res){

	std::cout << metadata.height << " " << metadata.width << " " << my_map.size() << robotOrigin.x << "" << robotOrigin.y << std::endl;
	if(metadata.height != 0 && metadata.width != 0 && my_map.size() == metadata.width * metadata.height){
		ReducedMap reducedMap = reduceMap(my_map, metadata.width, metadata.height);
		
		
		reducedGraph_t graph;

		graphFromReducedMap(reducedMap, graph, 8);

		std::pair<ReducedVertex, int> furthestPoint = findFurthestPointInReducedGraph(robotOrigin, metadata, graph, reducedMap);
		std::cout << furthestPoint.first.row << std::endl;
		if(furthestPoint.first.row != -1){
		
			geometry_msgs::PoseStamped msg;

			msg.header.seq = 0;
			msg.header.stamp = ros::Time::now();
			msg.header.frame_id = "/map1";

			msg.pose.position.x = (furthestPoint.first.end) * metadata.resolution + metadata.x;
			msg.pose.position.y = (furthestPoint.first.row) * metadata.resolution + metadata.y;
			msg.pose.orientation.w = 1.0;
			target_pub.publish(msg);
		}else{
			std_msgs::String msg;
                        msg.data = "fail";
                        fail_pub.publish(msg);
		}
	}

	return true;
}

// don't forget to run hector_mapping in order to get /slam_out_pose

int main(int argc, char* argv[]){

	ros::init(argc, argv, "furthest_point");

	ros::NodeHandle n; 

	// to receive the metadata of the map : width, height, resolution and origin
	ros::Subscriber sub_metadata = n.subscribe("/map1_metadata", 1000, getMetaData);

	// to receive the robot's position and orientation
	ros::Subscriber sub_robot = n.subscribe("/slam_out_pose", 1000, getRobotPos);

	// to receive the current map
	ros::Subscriber sub_map = n.subscribe("/map1", 1000, getMap);

	target_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1000);
	fail_pub = n.advertise<std_msgs::String>("/restartSmallMap", 1000);
	
	ros::ServiceServer service = n.advertiseService("Find_next_point", findNextPoint);

	ros::spin();
	
	return 0;
}
