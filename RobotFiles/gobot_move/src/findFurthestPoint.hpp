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

#define ROBOT_WIDTH 6

enum Color { White, Black };

/* Provides a struct for the metadata actually needed to compute the furthest point reachable by the robot */
struct Metadata {
	float resolution;
	uint32_t width;
	uint32_t height;
	double x;
	double y;
};

struct RobotPos {
	double x;
	double y;
};

/* provides a struct that will allow us to reduce a map so that a row of 0's or 1's is reduced to one object Row */
struct Sequence {
	Color color;
	int start;
	int end;
};

// a pair of coordinates in the initial map expressed in parameters used by the reducedMap
struct ReducedVertex {
	int start;
	int end;
	int row;
};

/* to store coordinates of the robot in pixels */
typedef std::pair<int, int> coordinates;

struct compReducedVertices {
	bool operator() (const ReducedVertex r, const ReducedVertex o) const {
		return r.row < o.row || (r.row == o.row && r.start < o.start) || (r.row == o.row && r.start == o.start && r.end < o.end);
	}
};

typedef std::vector<std::vector<Sequence> > ReducedMap;

typedef std::map<ReducedVertex, std::set<ReducedVertex, compReducedVertices>, compReducedVertices > reducedGraph_t;

void getMap(const nav_msgs::OccupancyGrid::ConstPtr& msg);

void getMetaData(const nav_msgs::MapMetaData::ConstPtr& msg);

void getRobotPos(const geometry_msgs::PoseStamped::ConstPtr& msg);

void addEdgeToReducedGraph(const ReducedVertex head, const ReducedVertex tail, reducedGraph_t & graph);

void getMapFromFile(const char* filename, int8_t _map[]);

/* to get the reduced map from the data sent through ROS */
ReducedMap reduceMap(const std::vector<int8_t>& _map, const uint32_t width, const uint32_t height);

void graphFromReducedMap(const ReducedMap& _map, reducedGraph_t & graph, 
	const int minSpaceRequired /* the space needed for the robot to move to a point, could be on the map a range of 13 white pixels */);

ReducedVertex reducedVertexFromPixels(const coordinates pixelsCoordinates, const ReducedMap& _map);

std::pair<ReducedVertex, int> findFurthestPointInReducedGraph(const RobotPos& robotOrigin, const Metadata& mapMetadata, const reducedGraph_t& graph, const ReducedMap& _reducedMap);	

#endif
