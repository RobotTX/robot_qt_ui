#include "cluster_optimized.h"

Cluster::Cluster(const std::vector<geometry_msgs::Pose>& posesV): poses(posesV)
{}

float Cluster::distancePoints(const geometry_msgs::Pose& A, const geometry_msgs::Pose& B) const {
	float xdist = pow((B.position.x - A.position.x), 2);
	float ydist = pow((B.position.y - A.position.y), 2);
	float zdist = pow((B.position.z - A.position.z), 2);
	return xdist + ydist + zdist;
}

void Cluster::calculateMaxDistPoint(void){
	
	float maxDist(-100000.0);
	float dist(0.0);
	int maxID(-1);

	for (size_t i = 0; i < poses.size(); i++){	
		dist = distancePoints(poses[i], centroid);
		if (dist > maxDist){
			maxDist = dist;
			maxID = i;
		}
	}

	maxPoint.first = maxID;
	maxPoint.second = maxDist;
}	

void Cluster::calculateCentroid(void) {
	
	float sumx(0.0);
	float sumy(0.0);
	float sumz(0.0);

	float maxDist(-1000000.0);
	float dist(0.0);

	for(size_t i = 0; i < poses.size(); i++){

		sumx = sumx + poses.at(i).position.x;
		sumy = sumy + poses.at(i).position.y;
		sumz = sumz + poses.at(i).position.z;
	}

	centroid.position.x = sumx / (poses.size()-1);
	centroid.position.y = sumy / (poses.size()-1);
	centroid.position.z = sumz / (poses.size()-1);
}

void Cluster::remove(const int i){
	if(i < poses.size())
		poses.erase(poses.begin() + i);
}