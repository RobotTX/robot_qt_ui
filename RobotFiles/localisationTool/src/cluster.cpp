
#include <math.h> 
#include "cluster.h"


Cluster::Cluster(std::vector<geometry_msgs::Pose> posesV): poses(posesV)
{
}


/*
float Cluster::calculateDiameter()
{

	float maxDist = -1000000;
	int i=1;
	float dist;
	while (i< poses.size())
	{
		 dist = distancePoints(poses[i],poses[i-1]);
		if(dist > maxDist ) 
				maxDist = dist;
		i=i+1;
	}
	return maxDist;
}

*/
float Cluster::distancePoints (geometry_msgs::Pose A,geometry_msgs::Pose B)
{
	float xdist = pow((B.position.x - A.position.x),2);
	float ydist = pow((B.position.y - A.position.y),2);
	float zdist = pow((B.position.z - A.position.z),2);
	return sqrt(xdist+ydist+zdist);
}

std::pair<int,float> Cluster::calculateMaxDistPoint ()
{
	
	float maxDist = -10000000;

	float dist;
	int maxID=-1;

	for (int i=0;i< poses.size();i++)
	{	
			dist = distancePoints(poses[i],centroid);
			if (dist > maxDist)
			{
				maxDist = dist;
				maxID= i;
			}
	}
	maxPoint.first = maxID;
	maxPoint.second = maxDist;


	return maxPoint;
}	

geometry_msgs::Pose Cluster::calculateCentroid()
{
	float sumx=0;
	float sumy=0;
	float sumz=0;

	float maxDist = -1000000;
	int i=1;
	float dist;



	for (float i=0;i< poses.size();i++)
	{

		sumx = sumx + poses.at(i).position.x;
		sumy = sumy + poses.at(i).position.y;
		sumz = sumz + poses.at(i).position.z;
		/*
		for (float j=0;j< poses.size();j++)
		{
			if(j!=i)
			{
				dist = distancePoints(poses[i],poses[j]);
				if(dist > maxDist ) 
					maxDist = dist;
			}
		}
		*/
	}
	//diameter = maxDist;
	float avex = sumx/ (poses.size()-1);
	float avey= sumy/(poses.size()-1);
	float avez = sumz/(poses.size()-1);
	
	centroid.position.x=avex;
	centroid.position.y=avey;
	centroid.position.z=avez;

	return centroid ;
}

void Cluster::remove(int i)
{
	poses.erase(poses.begin()+i);
}