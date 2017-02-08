#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseArray
import math 
from std_msgs.msg import Header
###
### hierarchical cluster algorithm 
###

seq = 0
def test(dataSet,numberData):
	maxSize = -1
	maxID = -1
	i=0
	for cluster in dataSet :
		if clusterSize(cluster) > maxSize :
			maxSize = clusterSize(cluster)
			maxID = i
		i=i+1
	biggestCluster = dataSet[maxID]

	pub = rospy.Publisher('biggestCluster', PoseArray, queue_size=10)

	global seq
	seq = seq+ 1
	header = Header()
	header.seq = seq
	header.stamp = rospy.Time.now()
	header.frame_id = "/map"
	pub.publish(PoseArray(header,biggestCluster))

	cs = clusterSize(biggestCluster)
	print cs
	if(cs >= (90*numberData)/100) :
		if diadiameterCluster(biggestCluster) <= 4:
			return 1
		else :
			return -1
	else:
		return 0 


def clusterSize(cluster):
	if  isinstance(cluster ,list):
		return len(cluster)
	else:
		return 1

def diameterCluster (cluster):
	maxDist = -1000000
	i=1
	while i< len(dataSet):
		dist = distancePoint(dataSet[i],dataSet[i-1])
		if dist > maxDist  :
				maxDist = dist
		i=i+1
	return maxDist

def distanceCluster (cluster1,cluster2):
	maxDist= -10000000
	if not isinstance(cluster1 ,list):
		cluster1=[cluster1]

	if not isinstance(cluster2 ,list):
		cluster2=[cluster2]

	for point1 in cluster1 :
		for point2 in cluster2 :
			dist = distancePoint(point1,point2)
			if dist > maxDist:
				maxDist = dist
	return maxDist



def distancePoint (A,B):
	xdist = pow((B.position.x - A.position.x),2)
	ydist = pow((B.position.y - A.position.y),2)
	zdist = pow((B.position.z - A.position.z),2)
	return math.sqrt(xdist+ydist+zdist)

def merge(cluster1, cluster2):
	if not isinstance(cluster1 ,list):
		cluster1= [cluster1]
	if not isinstance(cluster2 ,list):
		cluster2= [cluster2]
	return cluster1 + cluster2

def checkLocalisation(data):
	print "__________________________________"
	print "new position set"
	"""
	dataSet = []
	print len(data.poses)
	for pose in data.poses :
		tmp = [[pose.position.x,pose.position.y,pose.position.z ] , [pose.orientation.x,pose.orientation.y,pose.orientation.z ]] 
		dataSet.append(tmp)

	print dataSet
	"""	
	

	dataSet=data.poses
	numberData = len(data.poses)
	testVal = 0
	while( testVal == 0 and len(dataSet)>1 ):
		minDist = 100000000
		minID = -1
		i=0
		while i< len(dataSet):
	 		dist = distanceCluster(dataSet[i],dataSet[i-1])
	  		if dist < minDist :
	  			minDist = dist
	  			minID = i
	  		i=i+1
	  	dataSet[minID-1] = merge(dataSet[minID-1] , dataSet[minID])
		dataSet.pop(minID)
		testVal = test(dataSet,numberData)
		print len(dataSet)
	if(testVal == 1):
		print True
	else:
		print False
	


if __name__ == '__main__':
	rospy.init_node('isLocalised', anonymous=True)
	rospy.Subscriber("testSet", PoseArray, checkLocalisation)
    	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()



