/**
Created on February 19th 2015
Author : Anirudh Vemula
*/

#include "ControlUINode.h"
#include "ros/ros.h"
#include "tum_ardrone/keypoint_coord.h"
#include "ransacPlaneFit.h"
#include "ImageView.h"

#include <string>
#include <fstream>
#include <stdlib.h>
#include <sstream>

using namespace std;

pthread_mutex_t ControlUINode::keyPoint_CS = PTHREAD_MUTEX_INITIALIZER;


ControlUINode::ControlUINode() {
	command_channel = nh_.resolveName("ardrone/com");
	keypoint_channel = nh_.resolveName("/keypoint_coord");

	keypoint_coord_sub = nh_.subscribe(keypoint_channel, 10, &ControlUINode::keyPointDataCb, this);
	tum_ardrone_pub = nh_.advertise<std_msgs::String>(command_channel, 50);
	tum_ardrone_sub = nh_.subscribe(command_channel, 50, &ControlUINode::comCb, this);

	image_gui = new ImageView(this);

	ransacVerbose = false;
}

ControlUINode::~ControlUINode() {

}

void ControlUINode::keyPointDataCb (const tum_ardrone::keypoint_coordConstPtr coordPtr) {
	//ROS_INFO("Received keypoint data");
	pthread_mutex_lock(&keyPoint_CS);
	numPoints = coordPtr->num;
	load2dPoints(coordPtr->x_img, coordPtr->y_img);
	load3dPoints(coordPtr->x_w, coordPtr->y_w, coordPtr->z_w);
	loadLevels(coordPtr->levels);

	assert(_2d_points.size()==_3d_points.size() && _3d_points.size()==_levels.size());
	pthread_mutex_unlock(&keyPoint_CS);
	//fitPlane3d();
}

void ControlUINode::load2dPoints (std::vector<float> x_img, std::vector<float> y_img) {
	_2d_points.clear();
	for (int i = 0; i < numPoints; ++i)
	{
		std::vector<float> p;
		p.push_back(x_img[i]);
		p.push_back(y_img[i]);
		_2d_points.push_back(p);
	}
}

void ControlUINode::load3dPoints (std::vector<float> x_w, std::vector<float> y_w, std::vector<float> z_w) {
	_3d_points.clear();
	for (int i = 0; i < numPoints; ++i)
	{
		std::vector<float> p;
		p.push_back(x_w[i]);
		p.push_back(y_w[i]);
		p.push_back(z_w[i]);
		_3d_points.push_back(p);
	}
	//printf("Size of 3d points : %d\n", _3d_points.size());
}

void ControlUINode::loadLevels (std::vector<int> levels) {
	_levels.clear();
	for (int i = 0; i < numPoints; ++i)
	{
		_levels.push_back(levels[i]);
	}
}

void ControlUINode::fitPlane3d (std::vector<std::vector<int> > ccPoints) {

	std::vector<std::vector<float> > _in_points;

	pthread_mutex_lock(&keyPoint_CS);

	for(int i=0; i<_2d_points.size(); i++) {
		if(liesInside(ccPoints, _2d_points[i])) {
			//printf("%f, %f, %f\n", _3d_points[i][0], _3d_points[i][1], _3d_points[i][2]);
			_in_points.push_back(_3d_points[i]);
		}
	}

	pthread_mutex_unlock(&keyPoint_CS);
	//ROS_INFO("Number of keypoints inside %d", _in_points.size());
	//ROS_INFO("Total number of keypoints %d", _3d_points.size());
	_3d_plane = ransacPlaneFit(_in_points, ransacVerbose);
}

void ControlUINode::Loop () {
	while(nh_.ok()) {
		ros::spinOnce();
	}
}

void ControlUINode::comCb (const std_msgs::StringConstPtr str) {

}

float ControlUINode::distance (std::vector<int> p1, std::vector<float> p2) {
	return sqrt((p2[0]-p1[0])*(p2[0]-p1[0]) + (p2[1]-p1[1])*(p2[1]-p1[1]));
}

float ControlUINode::distance3D (std::vector<float> p1, std::vector<float> p2) {
	return sqrt((p2[0]-p1[0])*(p2[0]-p1[0]) + (p2[1]-p1[1])*(p2[1]-p1[1]) + (p2[2]-p1[2])*(p2[2]-p1[2]));
}

std::vector<float> ControlUINode::searchNearest (std::vector<int> pt, bool considerAllLevels) {

	pthread_mutex_lock(&keyPoint_CS);

	float min = -1;
	std::vector<float> minPt;

	if(!considerAllLevels) {
		for (int i=0; i<_2d_points.size(); i++)
		{
			if(_levels[i]==0) {
				if(min==-1) {
					min = distance(pt, _2d_points[i]);
					minPt = _3d_points[i];
				}
				else {
					float s = distance(pt, _2d_points[i]);
					if(s<min) {
						min = s;
						minPt = _3d_points[i];
					}
				}
			}
		}
	}
	else {
		for (int i=0; i<_2d_points.size(); i++)
		{
			if(min==-1) {
				min = distance(pt, _2d_points[i]);
				minPt = _3d_points[i];
			}
			else {
				float s = distance(pt, _2d_points[i]);
				if(s<min) {
					min = s;
					minPt = _3d_points[i];
				}
			}
		}
	}
	
	pthread_mutex_unlock(&keyPoint_CS);


	return minPt;
}

bool ControlUINode::get2DPoint (std::vector<float> pt, std::vector<int> &p, bool considerAllLevels) {

	pthread_mutex_lock(&keyPoint_CS);

	// ROS_INFO("Total num %d\n", numPoints);

	bool found = false;

	float minDist = 10000000.0;
	int min = -1;

	if(!considerAllLevels) {
		for (int i = 0; i < _3d_points.size(); ++i)
		{
			if(_levels[i]==0 && distance3D(pt, _3d_points[i]) < 0.05) {
				float s = distance3D(pt, _3d_points[i]);
				if(s<minDist) {
					minDist = s;
					min = i;
				}
			}
		}
	}
	else {
		for (int i = 0; i < _3d_points.size(); ++i)
		{
			if(distance3D(pt, _3d_points[i]) < 0.05) {
				float s = distance3D(pt, _3d_points[i]);
				if(s<minDist) {
					minDist = s;
					min = i;
				}
			}
		}
	}

	if(min!=-1) {
		found = true;
		p.push_back((int)_2d_points[min][0]);
		p.push_back((int)_2d_points[min][1]);
		//ROS_INFO("The minimum distance is %f", minDist);
	}

	pthread_mutex_unlock(&keyPoint_CS);

	return found;
}

bool ControlUINode::equal(std::vector<float> p1, std::vector<float> p2) {
	if(distance3D(p1, p2) < 0.001) {
		return true;
	}
	else {
		return false;
	}
}

int ControlUINode::getNumKP(bool considerAllLevels) {
	int c = 0;
	for (int i = 0; i < numPoints; ++i)
	{
		if(_levels[i]==0 && !considerAllLevels)
			c++;
		else if(considerAllLevels)
			c++;
	}
	return c;
}

void ControlUINode::saveKeyPointInformation (int numFile) {
	pthread_mutex_lock(&keyPoint_CS);

	//char * name = itoa(numFile);
	stringstream ss;
	ss << numFile;
	string s = ss.str();

	ofstream fp(s.c_str());

	fp<<numPoints<<std::endl;
	fp<<endl;
	for(int i=0; i<_3d_points.size(); i++) {
		fp<<_3d_points[i][0]<<","<<_3d_points[i][1]<<","<<_3d_points[i][2]<<","<<_levels[i]<<endl;
	}
	fp.close();

	pthread_mutex_unlock(&keyPoint_CS);
}