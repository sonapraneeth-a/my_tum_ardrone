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

using namespace std;

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
	numPoints = coordPtr->num;
	load2dPoints(coordPtr->x_img, coordPtr->y_img);
	load3dPoints(coordPtr->x_w, coordPtr->y_w, coordPtr->z_w);
	loadLevels(coordPtr->levels);
	fitPlane3d();
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

void ControlUINode::fitPlane3d () {
	_3d_plane = ransacPlaneFit(_3d_points, ransacVerbose);
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

std::vector<float> ControlUINode::searchNearest (std::vector<int> pt) {
	float min = -1;
	std::vector<float> minPt;
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

	return minPt;
}

/*bool ControlUINode::get2DPoint (std::vector<float> pt, std::vector<int> &p) {
	//std::vector<int> ret;
	bool found = false;
	float minDist = 100000;
	int min = -1;
	for(int i=0; i<_3d_points.size(); i++) {
		if(equal(pt, _3d_points[i])) {
			float s = distance3D(pt, _3d_points[i]);
			if(s < minDist) {
				minDist = s;
				min = i;
			}
		}
	}

	if(min!=-1) {
		found = true;
		p.push_back((int)_2d_points[min][0]);
		p.push_back((int)_2d_points[min][1]);
		//printf("Found")
	}
	return found;
}*/

bool ControlUINode::get2DPoint (std::vector<float> pt, std::vector<int> &p) {
	bool found = false;

	float minDist = 10000000.0;
	int min = -1;

	for (int i = 0; i < _3d_points.size(); ++i)
	{
		float s = distance3D(pt, _3d_points[i]);
		if(s<minDist) {
			minDist = s;
			min = i;
		}
	}

	if(distance3D(pt, _3d_points[min]) < 0.001) {
		found = true;
		p.push_back((int)_2d_points[min][0]);
		p.push_back((int)_2d_points[min][1]);
		ROS_INFO("The minimum distance is %f", minDist);
	}

	return found;
}

bool ControlUINode::equal(std::vector<float> p1, std::vector<float> p2) {
	/*for(int i=0; i<p1.size(); i++) {
		if(p1[i]==p2[i])
			continue;
		else
			return false;
	}
	return true;*/
	if(distance3D(p1, p2) < 0.001) {
		return true;
	}
	else {
		return false;
	}
}