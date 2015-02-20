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