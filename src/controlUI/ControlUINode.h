#pragma once
/**
Created on February 19th 2015
Author : Anirudh Vemula
*/

#ifndef _CONTROLUINODE_H
#define _CONTROLUINODE_H

#include "ros/ros.h"
#include "tum_ardrone/keypoint_coord.h"
#include "std_msgs/String.h"

#include <vector>
#include <string>

class ImageView;

class ControlUINode
{
private:

	ros::Subscriber keypoint_coord_sub;
	ros::Time lastKeyStamp;
	static pthread_mutex_t tum_ardrone_CS;
	ros::Subscriber tum_ardrone_sub;
	ros::Publisher tum_ardrone_pub;

	ros::NodeHandle nh_;

	std::vector<std::vector<float> > _3d_points;
	std::vector<std::vector<float> > _2d_points;
	int numPoints;

	std::vector<float> _3d_plane; // stored as a 4 length vector with constants a,b,c,d
								 // corresponding to ax+by+cz+d = 0

	std::string keypoint_channel;
	std::string command_channel;


public:

	ImageView *image_gui;

	ControlUINode ();
	~ControlUINode ();

	// ROS message callbacks
	void keyPointDataCb (const tum_ardrone::keypoint_coordConstPtr coordPtr);
	void comCb (const std_msgs::StringConstPtr str);

	// main loop
	void Loop ();

	//writes a string message to "/tum_ardrone/com"
	// is thread safe
	void publishCommand (std::string c);

	// Helper functions
	void load2dPoints (std::vector<float> t_2dPoints_x, std::vector<float> t_2dPoints_y);
	void load3dPoints (std::vector<float> t_3dPoints_x,
					  std::vector<float> t_3dPoints_y,
					  std::vector<float> t_3dPoints_z);



	// Algorithmic functions
	void fitPlane3d ();
};



#endif