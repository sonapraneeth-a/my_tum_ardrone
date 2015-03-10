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
	std::vector<int> _levels;
	int numPoints;

	std::vector<float> _3d_plane; // stored as a 4 length vector with constants a,b,c,d
								 // corresponding to ax+by+cz+d = 0

	std::string keypoint_channel;
	std::string command_channel;

	bool ransacVerbose;

	// distance between two 2d points
	float distance(std::vector<int> pt_int, std::vector<float> pt_float);

	// distance between two 3d points
	float distance3D(std::vector<float> p1, std::vector<float> p2);

	static pthread_mutex_t keyPoint_CS;


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

	void loadLevels (std::vector<int> levels);

	// Algorithmic functions
	void fitPlane3d ();

	// Search function : Given a 2d point, find the nearest 2d keypoint and return its 3d position
	std::vector<float> searchNearest(std::vector<int> pt, bool considerAllLevels);

	// Get 2d position of a key point given its 3d position. Return empty vector if keypoint not found in the current frame
	bool get2DPoint(std::vector<float> pt, std::vector<int> &p, bool considerAllLevels);

	// Equality function for 3d keypoints. Does it need to be exact equality? Or some heuristic based distance threshold
	bool equal(std::vector<float> p1, std::vector<float> p2);

	int getNumKP(bool considerAllLevels);

	void saveKeyPointInformation(int numFile);
};



#endif