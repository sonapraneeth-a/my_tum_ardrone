#pragma once
/**
Created on February 19th 2015
Author : Anirudh Vemula
*/

#ifndef _CONTROLUINODE_H
#define _CONTROLUINODE_H

#include "ros/ros.h"
#include "tum_ardrone/keypoint_coord.h"
#include "tum_ardrone/filter_state.h"
#include "std_msgs/String.h"
#include "helperFunctions.h"


#include <vector>
#include <string>

class ImageView;


struct gridSquare {
public:
	float u, v; // left upper point
	float width, height; // width and height of the gridSquare
	gridSquare(std::vector<float> lu, float width, float height) {
		this->width = width;
		this->height = height;
		u = lu[0];
		v = lu[1];
	}
	gridSquare(float u, float v, float width, float height) {
		this->width = width;
		this->height = height;
		this->u = u;
		this->v = v;
	}
	void debugPrint() {
		ROS_INFO("Square created at (%f, %f) with width %f and height %f", u, v, width, height);
	}
	void printCoord() {
		std::cout<<std::endl;
		std::cout<<u<<","<<v<<std::endl;
		std::cout<<u+width<<","<<v<<std::endl;
		std::cout<<u+width<<","<<v-height<<std::endl;
		std::cout<<u<<","<<v-height<<std::endl;
		std::cout<<std::endl;
	}
};

struct grid {
public:
	// bounds
	float minU, minV, maxU, maxV, width, height, overlap;
	int row;
	std::vector<std::vector<gridSquare> > rowSquares;
	grid(float mU, float mV, float maU, float maV, float width, float height, float overlap) {
		minU = mU;
		minV = mV;
		maxU = maU;
		maxV = maV;
		this->width = width;
		this->height = height;
		this->overlap = overlap;
		std::vector<gridSquare> v;
		rowSquares.push_back(v);
		row = 0;
	}
	void add(gridSquare square) {
		rowSquares[row].push_back(square);
	}
	bool translate(gridSquare g) {

		// Whenever row empty. Check v bounds too.
		if(rowSquares[row].empty()) {
			float unew = minU;
			float vnew = g.v - (1-overlap)*height;
			if(vnew-height >= minV) {
				// Clean down shift
				gridSquare gnew(unew, vnew, width, height);
				add(gnew);
			}
			else if(vnew - height < minV) {
				// grid with less height - ? No. grid square must always have a fixed height
				gridSquare gnew(unew, vnew, width, height);
				if(vnew - minV > 0)
					add(gnew);
				else
					return false;

			}
		}
		else {
			float unew = g.u + (1-overlap)*width;
			// Only right translation
			if(unew+width <= maxU) {
				// Clean right shift
				gridSquare gnew(unew, g.v, width, g.height);
				add(gnew);
			}
			else if(unew+width > maxU) {
				// grid with less width - ? No. grid square must always have a fixed width
				gridSquare gnew(unew, g.v, width, g.height);
				if(maxU - unew > 0) // lower bound on the width of the square - ?
					add(gnew);
				// Row completed
				row++;
				std::vector<gridSquare> v;
				rowSquares.push_back(v);
			}

		}
		return true;
	}
	gridSquare getLatest() {
		if(rowSquares[row].empty())
			return rowSquares[row-1].back();
		else
			return rowSquares[row].back();
	}
};

struct pGrid {
public:
	float au, av; //left upper point
	std::vector<float> rd; // right direction vector
	std::vector<float> dd; // down direction vector
	float width; // distance along rd
	float height; // height along dd
	float overlap; // overlap across grid squares
	int row;
	std::vector<std::vector<pGridSquare> > rowSquares;

	pGrid(float au, float av, std::vector<float> rd, std::vector<float> dd, float width, float height, float overlap) {
		this->au = au;
		this->av = av;
		this->rd = rd;
		this->dd = dd;
		this->width = width; this->height = height; this->overlap = overlap;
		std::vector<pGridSquare> v;
		rowSquares.push_back(v);
		row = 0;
	}
	void add(pGridSquare gs) {
		rowSquares[row].push_back(gs);
	}
	bool translate(pGridSquare gs) {

	}
	pGridSquare getLatest(){
		if(rowSquares[row].empty())
			return rowSquares[row-1].back();
		else
			return rowSquares[row].back();
	}
};

class ControlUINode
{
private:

	ros::Subscriber keypoint_coord_sub;
	ros::Subscriber pose_sub;
	ros::Time lastKeyStamp;
	static pthread_mutex_t tum_ardrone_CS;
	ros::Subscriber tum_ardrone_sub;
	ros::Publisher tum_ardrone_pub;

	ros::NodeHandle nh_;

	std::vector<std::vector<float> > _3d_points;
	std::vector<std::vector<float> > _2d_points;
	std::vector<int> _levels;
	int numPoints;
	float scale;
	float scale_z;
	float x_offset, y_offset, z_offset;

	std::vector<float> _3d_plane; // stored as a 4 length vector with constants a,b,c,d
								 // corresponding to ax+by+cz+d = 0. Enforcing the constraint that d = 1 for uniformity (except when d = 0)

	std::string keypoint_channel;
	std::string command_channel;
	std::string pose_channel;

	bool ransacVerbose;
	bool useScaleFactor;
	float threshold;

	// distance between two 2d points
	float distance(std::vector<int> pt_int, std::vector<float> pt_float);

	// distance between two 3d points
	float distance3D(std::vector<float> p1, std::vector<float> p2);

	static pthread_mutex_t keyPoint_CS;
	static pthread_mutex_t pose_CS;


public:

	ImageView *image_gui;

	ControlUINode ();
	~ControlUINode ();

	// ROS message callbacks
	void keyPointDataCb (const tum_ardrone::keypoint_coordConstPtr coordPtr);
	void poseCb (const tum_ardrone::filter_stateConstPtr statePtr);
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
	std::vector<float> fitPlane3d (std::vector<int> ccPoints, std::vector<std::vector<int> > pointsClicked);

	// Search function : Given a 2d point, find the nearest 2d keypoint and return its 3d position
	std::vector<float> searchNearest(std::vector<int> pt, bool considerAllLevels);

	// Get 2d position (with a threshold) of a key point given its 3d position. Return empty vector if keypoint not found in the current frame (within the threshold)
	bool get2DPoint(std::vector<float> pt, std::vector<int> &p, bool considerAllLevels);

	// Get 2d position of the nearest key point given a 3d position
	bool get2DPointNearest(std::vector<float> pt, std::vector<int> &p, bool considerAllLevels);

	// Equality function for 3d keypoints. Does it need to be exact equality? Or some heuristic based distance threshold
	bool equal(std::vector<float> p1, std::vector<float> p2);

	// A helper function to get the number of key points in the current frame
	int getNumKP(bool considerAllLevels);

	// Saves the 3d coordinates of the keypoints as a CSV file for external processing
	void saveKeyPointInformation(int numFile);

	// Translates the fitted plane by the given distance along its normal toward origin
	std::vector<float> translatePlane (float translateDistance);

	// Projects the 3d points onto the extracted plane
	std::vector<std::vector<float> >  projectPoints (std::vector<int> ccPoints, std::vector<std::vector<float> > keyPoints);

	// Builds the grid
	grid buildGrid (std::vector<std::vector<float> > pPoints);

	// Gets the target points given the grid and plane
	std::vector<std::vector<double> > getTargetPoints (grid g, std::vector<float> plane);
};



#endif