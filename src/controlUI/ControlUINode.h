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
#include <list>
#include <opencv2/core/core.hpp>


#include <vector>
#include <string>

using namespace std;
using namespace cv;
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

	void print(std::vector<float> plane) {
		for(unsigned int i=0; i<rowSquares.size(); i++) {
			for(unsigned int j=0; j<rowSquares[i].size(); j++) {
				float x = rowSquares[i][j].u;
				float z = rowSquares[i][j].v;
				float y = getY(x, z, plane);
				printf("%f %f %f\n", x, y, z);
				printf("%f %f %f\n", x + width, getY(x+width, z, plane), z);
				printf("%f %f %f\n", x + width, getY(x+width, z - height, plane), z - height);
				printf("%f %f %f\n", x, getY(x, z - height, plane), z - height);
			}
		}
	}
};

struct pGridSquare {
public:
	float u, v; // left upper point
	float width, height;
	std::vector<float> rd, dd; // right and down direction vector

	pGridSquare(float u, float v, float width, float height, std::vector<float> rd, std::vector<float> dd) {
		this->u = u;
		this->v = v;
		this->width = width;
		this->height = height;
		this->rd = rd;
		this->dd = dd;
	}
	void debugPrint() {
		ROS_INFO("Square created at (%f, %f) with width %f along (%f, %f) and height %f along (%f, %f)",
				u, v, width,
				rd[0], rd[1],height,dd[0],dd[1]);
	}
};

struct pGrid {
public:
	float au, av; //left upper point
	float maxR, maxD; // maximum right and down distances along rd and dd resp.
	std::vector<float> rd; // right direction vector
	std::vector<float> dd; // down direction vector
	float width; // distance along rd
	float height; // height along dd
	float overlap; // overlap across grid squares
	int row;
	std::vector<std::vector<pGridSquare> > rowSquares;

	pGrid(float au, float av, std::vector<float> rd, std::vector<float> dd, float width, float height, float overlap, float maxR, float maxD) {
		this->au = au;
		this->av = av;
		this->rd = rd;
		this->dd = dd;
		this->width = width; this->height = height; this->overlap = overlap;
		this->maxR = maxR; this->maxD = maxD;
		std::vector<pGridSquare> v;
		rowSquares.push_back(v);
		row = 0;
	}
	void add(pGridSquare gs) {
		rowSquares[row].push_back(gs);
	}

	bool translate(pGridSquare g) {
		// Whenever row empty. Check v bounds too.
		if(rowSquares[row].empty()) {
			float unew = au + (1-overlap)*height*dd[0];
			float vnew = g.v - (1-overlap)*height*dd[1];
			if(av - (vnew - height*dd[1]) <= maxD*dd[1]) {
				// Clean down shift
				pGridSquare gnew(unew, vnew, width, height, rd, dd);
				add(gnew);
			}
			else if(av - (vnew - height*dd[1]) > maxD*dd[1]) {
				// grid with less height - ? No. grid square must always have a fixed height
				pGridSquare gnew(unew, vnew, width, height, rd, dd);
				if(vnew > av - maxD*dd[1])
					add(gnew);
				else
					return false;

			}
		}
		else {
			float unew = g.u + (1-overlap)*width*rd[0];
			float vnew = g.v + (1-overlap)*width*rd[1];
			// Only right translation
			if(unew + width*rd[0] - au <= maxR*rd[0]) {
				// Clean right shift
				pGridSquare gnew(unew, vnew, width, height, rd, dd);
				add(gnew);
			}
			else if(unew + width*rd[0] - au > maxR*rd[0]) {
				// grid with less width - ? No. grid square must always have a fixed width
				pGridSquare gnew(unew, vnew, width, height, rd, dd);
				if(unew < au + maxR*rd[0]) // lower bound on the width of the square - ?
					add(gnew);
				// Row completed
				row++;
				std::vector<pGridSquare> v;
				rowSquares.push_back(v);
			}

		}
		return true;
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
	ros::Subscriber tum_ardrone_sub;
	ros::Publisher tum_ardrone_pub;

	ros::NodeHandle nh_;

	ros::ServiceClient video;

	ros::Timer timer_checkPos;
	ros::Timer timer_record;

	ros::Time last;

	// Key point information
	std::vector<std::vector<float> > _3d_points;
	std::vector<std::vector<float> > _2d_points;
	std::vector<int> _levels;
	cv::Mat cameraMatrix;
	cv::Mat distCoeffs;
	std::vector<cv::Mat> rvecs, tvecs;

	int numPoints;

	float scale; // PTAM X-Y scale
	float scale_z; // PTAM Z scale
	float x_offset, y_offset, z_offset; // PTAM offsets

	// Drone state variables
	float x_drone, y_drone, z_drone, roll, pitch, yaw; // drone pose variables

	std::vector<float> _3d_plane; // stored as a 4 length vector with constants a,b,c,d
								 // corresponding to ax+by+cz+d = 0. Enforcing the constraint that d = 1 for uniformity (except when d = 0)

	std::vector<double> targetPoint;
	std::list<std_msgs::String> commands;
	std::list<std::vector<double> > targetPoints;

	std::string keypoint_channel; // channel on which keypoint info is received
	std::string command_channel; // channel on which commands can be posted or received
	std::string pose_channel; // channel on which pose info is received

	bool ransacVerbose; // Whether we need the ransac verbose output or not
	bool useScaleFactor; // Using scale factors. MUST BE SET TO TRUE
	float threshold; // threshold allowed in finding keypoint in the current frame
	double error_threshold; // threshold allowed in the drone position
	double recordTime; // time to record the video
	bool record; // whether to record or not
	float pollingTime; // Interval at which polling is done to check the drone position
	bool targetSet;
	bool currentCommand;
	bool recordNow;
	bool notRecording;

	// distance between two 2d points
	float distance(std::vector<int> pt_int, std::vector<float> pt_float);

	// distance between two 3d points
	float distance3D(std::vector<float> p1, std::vector<float> p2);

	// Thread mutexes
	static pthread_mutex_t tum_ardrone_CS;
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

	// Fit multiple planes in 3D
	void fitMultiplePlanes3d (vector<int> &ccPoints, vector<vector<int> > &pointsClicked, vector<vector<float> >&planeParameters,vector< vector<Point3f> > & continuousBoundingBoxPoints);

	// Move Quadopter to required position
	void moveQuadcopter(
		vector< vector<float> >&planeParameters,
		vector< vector<Point3f> > &continuousBoundingBoxPoints);

	// Search function : Given a 2d point, find the nearest 2d keypoint and return its 3d position
	std::vector<float> searchNearest(std::vector<int> pt, bool considerAllLevels);

	// Get 2d position (with a threshold) of a key point given its 3d position. Return empty vector if keypoint not found in the current frame (within the threshold)
	bool get2DPoint(std::vector<float> pt, std::vector<int> &p, bool considerAllLevels);

	//Project World Pts on Image plane
	void project3DPointsOnImage(const vector<Point3f> &worldPts, vector<Point2f > & imagePts);

	//calibrate camera
	void calibrate();

	// Get 2d position of the nearest key point given a 3d position
	bool get2DPointNearest(std::vector<float> pt, std::vector<int> &p, bool considerAllLevels);

	// Equality function for 3d keypoints. Does it need to be exact equality? Or some heuristic based distance threshold
	bool equal(std::vector<float> p1, std::vector<float> p2);

	// Write 3D points obtained to a CSV file
	void write3DPointsToCSV(std::vector<std::vector<float> > &_3d_points);

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

	// Builds the PGrid
	//pGrid buildPGrid (std::vector<std::vector<float> > pPoints);
	pGrid buildPGrid(
		vector<float> uCoord,
		vector<float> vCoord );

	// Get the target points in XYZ for the grid for quadcopter movement
	void getPTargetPoints(
		const pGrid &grid,
		const vector<float> &planeParameters,
		vector< vector<double> > &targetPoints);

	// Gets the target points given the grid and plane
	std::vector<std::vector<double> > getTargetPoints (grid g, std::vector<float> plane);

	// Generate the appropriate goto commands according to the target points
	void moveDrone (std::vector<std::vector<double> > tPoints);

	// Checks the position of the drone and whether the error is less than a threshold
	void checkPos (const ros::TimerEvent&);

	// Records the video for a fixed amount of time
	// bool recordVideo ();
};



#endif
