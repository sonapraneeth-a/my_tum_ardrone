#pragma once
/*****************************************************************************************
 * ImageView.h
 *
 *       Created on: 21-Feb-2015
 *    Last Modified: 12-Sep-2016
 *  Original Author: Anirudh Vemula
 *   Current Author: Meghshyam Govind Prasad
 *   Current Author: Sona Praneeth Akula
 *          Project: Quadcopter
 *      Description: This file is to create a class which would enable a GL window displaying
 * 					 the camera feed of the drone and recording mouse clicks
 *
 * Date				Author							Modification
 * 12-Sep-2016	Sona Praneeth Akula	Added		Comments to the code
 *****************************************************************************************/

#ifndef _IMAGEVIEW_H
#define _IMAGEVIEW_H

#include "ros/ros.h"
#include "GLWindow2.h"
#include "sensor_msgs/Image.h"
#include "cvd/thread.h"
#include "cvd/image.h"
#include "cvd/byte.h"
#include "cvd/rgb.h"
#include "MouseKeyHandler.h"
#include "boost/thread.hpp"
#include "std_msgs/String.h"
#include <vector>
#include <opencv2/core/core.hpp>
#include "helperFunctions.h"


#include "Line2.hpp"
#include "AllHeaders.hpp"
#include <map>
#include "TopView.hpp"
#include "DebugUtility.hpp"
#include "LogUtility.hpp"
#include "makeBoundingRects.hpp"


// CGAL specific
/*#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/point_generators_3.h>
#include <CGAL/algorithm.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/convex_hull_3.h>*/

class ControlUINode;
struct grid;
struct gridSquare;
class TopView;

/**
 * @brief @todo 
 * @details @todo
 */
class ImageView : private CVD::Thread, private MouseKeyHandler
{

	private:
		// Communication with drone
		ros::Subscriber vid_sub;
		std::string video_channel;

		// Communication with tum_ardrone
		ros::Subscriber tum_ardrone_sub;
		ros::Publisher tum_ardrone_pub;
		std::string command_channel;

		// ROS
		ros::NodeHandle nh_;


		GLWindow2* myGLWindow;
		CVD::ImageRef desiredWindowSize;
		CVD::ImageRef defaultWindowSize;
		bool changeSizeNextRender;

		/**
		 * @brief The associated thread's run function
		 * @details Creates the 'DRONE CAMERA FEED' Window
		 * @param
		 * @return
		 */
		void run();

		CVD::Image<CVD::Rgb<CVD::byte> > mimFrameBW;
		CVD::Image<CVD::Rgb<CVD::byte> > mimFrameBW_workingCopy;
		int mimFrameTime;
		int mimFrameTime_workingCopy;
		unsigned int mimFrameSEQ;
		unsigned int mimFrameSEQ_workingCopy;
		ros::Time mimFrameTimeRos;
		ros::Time mimFrameTimeRos_workingCopy;
		int frameWidth, frameHeight;

		int videoFramePing;

		// keep running
		bool keepRunning;
		bool lockNextFrame;

		boost::condition_variable new_frame_signal;
		boost::mutex new_frame_signal_mutex;

		// resets
		/**
		 * @brief @todo Resets. But what does it reset
		 * @details 
		 * @param
		 * @return
		 */
		void ResetInternal();

		//ControlUINode
		ControlUINode *node;
		//
		int numPointsClicked;
		//
		int numKeyPointsDetected;

		// 2d image points clicked
		std::vector< std::vector<int> > pointsClicked;
		// the 3d keypoints of control node for nearest keypoints
		std::vector< std::vector<float> > keyPointsNearest;
		// corners of the convex hull
		std::vector<int> ccPoints;
		// corners of multiple planes bounding boxes in 3D
		std::vector< std::vector<cv::Point3f> > continuousBoundingBoxPoints;
		// Plane parameters
		std::vector< std::vector<float> > planeParameters;
		// corners of the bounding rectangle
		std::vector<int> bPoints;
		// Consider all levels or not
		bool considerAllLevels;
		// File number to be saved
		int numFile;
		// Render convex hull or not
		bool renderPoly;
		// Render bounding rectangle or not
		bool renderRect;
		// Distance by which the extracted plane should be translated
		float translateDistance;
		// 
		std::vector< std::vector<float> > projectedPoints;

	public:

		bool newImageAvailable; /*!< */

		/**
		 * @brief Constructor
		 * @details 
		 * @param
		 * @return
		 */
		ImageView(ControlUINode* node);

		/**
		 * @brief Destructor 
		 * @details 
		 * @param
		 * @return
		 */
		~ImageView();

		/**
		 * @brief 
		 * @details 
		 * @param
		 * @return
		 */
		void newImage(sensor_msgs::ImageConstPtr img);

		/**
		 * @brief Performs certain functions when a certain key is pressed
		 * @details Key r -> Need to reset all the entire points to initial state
		 * 					Key t -> Toggles rendering polygon. Calls
		 * 						extractBoundingPoly();
		 * 					Key b -> Calls extractBoundingRect();
		 * 					Key s -> Save all the keypoint information. Calls
		 * 						ControlUINode::saveKeyPointInformation(numFile);
		 * 					Key r -> Need to reset all the entire points to initial state. Turn off all rendering
		 * 					Key d -> Empty pointsClicked and keyPointsNearest vectors
		 * 					Key space -> Send control commands to drone. TODO need to implement
		 * 					Key e -> Extract multiple planes. Calls extractBoundingPoly();,
		 * 						ControlUINode::fitMultiplePlanes3d() . It prints the continuous bounding box
		 * 						points
		 * 					Key g -> While moving the quadcopter we don’t want bounding box to appear. Set
		 * 						renderRect to false and call ControlUINode::moveQuadcopter()
		 * @param [int] key - ASCII Value of pressed key
		 * @return
		*/
		virtual void on_key_down(int key);

		/**
		 * @brief Runs certain functions based on mouse click
		 * @details Store the co-ordinates of point clicked on the image screen (0, 0) -> (640, 360) to
		 * 					pointsClicked vector. 
		 * @param [CVD::ImageRef] where
		 * @param [int] state
		 * @param [int] button
		 * @return
		 */
		virtual void on_mouse_down(CVD::ImageRef where, int state, int button);

		/**
		 * @brief @todo May be this call initiates the drone_control GUI
		 * @details 
		 * @param
		 * @return
		 */
		void startSystem();

		/**
		 * @brief @todo May be this call terminates/closes the drone_control GUI
		 * @details 
		 * @param
		 * @return
		 */
		void stopSystem();

		/**
		 * @brief 
		 * @details Callback functions. Calls newImage() function
		 * @param
		 * @return
		 */
		void vidCb (const sensor_msgs::ImageConstPtr img);

		/**
		 * @brief 
		 * @details Callback functions
		 * @param
		 * @return
		 */
		void comCb (const std_msgs::StringConstPtr str);


		/**
		 * @brief The main rendering function
		 * @details 
		 * @param
		 * @return
		 */
		void renderFrame();

		/**
		 * @brief Search function : given a 2d image point searches for the nearest 2d keypoint in the same frame
		 * @details Calls ControlUINode::searchNearest() function. Adds the nearest key point to 
		 * 					keyPointsNearest vector and displays the 3d point found
		 * @param [vector<int>] pt 2d point clicked on the image
		 * @return
		 */
		void search(std::vector<int> pt);

		/**
		 * @brief Extracts convex hull of the clicked points
		 * @details 
		 * @param
		 * @return
		 */
		void extractBoundingPoly();

		/**
		 * @brief Extracts bounding rectangle of the clicked points
		 * @details Bounds: (MinXOfAllPoints, MinYOfAllPoints); (MaxXOfAllPoints,
		 * 					MaxYOfAllPoints). Uses pointsClicked vector and writes the bounding 
		 * 					rectangle points to bPoints vector
		 * @param
		 * @return
		 */
		void extractBoundingRect();

		/*** NEWER FUNCTIONS ***/

		void readInfo(string filename,
					vector< vector<float> > &sortedPlaneParameters,
					vector< vector<Point3f> > &boundingBoxPoints);

		void
		WriteInfoToFile(const vector<Point3f> &bounding_box_points, 
					const vector<float> &plane_parameters, 
					int plane_num, string filename);

		void split(	const string &s,
					char delim,
					vector<float> &elems);

		void split(	const string &s,
					vector<float> &elems);

		/**
		 * @brief Get the bounding box points for currently visible planes.
		 * @details Get the 2d points and corresponding plane labels
		 * @param [in] [vector< Point3f >] points - 3d world co-ordinates
		 * @param [in] [map< float, float >] 3d_to_2d - Mapping from 3d world co-ordinates to 2d image co-ordinates
		 * @param [out] [vector< vector<float> >] feature_2d_points - 2d image co-ordinates
		 * @param [out] [vector<int>] labels - Output labels for the feature_2d_points
		 * @param [out] sortedPlaneParameters - plane parameters corresponding to plane i after arranging the points by
		 * 					their distance from X axis
		 */
		int
		findMultiplePlanesIn2D(
				const vector<Point3f> &points,
				const map<float, float> &_3d_to_2d_map,
				vector<Point2f> &feature_2d_points,
				vector<int> &labels,
				vector< vector<float> > &sortedPlaneParameters,
				vector<Point2f> boundPoints);

};


#endif
