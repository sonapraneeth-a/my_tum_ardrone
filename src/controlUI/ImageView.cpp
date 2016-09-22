/*****************************************************************************************
 * ImageView.cpp
 *
 *       Created on: 21-Feb-2015
 *    Last Modified: 22-Sep-2016
 *  Original Author: Anirudh Vemula
 *   Current Author: Meghshyam Govind Prasad
 *   Current Author: Sona Praneeth Akula
 *          Project: Quadcopter
 *      Description: 
 *
 * Date				Author							Modification
 * 12-Sep-2016	Sona Praneeth Akula			* Added comments to the code
 * 19-Sep-2016	Sona Praneeth Akula			* Added code for on_key_down(); key c, g
 * 21-Sep-2016	Sona Praneeth Akula			* Added threading to TopView
 * 22-Sep-2016	Sona Praneeth Akula			* Added code to destroy TopView once its job is done
 *****************************************************************************************/


#include "ros/ros.h"
#include "ImageView.h"
#include <cvd/gl_helpers.h>
//#include <gvars3/instances.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "GLWindow2.h"
#include <iostream>
#include <fstream>
#include <string>
#include "std_msgs/String.h"
#include "../HelperFunctions.h"
#include "ControlUINode.h"

/*typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Polyhedron_3<K> Polyhedron_3;
typedef K::Segment_3 Segment_3;

typedef K::Point_3 Point_3;
typedef CGAL::Creator_uniform_3<double,Point_3> PointCreator;*/
using namespace std;

ImageView::ImageView(ControlUINode *cnode)
{
	frameWidth = frameHeight = 0;

	video_channel = nh_.resolveName("ardrone/front/image_raw"); // Change this for undistorted image
	command_channel = nh_.resolveName("tum_ardrone/com");

	vid_sub = nh_.subscribe(video_channel, 10, &ImageView::vidCb, this);
	tum_ardrone_sub = nh_.subscribe(command_channel, 50, &ImageView::comCb, this);

	tum_ardrone_pub = nh_.advertise<std_msgs::String>(command_channel, 50);

	node = cnode;

	numPointsClicked = 0;
	numKeyPointsDetected = 0;

	considerAllLevels = true;
	renderPoly = false;
	renderRect = false;

	numFile = 0;
	translateDistance = 0.5;
}

ImageView::~ImageView()
{

}

void
ImageView::vidCb (const sensor_msgs::ImageConstPtr img)
{
	newImage(img);
}

void
ImageView::comCb (const std_msgs::StringConstPtr str)
{

}

void
ImageView::startSystem()
{
	keepRunning = true;
	changeSizeNextRender = false;
	start();
}

void
ImageView::stopSystem()
{
	keepRunning = false;
	new_frame_signal.notify_all();
	join();
}

void
ImageView::ResetInternal()
{
	mimFrameBW.resize(CVD::ImageRef(frameWidth, frameHeight));
	mimFrameBW_workingCopy.resize(CVD::ImageRef(frameWidth, frameHeight));
}


void
ImageView::newImage(sensor_msgs::ImageConstPtr img)
{
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::RGB8);

	boost::unique_lock<boost::mutex> lock(new_frame_signal_mutex);

	// Copy to internal image, convert to bw, set flag
	if (ros::Time::now() - img->header.stamp > ros::Duration(30.0))
	{
		mimFrameTimeRos = (ros::Time::now()  - ros::Duration(0.001));
	}
	else
	{
		mimFrameTimeRos = img->header.stamp;
	}

	mimFrameTime = getMS(mimFrameTimeRos);

	mimFrameSEQ = img->header.seq;

	// Copy to mimFrame
	if (mimFrameBW.size().x != (int)img->width || mimFrameBW.size().y != (int)img->height)
		mimFrameBW.resize(CVD::ImageRef(img->width, img->height));

	memcpy(mimFrameBW.data(), cv_ptr->image.data, img->width * img->height * 3);
	newImageAvailable = true;

	lock.unlock();
	new_frame_signal.notify_all();
}


void
ImageView::run()
{
	while(!newImageAvailable)
		usleep(100000);
	newImageAvailable = false;
	while(!newImageAvailable)
		usleep(100000);

	// read image height and width
	frameHeight = mimFrameBW.size().y;
	frameWidth = mimFrameBW.size().x;

	ResetInternal();

	// Create window
	myGLWindow = new GLWindow2(CVD::ImageRef(frameWidth, frameHeight), "DRONE CAMERA FEED", this);
	myGLWindow->set_title("DRONE CAMERA FEED");

	changeSizeNextRender = true;
	if (frameWidth < 640)
	{
		desiredWindowSize = CVD::ImageRef(frameWidth*2, frameHeight*2);
	}
	else
	{
		desiredWindowSize = CVD::ImageRef(frameWidth, frameHeight);
	}

	boost::unique_lock<boost::mutex> lock(new_frame_signal_mutex);

	while(keepRunning)
	{
		if(newImageAvailable)
		{
			newImageAvailable = false;

			// Copy to working copy
			mimFrameBW_workingCopy.copy_from(mimFrameBW);
			mimFrameTime_workingCopy = mimFrameTime;
			mimFrameSEQ_workingCopy = mimFrameSEQ;
			mimFrameTimeRos_workingCopy = mimFrameTimeRos;

			//release lock
			lock.unlock();

			renderFrame();

			if (changeSizeNextRender)
			{
				myGLWindow->set_size(desiredWindowSize);
				changeSizeNextRender = false;
			}

			lock.lock();
		}
		else
			new_frame_signal.wait(lock);
	}

	lock.unlock();
	delete myGLWindow;

}

void
ImageView::renderFrame()
{

	// Setup
	myGLWindow->SetupViewport();
	myGLWindow->SetupVideoOrtho();
	myGLWindow->SetupVideoRasterPosAndZoom();

	// Render image from drone camera
	glDrawPixels(mimFrameBW_workingCopy);

	glPointSize(6);
	glEnable(GL_BLEND);
	glEnable(GL_POINT_SMOOTH);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glLineWidth(5);

	// Rendering clicked points

	/*glColor3f(1.0,0,0);
	glBegin(GL_POINTS);
	for (int i = 0; i < numPointsClicked; ++i)
	{
		glVertex2i(pointsClicked[i][0], pointsClicked[i][1]);
	}
	glEnd();*/
	
	// Render detected keyPoints
	if(!renderPoly)
	{
		glColor3f(0, 0, 1.0);
		glBegin(GL_POINTS);
		for (int i = 0; i < numKeyPointsDetected; ++i)
		{
			vector<int> p;
			bool found = node->get2DPoint(keyPointsNearest[i], p, considerAllLevels);
			if(found)
			{
				glVertex2i(p[0], p[1]);
			}
			else
			{

			}
		}
		glEnd();
	}
	else if(renderPoly)
	{
		// render convex hull polygon

		/*glColor3f(0, 0, 1.0);
		glBegin(GL_POINTS);
		for (int i = 0; i < numKeyPointsDetected; ++i)
		{
			vector<int> p;
			bool found = node->get2DPoint(keyPointsNearest[i], p, considerAllLevels);
			if(found) {
				glVertex2i(p[0], p[1]);
			}
			else {

			}
		}
		glEnd();*/


		glColor3f(0, 1.0, 0.0);
		glBegin(GL_LINE_STRIP);
		for(unsigned int i=0; i<ccPoints.size(); i++)
		{
			vector<int> p;
			bool found = node->get2DPoint(keyPointsNearest[ccPoints[i]], p, considerAllLevels);
			if(found)
			{
				glVertex2i(p[0], p[1]);
			}
			else
			{

			}
		}
		glEnd();
	}
	if(renderRect)
	{		
		int size = continuousBoundingBoxPoints.size();
		
		for(int planeIndex = 0; planeIndex<size; planeIndex++)
		{
			vector<Point3f> planeBoundingBoxPoints = continuousBoundingBoxPoints[planeIndex];
			vector< vector<int> > planePts2D;
			//glColor3f(0, 1.0, 0.0);
			glColor3f(1.0-1.0/(planeIndex+1), 0, 1.0/(planeIndex+1)); 
			glBegin(GL_LINE_STRIP);
			/*	
			for(int pointIndex = 0; pointIndex<planeBoundingBoxPoints.size(); pointIndex++) {
				vector<float> pt(3);
				pt[0] = planeBoundingBoxPoints[pointIndex].x;
				pt[1] = planeBoundingBoxPoints[pointIndex].y;
				pt[2] = planeBoundingBoxPoints[pointIndex].z;
				vector<int> p;
				bool found = node->get2DPoint(pt, p, true);
				if (found){
					planePts2D.push_back(p);
					glVertex2i(p[0],p[1]);
				}
			}
			*/
			vector<Point2f> imagePts;
			node->project3DPointsOnImage(planeBoundingBoxPoints, imagePts);
			for(int pointIndex = 0; pointIndex<imagePts.size(); pointIndex++)
			{
				vector<int> p(2);
				p[0] = imagePts[pointIndex].x;
				p[1] = imagePts[pointIndex].y;
				glVertex2i(p[0],p[1]);
			}
		
			glEnd();
		}
	}
	glDisable(GL_BLEND);

	myGLWindow->swap_buffers();
	myGLWindow->HandlePendingEvents();
}

// Keyboard input
void
ImageView::on_key_down(int key)
{
	// r - Reset all the vectors and rendering
	if(key == 114)
	{
		// node->publishCommand("i reset");
		// Need to reset all the entire points to initial state
		numPointsClicked = 0;
		numKeyPointsDetected = 0;
		pointsClicked.clear();
		keyPointsNearest.clear();
		renderPoly = false;
		renderRect= false;
	}
	// d - Clear the vectors 'pointsClicked' and 'keyPointsNearest'
	if(key == 100)
	{
		// node->publishCommand("i delete");
		// Need to delete the last point
		if(numPointsClicked != 0)
		{
			numPointsClicked--;
			pointsClicked.pop_back();
		}
		if(numKeyPointsDetected != 0)
		{
			numKeyPointsDetected--;
			keyPointsNearest.pop_back();
		}
	}
	// space
	if(key == 32)
	{
		// node->publishCommand("i run");
		// Send control commands to drone. TODO need to implement
	}
	// s - Saves the keypoint information '_3d_points' to file named by 'numFile'
	if(key == 115)
	{
		// save all the keypoint information
		node->saveKeyPointInformation(numFile);
		numFile++;
	}
	// t
	if(key == 116)
	{
		// renders the polygon
		//renderRect = false;
		renderPoly = !renderPoly;
		extractBoundingPoly();
	}
	// b
	if(key == 98)
	{
		// renders the bounding rectangle
		// renderPoly = false;
		// renderRect = !renderRect;
		extractBoundingRect();
	}
	// e - Extract Multiple planes using Jlinkage and store the bounding box points for the planess
	if(key == 101)
	{
		/*
		 * TODO Currently commented code which fits single plane and moves the drone accordingly
		 * Appropriate code needs to be added which will handle multiple planes
		 * For time being for multiple planes each plane is shown in different color
		 *
			vector<float> plane = node->fitPlane3d (ccPoints, pointsClicked);
			vector<vector<float> > pPoints =  node->projectPoints (ccPoints, keyPointsNearest);
			grid g = node->buildGrid(pPoints);
			vector<vector<double> > tPoints =  node->getTargetPoints(g, plane);
			node->moveDrone(tPoints);
		*/
		// Extract multiple planes
		extractBoundingPoly();
		renderRect = true;
		int size = continuousBoundingBoxPoints.size();
		assert(size == planeParameters.size());
		int i,j;
		for (i = 0; i < size; ++i)
		{
			continuousBoundingBoxPoints[i].clear();
			planeParameters[i].clear();
		}
		continuousBoundingBoxPoints.clear();
		planeParameters.clear();
		// Calls JLinkage
		node->fitMultiplePlanes3d(ccPoints, pointsClicked, planeParameters, continuousBoundingBoxPoints);
		cout << "[ DEBUG ] continuousBoundingBoxPoints from ImageView\n";
		size = continuousBoundingBoxPoints.size();
		for (i = 0; i < size; ++i)
		{
			for (j = 0; j < 5; ++j)
			{
				cout << continuousBoundingBoxPoints[i][j] << " ";
			}
			cout << "\n";
		}
		//vector<float> translatedPlane = node->translatePlane (translateDistance);
	}
	/* Below are the keys to be implemented properly fro mtp stage 01 */
	// g - Navigating the planes using the planned path and collecting the video footage
	if(key == 103)
	{
		// Cover multiple planes
		if(renderRect)
		{
			renderRect = false;  //While moving the quadcopter we don't want bounding box to appear
			node->moveQuadcopter(planeParameters, continuousBoundingBoxPoints);
		}
	}
	// n - Navigating the planes using the planned path and collecting the video footage
	// Uses the points written from a file
	if(key == 110)
	{
		// Read points from the file
		// For planeParameters and continuousBoundingBoxPoints

		// Cover multiple planes
		if(renderRect)
		{
			renderRect = false;  // While moving the quadcopter we don't want bounding box to appear
			node->moveQuadcopter(planeParameters, continuousBoundingBoxPoints);
		}
	}
	// Key c
	if(key == 99)
	{
		/* Launches GUI: Return approx. angles and orientations */
		/* Angle with which	quadcopter has to rotate to orient itself to the new plane */
		std::vector< double > main_angles;
		/* Direction in which quadcopter should rotate to orient its yaw with normal of new plane */
		std::vector< RotateDirection > main_direction;
		/* Initiating the GUI */
		TopView *top = new TopView();
		top->startSystem();
		/* GUI has been closed here */
		cout << "Calling the GUI for drawing top view sketch of the surface\n";
		while(!(top->getExitStatus()))
		{}
		/* Get the directions, angles and number of planes */
		top->getDirections(main_direction);
		top->getAngles(main_angles);
		/* Printing the information to the terminal */
		int number_of_planes = top->getNumberOfPlanes();
		if(number_of_planes == 0)
		{
			cout << "[ ERROR] No diagram drawn\n";
		}
		else
		{
			cout << "Number of planes: " << number_of_planes << "\n";
		}
		cout << "Max Height of the plane (as estimated by the user): " << top->getMaxHeightOfPlane() << "\n";
		if(top->getTypeOfSurface() == 0)
			cout << "Surface Drawn: " << "Open Surface" << "\n";
		if(top->getTypeOfSurface() == 1)
			cout << "Surface Drawn: " << "Closed Surface" << "\n";
		top->destroy();
		cout << "Angles: ";
		for (int i = 0; i < main_angles.size(); ++i)
		{
			cout << main_angles[i] << " ";
		}
		cout << "\n";
		cout << "Directions: ";
		for (int i = 0; i < main_direction.size(); ++i)
		{
			if(main_direction[i] == 0)
				cout << "CLOCKWISE" << " ";
			else if(main_direction[i] == 1)
				cout << "ANTI-CLOCKWISE" << " ";
		}
		cout << "\n";
		if(number_of_planes > 0)
		{
			// node->getMeTheMap(main_angles, main_directions);
			// Land the quadcopter on completion of the task
			cout << "Bounding Box Points collected\n";
			cout << "Landing the quadcopter\n";
			node->sendLand();
		}
		else
		{
			cout << "[ ERROR] No diagram drawn. So, landing the quadcopter\n";
			node->sendLand();
		}
		cout << "[ INFO] Quadcopter has landed.\n";
	}
	// Key n
	// Uses the points written from a file (obtained from key g)
	if(key == 110)
	{
		string inputDirectory = "./";
		string filename = inputDirectory + "Plane_3D_Points.txt";
		vector< vector<float> > sortedPlaneParameters;
		vector< vector<Point3f> > boundingBoxPoints;
		readInfo(filename, sortedPlaneParameters, boundingBoxPoints);
		cout << "Plane Parameters\n";
		for (int i = 0; i < sortedPlaneParameters.size(); ++i)
		{
			int param = sortedPlaneParameters[i].size();
			for (int j = 0; j < param; ++j)
			{
				cout << sortedPlaneParameters[i][j] << " ";
			}
			cout << "\n";
		}
		cout << "Bounding Box Points\n";
		for (int i = 0; i < boundingBoxPoints.size(); ++i)
		{
			int param = boundingBoxPoints[i].size();
			for (int j = 0; j < param; ++j)
			{
				cout << boundingBoxPoints[i][j] << "\n";
			}
			cout << "\n";
		}
		// Get the bounding box points
		getContinuousBoundingBox ( boundingBoxPoints, sortedPlaneParameters,
								continuousBoundingBoxPoints);
		// Path planning: Cover multiple planes
		if(renderRect)
		{
			renderRect = false;  // While moving the quadcopter we don't want bounding box to appear
			node->moveQuadcopter(planeParameters, continuousBoundingBoxPoints);
		}
	}
}

void
ImageView::on_mouse_down(CVD::ImageRef where, int state, int button)
{
	//double x = 4*(where.x/(double)this->myGLWindow->size().x - 0.5);
	//double y = -4*(where.y/(double)this->myGLWindow->size().y - 0.5);

	int x;
	int y;

	if(this->myGLWindow->size().x==640)
	{
		x = where.x;
	}
	else if(this->myGLWindow->size().x!=0)
	{
		x = (int)((640.0*where.x)/(this->myGLWindow->size().x));
	}

	if(this->myGLWindow->size().y==360)
	{
		y = where.y;
	}
	else if(this->myGLWindow->size().y!=0)
	{
		y = (int)((360.0*where.y)/(this->myGLWindow->size().y));
	}

	printf("X and Y of point clicked : (%d, %d)\n", x, y);

	numPointsClicked++;
	vector<int> v;
	v.push_back(x);
	v.push_back(y);
	pointsClicked.push_back(v);

	search(v);
}



void
ImageView::search(vector<int> pt)
{
	vector<float> kp = node->searchNearest(pt, considerAllLevels);

	keyPointsNearest.push_back(kp);
	numKeyPointsDetected++;
	printf("X, Y and Z of nearest keypoint : (%f, %f, %f)\n", kp[0], kp[1], kp[2]);
}

void
ImageView::extractBoundingPoly()
{
	// Check if the number of clicked points is exactly 4
	//assert(pointsClicked.size()==4);
	
	//vector<vector<int> > ccPoints;
	ccPoints.clear();

	vector<int> minXPoint;
	float minX = -1;
	int minXIndex = -1;
	for(unsigned int i=0; i<pointsClicked.size(); i++)
	{
		if(minX==-1)
		{
			minX = pointsClicked[i][0];
			minXPoint = pointsClicked[i];
			minXIndex = i;
		}
		else if(pointsClicked[i][0]<minX)
		{
			minX = pointsClicked[i][0];
			minXPoint = pointsClicked[i];
			minXIndex = i;
		}
	}

	//ccPoints.push_back(minXPoint);

	vector<int> endPoint;
	int endIndex;
	int i=0;
	do
	{
		ccPoints.push_back(minXIndex);
		endPoint = pointsClicked[0];
		endIndex = 0;
		for(unsigned int j=1;j<pointsClicked.size();j++)
		{
			if((endPoint[0]==minXPoint[0] && 
				endPoint[1]==minXPoint[1]) 
				|| 
				onLeft(pointsClicked[j], minXPoint, endPoint))
			{
				endPoint = pointsClicked[j];
				endIndex = j;
			}
		}
		i = i+1;
		minXPoint = endPoint;
		minXIndex = endIndex;

	} while(endPoint[0]!=pointsClicked[ccPoints[0]][0] 
		|| endPoint[1]!=pointsClicked[ccPoints[0]][1]);
	ccPoints.push_back(endIndex);

}


/*** NEWER FUNCTIONS ***/

void
ImageView::extractBoundingRect()
{

	bPoints.clear();

	int minX = pointsClicked[0][0], minY = pointsClicked[0][1];
	int maxX = pointsClicked[0][0], maxY = pointsClicked[0][1];
	int minXIndex = 0, minYIndex =0, maxXIndex = 0, maxYIndex = 0;

	for(unsigned int i=0; i<pointsClicked.size(); i++)
	{
		if(pointsClicked[i][0] < minX)
		{
			minX = pointsClicked[i][0];
			minXIndex = i;
		}
		if(pointsClicked[i][0] > maxX)
		{
			maxX = pointsClicked[i][0];
			maxXIndex = i;
		}
		if(pointsClicked[i][1] < minY)
		{
			minY = pointsClicked[i][1];
			minYIndex = i;
		}
		if(pointsClicked[i][1] > maxY)
		{
			maxY = pointsClicked[i][1];
			maxYIndex = i;
		}
	}

	bPoints.push_back(minXIndex);
	bPoints.push_back(minYIndex);
	bPoints.push_back(maxXIndex);
	bPoints.push_back(maxYIndex);
}

void
ImageView::readInfo(string filename,
					vector< vector<float> > &sortedPlaneParameters,
					vector< vector<Point3f> > &boundingBoxPoints)
{
	sortedPlaneParameters.clear();
	boundingBoxPoints.clear();
	ifstream plane_info;
	plane_info.open(filename.c_str(), std::ifstream::in);
	string line;
	vector<Point3f> box_points;
	Point3f location;
	vector<float> copyVector;
	string plane_string("Plane-Parameters");
	string box_string("Bounding-Box-Points");
	if (plane_info.is_open())
	{
		while ( getline (plane_info, line) )
		{
			/*cout << "Line: " << line << "\n";
			cout << "1-Compare: " << line.compare(plane_string) << "\n";
			cout << "2-Compare: " << line.compare(box_string) << "\n";*/
			if(!line.empty() && line.compare(plane_string)==1)
			{
				getline(plane_info, line);
				split(line, copyVector);
				sortedPlaneParameters.push_back(copyVector);
				copyVector.clear();
			}
			if(!line.empty() && line.compare(box_string)==1)
			{
				for (int i = 1; i <= 4; ++i)
				{
					getline(plane_info, line);
					split(line, copyVector);
					Point3f point;
					point.x = copyVector[0];
					point.y = copyVector[1];
					point.z = copyVector[2];
					box_points.push_back(point);
					copyVector.clear();
				}
				boundingBoxPoints.push_back(box_points);
				box_points.clear();
			}
		}
	}
	else
	{
		cerr << "[ DEBUG] File " << filename << " doesn't open\n";
		return ;
	}
	plane_info.close();
}

void
ImageView::split(	const string &s,
					vector<float> &elems)
{
	istringstream ss(s);
	while( ! ss.eof() )
	{
		float tmp_f;
		if ( ss >> tmp_f )
		{
			elems.push_back(tmp_f);
		}
	}
}

/**
 * @brief Write the plane parameters and bounding box points of all planes to file
 * @details See points_00.csv for details of the written file
 * @param [vector<Point3f>] bounding_box_points - Bounding box points of the plane
 * @param [vector<float>] plane_parameters - (a, b, c, d) of the plane
 * @param [int] plane_num - Plane number whose points are written to file
 * @param [string] file_name - Name of the file to which the points are written
 */
void
ImageView::WriteInfoToFile(const vector<Point3f> &bounding_box_points, 
			const vector<float> &plane_parameters, 
			int plane_num, string filename)
{
	int num_bounding_box_points = bounding_box_points.size();
	const char* outFilename = filename.c_str();
	ofstream outFile;
	// Open the object in writing and appending mode
	outFile.open(outFilename, ios::app);
	// Check if the file is open
	if (!outFile.is_open())
	{
		cerr << "\nFile " << filename << " cannot be opened for writing.\n";
		cerr << "Please check if the file is existing and has required permissions ";
		cerr << " for writing.\n";
	}
	outFile << "Plane " << std::setfill ('0') << std::setw (2) << to_string(plane_num) << "\n";
	outFile << "Bounding-Box-Points\n";
	for (int i = 0; i < num_bounding_box_points; ++i)
	{
		outFile << bounding_box_points[i].x << " "
			<< bounding_box_points[i].y << " "
			<< bounding_box_points[i].z << "\n";
	}
	outFile << "Plane-Parameters\n";
	for (int i = 0; i < plane_parameters.size(); ++i)
	{
		if(i != plane_parameters.size()-1)
			outFile << plane_parameters[i] << " ";
		else
			outFile << plane_parameters[i] << "\n";
	}
	// Close the file
	outFile.close();
	return ;
}


// /**
//  * @brief Get the bounding box points for currently visible planes.
//  * @details Get the 2d points and corresponding plane labels
//  * @param [in] [vector< Point3f >] points - 3d world co-ordinates
//  * @param [in] [map< float, float >] 3d_to_2d - Mapping from 3d world co-ordinates to 2d image co-ordinates
//  * @param [out] [vector< vector<float> >] feature_2d_points - 2d image co-ordinates
//  * @param [out] [vector<int>] labels - Output labels for the feature_2d_points
//  * @param [out] sortedPlaneParameters - plane parameters corresponding to plane i after arranging the points by
//  * 					their distance from X axis
//  */
// int
// ImageView::findMultiplePlanesIn2D(
// 		const vector<Point3f> &points,
// 		const map<float, float> 3d_to_2d,
// 		vector<Point2f> &feature_2d_points,
// 		vector<int> &labels,
// 		vector< vector<float> > &sortedPlaneParameters,
// 		vector<Point2f> boundPoints)
// {
// 	vector<Point3f> _in_points;
// 	vector< vector<int> > points;
// 	for(unsigned int i=0; i<ccPoints.size(); i++)
// 	{
// 		points.push_back(pointsClicked[ccPoints[i]]);
// 	}
// 	pthread_mutex_lock(&keyPoint_CS);
// 	for(unsigned int i=0; i<_2d_points.size(); i++)
// 	{
// 		if(liesInside(points, _2d_points[i]))
// 		{
// 			Point3f featurePt;
// 			featurePt.x = _3d_points[i][0];
// 			featurePt.y = _3d_points[i][1];
// 			featurePt.z = _3d_points[i][2];
// 			_in_points.push_back(featurePt);
// 		}
// 	}
// 	pthread_mutex_unlock(&keyPoint_CS);
// 	// Step 1: Performing JLinkage to find multiple models
// 	// This vector describes to which plane does point i belong to
// 	// This is an output from JLinkage
// 	vector<int> oldPlaneIndices;
// 	callJLinkage(_in_points, oldPlaneIndices);
// 	// Step 2: Remove points which contain less than 'minimumNumberOfPoints'
// 	// We have taken 'minimumNumberOfPoints' to be 30
// 	// Minimum number of points required to be qualified to be in data
// 	int minPointsPerPlane = 30;
// 	// Number of points in 'points' during initial stage
// 	int numberOfPointsInData = points.size();
// 	// Mapping planeIndex -> number of points in that plane
// 	map<int, int> numberOfPointsPerPlane;
// 	// Plane Indices after removing unnecessary planes
// 	vector<int> newPlaneIndices;
// 	vector<Point3f> newPoints;
// 	// Number of planes after removing unnecessary planes
// 	int numberOfPlanes;
// 	removeUnnecessaryPlanes( _in_points, oldPlaneIndices, minPointsPerPlane,
// 			numberOfPointsPerPlane, newPoints, newPlaneIndices, numberOfPlanes);
// 	// Try to find for multiple planes only if there are more than 1 plane.
// 	if( numberOfPlanes > 1 )
// 	{
// 		// Step 3: Perform K-means for the new set of planes obtained
// 		Mat pointsMatrix = Mat(newPoints.size(), 3, CV_32F);
// 		// Create a pointsMatrix from vector<Point3f> data
// 		for(int i = 0; i < newPoints.size(); i++) {
// 			float *ptr = (float*)(&pointsMatrix.at<float>(i,0));
// 			ptr[0] = newPoints[i].x;
// 			ptr[1] = newPoints[i].y;
// 			ptr[2] = newPoints[i].z;
// 		}
// 		// Cluster centroid being returned from kmeans
// 		Mat clusterCentroids;
// 		/* Number of rounds for k-means */
// 		int numberOfRounds = 10;
// 		// Perform k-means
// 		// Reference: http://docs.opencv.org/2.4/modules/core/doc/clustering.html
// 		// float kmeans(const cv::_InputArray &,
// 		// int, const cv::_OutputArray &,
// 		// cv::TermCriteria, int, int, const cv::_OutputArray &);
// 		// Performing kmeans using initial labels we've obtained in step 1
// 		kmeans(pointsMatrix, numberOfPlanes, newPlaneIndices,
// 				TermCriteria( CV_TERMCRIT_ITER, 10, 2.0), numberOfRounds,
// 				KMEANS_USE_INITIAL_LABELS, clusterCentroids);
// 		// Plane Parameters (a, b, c, d) for plane i
// 		vector< vector<float> > planeParameters;
// 		// Obtain the plane parameters for the set of points obtained after Step 2
// 		// Arrange plane points belonging to a particular plane i
// 		vector< vector<Point3f> > planeOrderedPoints;
// 		getPlaneParameters(newPoints, newPlaneIndices, planeParameters, planeOrderedPoints);
// 		// Step 4: Remove points which far from the estimated plane after performing k-means
// 		// Get 3D Projection of points onto the plane
// 		vector<float> distanceMatrix;
// 		vector< vector<int> > planePointsIndexMapping;
// 		vector<Point3f> newSortedPoints;
// 		vector< vector<float> > newPlaneParameters;
// 		map<int, pair<int, int> > planeIndexBounds;
// 		vector<Point3f> projectionsOf3DPoints;
// 		// Calculate the distances of the points from their respective planes
// 		calculateDistanceFromPlane( newPoints, planeParameters, newPlaneIndices,
// 				distanceMatrix, planePointsIndexMapping);
// 		// Step 5
// 		// Order plane by left to right as viewed from camera
// 		vector<Point3f> sortedProjectionsOf3DPoints;
// 		map<int, pair<int, int> > sortedPlaneIndexBounds;
// 		vector< vector<Point3f> > boundingBoxPoints;
// 		orderPlanePointsByCentroids( projectionsOf3DPoints, newPlaneParameters, planeIndexBounds,
// 				sortedProjectionsOf3DPoints, sortedPlaneParameters, sortedPlaneIndexBounds);
// 	}
// 	else
// 	{
// 		vector<float> planeParameters;
// 		vector<Point3f> projectionsOf3DPoints;
// 		fitPlane3D(newPoints, planeParameters);
// 		float a = planeParameters[0];
// 		float b = planeParameters[1];
// 		float c = planeParameters[2];
// 		float d = planeParameters[3];
// 		for (int j = 0; j < newPoints.size(); ++j)
// 		{
// 			float x0 = newPoints[j].x;
// 			float y0 = newPoints[j].y;
// 			float z0 = newPoints[j].z;
// 			float t = ((-1)*(a*x0+b*y0+c*z0+d))/(a*a+b*b+c*c);
// 			float projX0 = x0 + a*t;
// 			float projY0 = y0 + b*t;
// 			float projZ0 = z0 + c*t;
// 			projectionsOf3DPoints.push_back(Point3f(projX0, projY0, projZ0));
// 		}
// 		sortedPlaneParameters.push_back(planeParameters);
// 	}
// 	for (int i = 0; i < newPlaneIndices.size(); ++i)
// 	{
// 		labels.push_back(newPlaneIndices);
// 	}
// 	for (int i = 0; i < newPoints.size(); ++i)
// 	{
// 		feature_2d_points.push_back(3d_to_2d[newPoints[i]]);
// 	}
// 	extractBoundingRectIn2D(feature_2d_points, boundPoints);
// 	return 0;

// }


// void
// ImageView::split(	const string &s,
// 					char delim,
// 					vector<float> &elems)
// {
// 	stringstream ss(s);
// 	string item;
// 	while (getline(ss, item, delim))
// 	{
// 		elems.push_back(stof(item));
// 	}
// }
