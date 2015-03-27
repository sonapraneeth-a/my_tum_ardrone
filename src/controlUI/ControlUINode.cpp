/**
Created on February 19th 2015
Author : Anirudh Vemula
*/

#include "ControlUINode.h"
#include "ros/ros.h"
#include "tum_ardrone/keypoint_coord.h"
#include "tum_ardrone/filter_state.h"
#include "ransacPlaneFit.h"
#include "ImageView.h"

// OpenCV related stuff
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <string>
#include <fstream>
#include <stdlib.h>
#include <sstream>

using namespace std;

pthread_mutex_t ControlUINode::keyPoint_CS = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t ControlUINode::pose_CS = PTHREAD_MUTEX_INITIALIZER;


ControlUINode::ControlUINode() {
	command_channel = nh_.resolveName("ardrone/com");
	keypoint_channel = nh_.resolveName("/keypoint_coord");
	pose_channel = nh_.resolveName("ardrone/predictedPose");

	keypoint_coord_sub = nh_.subscribe(keypoint_channel, 10, &ControlUINode::keyPointDataCb, this);
	pose_sub = nh_.subscribe(pose_channel, 10, &ControlUINode::poseCb, this);

	tum_ardrone_pub = nh_.advertise<std_msgs::String>(command_channel, 50);
	tum_ardrone_sub = nh_.subscribe(command_channel, 50, &ControlUINode::comCb, this);

	image_gui = new ImageView(this);

	ransacVerbose = true;
	useScaleFactor = true;
	threshold = 0.1;
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

void ControlUINode::poseCb (const tum_ardrone::filter_stateConstPtr statePtr) {
	pthread_mutex_lock(&pose_CS);

	scale = statePtr->scale;
	scale_z = statePtr->scale_z;
	x_offset = statePtr->x_offset;
	y_offset = statePtr->y_offset;
	z_offset = statePtr->z_offset;

	pthread_mutex_unlock(&pose_CS);
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
	pthread_mutex_lock(&pose_CS);

	_3d_points.clear();
	for (int i = 0; i < numPoints; ++i)
	{
		std::vector<float> p;
		if(!useScaleFactor) {
			p.push_back(x_w[i]);
			p.push_back(y_w[i]);
			p.push_back(z_w[i]);
		}
		else {
			p.push_back(x_w[i]*scale + x_offset);
			p.push_back(y_w[i]*scale + y_offset);
			p.push_back(z_w[i]*scale_z + z_offset);
		}
		_3d_points.push_back(p);
	}

	pthread_mutex_unlock(&pose_CS);
	//printf("Size of 3d points : %d\n", _3d_points.size());
}

void ControlUINode::loadLevels (std::vector<int> levels) {
	_levels.clear();
	for (int i = 0; i < numPoints; ++i)
	{
		_levels.push_back(levels[i]);
	}
}

std::vector<float> ControlUINode::fitPlane3d (std::vector<int> ccPoints, std::vector<std::vector<int> > pointsClicked) {

	std::vector<std::vector<float> > _in_points;

	std::vector<std::vector<int> > points;
	for(unsigned int i=0; i<ccPoints.size(); i++) {
		points.push_back(pointsClicked[ccPoints[i]]);
	}

	pthread_mutex_lock(&keyPoint_CS);

	for(unsigned int i=0; i<_2d_points.size(); i++) {
		if(liesInside(points, _2d_points[i])) {
			//printf("%f, %f, %f\n", _3d_points[i][0], _3d_points[i][1], _3d_points[i][2]);
			_in_points.push_back(_3d_points[i]);
		}
	}

	pthread_mutex_unlock(&keyPoint_CS);
	//ROS_INFO("Number of keypoints inside %d", _in_points.size());
	//ROS_INFO("Total number of keypoints %d", _3d_points.size());
	_3d_plane = ransacPlaneFit(_in_points, ransacVerbose);

	return _3d_plane;
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
		for (unsigned int i=0; i<_2d_points.size(); i++)
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
		for (unsigned int i=0; i<_2d_points.size(); i++)
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
		for (unsigned int i = 0; i < _3d_points.size(); ++i)
		{
			if(_levels[i]==0 && distance3D(pt, _3d_points[i]) < threshold) {
				float s = distance3D(pt, _3d_points[i]);
				if(s<minDist) {
					minDist = s;
					min = i;
				}
			}
		}
	}
	else {
		for (unsigned int i = 0; i < _3d_points.size(); ++i)
		{
			if(distance3D(pt, _3d_points[i]) < threshold) {
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

bool ControlUINode::get2DPointNearest (std::vector<float> pt, std::vector<int> &p, bool considerAllLevels) {
	pthread_mutex_lock(&keyPoint_CS);

	// ROS_INFO("Total num %d\n", numPoints);

	bool found = false;

	float minDist = 10000000.0;
	int min = -1;

	if(!considerAllLevels) {
		for (unsigned int i = 0; i < _3d_points.size(); ++i)
		{
			if(_levels[i]==0) {
				float s = distance3D(pt, _3d_points[i]);
				if(s<minDist) {
					minDist = s;
					min = i;
				}
			}
		}
	}
	else {
		for (unsigned int i = 0; i < _3d_points.size(); ++i)
		{
			//if(distance3D(pt, _3d_points[i]) < 0.05) {
				float s = distance3D(pt, _3d_points[i]);
				if(s<minDist) {
					minDist = s;
					min = i;
				}
			//}
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
	for(unsigned int i=0; i<_3d_points.size(); i++) {
		fp<<_3d_points[i][0]<<","<<_3d_points[i][1]<<","<<_3d_points[i][2]<<","<<_levels[i]<<endl;
	}
	fp.close();

	pthread_mutex_unlock(&keyPoint_CS);
}

std::vector<float> ControlUINode::translatePlane (float translateDistance) {
	
	std::vector<float> translatedPlane;

	float a = _3d_plane[0];
	float b = _3d_plane[1];
	float c = _3d_plane[2];

	float norm = sqrt(a*a + b*b + c*c);

	std::vector<float> unitNorm;
	unitNorm.push_back(a/norm);
	unitNorm.push_back(b/norm);
	unitNorm.push_back(c/norm);

	std::vector<float> pointLyingOnPlane;
	if(b!=0) {
		pointLyingOnPlane.push_back(0);
		pointLyingOnPlane.push_back(-1/b);
		pointLyingOnPlane.push_back(0);
	}
	else if(a!=0) {
		pointLyingOnPlane.push_back(-1/a);
		pointLyingOnPlane.push_back(0);
		pointLyingOnPlane.push_back(0);
	}
	else if(c!=0) {
		pointLyingOnPlane.push_back(0);
		pointLyingOnPlane.push_back(0);
		pointLyingOnPlane.push_back(-1/c);
	}
	else {
		ROS_INFO("Invalid Plane");
	}

	std::vector<float> vectorConnectingOriginToPoint;
	vectorConnectingOriginToPoint.push_back(-pointLyingOnPlane[0]);
	vectorConnectingOriginToPoint.push_back(-pointLyingOnPlane[1]);
	vectorConnectingOriginToPoint.push_back(-pointLyingOnPlane[2]);

	int dir = sign(innerProduct(vectorConnectingOriginToPoint, unitNorm));
	if(dir==1) {
		//correct side
		translatedPlane.push_back(a);
		translatedPlane.push_back(b);
		translatedPlane.push_back(c);
		translatedPlane.push_back(1 - translateDistance);
	}
	else if(dir==-1){
		//opposite side
		/*unitNorm[0] = -unitNorm[0];
		unitNorm[1] = -unitNorm[1];
		unitNorm[2] = -unitNorm[2];*/
		translatedPlane.push_back(a);
		translatedPlane.push_back(b);
		translatedPlane.push_back(c);
		translatedPlane.push_back(1 + translateDistance);

	}
	else {
		// origin lies on the plane
		ROS_INFO("Origin lies on the plane");
		translatedPlane.push_back(a);
		translatedPlane.push_back(b);
		translatedPlane.push_back(c);
		translatedPlane.push_back(1);
	}

	return translatedPlane;

}

std::vector<std::vector<float> > ControlUINode::projectPoints (std::vector<int> ccPoints, std::vector<std::vector<float> > keyPoints) {
	std::vector<std::vector<float> > pPoints;
	for(unsigned int i=0; i<ccPoints.size(); i++) {
		std::vector<float> v = projectPoint(_3d_plane, keyPoints[ccPoints[i]]);
		pPoints.push_back(v);
	}
	return pPoints;
}

grid ControlUINode::buildGrid (std::vector<std::vector<float> > pPoints) {
	std::vector<float> lu;
	float width, height;

	float squareWidth = 0.6, squareHeight = 0.34, overlap = 0.5;

	// Assuming that the plane is always parallel to XZ plane - ? Gotta change this
	getDimensions(pPoints, lu, width, height);

	grid g(lu[0], lu[1]-height, lu[0]+width, lu[1], squareWidth, squareHeight, overlap);
	gridSquare gs(lu, squareWidth, squareHeight);
	g.add(gs);
	//gs.debugPrint();
	while(g.translate(gs)) {
		gs = g.getLatest();
		//gs.debugPrint();

	}

	//ROS_INFO("Number of rows in grid : %d", g.row+1);
	//g.print();

	return g;
}

/*pGrid ControlUINode::buildPGrid (std::vector<std::vector<float> > pPoints) {
	std::vector<float> lu, rd, dd;
	float maxD, maxR;

	float squareWidth = 0.4, squareHeight = 0.4, overlap = 0.5;

	getPDimensions (pPoints, lu, rd, dd, maxD, maxR);
}*/

std::vector<std::vector<double> > ControlUINode::getTargetPoints(grid g, std::vector<float> plane) {

	g.print(plane);

	std::vector<std::vector<double> > tPoints;
	std::vector<std::vector<double> > tPoints_z;

	std::vector<cv::Point2d> imgPoints;
	imgPoints.push_back(cv::Point2d(0,0));
	imgPoints.push_back(cv::Point2d(640,0));
	imgPoints.push_back(cv::Point2d(640,360));
	imgPoints.push_back(cv::Point2d(0,360));
	cv::Mat imgPoints_mat(4,1, CV_64FC2);
	for(int i=0; i<4; i++)
		imgPoints_mat.at<cv::Point2d>(i,0) = imgPoints[i];
	

	cv::Mat cameraMatrix(3,3,cv::DataType<double>::type);
	// Setting camera matrix for vga quality
	//From calibration done on our drone
	cameraMatrix.at<double>(0,0) = 565.710890694431;
	cameraMatrix.at<double>(0,1) = 0;
	cameraMatrix.at<double>(0,2) = 329.70046366652;
	cameraMatrix.at<double>(1,0) = 0;
	cameraMatrix.at<double>(1,1) = 565.110297594854;
	cameraMatrix.at<double>(1,2) = 169.873085097623;
	cameraMatrix.at<double>(2,0) = 0;
	cameraMatrix.at<double>(2,1) = 0;
	cameraMatrix.at<double>(2,2) = 1;
	
	/* From ARDRone package
	cameraMatrix.at<double>(0,0) = 569.883158064802;
	cameraMatrix.at<double>(0,1) = 0;
	cameraMatrix.at<double>(0,2) = 331.403348466206;
	cameraMatrix.at<double>(1,0) = 0;
	cameraMatrix.at<double>(1,1) = 568.007065238522;
	cameraMatrix.at<double>(1,2) = 135.879365106014;
	cameraMatrix.at<double>(2,0) = 0;
	cameraMatrix.at<double>(2,1) = 0;
	cameraMatrix.at<double>(2,2) = 1;
	*/

	cv::Mat distCoeffs(5,1,cv::DataType<double>::type);
	// Setting distortion coefficients
	//From calibration done on our drone
	distCoeffs.at<double>(0) = -0.516089772391501;
	distCoeffs.at<double>(1) = 0.285181914111246;
	distCoeffs.at<double>(2) = -0.000466469917823537;
	distCoeffs.at<double>(3) = 0.000864792975814983;
	distCoeffs.at<double>(4) = 0;

	/* From ARDrone package
	distCoeffs.at<double>(0) = -0.526629354780687;
	distCoeffs.at<double>(1) = 0.274357114262035;
	distCoeffs.at<double>(2) = 0.0211426202132638;
	distCoeffs.at<double>(3) = -0.0063942451330052;
	distCoeffs.at<double>(4) = 0;
	*/
	cv::Mat rvec(3,1,cv::DataType<double>::type);
	cv::Mat tvec(3,1,cv::DataType<double>::type);



	std::vector<cv::Point3d> objPoints;


	bool forward = true; // Need to iterate forward or backward
	for(unsigned int i=0; i < g.rowSquares.size()-1; i++) {


		/** In drone coordinate system,

			(gs.u, getY(gs.u, gs.v, plane), gs.v) --------------- (gs.u + gs.width, getY(gs.u+gs.width,gs.v, plane), gs.v)
						|																|
						|																|
						|																|
						|																|
			(gs.u, getY(gs.u, gs.v - gs.height, gs.v), gs.v - gs.height) ------------ (gs.u + gs.width, getY(gs.u+gs.width, gs.v-gs.height, plane), gs.v-gs.height)
		*/


		// Iterating across rows
		//ROS_INFO("Starting %dth row", i);
		//ROS_INFO("Size of %dth row is %d", i, g.rowSquares[i].size());
		if(forward) {
			for(unsigned int j=0; j < g.rowSquares[i].size(); j++) {
				//ROS_INFO("Accessing %dth square of %dth row", j, i);
				objPoints.clear();
				gridSquare gs = g.rowSquares[i][j];
				// objPoints.push_back(cv::Point3d(gs.u, getY(gs.u, gs.v, plane), gs.v));
				objPoints.push_back(cv::Point3d(gs.u, - gs.v, getY(gs.u, gs.v, plane)));
				// objPoints.push_back(cv::Point3d(gs.u + gs.width, getY(gs.u + gs.width, gs.v, plane), gs.v));
				objPoints.push_back(cv::Point3d(gs.u + gs.width, - gs.v, getY(gs.u + gs.width, gs.v, plane)));
				// objPoints.push_back(cv::Point3d(gs.u + gs.width, getY(gs.u + gs.width, gs.v - gs.height, plane), gs.v - gs.height));
				objPoints.push_back(cv::Point3d(gs.u + gs.width, - (gs.v - gs.height), getY(gs.u + gs.width, gs.v - gs.height, plane)));
				// objPoints.push_back(cv::Point3d(gs.u, getY(gs.u, gs.v - gs.height, plane), gs.v - gs.height));
				objPoints.push_back(cv::Point3d(gs.u, - (gs.v - gs.height), getY(gs.u, gs.v - gs.height, plane)));

				//gs.printCoord();
				//std::cout<<objPoints<<std::endl;
				
				cv::Mat objPoints_mat(4,1, CV_64FC3);				
				for(int i=0; i<4; i++)
					objPoints_mat.at<cv::Point3d>(i,0) = objPoints[i];

				//[MGP]Dont know but we have to call undistortPoints as a dummy call
				//Something to do with older version of opencv which gets linked by mrpt
				cv::Mat dummy;
				cv::undistortPoints(imgPoints_mat, dummy, cameraMatrix, distCoeffs);
				cv::Mat rot_guess = cv::Mat::eye(3,3, CV_64F);
				cv::Rodrigues(rot_guess, rvec);
				tvec.at<double>(0)  = -(gs.u + (gs.width/2));
				tvec.at<double>(1)  = gs.v - (gs.height/2);
				tvec.at<double>(2)  = -(getY(gs.u + (gs.width/2), gs.v - (gs.height/2), plane) - 0.3);

				cv::solvePnP(objPoints_mat, imgPoints_mat, cameraMatrix, distCoeffs, rvec, tvec, true, CV_ITERATIVE);
		
				// std::cout<<"rvec : "<<rvec<<std::endl;
				// std::cout<<"tvec : "<<tvec<<std::endl;

				cv::Mat rot(3,3, cv::DataType<double>::type);
				cv::Rodrigues(rvec, rot);

				cv::Mat rotinv;
				transpose(rot, rotinv);

				tvec = -rotinv * tvec;

				//std::cout<<"rotated tvec : "<<tvec<<std::endl;

				std::vector<double> pt;
				pt.push_back(tvec.at<double>(0));
				pt.push_back(tvec.at<double>(2));
				pt.push_back(-tvec.at<double>(1));
				tPoints.push_back(pt);

				pt.clear();
				pt.push_back(gs.u + (gs.width/2));
				pt.push_back(tvec.at<double>(2));
				pt.push_back(gs.v - (gs.height/2));
				tPoints_z.push_back(pt);
			}
		}
		else {
			for(int j = g.rowSquares[i].size()-1; j>=0 ; j--) {
				//ROS_INFO("Accessing %dth square of %dth row", j, i);
				objPoints.clear();
				gridSquare gs = g.rowSquares[i][j];
				// objPoints.push_back(cv::Point3d(gs.u, getY(gs.u, gs.v, plane), gs.v));
				objPoints.push_back(cv::Point3d(gs.u, -gs.v, getY(gs.u, gs.v, plane)));
				// objPoints.push_back(cv::Point3d(gs.u + gs.width, getY(gs.u + gs.width, gs.v, plane), gs.v));
				objPoints.push_back(cv::Point3d(gs.u + gs.width, -gs.v, getY(gs.u + gs.width, gs.v, plane)));
				// objPoints.push_back(cv::Point3d(gs.u + gs.width, getY(gs.u + gs.width, gs.v - gs.height, plane), gs.v - gs.height));
				objPoints.push_back(cv::Point3d(gs.u + gs.width, -(gs.v - gs.height), getY(gs.u + gs.width, gs.v - gs.height, plane)));
				// objPoints.push_back(cv::Point3d(gs.u, getY(gs.u, gs.v - gs.height, plane), gs.v - gs.height));
				objPoints.push_back(cv::Point3d(gs.u, -(gs.v - gs.height), getY(gs.u, gs.v - gs.height, plane)));

				//gs.printCoord();
				//std::cout<<objPoints<<std::endl;

				cv::Mat objPoints_mat(4,1, CV_64FC3);				
				for(int i=0; i<4; i++)
					objPoints_mat.at<cv::Point3d>(i,0) = objPoints[i];

				//[MGP]Dont know but we have to call undistortPoints as a dummy call
				//Something to do with older version of opencv which gets linked by mrpt
				cv::Mat dummy;
				cv::undistortPoints(imgPoints_mat, dummy, cameraMatrix, distCoeffs);
				cv::Mat rot_guess = cv::Mat::eye(3,3, CV_64F);
				cv::Rodrigues(rot_guess, rvec);
				tvec.at<double>(0)  = -(gs.u + (gs.width/2));
				tvec.at<double>(1)  = gs.v - (gs.height/2);
				tvec.at<double>(2)  = -(getY(gs.u + (gs.width/2), gs.v - (gs.height/2), plane) - 0.3);

				cv::solvePnP(objPoints_mat, imgPoints_mat, cameraMatrix, distCoeffs, rvec, tvec, true, CV_ITERATIVE);


				// std::cout<<"rvec : "<<rvec<<std::endl;
				// std::cout<<"tvec : "<<tvec<<std::endl;

				cv::Mat rot(3,3, cv::DataType<double>::type);
				cv::Rodrigues(rvec, rot);

				cv::Mat rotinv;
				transpose(rot, rotinv);

				tvec = -rotinv * tvec;

				//std::cout<<"rotated tvec : "<<tvec<<std::endl;

				std::vector<double> pt;
				pt.push_back(tvec.at<double>(0));
				pt.push_back(tvec.at<double>(2));
				pt.push_back(-tvec.at<double>(1));
				tPoints.push_back(pt);

				pt.clear();
				pt.push_back(gs.u + (gs.width/2));
				pt.push_back(tvec.at<double>(2));
				pt.push_back(gs.v - (gs.height/2));
				tPoints_z.push_back(pt);
			}
		}

		forward = !forward;
	}
	printf("\ntarget points\n\n");
	print3dPoints(tPoints);
	//printf("Points with X and Z just midpoints\n");
	//print3dPoints(tPoints_z);

	return tPoints;
}
