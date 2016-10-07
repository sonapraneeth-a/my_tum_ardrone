/*****************************************************************************************
 * ControlUINode.cpp
 *
 *       Created on: 19-Feb-2015
 *    Last Modified: 25-Sep-2016
 *  Original Author: Anirudh Vemula
 *   Current Author: Meghshyam Govind Prasad
 *   Current Author: Sona Praneeth Akula
 *          Project: Quadcopter
 *      Description:
 *
 * Date				Author							Modification
 * 12-Sep-2016	Sona Praneeth Akula			* Added comments to the code
 * 21-Sep-2016	Sona Praneeth Akula			* Added code to land the quadcopter
 *****************************************************************************************/

#include "ControlUINode.h"
#include "ros/ros.h"
#include "tum_ardrone/keypoint_coord.h"
#include "tum_ardrone/filter_state.h"
#include "ransacPlaneFit.h"
#include "ImageView.h"
#include "allHeaders.hpp"


// OpenCV related stuff
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "ardrone_autonomy/RecordEnable.h"
#include "Multiple-Plane-JLinkage/conversion.hpp"
#include "Multiple-Plane-JLinkage/utilities.hpp"
#include "Multiple-Plane-JLinkage/makeBoundingRects.hpp"
#include "Multiple-Plane-JLinkage/multiplePlanes.hpp"

#include "std_msgs/String.h"
#include "std_msgs/Empty.h"

#include <string>
#include <fstream>
#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <list>

using namespace std;
using namespace cv;

pthread_mutex_t ControlUINode::keyPoint_CS = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t ControlUINode::pose_CS = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t ControlUINode::tum_ardrone_CS = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t ControlUINode::command_CS = PTHREAD_MUTEX_INITIALIZER;


ControlUINode::ControlUINode()
{
	// Command channel for sending/receiving commands (goto)
	command_channel = nh_.resolveName("tum_ardrone/com");
	// Channel for receiving the keypoints at various levels (1 to 4)
	keypoint_channel = nh_.resolveName("/keypoint_coord");
	// Channel for getting the current quadcopter position (x, y, z, roll, pitch, yaw)
	pose_channel = nh_.resolveName("ardrone/predictedPose");
	// Channel for landing the quadcopter
	land_channel = nh_.resolveName("ardrone/land");

	// Subscribing for key point channel
	keypoint_coord_sub = nh_.subscribe(keypoint_channel, 10, &ControlUINode::keyPointDataCb, this);
	// Subscribing for pose channel
	pose_sub = nh_.subscribe(pose_channel, 10, &ControlUINode::poseCb, this);

	tum_ardrone_pub = nh_.advertise<std_msgs::String>(command_channel, 50);
	tum_ardrone_sub = nh_.subscribe(command_channel, 50, &ControlUINode::comCb, this);
	land_pub	   = nh_.advertise<std_msgs::Empty>(land_channel,1);

	// For recording video
	video = nh_.serviceClient<ardrone_autonomy::RecordEnable>("ardrone/setrecord");

	// Initiating image view class which displays the "drone_controlUI" window
	image_gui = new ImageView(this);

	ransacVerbose = true;
	useScaleFactor = true;
	threshold = 0.1;
	error_threshold = 0.2;
	recordTime = 3.5; // a second
	pollingTime = 0.5;
	record = true;
	targetSet = false;

	currentCommand = false;
	justNavigation = false; got_points = false;
	recordNow = false;
	notRecording = true;
	planeIndex = 0;

	timer_checkPos = nh_.createTimer(ros::Duration(pollingTime), &ControlUINode::checkPos, this);
	// timer_record = nh_.createTimer(ros::Duration(recordTime), &ControlUINode::recordVideo);
}

ControlUINode::~ControlUINode()
{

}

void
ControlUINode::keyPointDataCb (const tum_ardrone::keypoint_coordConstPtr coordPtr)
{
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

void
ControlUINode::poseCb (const tum_ardrone::filter_stateConstPtr statePtr)
{
	pthread_mutex_lock(&pose_CS);

	scale = statePtr->scale;
	scale_z = statePtr->scale_z;
	x_offset = statePtr->x_offset;
	y_offset = statePtr->y_offset;
	z_offset = statePtr->z_offset;

	x_drone = statePtr->x;
	y_drone = statePtr->y;
	z_drone = statePtr->z;
	yaw = statePtr->yaw;
	roll = statePtr->roll;
	pitch = statePtr->pitch;

	pthread_mutex_unlock(&pose_CS);

	// Goto commands left to be executed

	pthread_mutex_lock(&command_CS);
	static int numCommands = 0;
	if(commands.size() > 0 && !currentCommand)
	{
		currentCommand = true;
		numCommands++;
		pthread_mutex_lock(&tum_ardrone_CS);
		tum_ardrone_pub.publish(commands.front());
		pthread_mutex_unlock(&tum_ardrone_CS);
		targetPoint = targetPoints.front();
		printf(" Current target: %lf %lf %lf\n", targetPoint[0], targetPoint[1] , targetPoint[2] );
	}
	else if(currentCommand && !recordNow)
	{
		static int planeIndexCurrent = 0;
		if(planeIndexCurrent< (numberOfPlanes-1) &&
			numCommands > startTargePtIndex[planeIndexCurrent+1])
			planeIndexCurrent++;

		if(numCommands < startTargePtIndex[planeIndexCurrent]+8)
		{
			ros::Duration(1).sleep();
			currentCommand = false;
			commands.pop_front();
			targetPoints.pop_front();
			pthread_mutex_unlock(&command_CS);
			return;
		}
		double x = targetPoint[0];
		double y = targetPoint[1];
		double z = targetPoint[2];

		pthread_mutex_lock(&pose_CS);
		double ea = sqrt(pow(x - x_drone, 2) + pow(y - y_drone, 2) + pow(z - z_drone, 2));
		//printf("Error %lf\n", ea);
		pthread_mutex_unlock(&pose_CS);
		if(ea < error_threshold)
		{
			//printf("reached\n");
			recordNow = true;
			ros::Duration(3).sleep();
			last= ros::Time::now();
		}
		else
		{
			recordNow = false;
		}
	}
	else if(recordNow)
	{
		if(ros::Time::now() - last < ros::Duration(recordTime))
		{
			if(record && notRecording)
			{
				ardrone_autonomy::RecordEnable srv;
				srv.request.enable = true;
				video.call(srv);
				notRecording = false;
				popen("rosbag record /ardrone/image_raw /ardrone/predictedPose --duration=3", "r");
			}
			else if(!notRecording)
			{

			}
		}
		else
		{
			ardrone_autonomy::RecordEnable srv;
			srv.request.enable = false;
			video.call(srv);
			currentCommand = false;
			notRecording = true;
			recordNow = false;
			commands.pop_front();
			targetPoints.pop_front();
			ros::Duration(3).sleep();
		}
	}
	else
	{
		// do nothing
	}
	pthread_mutex_unlock(&command_CS);
}

void
ControlUINode::load2dPoints (vector<float> x_img, vector<float> y_img)
{
	_2d_points.clear();
	for (int i = 0; i < numPoints; ++i)
	{
		vector<float> p;
		p.push_back(x_img[i]);
		p.push_back(y_img[i]);
		_2d_points.push_back(p);
	}
}

void
ControlUINode::write3DPointsToCSV(vector<vector<float> > &_3d_points)
{

	int i, j;
	int numberOfPoints = _3d_points.size();

	string filename = "points_fckohli_test_08042016.csv";
	const char* outFilename = filename.c_str();
	ofstream outFile;
	// Open the object in writing mode
	outFile.open(outFilename, ios::out);
	// Check if the file is open
	if (!outFile.is_open())
	{
		cerr << "\nFile " << filename << " cannot be opened for writint.\n";
		cerr << "Please check if the file is existing and has required permissions ";
		cerr << " for writing.\n";
	}

	for (i = 0; i < numberOfPoints; ++i)
	{
		int dimensions = _3d_points[i].size();
		for (j = 0; j < dimensions; ++j)
		{
			if(j != dimensions-1)
			{
				outFile << _3d_points[i][j] << ", ";
			}
			else
			{
				outFile << _3d_points[i][j] << "\n";
			}
		}
	}

	// Close the file
	outFile.close();
	return ;

}

void
ControlUINode::load3dPoints (vector<float> x_w, vector<float> y_w, vector<float> z_w)
{
	pthread_mutex_lock(&pose_CS);

	_3d_points.clear();
	for (int i = 0; i < numPoints; ++i)
	{
		vector<float> p;
		if(!useScaleFactor)
		{
			p.push_back(x_w[i]);
			p.push_back(y_w[i]);
			p.push_back(z_w[i]);
		}
		else
		{
			p.push_back(x_w[i]*scale + x_offset);
			p.push_back(y_w[i]*scale + y_offset);
			p.push_back(z_w[i]*scale_z + z_offset);
		}
		_3d_points.push_back(p);
	}

	pthread_mutex_unlock(&pose_CS);
	//write3DPointsToCSV(_3d_points);
}

void
ControlUINode::loadLevels (vector<int> levels)
{
	_levels.clear();
	for (int i = 0; i < numPoints; ++i)
	{
		_levels.push_back(levels[i]);
	}
}

vector<float>
ControlUINode::fitPlane3d (vector<int> ccPoints, vector<vector<int> > pointsClicked)
{

	vector<vector<float> > _in_points;

	vector<vector<int> > points;
	for(unsigned int i=0; i<ccPoints.size(); i++)
	{
		points.push_back(pointsClicked[ccPoints[i]]);
	}

	pthread_mutex_lock(&keyPoint_CS);

	for(unsigned int i=0; i<_2d_points.size(); i++)
	{
		if(liesInside(points, _2d_points[i]))
		{
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

void
ControlUINode::fitMultiplePlanes3d (vector<int> &ccPoints, vector<vector<int> > &pointsClicked,
									vector<vector<float> >&planeParameters,
									vector< vector<Point3f> > & continuousBoundingBoxPoints)
{
	vector<Point3f> _in_points;

	vector<vector<int> > points;
	for(unsigned int i = 0; i < ccPoints.size(); i++)
	{
		points.push_back(pointsClicked[ccPoints[i]]);
	}

	pthread_mutex_lock(&keyPoint_CS);

	for(unsigned int i = 0; i < _2d_points.size(); i++)
	{
		if(liesInside(points, _2d_points[i]))
		{
			//printf("%f, %f, %f\n", _3d_points[i][0], _3d_points[i][1], _3d_points[i][2]);
			Point3f featurePt;
			featurePt.x = _3d_points[i][0];
			featurePt.y = _3d_points[i][1];
			featurePt.z = _3d_points[i][2];
			_in_points.push_back(featurePt);
		}
	}

	pthread_mutex_unlock(&keyPoint_CS);
	findMultiplePlanes(_in_points, planeParameters, continuousBoundingBoxPoints);
}

void
ControlUINode::moveQuadcopter(
		const vector< vector<float> > &planeParameters,
		const vector< vector<Point3f> > &continuousBoundingBoxPoints)
{

	numberOfPlanes = planeParameters.size();
	int i, j;

	vector<Point2f> uvCoordinates;
	vector<Point3f> uvAxes, xyzGridCoord;
	vector<float> uCoord, vCoord, uVector, vVector;
	startTargePtIndex.resize(numberOfPlanes, 0);
	startTargePtIndex[0] = 0;
	vector<double> prevPosition(3);
	double prevYaw = 0;
	pthread_mutex_lock(&command_CS);
	for (i = 0; i < numberOfPlanes; ++i)
	{

		// TODO: Move the quadcopter to face the plane i (i>0)

		// Make parameters for making the grid
		float a = planeParameters[i][0];
		float b = planeParameters[i][1];
		float c = planeParameters[i][2];
		float d = planeParameters[i][3];
		uvCoordinates.clear();
		uvAxes.clear();
		// Convert XYZ bounding points to UV coordinates
		AllXYZToUVCoordinates(
			continuousBoundingBoxPoints[i], planeParameters[i],
			uvCoordinates, uvAxes);
		uVector.clear();
		uVector.push_back(uvAxes[0].x);
		uVector.push_back(uvAxes[0].y);
		uVector.push_back(uvAxes[0].z);
		vVector.clear();
		vVector.push_back(uvAxes[1].x);
		vVector.push_back(uvAxes[1].y);
		vVector.push_back(uvAxes[1].z);
		uCoord.clear();

		pGrid grid = buildPGrid(uvCoordinates);
		//printGrid(grid, uvAxes, planeParameters[i]);
		vector< vector<double> > pTargetPoints;
		getPTargetPoints(grid, planeParameters[i], uvAxes, pTargetPoints);
		double desiredYaw = 0;
		Point3f projectedNormal(planeParameters[i][0], planeParameters[i][1], 0);
		Point3f yAxis(0,1,0);
		desiredYaw= findAngle(projectedNormal, yAxis);
		desiredYaw= desiredYaw*180/M_PI;
		if(i ==0)
		{
			prevPosition[0] = x_drone;
			prevPosition[1] = y_drone;
			prevPosition[2] = z_drone;
			prevYaw = yaw;
		}
		moveDrone(prevPosition, pTargetPoints, prevYaw, desiredYaw);
		int numTargetPoints = pTargetPoints.size();
		prevPosition[0] = pTargetPoints[numTargetPoints-1][0];
		prevPosition[1] = pTargetPoints[numTargetPoints-1][1] - 0.6;
		prevPosition[2] = pTargetPoints[numTargetPoints-1][2];
		prevYaw = desiredYaw;
		planeIndex++;
	}
	pthread_mutex_unlock(&command_CS);

	return ;

}

void
ControlUINode::printGrid(const pGrid &g, const vector<Point3f> &uvAxes, const vector<float> &plane)
{
	vector<Point2f> uvCoordinates;
	vector<Point3f> xyzCorners;
	cout<<"[ Debug ] UV Grid \n";
	for(unsigned int i=0; i<g.rowSquares.size(); i++)
	{
		for(unsigned int j=0; j<g.rowSquares[i].size(); j++)
		{
			float u = g.rowSquares[i][j].u;
			float v = g.rowSquares[i][j].v;
			Point2f uv(u,v);
			uvCoordinates.push_back(uv);
		}
	}

	AllUVToXYZCoordinates(uvCoordinates, uvAxes, plane[3], xyzCorners);
	cout << uvCoordinates<<"\n";
	cout << xyzCorners<<"\n";
}



void
ControlUINode::Loop ()
{
	while(nh_.ok())
	{
		ros::spinOnce();
	}
}

void
ControlUINode::comCb (const std_msgs::StringConstPtr str)
{

}

float
ControlUINode::distance (vector<int> p1, vector<float> p2)
{
	return sqrt((p2[0]-p1[0])*(p2[0]-p1[0]) + (p2[1]-p1[1])*(p2[1]-p1[1]));
}

float
ControlUINode::distance3D (vector<float> p1, vector<float> p2)
{
	return sqrt((p2[0]-p1[0])*(p2[0]-p1[0]) +
				(p2[1]-p1[1])*(p2[1]-p1[1]) +
				(p2[2]-p1[2])*(p2[2]-p1[2]));
}

vector<float>
ControlUINode::searchNearest (vector<int> pt, bool considerAllLevels)
{

	pthread_mutex_lock(&keyPoint_CS);

	float min = -1;
	vector<float> minPt;

	if(!considerAllLevels)
	{
		for (unsigned int i=0; i<_2d_points.size(); i++)
		{
			if(_levels[i]==0)
			{
				if(min==-1)
				{
					min = distance(pt, _2d_points[i]);
					minPt = _3d_points[i];
				}
				else
				{
					float s = distance(pt, _2d_points[i]);
					if(s<min)
					{
						min = s;
						minPt = _3d_points[i];
					}
				}
			}
		}
	}
	else
	{
		for (unsigned int i=0; i<_2d_points.size(); i++)
		{
			if(min==-1)
			{
				min = distance(pt, _2d_points[i]);
				minPt = _3d_points[i];
			}
			else
			{
				float s = distance(pt, _2d_points[i]);
				if(s<min)
				{
					min = s;
					minPt = _3d_points[i];
				}
			}
		}
	}

	pthread_mutex_unlock(&keyPoint_CS);


	return minPt;
}

bool
ControlUINode::get2DPoint (vector<float> pt, vector<int> &p,
							bool considerAllLevels)
{

	pthread_mutex_lock(&keyPoint_CS);

	// ROS_INFO("Total num %d\n", numPoints);

	bool found = false;

	float minDist = 10000000.0;
	int min = -1;

	if(!considerAllLevels)
	{
		for (unsigned int i = 0; i < _3d_points.size(); ++i)
		{
			if(_levels[i]==0 && distance3D(pt, _3d_points[i]) < threshold)
			{
				float s = distance3D(pt, _3d_points[i]);
				if(s<minDist)
				{
					minDist = s;
					min = i;
				}
			}
		}
	}
	else
	{
		for (unsigned int i = 0; i < _3d_points.size(); ++i)
		{
			if(distance3D(pt, _3d_points[i]) < threshold)
			{
				float s = distance3D(pt, _3d_points[i]);
				if(s<minDist)
				{
					minDist = s;
					min = i;
				}
			}
		}
	}

	if(min!=-1)
	{
		found = true;
		p.push_back((int)_2d_points[min][0]);
		p.push_back((int)_2d_points[min][1]);
		//ROS_INFO("The minimum distance is %f", minDist);
	}

	pthread_mutex_unlock(&keyPoint_CS);

	return found;
}

bool
ControlUINode::get2DPointNearest (vector<float> pt, vector<int> &p,
									bool considerAllLevels)
{
	pthread_mutex_lock(&keyPoint_CS);

	// ROS_INFO("Total num %d\n", numPoints);

	bool found = false;

	float minDist = 10000000.0;
	int min = -1;

	if(!considerAllLevels)
	{
		for (unsigned int i = 0; i < _3d_points.size(); ++i)
		{
			if(_levels[i]==0)
			{
				float s = distance3D(pt, _3d_points[i]);
				if(s<minDist)
				{
					minDist = s;
					min = i;
				}
			}
		}
	}
	else
	{
		for (unsigned int i = 0; i < _3d_points.size(); ++i)
		{
			//if(distance3D(pt, _3d_points[i]) < 0.05) {
				float s = distance3D(pt, _3d_points[i]);
				if(s<minDist)
				{
					minDist = s;
					min = i;
				}
			//}
		}
	}

	if(min!=-1)
	{
		found = true;
		p.push_back((int)_2d_points[min][0]);
		p.push_back((int)_2d_points[min][1]);
		//ROS_INFO("The minimum distance is %f", minDist);
	}

	pthread_mutex_unlock(&keyPoint_CS);

	return found;
}

bool
ControlUINode::equal(vector<float> p1, vector<float> p2)
{
	if(distance3D(p1, p2) < 0.001)
	{
		return true;
	}
	else
	{
		return false;
	}
}

int
ControlUINode::getNumKP(bool considerAllLevels)
{
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

void
ControlUINode::saveKeyPointInformation (int numFile)
{
	pthread_mutex_lock(&keyPoint_CS);

	//char * name = itoa(numFile);
	stringstream ss;
	ss << numFile;
	string s = ss.str();

	ofstream fp(s.c_str());

	fp << numPoints << endl;
	fp << endl;
	for(unsigned int i=0; i<_3d_points.size(); i++)
	{
		fp << _3d_points[i][0] << ", " << _3d_points[i][1]
				<< ", " << _3d_points[i][2] << ", " << _levels[i] << endl;
	}
	fp.close();

	pthread_mutex_unlock(&keyPoint_CS);
}

vector<float>
ControlUINode::translatePlane (float translateDistance)
{

	vector<float> translatedPlane;

	float a = _3d_plane[0];
	float b = _3d_plane[1];
	float c = _3d_plane[2];

	float norm = sqrt(a*a + b*b + c*c);

	vector<float> unitNorm;
	unitNorm.push_back(a/norm);
	unitNorm.push_back(b/norm);
	unitNorm.push_back(c/norm);

	vector<float> pointLyingOnPlane;
	if(b != 0)
	{
		pointLyingOnPlane.push_back(0);
		pointLyingOnPlane.push_back(-1/b);
		pointLyingOnPlane.push_back(0);
	}
	else if(a != 0)
	{
		pointLyingOnPlane.push_back(-1/a);
		pointLyingOnPlane.push_back(0);
		pointLyingOnPlane.push_back(0);
	}
	else if(c != 0)
	{
		pointLyingOnPlane.push_back(0);
		pointLyingOnPlane.push_back(0);
		pointLyingOnPlane.push_back(-1/c);
	}
	else
	{
		ROS_INFO("Invalid Plane");
	}

	vector<float> vectorConnectingOriginToPoint;
	vectorConnectingOriginToPoint.push_back(-pointLyingOnPlane[0]);
	vectorConnectingOriginToPoint.push_back(-pointLyingOnPlane[1]);
	vectorConnectingOriginToPoint.push_back(-pointLyingOnPlane[2]);

	int dir = sign(innerProduct(vectorConnectingOriginToPoint, unitNorm));
	if(dir == 1)
	{
		//correct side
		translatedPlane.push_back(a);
		translatedPlane.push_back(b);
		translatedPlane.push_back(c);
		translatedPlane.push_back(1 - translateDistance);
	}
	else if(dir == -1)
	{
		//opposite side
		/*unitNorm[0] = -unitNorm[0];
		unitNorm[1] = -unitNorm[1];
		unitNorm[2] = -unitNorm[2];*/
		translatedPlane.push_back(a);
		translatedPlane.push_back(b);
		translatedPlane.push_back(c);
		translatedPlane.push_back(1 + translateDistance);
	}
	else
	{
		// origin lies on the plane
		ROS_INFO("Origin lies on the plane");
		translatedPlane.push_back(a);
		translatedPlane.push_back(b);
		translatedPlane.push_back(c);
		translatedPlane.push_back(1);
	}

	return translatedPlane;

}

vector< vector<float> >
ControlUINode::projectPoints (vector<int> ccPoints, vector<vector<float> > keyPoints)
{
	vector<vector<float> > pPoints;
	for(unsigned int i=0; i<ccPoints.size(); i++)
	{
		vector<float> v = projectPoint(_3d_plane, keyPoints[ccPoints[i]]);
		pPoints.push_back(v);
	}
	return pPoints;
}

grid
ControlUINode::buildGrid (vector<vector<float> > pPoints)
{
	vector<float> lu;
	float width, height;

	float squareWidth = 0.8, squareHeight = 0.45, overlap = 0.334;

	// Assuming that the plane is always parallel to XZ plane - ? Gotta change this
	getDimensions(pPoints, lu, width, height);

	grid g(lu[0], lu[1]-height, lu[0]+width, lu[1], squareWidth, squareHeight, overlap);
	gridSquare gs(lu, squareWidth, squareHeight);
	g.add(gs);
	//gs.debugPrint();
	while(g.translate(gs))
	{
		gs = g.getLatest();
		//gs.debugPrint();
	}

	//ROS_INFO("Number of rows in grid : %d", g.row+1);
	//g.print();

	return g;
}

/*pGrid ControlUINode::buildPGrid (vector<vector<float> > pPoints) {
	vector<float> lu, rd, dd;
	float maxD, maxR;

	float squareWidth = 0.4, squareHeight = 0.4, overlap = 0.5;

	getPDimensions (pPoints, lu, rd, dd, maxD, maxR);
}*/

pGrid
ControlUINode::buildPGrid(const vector<Point2f> &uvCoordinates)
{
	vector<float> uCoord, vCoord;
	vector<Point2f> sortedUVCoordinates;
	sortUVCorners(uvCoordinates, sortedUVCoordinates);
	int j;
	// Make the X Co-ordinates of plane bounding box points
	for (j = 0; j < 4; ++j)
	{
		uCoord.push_back(sortedUVCoordinates[j].x);
	}
	// Make the Y Co-ordinates of plane bouding box points
	for (j = 0; j < 4; ++j)
	{
		vCoord.push_back(sortedUVCoordinates[j].y);
	}
	vector<float> uVector(2), vVector(2);
	uVector[0] = uCoord[1]-uCoord[0];
	uVector[1] = vCoord[1]-vCoord[0];

	vVector[0] = uCoord[0]-uCoord[3];
	vVector[1] = vCoord[0]-vCoord[3];

	float horizDist1 = sqrt(pow(uCoord[0]-uCoord[1],2)+pow(vCoord[0]-vCoord[1],2));
	float horizDist2 = sqrt(pow(uCoord[2]-uCoord[3],2)+pow(vCoord[2]-vCoord[3],2));
	float vertDist1 = sqrt(pow(uCoord[2]-uCoord[1],2)+pow(vCoord[2]-vCoord[1],2));
	float vertDist2 = sqrt(pow(uCoord[0]-uCoord[3],2)+pow(vCoord[0]-vCoord[3],2));
	float squareWidth = 0.8;
	float squareHeight = 0.45;
	float overlap = 0.334;
	float maxR = max(horizDist1, horizDist2);
	float maxD = max(vertDist1, vertDist2) - squareHeight;
	uVector[0] /= horizDist1;
	uVector[1] /= horizDist1;
	vVector[0] /= vertDist1;
	vVector[1] /= vertDist1;
	// pGridSquare width and height
	pGrid grid(uCoord[0], vCoord[0], uVector, vVector, squareWidth, squareHeight, overlap, maxR, maxD);
	pGridSquare gridSquare = pGridSquare(uCoord[0], vCoord[0], squareWidth, squareHeight, uVector, vVector);
	grid.add(gridSquare);
	while (grid.translate(gridSquare))
	{
		gridSquare = grid.getLatest();
	}
	return grid;
}

void
ControlUINode::calibrate()
{
	cameraMatrix = Mat(3,3, DataType<float>::type);
	/*
	cameraMatrix.at<float>(0,0) = 374.6706070969281;
	cameraMatrix.at<float>(0,1) = 0;
	cameraMatrix.at<float>(0,2) = 320.5;
	cameraMatrix.at<float>(1,0) = 0;
	cameraMatrix.at<float>(1,1) = 374.6706070969281;
	cameraMatrix.at<float>(1,2) = 180.5;
	cameraMatrix.at<float>(2,0) = 0;
	cameraMatrix.at<float>(2,1) = 0;
	cameraMatrix.at<float>(2,2) = 1;
	*/
	cameraMatrix.at<float>(0,0) = 565.710890694431;
	cameraMatrix.at<float>(0,1) = 0;
	cameraMatrix.at<float>(0,2) = 329.70046366652;
	cameraMatrix.at<float>(1,0) = 0;
	cameraMatrix.at<float>(1,1) = 565.110297594854;
	cameraMatrix.at<float>(1,2) = 169.873085097623;
	cameraMatrix.at<float>(2,0) = 0;
	cameraMatrix.at<float>(2,1) = 0;
	cameraMatrix.at<float>(2,2) = 1;


	distCoeffs = Mat::zeros(5,1,DataType<float>::type);

	vector<Vec3f> object_points;
	vector<Vec2f> image_pts;
	pthread_mutex_lock(&keyPoint_CS);
	assert(_3d_points.size() == _2d_points.size());
	int numPts = _3d_points.size();
	for(int i = 0; i < numPts; i++)
	{
		Vec3f obj_pt(_3d_points[i][0], _3d_points[i][1], _3d_points[i][2]);
		Vec2f img_pt(_2d_points[i][0], _2d_points[i][1]);
		object_points.push_back(obj_pt);
		image_pts.push_back(img_pt);
	}
	vector<vector<Vec3f>  > object;
	vector<vector<Vec2f>  > image;
	object.push_back(object_points);
	image.push_back(image_pts);
	calibrateCamera(object, image, Size(640, 360), cameraMatrix, distCoeffs, rvecs, tvecs, CV_CALIB_USE_INTRINSIC_GUESS);
	pthread_mutex_unlock(&keyPoint_CS);
	calibrated= true;
}

void
ControlUINode::project3DPointsOnImage(const vector<Point3f> &worldPts, vector<Point2f > & imagePts)
{
/*
	Mat cameraMatrix(3,3,DataType<float>::type);
	// Setting camera matrix for vga quality
	//From calibration done on our drone
	vector<Point3f> transformedWorldPts;
	for(int i=0; i<worldPts.size(); i++){
		Point3f pt = worldPts[i];
		Point3f trPoint;
		trPoint.x = pt.x;
		trPoint.y = -pt.z;
		trPoint.z = pt.y;
		transformedWorldPts.push_back(trPoint);
	}
//Gazebo K: [374.6706070969281, 0.0, 320.5, 0.0, 374.6706070969281, 180.5, 0.0, 0.0, 1.0]
	cameraMatrix.at<float>(0,0) = 374.6706070969281;
	cameraMatrix.at<float>(0,1) = 0;
	cameraMatrix.at<float>(0,2) = 320.5;
	cameraMatrix.at<float>(1,0) = 0;
	cameraMatrix.at<float>(1,1) = 374.6706070969281;
	cameraMatrix.at<float>(1,2) = 180.5;
	cameraMatrix.at<float>(2,0) = 0;
	cameraMatrix.at<float>(2,1) = 0;
	cameraMatrix.at<float>(2,2) = 1;


	cameraMatrix.at<float>(0,0) = 565.710890694431;
	cameraMatrix.at<float>(0,1) = 0;
	cameraMatrix.at<float>(0,2) = 329.70046366652;
	cameraMatrix.at<float>(1,0) = 0;
	cameraMatrix.at<float>(1,1) = 565.110297594854;
	cameraMatrix.at<float>(1,2) = 169.873085097623;
	cameraMatrix.at<float>(2,0) = 0;
	cameraMatrix.at<float>(2,1) = 0;
	cameraMatrix.at<float>(2,2) = 1;


	Mat distCoeffs = Mat::zeros(5,1,DataType<float>::type);
	// Setting distortion coefficients
	//From calibration done on our drone

	distCoeffs.at<float>(0) = -0.516089772391501;
	distCoeffs.at<float>(1) = 0.285181914111246;
	distCoeffs.at<float>(2) = -0.000466469917823537;
	distCoeffs.at<float>(3) = 0.000864792975814983;
	distCoeffs.at<float>(4) = 0;


	Mat R = getRotationMatrix(roll,pitch, yaw);
	Mat rvec;
	Rodrigues(R, rvec);
	//cout<<"Rotation matrix"<<R<<"\n";

	Mat tvec(3, 1, DataType<float>::type);
	tvec.at<float>(0,0) = x_drone;
	tvec.at<float>(1,0) = -z_drone;
	tvec.at<float>(2,0) = y_drone;
	*/
	calibrate();
	cv::projectPoints(worldPts, rvecs[0], tvecs[0], cameraMatrix, distCoeffs, imagePts);
	int numPoints = imagePts.size();
	//cout<<"Projected points:"<<numPoints<<"\n";
	//cout << imagePts<<"\n";
}

vector< vector<double> >
ControlUINode::getTargetPoints(grid g, vector<float> plane)
{

	g.print(plane);

	vector<vector<double> > tPoints;
	vector<vector<double> > tPoints_z;

	vector<Point2d> imgPoints;
	imgPoints.push_back(Point2d(0,0));
	imgPoints.push_back(Point2d(320,0));
	imgPoints.push_back(Point2d(640,0));
	imgPoints.push_back(Point2d(640,180));
	imgPoints.push_back(Point2d(640,360));
	imgPoints.push_back(Point2d(320,360));
	imgPoints.push_back(Point2d(0,360));
	imgPoints.push_back(Point2d(0,180));
	imgPoints.push_back(Point2d(320,180));

	Mat imgPoints_mat(9,1, CV_64FC2);
	for(int i=0; i<9; i++)
		imgPoints_mat.at<Point2d>(i,0) = imgPoints[i];


	Mat cameraMatrix(3,3,DataType<double>::type);
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

	Mat distCoeffs(5,1,DataType<double>::type);
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
	Mat rvec(3, 1, DataType<double>::type);
	Mat tvec(3, 1, DataType<double>::type);



	vector<Point3d> objPoints;


	bool forward = true; // Need to iterate forward or backward
	for(unsigned int i = 0; i < g.rowSquares.size()-1; i++)
	{


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
		if(forward)
		{
			for(unsigned int j=0; j < g.rowSquares[i].size(); j++)
			{
				//ROS_INFO("Accessing %dth square of %dth row", j, i);
				objPoints.clear();
				gridSquare gs = g.rowSquares[i][j];
				Point3d corner1 = Point3d(gs.u, - gs.v, getY(gs.u, gs.v, plane));
				Point3d corner2 = Point3d(gs.u + gs.width, - gs.v, getY(gs.u + gs.width, gs.v, plane));
				Point3d corner3 = Point3d(gs.u + gs.width, - (gs.v - gs.height), getY(gs.u + gs.width, gs.v - gs.height, plane));
				Point3d corner4 = Point3d(gs.u, - (gs.v - gs.height), getY(gs.u, gs.v - gs.height, plane));
				Point3d mid1 = (corner1 + corner2)*0.5;
				mid1.z = getY(mid1.x, mid1.y, plane);
				Point3d mid2 = (corner2 + corner3)*0.5;
				mid2.z = getY(mid2.x, mid2.y, plane);
				Point3d mid3 = (corner3 + corner4)*0.5;
				mid3.z = getY(mid3.x, mid3.y, plane);
				Point3d mid4 = (corner4 + corner1)*0.5;
				mid4.z = getY(mid4.x, mid4.y, plane);
				Point3d center = (mid1 + mid3)*0.5;
				center.z = getY(center.x, center.y,plane)*0.5;

				objPoints.push_back(corner1);
				objPoints.push_back(mid1);
				objPoints.push_back(corner2);
				objPoints.push_back(mid2);
				objPoints.push_back(corner3);
				objPoints.push_back(mid3);
				objPoints.push_back(corner4);
				objPoints.push_back(mid4);
				objPoints.push_back(center);

				//gs.printCoord();
				//cout << objPoints << endl;

				Mat objPoints_mat(9,1, CV_64FC3);
				for(int i=0; i<9; i++)
					objPoints_mat.at<Point3d>(i,0) = objPoints[i];

				//[MGP]Dont know but we have to call undistortPoints as a dummy call
				//Something to do with older version of opencv which gets linked by mrpt
				Mat dummy;
				undistortPoints(imgPoints_mat, dummy, cameraMatrix, distCoeffs);
				Mat rot_guess = Mat::eye(3,3, CV_64F);
				Rodrigues(rot_guess, rvec);
				tvec.at<double>(0)  = -(gs.u + (gs.width/2));
				tvec.at<double>(1)  = gs.v - (gs.height/2);
				tvec.at<double>(2)  = -(getY(gs.u + (gs.width/2), gs.v - (gs.height/2), plane) - 0.6);

				solvePnP(objPoints_mat, imgPoints_mat, cameraMatrix, distCoeffs, rvec, tvec, true, CV_ITERATIVE);

				// cout<<"rvec : "<<rvec << endl;
				// cout<<"tvec : "<<tvec << endl;

				Mat rot(3,3, DataType<double>::type);
				Rodrigues(rvec, rot);

				Mat rotinv;
				transpose(rot, rotinv);

				tvec = -rotinv * tvec;

				//cout<<"rotated tvec : "<<tvec << endl;

				vector<double> pt;
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
		else
		{
			for(int j = g.rowSquares[i].size()-1; j >= 0 ; j--)
			{
				//ROS_INFO("Accessing %dth square of %dth row", j, i);
				objPoints.clear();
				gridSquare gs = g.rowSquares[i][j];
				Point3d corner1 = Point3d(gs.u, - gs.v, getY(gs.u, gs.v, plane));
				Point3d corner2 = Point3d(gs.u + gs.width, - gs.v, getY(gs.u + gs.width, gs.v, plane));
				Point3d corner3 = Point3d(gs.u + gs.width, - (gs.v - gs.height), getY(gs.u + gs.width, gs.v - gs.height, plane));
				Point3d corner4 = Point3d(gs.u, - (gs.v - gs.height), getY(gs.u, gs.v - gs.height, plane));
				Point3d mid1 = (corner1 + corner2)*0.5;
				mid1.z = getY(mid1.x, mid1.y, plane);
				Point3d mid2 = (corner2 + corner3)*0.5;
				mid2.z = getY(mid2.x, mid2.y, plane);
				Point3d mid3 = (corner3 + corner4)*0.5;
				mid3.z = getY(mid3.x, mid3.y, plane);
				Point3d mid4 = (corner4 + corner1)*0.5;
				mid4.z = getY(mid4.x, mid4.y, plane);
				Point3d center = (mid1 + mid3)*0.5;
				center.z = getY(center.x, center.y,plane)*0.5;

				objPoints.push_back(corner1);
				objPoints.push_back(mid1);
				objPoints.push_back(corner2);
				objPoints.push_back(mid2);
				objPoints.push_back(corner3);
				objPoints.push_back(mid3);
				objPoints.push_back(corner4);
				objPoints.push_back(mid4);
				objPoints.push_back(center);

				Mat objPoints_mat(9,1, CV_64FC3);
				for(int i=0; i<9; i++)
					objPoints_mat.at<Point3d>(i,0) = objPoints[i];

				//[MGP]Dont know but we have to call undistortPoints as a dummy call
				//Something to do with older version of opencv which gets linked by mrpt
				Mat dummy;
				undistortPoints(imgPoints_mat, dummy, cameraMatrix, distCoeffs);
				Mat rot_guess = Mat::eye(3,3, CV_64F);
				Rodrigues(rot_guess, rvec);
				tvec.at<double>(0)  = -(gs.u + (gs.width/2));
				tvec.at<double>(1)  = gs.v - (gs.height/2);
				tvec.at<double>(2)  = -(getY(gs.u + (gs.width/2), gs.v - (gs.height/2), plane) - 0.6);

				solvePnP(objPoints_mat, imgPoints_mat, cameraMatrix, distCoeffs, rvec, tvec, true, CV_ITERATIVE);


				// cout<<"rvec : "<<rvec << endl;
				// cout<<"tvec : "<<tvec << endl;

				Mat rot(3,3, DataType<double>::type);
				Rodrigues(rvec, rot);

				Mat rotinv;
				transpose(rot, rotinv);

				tvec = -rotinv * tvec;

				//cout<<"rotated tvec : "<<tvec << endl;

				vector<double> pt;
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

void
ControlUINode::moveDrone (const vector<double> &prevPosition,
							vector<vector<double> > tPoints,
							double prevYaw, double desiredYaw)
{
	double drone_length = 0.6;
	for (unsigned int i = 0; i < tPoints.size(); ++i)
	{
		vector<double> p = tPoints[i];
		p[1] = p[1] - drone_length;

		char buf[100];
		if(i == 0)
		{
			vector<vector <double > > xyz_yaw;
			getInitialPath(prevPosition, p, prevYaw, desiredYaw, xyz_yaw);
			for(int j=0; j<xyz_yaw.size(); j++){
				vector<double> interm_point;
				interm_point = xyz_yaw[j];
				snprintf(buf, 100, "c goto %lf %lf %lf %lf",
					interm_point[0], interm_point[1], interm_point[2], interm_point[3]);
				std_msgs::String s;
				s.data = buf;
				ROS_INFO("Message: ");
				ROS_INFO(buf);
				commands.push_back(s);
				targetPoints.push_back(interm_point);
			}
			/*
			pthread_mutex_lock(&pose_CS);
			double half_y = (y_drone + p[1])/2;
			vector<double> interm_point(3);
			interm_point[0] = x_drone;
			interm_point[1] = half_y;
			interm_point[2] = z_drone;

			snprintf(buf, 100, "c goto %lf %lf %lf %lf", x_drone, half_y, z_drone, yaw);
			std_msgs::String s;
			s.data = buf;
			ROS_INFO("Message: ");
			ROS_INFO(buf);
			commands.push_back(s);
			targetPoints.push_back(interm_point);

			interm_point[1] = p[1];
			snprintf(buf, 100, "c goto %lf %lf %lf %lf", x_drone, p[1], z_drone, yaw);
			std_msgs::String s1;
			s1.data = buf;
			ROS_INFO("Message: ");
			ROS_INFO(buf);
			commands.push_back(s1);
			targetPoints.push_back(interm_point);

			interm_point[0] = p[0];
			snprintf(buf, 100, "c goto %lf %lf %lf %lf", p[0], p[1], z_drone, yaw);
			std_msgs::String s2;
			s2.data = buf;
			ROS_INFO("Message: ");
			ROS_INFO(buf);
			commands.push_back(s2);
			targetPoints.push_back(interm_point);

			interm_point[2] = p[2];
			snprintf(buf, 100, "c goto %lf %lf %lf %lf", p[0], p[1], p[2], yaw);
			std_msgs::String s3;
			s3.data = buf;
			ROS_INFO("Message: ");
			ROS_INFO(buf);
			commands.push_back(s3);
			targetPoints.push_back(p);

			snprintf(buf, 100, "c goto %lf %lf %lf %lf", p[0], p[1], p[2], desiredYaw);
			std_msgs::String s4;
			s4.data = buf;
			ROS_INFO("Message: ");
			ROS_INFO(buf);
			commands.push_back(s4);
			targetPoints.push_back(p);
			pthread_mutex_unlock(&pose_CS);
			*/
		}
		else
		{
			snprintf(buf, 100, "c goto %lf %lf %lf %lf", p[0], p[1], p[2], desiredYaw);
			std_msgs::String s;
			s.data = buf;
			ROS_INFO("Message: ");
			ROS_INFO(buf);
			commands.push_back(s);
			targetPoints.push_back(p);
		}
	}
		if(planeIndex < (numberOfPlanes - 1) )
			startTargePtIndex[planeIndex+1] = targetPoints.size();
}


void
ControlUINode::checkPos(const ros::TimerEvent&)
{
	if(targetSet)
	{
		double x = targetPoint[0];
		double y = targetPoint[1];
		double z = targetPoint[2];

		pthread_mutex_lock(&pose_CS);
		double ea = sqrt(pow(x - x_drone, 2) + pow(y - y_drone, 2) + pow(z - z_drone, 2));
		pthread_mutex_unlock(&pose_CS);

		if(ea < error_threshold)
			targetSet = false;
		else
			targetSet = true;
	}
}

// void ControlUINode::recordVideo(const ros::TimerEvent&) {
	// ardrone_autonomy::RecordEnable srv;
	// srv.request.enable = true;

	// if(video.call(srv)) {
	// 	ros::Duration(recordTime).sleep(); // record for half a second
	// 	srv.request.enable = false;
	// 	video.call(srv);
	// 	return true;
// 	}
// 	else {
// 		return false;
// 	}
// }
//
void
ControlUINode::getPTargetPoints(const pGrid &g, const vector<float> & plane,
									const vector<Point3f> &uvAxes,
									vector< vector<double> > &sortedTPoints)
{

		// TODO: Copy getTargetPoints into new function. - Done
		// NOTE: Y and Z swapped oin camera co-ordinates
		// TODO: Change the corners. We have UV from 'grid'. Change that to XYZ
		// TODO: Arrange the XYZ in X,-Z,Y
		// TODO: Make the other points from width and height and min and max points

	if(!calibrated)
		calibrate();
	vector<vector<double> > tPoints;
	vector<vector<double> > tPoints_z;

	vector<Point2d> imgPoints;
	imgPoints.push_back(Point2d(0,0));
	imgPoints.push_back(Point2d(320,0));
	imgPoints.push_back(Point2d(640,0));
	imgPoints.push_back(Point2d(640,180));
	imgPoints.push_back(Point2d(640,360));
	imgPoints.push_back(Point2d(320,360));
	imgPoints.push_back(Point2d(0,360));
	imgPoints.push_back(Point2d(0,180));
	imgPoints.push_back(Point2d(320,180));

	Mat imgPoints_mat(9,1, CV_64FC2);
	for(int i=0; i<9; i++)
		imgPoints_mat.at<Point2d>(i,0) = imgPoints[i];


	Mat cameraMatrix(3,3,DataType<double>::type);
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

	Mat distCoeffs(5,1,DataType<double>::type);
	// Setting distortion coefficients
	//From calibration done on our drone
	distCoeffs.at<double>(0) = -0.516089772391501;
	distCoeffs.at<double>(1) = 0.285181914111246;
	distCoeffs.at<double>(2) = -0.000466469917823537;
	distCoeffs.at<double>(3) = 0.000864792975814983;
	distCoeffs.at<double>(4) = 0;


	Mat rvec(3,1,DataType<double>::type);
	Mat tvec(3,1,DataType<double>::type);

	vector<Point3d> objPoints;


	bool forward = true; // Need to iterate forward or backward
	cout<<"\nXYZ corners:\n";
	for(unsigned int i=0; i < g.rowSquares.size()-1; i++)
	{

		if(forward)
		{
			for(unsigned int j=0; j < g.rowSquares[i].size(); j++)
			{
	//ROS_INFO("Accessing %dth square of %dth row", j, i);
				objPoints.clear();
				pGridSquare gs = g.rowSquares[i][j];
				vector<Point2f> uvCorners;
				vector<Point3f> xyzCorners, sortedXYZCorners;
				getGridSquareUVCorners(gs, uvCorners);
				AllUVToXYZCoordinates(uvCorners, uvAxes, plane[3], xyzCorners);
				sortXYZCorners(xyzCorners, sortedXYZCorners);
				cout << sortedXYZCorners<<"\n";

				Point3d corner1 = Point3d(sortedXYZCorners[0].x, -sortedXYZCorners[0].z, sortedXYZCorners[0].y);
				Point3d corner2 = Point3d(sortedXYZCorners[1].x, -sortedXYZCorners[1].z, sortedXYZCorners[1].y);
				Point3d corner3 = Point3d(sortedXYZCorners[2].x, -sortedXYZCorners[2].z, sortedXYZCorners[2].y);
				Point3d corner4 = Point3d(sortedXYZCorners[3].x, -sortedXYZCorners[3].z, sortedXYZCorners[3].y);
				Point3d mid1 = (corner1 + corner2)*0.5;
				mid1.z = getY(mid1.x, -mid1.y, plane);
				Point3d mid2 = (corner2 + corner3)*0.5;
				mid2.z = getY(mid2.x, -mid2.y, plane);
				Point3d mid3 = (corner3 + corner4)*0.5;
				mid3.z = getY(mid3.x, -mid3.y, plane);
				Point3d mid4 = (corner4 + corner1)*0.5;
				mid4.z = getY(mid4.x, -mid4.y, plane);
				Point3d center = (mid1 + mid3)*0.5;
				center.z = getY(center.x, -center.y,plane);

				objPoints.push_back(corner1);
				objPoints.push_back(mid1);
				objPoints.push_back(corner2);
				objPoints.push_back(mid2);
				objPoints.push_back(corner3);
				objPoints.push_back(mid3);
				objPoints.push_back(corner4);
				objPoints.push_back(mid4);
				objPoints.push_back(center);

				//gs.printCoord();
				//cout << objPoints << endl;

				Mat objPoints_mat(9,1, CV_64FC3);
				for(int i=0; i<9; i++)
					objPoints_mat.at<Point3d>(i,0) = objPoints[i];

				//[MGP]Dont know but we have to call undistortPoints as a dummy call
				//Something to do with older version of opencv which gets linked by mrpt
				Mat dummy;
				undistortPoints(imgPoints_mat, dummy, cameraMatrix, distCoeffs);
				Mat rot_guess = Mat::eye(3,3, CV_64F);
				Rodrigues(rot_guess, rvec);
				tvec.at<double>(0)  = -(center.x-0.6*plane[0]);
				tvec.at<double>(1)  = -(center.y+0.6*plane[2]);
				tvec.at<double>(2)  = -(center.z - 0.6*plane[1]);

				solvePnP(objPoints_mat, imgPoints_mat, cameraMatrix, distCoeffs, rvec, tvec, true, CV_ITERATIVE);

				// cout<<"rvec : "<<rvec << endl;
				// cout<<"tvec : "<<tvec << endl;

				Mat rot(3,3, DataType<double>::type);
				Rodrigues(rvec, rot);

				Mat rotinv;
				transpose(rot, rotinv);

				tvec = -rotinv * tvec;

				//cout<<"rotated tvec : "<<tvec << endl;

				vector<double> pt;
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
		else
		{
			for(int j = g.rowSquares[i].size()-1; j>=0 ; j--)
			{
				//ROS_INFO("Accessing %dth square of %dth row", j, i);
				objPoints.clear();
				pGridSquare gs = g.rowSquares[i][j];
				vector<Point2f> uvCorners;
				vector<Point3f> xyzCorners, sortedXYZCorners;
				getGridSquareUVCorners(gs, uvCorners);
				AllUVToXYZCoordinates(uvCorners, uvAxes, plane[3], xyzCorners);
				sortXYZCorners(xyzCorners, sortedXYZCorners);
				cout << sortedXYZCorners<<"\n";

				Point3d corner1 = Point3d(sortedXYZCorners[0].x, -sortedXYZCorners[0].z, sortedXYZCorners[0].y);
				Point3d corner2 = Point3d(sortedXYZCorners[1].x, -sortedXYZCorners[1].z, sortedXYZCorners[1].y);
				Point3d corner3 = Point3d(sortedXYZCorners[2].x, -sortedXYZCorners[2].z, sortedXYZCorners[2].y);
				Point3d corner4 = Point3d(sortedXYZCorners[3].x, -sortedXYZCorners[3].z, sortedXYZCorners[3].y);

				Point3d mid1 = (corner1 + corner2)*0.5;
				mid1.z = getY(mid1.x, -mid1.y, plane);
				Point3d mid2 = (corner2 + corner3)*0.5;
				mid2.z = getY(mid2.x, -mid2.y, plane);
				Point3d mid3 = (corner3 + corner4)*0.5;
				mid3.z = getY(mid3.x, -mid3.y, plane);
				Point3d mid4 = (corner4 + corner1)*0.5;
				mid4.z = getY(mid4.x, -mid4.y, plane);
				Point3d center = (mid1 + mid3)*0.5;
				center.z = getY(center.x, -center.y,plane);

				objPoints.push_back(corner1);
				objPoints.push_back(mid1);
				objPoints.push_back(corner2);
				objPoints.push_back(mid2);
				objPoints.push_back(corner3);
				objPoints.push_back(mid3);
				objPoints.push_back(corner4);
				objPoints.push_back(mid4);
				objPoints.push_back(center);
				Mat objPoints_mat(9,1, CV_64FC3);
				for(int i=0; i<9; i++)
					objPoints_mat.at<Point3d>(i,0) = objPoints[i];
				//[MGP]Dont know but we have to call undistortPoints as a dummy call
				//Something to do with older version of opencv which gets linked by mrpt
				Mat dummy;
				undistortPoints(imgPoints_mat, dummy, cameraMatrix, distCoeffs);
				Mat rot_guess = Mat::eye(3,3, CV_64F);
				Rodrigues(rot_guess, rvec);
				/*
				tvec.at<double>(0)  = -(gs.u + (gs.width/2));
				tvec.at<double>(1)  = gs.v - (gs.height/2);
				tvec.at<double>(2)  = -(getY(gs.u + (gs.width/2), gs.v - (gs.height/2), plane) - 0.6);
				*/
				/*
				tvec.at<double>(0) = center.x;
				tvec.at<double>(1) = center.y;
				tvec.at<double>(2) = -(center.z - 0.6);
				*/
				tvec.at<double>(0)  = -(center.x-0.6*plane[0]);
				tvec.at<double>(1)  = -(center.y+0.6*plane[2]);
				tvec.at<double>(2)  = -(center.z - 0.6*plane[1]);


				solvePnP(objPoints_mat, imgPoints_mat, cameraMatrix, distCoeffs, rvec, tvec, true, CV_ITERATIVE);

				// cout<<"rvec : "<<rvec << endl;
				// cout<<"tvec : "<<tvec << endl;

				Mat rot(3,3, DataType<double>::type);
				Rodrigues(rvec, rot);

				Mat rotinv;
				transpose(rot, rotinv);

				tvec = -rotinv * tvec;

				//cout<<"rotated tvec : "<<tvec << endl;
				vector<double> pt;
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
	int numRows= g.rowSquares.size()-1;
	vector<int> numColsPerRow;
	for(int i=0; i<numRows; i++)
	{
		int n  = g.rowSquares[i].size();
		numColsPerRow.push_back(n);
	}
	sortTargetPoints(numRows, numColsPerRow, tPoints, sortedTPoints);
	printf("\ntarget points\n\n");
	print3dPoints(sortedTPoints);
	//printf("Points with X and Z just midpoints\n");
	//print3dPoints(tPoints_z);
}

void
ControlUINode::getGridSquareUVCorners(const pGridSquare &gs, vector<Point2f> &uvCorners)
{
	Point2f corner1 = Point2f(gs.u, gs.v);
	Point2f corner2 = Point2f(gs.u+gs.width, gs.v);
	Point2f corner3 = Point2f(gs.u+gs.width, gs.v - gs.height);
	Point2f corner4 = Point2f(gs.u, gs.v-gs.height);
	uvCorners.push_back(corner1);
	uvCorners.push_back(corner2);
	uvCorners.push_back(corner3);
	uvCorners.push_back(corner4);
}

void
ControlUINode::sortTargetPoints(int numRows, const vector<int> &numColsPerRow,
									const vector< vector<double> > &tPoints,
									vector< vector<double> > &sortedTPoints)
{
	vector<float> z_coord(numRows);
	vector<float> sortedZcoord;
	vector<int> rowStartIndex;
	int rowStart = 0;
	for(int i = 0; i < numRows; i++)
	{
		rowStartIndex.push_back(rowStart);
		int index = rowStart;
		z_coord[i] = tPoints[index][2];
		rowStart += numColsPerRow[i];
	}
	vector<int> indices;
	sortData(z_coord, sortedZcoord, indices, false);
	for(int i=0; i<numRows; i++)
	{
		int index= rowStartIndex[indices[i]] ;
		int numCols = numColsPerRow[indices[i]];
		vector< vector<double> > rowTPoints;
		for(int j=0; j < numCols; j++)
		{
			vector<double> tPoint = tPoints[index+j];
			rowTPoints.push_back(tPoint);
		}
		if( i%2 == (indices[i])%2) //if new index and old index have same parity then the order need not be changed
		{
			sortedTPoints.insert(sortedTPoints.end(), rowTPoints.begin(), rowTPoints.end());
		}
		else //otherwise need to reverse the order
		{
			for(int j = numCols-1; j >= 0; j--)
			{
				sortedTPoints.push_back(rowTPoints[j]);
			}
		}
	}
}

void
ControlUINode::getInitialPath(const vector<double> &prevPosition, const vector<double> &tPoint,
								double prevYaw, double desiredYaw, vector<vector<double> > &xyz_yaw)
{
	vector<double> interm_point(4);
	interm_point[0] = prevPosition[0];
	interm_point[1] = prevPosition[1];
	interm_point[2] = prevPosition[2];
	interm_point[3] = 0;
	for(int i = 0; i < 6; i++)
	{
		int m = i/2 +1;
		int n = 2 - i/2;
		if(i%2 == 0)
		{
			interm_point[0] = (m*tPoint[0] + n*prevPosition[0])/3;
		}
		else{
			interm_point[1] = (m*tPoint[1] + n*prevPosition[1])/3;
		}
		interm_point[3] = prevYaw*(5-i)/5 + desiredYaw*i/5;
		xyz_yaw.push_back(interm_point);
	}
	interm_point[2] = prevPosition[2]/2+ tPoint[2]/2;
	xyz_yaw.push_back(interm_point);
	interm_point[2] = tPoint[2];
	xyz_yaw.push_back(interm_point);
}

/***
	New additions to the code
***/

/**
 * @brief Fits 3d points for a single plane
 * @details Assumption: Only one plane is visible  at the current instant
 */
vector<float>
ControlUINode::fitPlane3dForTheCurrentPlane ()
{
	vector<vector<float> > _in_points;
	pthread_mutex_lock(&keyPoint_CS);
	for(unsigned int i = 0; i < _3d_points.size(); i++)
	{
		_in_points.push_back(_3d_points[i]);
	}
	pthread_mutex_unlock(&keyPoint_CS);
	_3d_plane = ransacPlaneFit(_in_points, ransacVerbose);
	_in_points.clear();
	return _3d_plane;
}

/**
 * @brief Land the quadcopter on calling this function
 * @details
 */
void
ControlUINode::sendLand()
{
	land_pub.publish(std_msgs::Empty());
}

/**
 * @brief Gets the current position of the drone
 * @details Returns the position of the drone when the function is called
 */
void
ControlUINode::getCurrentPositionOfDrone(vector<double> &curr_pos_of_drone)
{
	curr_pos_of_drone.clear();
	pthread_mutex_lock(&pose_CS);
		curr_pos_of_drone.push_back((double)x_drone);
		curr_pos_of_drone.push_back((double)y_drone);
		curr_pos_of_drone.push_back((double)z_drone);
		curr_pos_of_drone.push_back((double)yaw);
	pthread_mutex_unlock(&pose_CS);
}

/**
 * @brief Calculate the distance to see the plane completely from top to bottom
 * @details From tyhis position the leftmost and rightmost edges of the plane may not be visible
 */
float
ControlUINode::getDistanceToSeePlane(int height)
{
	// Read points
	vector<Point2f> imagePoints = GenerateMy2DPoints();
	float width = (16.0/9.0)*height;
	float drone_length = 0.6;
	vector<Point3f> objectPoints = GenerateMy3DPoints(width, height);
	Mat cameraMatrix(3, 3, DataType<double>::type);
	setIdentity(cameraMatrix);
	// From calibration done on our drone
	cameraMatrix.at<double>(0,0) = 565.710890694431;
	cameraMatrix.at<double>(0,1) = 0;
	cameraMatrix.at<double>(0,2) = 329.70046366652;
	cameraMatrix.at<double>(1,0) = 0;
	cameraMatrix.at<double>(1,1) = 565.110297594854;
	cameraMatrix.at<double>(1,2) = 169.873085097623;
	cameraMatrix.at<double>(2,0) = 0;
	cameraMatrix.at<double>(2,1) = 0;
	cameraMatrix.at<double>(2,2) = 1;
	Mat distCoeffs(5, 1, DataType<double>::type);
	// From calibration done on our drone
	distCoeffs.at<double>(0) = -0.516089772391501;
	distCoeffs.at<double>(1) = 0.285181914111246;
	distCoeffs.at<double>(2) = -0.000466469917823537;
	distCoeffs.at<double>(3) = 0.000864792975814983;
	distCoeffs.at<double>(4) = 0;
	Mat rvec(3, 1, DataType<double>::type);
	Mat tvec(3, 1, DataType<double>::type);
	Mat dummy;
	undistortPoints(imagePoints, dummy, cameraMatrix, distCoeffs);
	Mat rot_guess = Mat::eye(3, 3, CV_64F);
	Rodrigues(rot_guess, rvec);
	tvec.at<double>(0)  = 0.0;
	tvec.at<double>(1)  = 0.0;
	tvec.at<double>(2)  = -(height - drone_length);
	solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);
	Mat rot(3, 3, DataType<double>::type);
	Rodrigues(rvec, rot);
	Mat rotinv;
	transpose(rot, rotinv);
	tvec = -rotinv * tvec;
	cout << "Expected Quadcopter Location to see height: " << height << "and width: " << width 
		<< ": (" << tvec.at<double>(0) << ", " << tvec.at<double>(2) << ", " << -tvec.at<double>(1) << ")\n";
	double dist = tvec.at<double>(2);
	return dist;
}

/**
 * @brief Generates 2d points on the image on the left, middle and right edge of plane
 * @details
 */
vector<Point2f>
ControlUINode::GenerateMy2DPoints()
{
	vector<Point2f> points;
	float x, y;
	/* Point 1 */
	x = 0; y = 0;
	points.push_back(Point2f(x, y));
	/* Point 2 */
	x = 0; y = 180;
	points.push_back(Point2f(x, y));
	/* Point 3 */
	x = 0; y = 360;
	points.push_back(Point2f(x, y));
	/* Point 4 */
	x = 320; y = 0;
	points.push_back(Point2f(x, y));
	/* Point 5 */
	x = 320; y = 180;
	points.push_back(Point2f(x, y));
	/* Point 6 */
	x = 320; y = 360;
	points.push_back(Point2f(x, y));
	/* Point 1 */
	x = 640; y = 0;
	points.push_back(Point2f(x, y));
	/* Point 8 */
	x = 640; y = 180;
	points.push_back(Point2f(x, y));
	/* Point 9 */
	x = 640; y = 360;
	points.push_back(Point2f(x, y));
	return points;
}

/**
 * @brief Generates 3d points on the z = 0 plane for a specific width and height
 * @details
 */
vector<Point3f>
ControlUINode::GenerateMy3DPoints(float width, float height)
{
	vector<Point3f> points;
	float x, y, z;
	/*              4
	  1 +-----------+------------+ 7
		|           | 5          |
	  2 +-----------+------------+ 8
		|           |            |
	  3 +-----------+------------+ 9
					6 */
	/* Point 1 */
	x = -width/2; y = height/2; z = 0.0;
	points.push_back(Point3f(x,y,z));
	/* Point 2 */
	x = -width/2; y = 0.0; z = 0.0;
	points.push_back(Point3f(x,y,z));
	/* Point 3 */
	x = -width/2; y = -height/2; z = 0.0;
	points.push_back(Point3f(x,y,z));
	/* Point 4 */
	x = 0.0; y = height/2; z = 0.0;
	points.push_back(Point3f(x,y,z));
	/* Point 5: Origin at the center of the plane */
	x = 0.0; y = 0.0; z = 0.0;
	points.push_back(Point3f(x,y,z));
	/* Point 6 */
	x = 0.0; y = -height/2; z = 0.0;
	points.push_back(Point3f(x,y,z));
	/* Point 7 */
	x = width/2; y = height/2; z = 0.0;
	points.push_back(Point3f(x,y,z));
	/* Point 8 */
	x = width/2; y = 0.0; z = 0.0;
	points.push_back(Point3f(x,y,z));
	/* Point 9 */
	x = width/2; y = -height/2; z = 0.0;
	points.push_back(Point3f(x,y,z));
	return points;
}

/**
 * @brief Get all the 3d planes present within the boundary of the clicked points on he image
 * @details
 */
void
ControlUINode::getMultiplePlanes3d (const vector<int> &ccPoints, const vector< vector<int> > &pointsClicked,
									vector< vector<float> > &planeParameters,
									vector< vector<Point3f> > &continuousBoundingBoxPoints,
									vector< vector<Point3f> > &sorted_3d_points,
									vector<float> &percentageOfEachPlane)
{
	cout << "[ DEBUG] [getMultiplePlanes3d] Started\n";
	vector<Point3f> _in_points;
	vector< vector<int> > points;
	_in_points.clear();
	for(unsigned int i = 0; i < ccPoints.size(); i++)
	{
		points.push_back(pointsClicked[ccPoints[i]]);
	}
	pthread_mutex_lock(&keyPoint_CS);
	for(unsigned int i = 0; i < _2d_points.size(); i++)
	{
		if(liesInside(points, _2d_points[i]))
		{
			Point3f featurePt;
			featurePt.x = _3d_points[i][0];
			featurePt.y = _3d_points[i][1];
			featurePt.z = _3d_points[i][2];
			_in_points.push_back(featurePt);
		}
	}
	pthread_mutex_unlock(&keyPoint_CS);
	cout << "[ DEBUG] [getMultiplePlanes3d] Captured the 3d points within the clicked points\n";
	// See multiplePlanes.cpp
	findPercBoundEachPlane(_in_points, planeParameters, continuousBoundingBoxPoints, sorted_3d_points, percentageOfEachPlane);
	cout << "[ DEBUG] [getMultiplePlanes3d] Completed\n";
	return ;
}

/**
 * @brief Get all the 3d planes from the visible feature points in the scene
 * @details
 */
void
ControlUINode::getMultiplePlanes3d (vector< vector<float> > &planeParameters,
									vector< vector<Point3f> > &continuousBoundingBoxPoints,
									vector< vector<Point3f> > &sorted_3d_points,
									vector<float> &percentageOfEachPlane)
{
	cout << "[ DEBUG] [getMultiplePlanes3d] Started\n";
	vector< Point3f > _in_points;
	_in_points.clear();
	pthread_mutex_lock(&keyPoint_CS);
	for(unsigned int i = 0; i < _3d_points.size(); i++)
	{
		Point3f featurePt;
		featurePt.x = _3d_points[i][0];
		featurePt.y = _3d_points[i][1];
		featurePt.z = _3d_points[i][2];
		_in_points.push_back(featurePt);
	}
	pthread_mutex_unlock(&keyPoint_CS);
	cout << "[ DEBUG] [getMultiplePlanes3d] Captured all the 3d points\n";
	// See multiplePlanes.cpp
	findPercBoundEachPlane(_in_points, planeParameters, continuousBoundingBoxPoints,
							sorted_3d_points, percentageOfEachPlane);
	cout << "[ DEBUG] [getMultiplePlanes3d] Completed\n";
	return ;
}

float
ControlUINode::getPointPlaneDistance(const vector<double> &current_pos_of_drone, const vector<float> &planeParameters)
{
	float ans = 0.0;
	float a = planeParameters[0];
	float b = planeParameters[1];
	float c = planeParameters[2];
	float d = planeParameters[3];
	float mag = sqrt(a*a+b*b+c*c);
	float x0 = current_pos_of_drone[0];
	float y0 = current_pos_of_drone[1];
	float z0 = current_pos_of_drone[2];
	float dist = abs(a*x0 + b*y0 + c*z0 + d)/mag;
	return dist;
}

/**
 * @brief Get index of the plane for which the quadcopter has to capture information at that instant
 * @details Uses the information from previously visited plane normlas
 * @todo Please test this extensively
 */
int
ControlUINode::getCurrentPlaneIndex(const vector< vector<float> > &plane_parameters,
									const vector< vector<float> > &temp_plane_parameters,
									const vector<float> &percentagePlane)
{
	int planeIndex = -1;
	float plane_heuristic = 0.984; // +-10 degrees variation
	if(plane_parameters.size() == 0)
	{
		vector<float> yaw_axis;
		yaw_axis.push_back(0.0);
		yaw_axis.push_back(1.0); //@todo-me Check if it is +1.0 or -1.0
		yaw_axis.push_back(0.0);
		for (int i = 0; i < temp_plane_parameters.size(); ++i)
		{
			float a = temp_plane_parameters[i][0];
			float b = temp_plane_parameters[i][1];
			float c = temp_plane_parameters[i][2];
			float dot_p = yaw_axis[0]*a + yaw_axis[1]*b + yaw_axis[2]*c;
			if(dot_p >= plane_heuristic)
			{
				planeIndex = i; break;
			}
		}
	}
	else
	{
		for (int i = 0; i < temp_plane_parameters.size(); ++i)
		{
			float dot_p;
			float in_a, in_b, in_c;
			float out_a, out_b, out_c;
			out_a = temp_plane_parameters[i][0];
			out_b = temp_plane_parameters[i][1];
			out_c = temp_plane_parameters[i][2];
			bool found = false;
			for (int j = 0; j < plane_parameters.size(); ++j)
			{
				in_a = plane_parameters[i][0];
				in_b = plane_parameters[i][1];
				in_c = plane_parameters[i][2];
				dot_p = in_a*out_a + in_b*out_b + in_c*out_c;
				if(dot_p >= 0.9)
				{
					found = true; break;
				}
			}
			if(!found)
			{
				planeIndex = i;
				break;
			}
		}
		/*vector<double> current_pos_of_drone;
		vector<double> dest_pos_of_drone;
		dest_pos_of_drone.push_back(0.0);
		dest_pos_of_drone.push_back(1.0);
		dest_pos_of_drone.push_back(0.0);
		vector<double> ac_dest_pos_of_drone;
		getCurrentPositionOfDrone(current_pos_of_drone);
		convertWRTQuadcopterOrigin(current_pos_of_drone, dest_pos_of_drone, ac_dest_pos_of_drone);
		bool found = false;
		float out_a, out_b, out_c;
		out_a = ac_dest_pos_of_drone[0]-current_pos_of_drone[0];
		out_b = ac_dest_pos_of_drone[1]-current_pos_of_drone[1];
		out_c = ac_dest_pos_of_drone[2]-current_pos_of_drone[2];
		for (int j = 0; j < plane_parameters.size(); ++j)
		{
			in_a = plane_parameters[i][0];
			in_b = plane_parameters[i][1];
			in_c = plane_parameters[i][2];
			dot_p = in_a*out_a + in_b*out_b + in_c*out_c;
			if(dot_p >= plane_heuristic)
			{
				found = true; break;
			}
		}
		if(!found)
		{
			planeIndex = i;
			break;
		}*/
	}
	cout << "[ DEBUG] [getCurrentPlaneIndex] Current Plane Index: " << planeIndex << "\n";
	return planeIndex;
}

/**
 * @brief Move the drone to the destination point
 * @details The destination point is represented as (x, y, z, yaw)
 * @param [vector<double>] dest_point - Final Destination of quadcopter
 * 									Includes yaw in the vector
 * @return
 */
void
ControlUINode::moveDroneToPosition(const vector<double> &dest_point)
{
	justNavigation = true;
	char buf[100];
	just_navigation_commands.clear();
	targetPoints.clear();
	snprintf(buf, 100, "c goto %lf %lf %lf %lf",
		dest_point[0], dest_point[1], dest_point[2], dest_point[3]);
	std_msgs::String s;
	s.data = buf;
	ROS_INFO("Message: ");
	ROS_INFO(buf);
	just_navigation_commands.push_back(s);
	targetPoints.push_back(dest_point);
	pthread_mutex_lock(&command_CS);
	if(just_navigation_commands.size() > 0)
	{
		while(!just_navigation_commands.empty())
		{
			pthread_mutex_lock(&tum_ardrone_CS);
			tum_ardrone_pub.publish(just_navigation_commands.front());
			pthread_mutex_unlock(&tum_ardrone_CS);
			targetPoint = targetPoints.front();
			printf("Moving to Current target: %lf %lf %lf\n", targetPoint[0], targetPoint[1] , targetPoint[2] );
			just_navigation_commands.pop_front();
			targetPoints.pop_front();
			ros::Duration(2).sleep();
		}
		justNavigation = false;
	}
	pthread_mutex_unlock(&command_CS);
	vector<double> current_pos_of_drone;
	getCurrentPositionOfDrone(current_pos_of_drone);
	cout << "[ DEBUG] Drone at: (" << current_pos_of_drone[0] << ", " 
		<< current_pos_of_drone[1] << ", " << current_pos_of_drone[2] << ", "
		<< current_pos_of_drone[3] << ")\n";
}

/**
 * @brief Move the drone to the destination point via a set of points
 * @details Each point is represented as (x, y, z, yaw)
 * @param [vector< vector<double> >] dest_points - Points to where quadcopter has to travel
 * 									Includes yaw in the vector
 * @return
 */
void
ControlUINode::moveDroneViaSetOfPoints(const vector< vector<double> > &dest_points)
{
	cout << "[ DEBUG] [moveDroneViaSetOfPoints] Started\n";
	justNavigation = true;
	char buf[100];
	commands.clear();
	just_navigation_commands.clear();
	targetPoints.clear();
	double a, b, c, d;
	for (int i = 0; i < dest_points.size(); ++i)
	{
		if(i == 0)
		{
			snprintf(buf, 100, "c goto %lf %lf %lf %lf",
				dest_points[i][0], dest_points[i][1], dest_points[i][2], dest_points[i][3]);
			std_msgs::String s;
			s.data = buf;
			ROS_INFO("Message: ");
			ROS_INFO(buf);
			just_navigation_commands.push_back(s);
			targetPoints.push_back(dest_points[i]);
			a = dest_points[i][0]; b = dest_points[i][1];
			c = dest_points[i][2]; d = dest_points[i][3];
		}
		else
		{
			if(!( (a-dest_points[i][0]<=std::numeric_limits<double>::epsilon()) &&
				(b-dest_points[i][1]<=std::numeric_limits<double>::epsilon()) &&
				(c-dest_points[i][2]<=std::numeric_limits<double>::epsilon()) &&
				(d-dest_points[i][3]<=std::numeric_limits<double>::epsilon()) ))
			{
				snprintf(buf, 100, "c goto %lf %lf %lf %lf",
				dest_points[i][0], dest_points[i][1], dest_points[i][2], dest_points[i][3]);
				std_msgs::String s;
				s.data = buf;
				ROS_INFO("Message: ");
				ROS_INFO(buf);
				just_navigation_commands.push_back(s);
				targetPoints.push_back(dest_points[i]);
				a = dest_points[i][0]; b = dest_points[i][1];
				c = dest_points[i][2]; d = dest_points[i][3];
			}
		}
	}
	cout << "[ DEBUG] [moveDroneViaSetOfPoints] Commands Ready\n";
	cout << "[ DEBUG] [moveDroneViaSetOfPoints] Commands to execute: " << just_navigation_commands.size() << "\n";
	pthread_mutex_lock(&command_CS);
	if(just_navigation_commands.size() > 0)
	{
		while(!just_navigation_commands.empty())
		{
			pthread_mutex_lock(&tum_ardrone_CS);
			tum_ardrone_pub.publish(just_navigation_commands.front());
			pthread_mutex_unlock(&tum_ardrone_CS);
			targetPoint = targetPoints.front();
			printf("Moving to Current target: %lf %lf %lf %lf\n", targetPoint[0], targetPoint[1] , targetPoint[2], targetPoint[3] );
			just_navigation_commands.pop_front();
			targetPoints.pop_front();
			ros::Duration(2).sleep();
		}
		justNavigation = false;
	}
	pthread_mutex_unlock(&command_CS);
	vector<double> current_pos_of_drone;
	getCurrentPositionOfDrone(current_pos_of_drone);
	cout << "[ DEBUG] Drone at: (" << current_pos_of_drone[0] << ", " 
		<< current_pos_of_drone[1] << ", " << current_pos_of_drone[2] << ", "
		<< current_pos_of_drone[3] << ")\n";
	cout << "[ DEBUG] Done\n";
}

/**
 * @brief Generates the set of points (smoothly distributed) which drone has to follow to move from start to end
 * @details The initial and end points are represented as (x, y, z, yaw)
 * @param [in] [vector< double >] start - Starting position of quadcopter (With yaw)
 * @param [in] [vector< double >] end - Ending position of quadcopter (With yaw)
 * @param [out] [vector< vector<double> >] path - Intermediate points from start to end (With yaw)
 * @return
 */
void
ControlUINode::designPathForDrone(const vector< double > &start,
					const vector< double > &end,
					vector< vector<double> > &path)
{
	clearDoubleVector(path);
	// See getInitialPath for help
	vector<double> start_3d_pos, end_3d_pos;
	for (int i = 0; i < 3; ++i)
	{
		start_3d_pos.push_back(start[i]);
		end_3d_pos.push_back(end[i]);
	}
	getInitialPath(start_3d_pos, end_3d_pos, start[3], end[3], path);
}

/**
 * @brief Generates the set of points (smoothly distributed) which drone has to follow to change its yaw by large angles
 * @details The current point is represented as (x, y, z, yaw)
 * @param [vector< vector<double> >] dest_points - Points to where quadcopter has to travel
 * 									Includes yaw in the vector
 * @return
 */
void
ControlUINode::designPathToChangeYaw(const vector<double> &curr_point,
				double dest_yaw,
				vector< vector<double> > &xyz_yaw)
{
	clearDoubleVector(xyz_yaw);
	double yaw_diff = dest_yaw - curr_point[3];
	vector<double> interm_point;
	double prevYaw = curr_point[3], desiredYaw = dest_yaw;
	cout << "[ DEBUG] [designPathToChangeYaw] Started\n";
	for(int i = 0; i < 6; i++)
	{
		interm_point.clear();
		interm_point.push_back(curr_point[0]);
		interm_point.push_back(curr_point[1]);
		interm_point.push_back(curr_point[2]);
		interm_point.push_back(prevYaw*(5-i)/5 + desiredYaw*i/5);
		xyz_yaw.push_back(interm_point);
	}
	cout << "[ DEBUG] [designPathToChangeYaw] Completed\n";
}

/**
 * @brief Clears a double vector
 * @details
 */
void
ControlUINode::clearDoubleVector(vector< vector<double> > &xyz_yaw)
{
	int size = xyz_yaw.size();
	for (int i = 0; i < size; ++i)
	{
		xyz_yaw[i].clear();
	}
	xyz_yaw.clear();
}

/**
 * @brief Converts a point given from quadcopter's current position as origin to quadcopter's actual origin
 * @details This is required for generating points for quadcopter's autonomous movement
 */
void
ControlUINode::convertWRTQuadcopterOrigin(const vector<double> &current_pos_of_drone,
											const vector<double> &dest_pos_of_drone,
											vector<double> &ac_dest_pos_of_drone)
{
	ac_dest_pos_of_drone.clear();
	Mat rotationMatrix = Mat::eye(3, 3, CV_64F);
	double angle = current_pos_of_drone[3];
	angle = (angle*3.14)/180.0;
	rotationMatrix.at<double>(0, 0) = cos(angle);
	rotationMatrix.at<double>(0, 1) = -sin(angle);
	rotationMatrix.at<double>(1, 0) = sin(angle);
	rotationMatrix.at<double>(1, 1) = cos(angle);
	cout << "[ DEBUG] Rotation Matrix: " << rotationMatrix << "\n";
	Mat translationVector(3, 1, DataType<double>::type);
	translationVector.at<double>(0, 0) = current_pos_of_drone[0];
	translationVector.at<double>(1, 0) = current_pos_of_drone[1];
	translationVector.at<double>(2, 0) = current_pos_of_drone[2];
	cout << "[ DEBUG] Translation Vector: " << translationVector << "\n";
	Mat dest_point_drone_origin_mat(3, 1, DataType<double>::type);
	dest_point_drone_origin_mat.at<double>(0, 0) = dest_pos_of_drone[0];
	dest_point_drone_origin_mat.at<double>(1, 0) = dest_pos_of_drone[1];
	dest_point_drone_origin_mat.at<double>(2, 0) = dest_pos_of_drone[2];
	Mat sub = rotationMatrix*translationVector;
	Mat b = dest_point_drone_origin_mat + sub;
	// How do I solve Ax = b?
	// Will this always be solvable?
	Mat x = rotationMatrix.inv() * b;
	ac_dest_pos_of_drone.push_back(x.at<double>(0, 0));
	ac_dest_pos_of_drone.push_back(x.at<double>(1, 0));
	ac_dest_pos_of_drone.push_back(x.at<double>(2, 0));
	ac_dest_pos_of_drone.push_back(angle+dest_pos_of_drone[3]);
}

/**
 * @brief Get the best fit plane for a set of 3d points
 * @details Uses least squares and svd method
 * @todo-me Decide on which method to use finally after testing
 */
vector<float>
ControlUINode::bestFitPlane(const vector<Point3f> &threed_points)
{
	// http://stackoverflow.com/questions/1400213/3d-least-squares-plane
	float x_c = 0.0, y_c = 0.0, z_c = 0.0;
	float a, b, c, d;
	Mat X(3, threed_points.size(), DataType<float>::type);
	for (int i = 0; i < threed_points.size(); ++i)
	{
		x_c += threed_points[i].x;
		y_c += threed_points[i].y;
		z_c += threed_points[i].z;
		X.at<float>(0, i) = threed_points[i].x;
		X.at<float>(1, i) = threed_points[i].y;
		X.at<float>(2, i) = threed_points[i].z;
	}
	x_c /= threed_points.size();
	y_c /= threed_points.size();
	z_c /= threed_points.size();
	// Centering the points
	for (int i = 0; i < threed_points.size(); ++i)
	{
		X.at<float>(0, i) = X.at<float>(0, i) - x_c;
		X.at<float>(1, i) = X.at<float>(1, i) - y_c;
		X.at<float>(2, i) = X.at<float>(2, i) - z_c;
	}
	// http://www.ilikebigbits.com/blog/2015/3/2/plane-from-points
	// Method 1: Least squares
	/*float val11 = 0.0, val12 = 0.0;
	float val21 = 0.0, val22 = 0.0;
	float val1 = 0.0, val2 = 0.0;
	for (int i = 0; i < threed_points.size(); ++i)
	{
		val11 += (X.at<float>(0,i) * X.at<float>(0,i));
		val12 += (X.at<float>(0,i) * X.at<float>(1,i));
		val21 += (X.at<float>(1,i) * X.at<float>(0,i));
		val22 += (X.at<float>(1,i) * X.at<float>(1,i));
		val1 += (X.at<float>(0,i) * X.at<float>(2,i));
		val2 += (X.at<float>(1,i) * X.at<float>(2,i));
	}
	float D = (val11*val22) - (val12*val21);
	float a = ((val2*val12) - (val1*val22))/D;
	float b = ((val12*val1) - (val11*val2))/D;
	float c = 1.0;
	float mag = sqrt(a*a + b*b + c*c);
	cout << "Method 1: (" << a/mag << ", " << b/mag << ", " << c/mag << ")\n";*/
	// Method 2: SVD Method
	// http://math.stackexchange.com/questions/99299/best-fitting-plane-given-a-set-of-points
	// http://www.ltu.se/cms_fs/1.51590!/svd-fitting.pdf
	Mat w(3, threed_points.size(), DataType<float>::type);
	Mat vt(threed_points.size(), threed_points.size(), DataType<float>::type);
	Mat u(3, 3, DataType<float>::type);
	SVD::compute(X, w, u, vt);
	cout << "U:\n" << u << "\n";
	cout << "S:\n" << w << "\n";
	cout << "Method 2: (" << u.at<float>(0, 2) << ", " << u.at<float>(1, 2) << ", " << u.at<float>(2, 2) << ")\n";
	vector<float> normal;
	a = u.at<float>(0, 2);
	b = u.at<float>(1, 2);
	c = u.at<float>(2, 2);
	normal.push_back(a);
	normal.push_back(b);
	normal.push_back(c);
	normal.push_back(0.0);
	return normal;
}

/**
 * @brief Get the bounding box points of each plane by autonomously navigating the quadcopter
 * @details Write the information to the file "Plane_Info.txt"
 * @param [in] [vector<double>] angles - Angles by which drone has to rotate to see the new plane
 * @param [in] [vector<RotateDirection>] directions - Directiosn in which drone has to rotate to see the new plane
 * @param [in] [float] min_distance - Minimum distance drone has to be from the plane to see the min. height
 * @param [in] [float] max_distance - Minimum distance drone has to be from the plane to see the allowed max. height
 */
void
ControlUINode::getMeTheMap(const vector< double > &angles,
							const vector< RotateDirection > &directions,
							float min_distance,
							float max_distance)
{
	vector<float> current_pos_of_drone, start_pos, end_pos;
	int plane_num = 01;
	// Corners of all multiple planes bounding boxes in 3D
	vector< vector<Point3f> > all_planes_bounding_box_points;
	// Plane parameters of all multiple planes being covered
	vector< vector<float> > all_planes_plane_parameters;
	// Corners of current plane bounding box in 3D
	vector< Point3f > current_bounding_box_points;
	// Plane parameters of current plane being covered
	vector< float > current_plane_parameters;
	if(directions.size()==0)
	{
		// CLOCKWISE - to prevent is crashing it with nearby adjacent plane (if it exists)
		cout << "[ INFO] There is only one plane to cover\n";
		cout << "[ INFO] Covering the current plane no." << plane_num << "\n";
		CoverTheCurrentPlane(plane_num, min_distance, max_distance, CLOCKWISE, 0.0,
							all_planes_bounding_box_points, all_planes_plane_parameters,
							current_bounding_box_points, current_plane_parameters);
		all_planes_bounding_box_points.push_back(current_bounding_box_points);
		all_planes_plane_parameters.push_back(current_plane_parameters);
		cout << "[ INFO] Added plane parameters and bounding box points of plane no. " << plane_num << " to the vector.\n";
	}
	else
	{
		cout << "[ INFO] There are " << directions.size()+1 << " planes to cover\n";
		cout << "[ INFO] Covering the current plane no." << plane_num << "\n";
		CoverTheCurrentPlane(plane_num, min_distance, max_distance, directions[0], angles[0],
							all_planes_bounding_box_points, all_planes_plane_parameters,
							current_bounding_box_points, current_plane_parameters);
		all_planes_bounding_box_points.push_back(current_bounding_box_points);
		all_planes_plane_parameters.push_back(current_plane_parameters);
		cout << "[ INFO] Added plane parameters and bounding box points of plane no. " << plane_num << " to the vector.\n";
		/* directions*/
		int number_of_rotations = directions.size();
		for(int i = 0; i < number_of_rotations; i++)
		{
			double angle = fabs(0.75*angles[i]);
			plane_num++;
			cout << "[ INFO] Adjusting the quadcopter to see plane no." << plane_num << "\n";
			MoveQuadcopterToNextPlane(directions[i], angle, all_planes_plane_parameters);
			if(i == number_of_rotations-1)
			{
				cout << "[ INFO] Covering the current and the last plane no." << plane_num << "\n";
				CoverTheCurrentPlane(plane_num, min_distance, max_distance, CLOCKWISE, 0.0,
								all_planes_bounding_box_points, all_planes_plane_parameters,
								current_bounding_box_points, current_plane_parameters);
			}
			else
			{
				cout << "[ INFO] Covering the current plane no." << plane_num << "\n";
				CoverTheCurrentPlane(plane_num, min_distance, max_distance, directions[i+1], angles[i+1],
									all_planes_bounding_box_points, all_planes_plane_parameters,
									current_bounding_box_points, current_plane_parameters);
			}
			all_planes_bounding_box_points.push_back(current_bounding_box_points);
			all_planes_plane_parameters.push_back(current_plane_parameters);
			cout << "[ INFO] Added plane parameters and bounding box points of plane no. " << plane_num << " to the vector.\n";
		}
	}
	cout << "[ INFO] Land command has been issued to the quadcopter\n";
	// Mission accomplished. Land the quadcopter :)
	sendLand();
	/* Write the bounding box points to file Plane_Info.txt */
	/* Write the plane parameters to the same file */
	string file_name = "Plane_Info.txt";
	cout << "[ INFO] Writing the information to " << file_name << ".\n";
	for (int i = 0; i < all_planes_plane_parameters.size(); ++i)
	{
		image_gui->WriteInfoToFile(all_planes_bounding_box_points[i], all_planes_plane_parameters[i], i, file_name);
	}
	cout << "[ INFO] Gathering points completed.\n";
	return ;
}

void
ControlUINode::projectPointsOnPlane (const vector<Point3f> &points, const vector<float> &planeParameters,
								vector<Point3f> &projectedPoints)
{
	projectedPoints.clear();
	vector<float> point, v;
	Point3f pp;
	for(unsigned int i = 0; i < points.size(); i++)
	{
		point.clear();
		point.push_back(points[i].x);
		point.push_back(points[i].y);
		point.push_back(points[i].z);
		v = projectPoint(planeParameters, point);
		pp.x = v[0]; pp.y = v[1]; pp.z = v[2];
		projectedPoints.push_back(pp);
		v.clear();
	}
	return ;
}

void
ControlUINode::alignQuadcopterToCurrentPlane(const vector<double> &current_pos_of_drone,
								const vector<float> &planeParameters)
{
	cout << "[ DEBUG] [alignQuadcopterToCurrentPlane] Started\n";
	vector<double> dest_pos_of_drone, ac_dest_pos_of_drone;
	dest_pos_of_drone.push_back(0.0);
	dest_pos_of_drone.push_back(1.0);
	dest_pos_of_drone.push_back(0.0);
	dest_pos_of_drone.push_back(0.0);
	convertWRTQuadcopterOrigin(current_pos_of_drone, dest_pos_of_drone, ac_dest_pos_of_drone);
	Point3f pYAxis(ac_dest_pos_of_drone[0],ac_dest_pos_of_drone[1],ac_dest_pos_of_drone[2]);
	//Point3f pYAxis(0.0,-1.0,0.0);
	Point3f pOrigin(current_pos_of_drone[0],current_pos_of_drone[1],current_pos_of_drone[2]);
	Point3f projectedNormal(pYAxis-pOrigin);
	//Point3f projectedNormal(pYAxis);
	Point3f plane_params(planeParameters[0],planeParameters[1],0.0);
	cout << "[ DEBUG] [alignQuadcopterToCurrentPlane] pYAxis: " << pYAxis << "\n";
	cout << "[ DEBUG] [alignQuadcopterToCurrentPlane] pOrigin: " << pOrigin << "\n";
	cout << "[ DEBUG] [alignQuadcopterToCurrentPlane] projectedNormal: " << projectedNormal << "\n";
	cout << "[ DEBUG] [alignQuadcopterToCurrentPlane] PP: " << plane_params << "\n";
	float angle = findAngle(projectedNormal, plane_params);
	cout << "[ DEBUG] Angle (radians): " << angle << "\n";
	angle = -angle*180/M_PI;
	cout << "[ DEBUG] Angle to rotate: " << angle << "\n";
	vector< vector<double> > path;
	designPathToChangeYaw(current_pos_of_drone, current_pos_of_drone[3]+angle, path);
	moveDroneViaSetOfPoints(path);
	cout << "[ DEBUG] [alignQuadcopterToCurrentPlane] Completed\n";
	return ;
}


void
ControlUINode::moveForward(float min_distance, float max_distance)
{
	float step_distance = (min_distance+max_distance)/5.0;
	vector<double> current_pos_of_drone;
	getCurrentPositionOfDrone(current_pos_of_drone);
	vector<double> dest_pos_of_drone, ac_dest_pos_of_drone;
	dest_pos_of_drone.push_back(0.0);
	dest_pos_of_drone.push_back(step_distance);
	dest_pos_of_drone.push_back(0.0);
	dest_pos_of_drone.push_back(current_pos_of_drone[3]);
	convertWRTQuadcopterOrigin(current_pos_of_drone, dest_pos_of_drone, ac_dest_pos_of_drone);
	vector< vector<double> > path;
	designPathForDrone(current_pos_of_drone, ac_dest_pos_of_drone, path);
	moveDroneViaSetOfPoints(path);
	return ;
}

void
ControlUINode::moveBackward(float min_distance, float max_distance)
{
	float step_distance = (min_distance+max_distance)/2.0;
	vector<double> current_pos_of_drone;
	getCurrentPositionOfDrone(current_pos_of_drone);
	vector<double> dest_pos_of_drone, ac_dest_pos_of_drone;
	dest_pos_of_drone.push_back(0.0);
	dest_pos_of_drone.push_back(-step_distance);
	dest_pos_of_drone.push_back(0.0);
	dest_pos_of_drone.push_back(current_pos_of_drone[3]);
	convertWRTQuadcopterOrigin(current_pos_of_drone, dest_pos_of_drone, ac_dest_pos_of_drone);
	vector< vector<double> > path;
	designPathForDrone(current_pos_of_drone, ac_dest_pos_of_drone, path);
	moveDroneViaSetOfPoints(path);
	return ;
}

bool
ControlUINode::isNewPlaneVisible(const vector< vector<float> > &plane_parameters,
					const vector< vector<float> > &planeParameters,
					const vector<float> &percentagePlane)
{
	bool flag = true;
	int plane_index = getCurrentPlaneIndex(plane_parameters, planeParameters, percentagePlane);
	if(plane_index < 0)
	{
		flag = false;
	}
	else
	{
		flag = true;
	}
	return flag;
}

/**
 * @brief Cover the current plane completely from its leftmost edge to the rightmost edge
 * @details At the end of CoverTheCurrentPlane(), the quadcopter is expected at
 * 			the center of the last visited segment of the plane (both wrt width and height) it is currently seeing
 * @param [int] plane_num - Which plane you're covering
 * @param [float] max_height - Maximum height of the plane
 * @return
 */
void
ControlUINode::CoverTheCurrentPlane (int plane_num, float min_distance,
										float max_distance, RotateDirection dir, double next_plane_angle,
										const vector< vector<Point3f> > &all_planes_bounding_box_points,
										const vector< vector<float> > &all_planes_plane_parameters,
										vector< Point3f > &current_bounding_box_points,
										vector< float > &current_plane_parameters)
{
	current_bounding_box_points.clear();
	current_plane_parameters.clear();
	/* Push that position to vector */
	vector<double> current_pos_of_drone;
	vector<double> dest_pos_of_drone;
	vector<double> ac_dest_pos_of_drone;
	// To check if the plane is covered completely
	bool planeCovered = false;
	/* As obtained from ControlUINode.cpp Line 429 */
	bool stage = true; // Variable to indicate whether it's the first time seeing the plane
	// Corners of all multiple planes bounding boxes in 3D
	vector< vector<Point3f> > temp_bounding_box_points;
	// Plane parameters of all multiple planes being covered
	vector< vector<float> > temp_plane_parameters;
	vector< vector<double> > path;
	vector< vector<float> > planeParameters;
	vector< vector<Point3f> > continuousBoundingBoxPoints;
	// Plane parameters required to see if the plane is complete.
	// Includes the parameters for the current plane
	vector< vector<float> > test_plane_parameters;
	// Vectors for storing the 3d points to do plane fitting if there are multiple visits to the plane
	vector<Point3f> plane_3d_points;
	// Vectors for storing the bounding box points to project onto plane after plane fitting
	vector<Point3f> plane_bounding_box_points;
	vector< vector<Point3f> > sorted_3d_points;
	// Index of plane which is currently under scrutiny
	int significantPlaneIndex;
	// Variable to indicate if the plane has been covered in multiple attempts
	bool isBigPlane = false;
	plane_3d_points.clear();
	plane_bounding_box_points.clear();
	// 2d image points clicked on the DRONE CAMERA FEED Screen
	vector< vector<int> > points_clicked;
	// the 3d keypoints of control node for nearest keypoints
	vector< vector<float> > key_points_nearest;
	// corners of the convex hull
	vector<int> cc_points;
	// Indicating whether the current plane is completed!!!
	bool flag;
	int size;
	// Temporarily storing bouding box points of single plane multiple scans
	// for later reprojecting onto the estimated plane
	vector<float> percentageOfEachPlane;
	vector<Point3f> new_bounding_box_points;
	double width_of_3d_plane;
	while (!planeCovered)
	{
		cout << "[ DEBUG] [CoverTheCurrentPlane] Covering plane no.: " << plane_num << "\n";
		// First param: Number of points clicked on the screen
		// Second param: Number of Key points detected
		image_gui->setNumberOfPoints(0, 0);
		// RendeRect: false, RenderPoly: false, RenderSignificantPlane: false
		image_gui->setRender(false, false, false);
		// Clear all Vectors
		image_gui->clearInputVectors();
		/* Adjust the quadcopter to see the top and bottom edge of current plane (plane in view) */
		/* Returns TRUE if the plane in view is covered completely, else FALSE*/
		cout << "[ INFO] [CoverTheCurrentPlane] Performing the adjustments to see the top and bottom edge of the current plane\n";
		flag = AdjustToSeeCurrentPlane(min_distance, max_distance, all_planes_plane_parameters, stage );
		/* Mark the bounding points for the plane to be scanned */
		/* Output: vector<Point2f> pointsClicked */
		/* Loop until 4 points are clicked on the image */
		// Stop the system until all 4 points are clicked
		cout << "[ ORDER] [CoverTheCurrentPlane] Start Clicking the bounding box points\n";
		image_gui->getPointsClicked(points_clicked);
		while(points_clicked.size()!=4) {image_gui->getPointsClicked(points_clicked);}
		cout << "[ INFO] [CoverTheCurrentPlane] Extracting Bounding Poly\n";
		image_gui->extractBoundingPoly();
		int significantPlaneIndex = 0;
		image_gui->getCCPoints(cc_points);
		image_gui->getPointsClicked(points_clicked);
		size = temp_bounding_box_points.size();
		for (int i = 0; i < size; ++i)
		{
			temp_bounding_box_points[i].clear();
			temp_plane_parameters[i].clear();
		}
		temp_bounding_box_points.clear();
		size = temp_plane_parameters.size();
		for (int i = 0; i < size; ++i)
		{
			temp_plane_parameters[i].clear();
		}
		temp_plane_parameters.clear();
		cout << "[ INFO] Get multiple planes from the clicked points using JLinkage\n";
		// Calls JLinkage and finds all planes within the clicked region
		getMultiplePlanes3d(cc_points, points_clicked, temp_plane_parameters, temp_bounding_box_points,
							sorted_3d_points, percentageOfEachPlane);
		significantPlaneIndex = getCurrentPlaneIndex(all_planes_plane_parameters, temp_plane_parameters, percentageOfEachPlane);
		// @todo-me See 288-328 for implementation
		// Render significant plane
		image_gui->setRender(false, true, true);
		cout << "[ INFO] Rendering the frames in the DRONE CAMERA FEED GUI\n";
		image_gui->setSigPlaneBoundingBoxPoints(temp_bounding_box_points[planeIndex]);
		image_gui->renderFrame();
		if (flag)
		{
			planeCovered = true;
			if(!isBigPlane)
			{
				cout << "[ INFO] Not a big plane. Covered in one go. Copying the necessary information\n";
				for (int i = 0; i < temp_bounding_box_points[significantPlaneIndex].size(); ++i)
				{
					current_bounding_box_points.push_back(temp_bounding_box_points[significantPlaneIndex][i]);
				}
				for (int i = 0; i < temp_plane_parameters[significantPlaneIndex].size(); ++i)
				{
					current_plane_parameters.push_back(temp_plane_parameters[significantPlaneIndex][i]);
				}
			}
			else
			{
				cout << "[ INFO] A big plane. Could not cover in one go. Estimating the best plane\n";
				vector<float> new_plane_params = bestFitPlane(plane_3d_points);
				for (int i = 0; i < new_plane_params.size(); ++i)
				{
					current_plane_parameters.push_back(new_plane_params[i]);
				}
				new_bounding_box_points.clear();
				projectPointsOnPlane(plane_bounding_box_points, new_plane_params, new_bounding_box_points);
				for (int i = 0; i < new_bounding_box_points.size(); ++i)
				{
					current_bounding_box_points.push_back(new_bounding_box_points[i]);
				}
			}
		}
		else
		{
			cout << "[ INFO] Adding the 3d points to be used later for best fit\n";
			for (int i = 0; i < sorted_3d_points[significantPlaneIndex].size(); ++i)
			{
				plane_3d_points.push_back(sorted_3d_points[significantPlaneIndex][i]);
			}
			if(stage)
			{
				for (int i = 0; i < temp_bounding_box_points[significantPlaneIndex].size(); ++i)
				{
					plane_bounding_box_points.push_back(temp_bounding_box_points[significantPlaneIndex][i]);
				}
			}
			else
			{
				Point3f a0, a1, a2, a3;
				a0 = plane_bounding_box_points[0];
				a1 = temp_bounding_box_points[significantPlaneIndex][1];
				a2 = temp_bounding_box_points[significantPlaneIndex][2];
				a3 = plane_bounding_box_points[3];
				plane_bounding_box_points.clear();
				plane_bounding_box_points.push_back(a0);
				plane_bounding_box_points.push_back(a1);
				plane_bounding_box_points.push_back(a2);
				plane_bounding_box_points.push_back(a3);
			}
			Point3f top_right = temp_bounding_box_points[significantPlaneIndex][1];
			Point3f top_left = temp_bounding_box_points[significantPlaneIndex][2];
			width_of_3d_plane = (double)fabs(sqrt( (top_right.x - top_left.x)*(top_right.x - top_left.x) +
												(top_right.y - top_left.y)*(top_right.y - top_left.y) +
												(top_right.z - top_left.z)*(top_right.z - top_left.z) ));
			cout << "[ INFO] Width of the plane is: " << width_of_3d_plane << "\n";
			getCurrentPositionOfDrone(current_pos_of_drone);
			dest_pos_of_drone.clear();
			// Direction of next plane
			if(dir == CLOCKWISE)
			{
				cout << "[ INFO] Next plane is CLOCKWISE wrt current plane\n";
				cout << "[ INFO] Changing the yaw clocwise by " << (next_plane_angle/2.0) << "\n";
				designPathToChangeYaw(current_pos_of_drone, current_pos_of_drone[3]+(next_plane_angle/2.0), path);
				moveDroneViaSetOfPoints(path);
				cout << "[ INFO] Estimating multiple planes -> call to JLinkage\n";
				getMultiplePlanes3d (planeParameters, continuousBoundingBoxPoints, sorted_3d_points, percentageOfEachPlane);
				size = test_plane_parameters.size();
				for(int i = 0; i < size; i++)
				{
					test_plane_parameters[i].clear();
				}
				test_plane_parameters.clear();
				// Adding the currently seeing plane to find out if a new plane another than the
				// current one is visible by rotating the drone
				for (int i = 0; i < all_planes_plane_parameters.size(); ++i)
				{
					test_plane_parameters.push_back(all_planes_plane_parameters[i]);
				}
				test_plane_parameters.push_back(temp_plane_parameters[significantPlaneIndex]);
				if(isNewPlaneVisible(test_plane_parameters, planeParameters, percentageOfEachPlane))
				{
					planeCovered = true;
					if(!isBigPlane)
					{
						cout << "[ INFO] Not a big plane. Covered in one go. Copying the necessary information\n";
						for (int i = 0; i < temp_bounding_box_points[significantPlaneIndex].size(); ++i)
						{
							current_bounding_box_points.push_back(temp_bounding_box_points[significantPlaneIndex][i]);
						}
						for (int i = 0; i < temp_plane_parameters[significantPlaneIndex].size(); ++i)
						{
							current_plane_parameters.push_back(temp_plane_parameters[significantPlaneIndex][i]);
						}
					}
					else
					{
						cout << "[ INFO] A big plane. Could not cover in one go. Estimating the best plane\n";
						vector<float> new_plane_params = bestFitPlane(plane_3d_points);
						for (int i = 0; i < new_plane_params.size(); ++i)
						{
							current_plane_parameters.push_back(new_plane_params[i]);
						}
						new_bounding_box_points.clear();
						projectPointsOnPlane(plane_bounding_box_points, new_plane_params, new_bounding_box_points);
						for (int i = 0; i < new_bounding_box_points.size(); ++i)
						{
							current_bounding_box_points.push_back(new_bounding_box_points[i]);
						}
					}
					/*getCurrentPositionOfDrone(current_pos_of_drone);
					designPathToChangeYaw(current_pos_of_drone, current_pos_of_drone[3]-(next_plane_angle/2.0), path);
					moveDroneViaSetOfPoints(path);*/
				}
				else
				{
					planeCovered = false;
					isBigPlane = true;
					getCurrentPositionOfDrone(current_pos_of_drone);
					dest_pos_of_drone.clear();
					dest_pos_of_drone.push_back(width_of_3d_plane/(double)2.0);
					dest_pos_of_drone.push_back(0.0);
					dest_pos_of_drone.push_back(0.0);
					dest_pos_of_drone.push_back(-(next_plane_angle/2.0));
					convertWRTQuadcopterOrigin(current_pos_of_drone, dest_pos_of_drone, ac_dest_pos_of_drone);
					designPathForDrone(current_pos_of_drone, ac_dest_pos_of_drone, path);
					moveDroneViaSetOfPoints(path);
				}
			}
			else if(dir == COUNTERCLOCKWISE)
			{
				getCurrentPositionOfDrone(current_pos_of_drone);
				dest_pos_of_drone.clear();
				dest_pos_of_drone.push_back(width_of_3d_plane);
				dest_pos_of_drone.push_back(0.0);
				dest_pos_of_drone.push_back(0.0);
				dest_pos_of_drone.push_back(0.0);
				convertWRTQuadcopterOrigin(current_pos_of_drone, dest_pos_of_drone, ac_dest_pos_of_drone);
				designPathForDrone(current_pos_of_drone, ac_dest_pos_of_drone, path);
				moveDroneViaSetOfPoints(path);
				designPathToChangeYaw(current_pos_of_drone, current_pos_of_drone[3]-(next_plane_angle/2.0), path);
				moveDroneViaSetOfPoints(path);
				getMultiplePlanes3d (planeParameters, continuousBoundingBoxPoints, sorted_3d_points, percentageOfEachPlane);
				size = test_plane_parameters.size();
				for(int i = 0; i < size; i++)
				{
					test_plane_parameters[i].clear();
				}
				test_plane_parameters.clear();
				for (int i = 0; i < all_planes_plane_parameters.size(); ++i)
				{
					test_plane_parameters.push_back(all_planes_plane_parameters[i]);
				}
				test_plane_parameters.push_back(temp_plane_parameters[significantPlaneIndex]);
				if(isNewPlaneVisible(test_plane_parameters, planeParameters, percentageOfEachPlane))
				{
					planeCovered = true;
					if(!isBigPlane)
					{
						cout << "[ INFO] Not a big plane. Covered in one go. Copying the necessary information\n";
						for (int i = 0; i < temp_bounding_box_points[significantPlaneIndex].size(); ++i)
						{
							current_bounding_box_points.push_back(temp_bounding_box_points[significantPlaneIndex][i]);
						}
						for (int i = 0; i < temp_plane_parameters[significantPlaneIndex].size(); ++i)
						{
							current_plane_parameters.push_back(temp_plane_parameters[significantPlaneIndex][i]);
						}
					}
					else
					{
						cout << "[ INFO] A big plane. Could not cover in one go. Estimating the best plane\n";
						vector<float> new_plane_params = bestFitPlane(plane_3d_points);
						for (int i = 0; i < new_plane_params.size(); ++i)
						{
							current_plane_parameters.push_back(new_plane_params[i]);
						}
						projectPointsOnPlane(plane_bounding_box_points, new_plane_params, new_bounding_box_points);
						for (int i = 0; i < new_bounding_box_points.size(); ++i)
						{
							current_bounding_box_points.push_back(new_bounding_box_points[i]);
						}
					}
					/*getCurrentPositionOfDrone(current_pos_of_drone);
					designPathToChangeYaw(current_pos_of_drone, current_pos_of_drone[3]+(next_plane_angle/2.0), path);
					moveDroneViaSetOfPoints(path);*/
				}
				else
				{
					planeCovered = false;
					isBigPlane = true;
					getCurrentPositionOfDrone(current_pos_of_drone);
					designPathToChangeYaw(current_pos_of_drone, current_pos_of_drone[3]+(next_plane_angle/2.0), path);
					moveDroneViaSetOfPoints(path);
					getCurrentPositionOfDrone(current_pos_of_drone);
					dest_pos_of_drone.clear();
					dest_pos_of_drone.push_back(width_of_3d_plane);
					dest_pos_of_drone.push_back(0.0);
					dest_pos_of_drone.push_back(0.0);
					dest_pos_of_drone.push_back(0.0);
					convertWRTQuadcopterOrigin(dest_pos_of_drone, dest_pos_of_drone, ac_dest_pos_of_drone);
					designPathForDrone(current_pos_of_drone, ac_dest_pos_of_drone, path);
					moveDroneViaSetOfPoints(path);
				}
			}
		}
		// Now we've seen this plane 1 time
		stage = false;
	}
}

/**
 * @brief Calculate the optimal position of quadcopter such that it can see the top and bottom edge of the plane
 * @details
 * @param
 * @return
 */
bool
ControlUINode::AdjustToSeeCurrentPlane(float min_distance, float max_distance,
										const vector< vector<float> > &plane_parameters, bool stage)
{
	cout << "[ DEBUG] [AdjustToSeeCurrentPlane] Started\n";
	bool flag = false; // flag indicates whether or not the plane is covered completely
	// Variables for handling drone's position
	vector<double> dest_pos_of_drone;
	vector<double> ac_dest_pos_of_drone;
	vector<double> current_pos_of_drone;
	vector< vector<double> > path;
	// Variables for understanding the current plane
	vector< vector<float> > planeParameters;
	vector< vector<Point3f> > continuousBoundingBoxPoints;
	vector< vector<Point3f> > sorted_3d_points;
	vector<float> percentageOfEachPlane;
	float dest_yaw;
	int planeIndex, move;
	bool planeTopBottomVisible, planeLeftVisible;
	if(stage)
	{
		cout << "[ DEBUG] [AdjustToSeeCurrentPlane] See the plane no.: " << plane_parameters.size()
					<< " for the first time\n";
		cout << "[ DEBUG] [AdjustToSeeCurrentPlane] Calling getMultiplePlanes3d\n";
		getMultiplePlanes3d (planeParameters, continuousBoundingBoxPoints, sorted_3d_points, percentageOfEachPlane);
		cout << "[ DEBUG] [AdjustToSeeCurrentPlane] Plane Parameters derived\n";
		for (int i = 0; i < planeParameters.size(); ++i)
		{
			cout << "[ INFO] Parameters (a, b, c, d) for plane " << i << ": ("
					<< percentageOfEachPlane[i] << ")\n";
			for (int j = 0; j < planeParameters[i].size(); ++j)
			{
				cout << planeParameters[i][j] << " ";
			}
			cout << "\n";
		}
		for (int i = 0; i < continuousBoundingBoxPoints.size(); ++i)
		{
			cout << "[ INFO] Bounding box points for plane " << i << "\n";
			for (int j = 0; j < continuousBoundingBoxPoints[i].size(); ++j)
			{
				cout << continuousBoundingBoxPoints[i][j] << " ";
			}
			cout << "\n";
		}
		image_gui->setContinuousBoundingBoxPoints(continuousBoundingBoxPoints);
		image_gui->setRender(false, true, false);
		cout << "Rendering frame\n";
		image_gui->renderFrame();
		/*for (int i = 0; i < planeParameters.size(); ++i)
		{
			cout << "(" << planeParameters[i][0] << ", " << planeParameters[i][1] << ", "
				<< planeParameters[i][2] << ", " << planeParameters[i][3] << ")\n";
		}*/
		planeIndex = getCurrentPlaneIndex(plane_parameters, planeParameters, percentageOfEachPlane);
		getCurrentPositionOfDrone(current_pos_of_drone);
		cout << "[ DEBUG] [AdjustToSeeCurrentPlane] Curr Pos of Drone: (" << current_pos_of_drone[0]
			<< ", " << current_pos_of_drone[1] << ", "
			<< current_pos_of_drone[2] << ", " << current_pos_of_drone[3] << ")\n";
		alignQuadcopterToCurrentPlane(current_pos_of_drone, planeParameters[planeIndex]);
		getCurrentPositionOfDrone(current_pos_of_drone);
		float distance = getPointPlaneDistance(current_pos_of_drone, planeParameters[planeIndex]);
		cout << "[ DEBUG] [AdjustToSeeCurrentPlane] PP: (" << planeParameters[planeIndex][0]
			<< ", " << planeParameters[planeIndex][1] << ", "
			<< planeParameters[planeIndex][2] << ", " << planeParameters[planeIndex][3] << ")\n";
		cout << "[ DEBUG] [AdjustToSeeCurrentPlane] Distance from the plane: " << distance << "\n";
		bool user_control = true;
		if(!user_control)
		{
			float mind = min_distance;
			float maxd = max_distance;
			bool planeTopBottomVisible = false;
			float mid_distance = mind + (maxd - mind) / 2.0;
			// Move the quadcopter to mid distance
			dest_pos_of_drone.push_back(0.0);
			dest_pos_of_drone.push_back(0.0);
			dest_pos_of_drone.push_back(0.0);
			dest_pos_of_drone.push_back(0.0);
			if(distance >= mid_distance)
			{
				dest_pos_of_drone[1] += fabs(distance - mid_distance);
			}
			else
			{
				dest_pos_of_drone[1] -= fabs(distance - mid_distance);
			}
			convertWRTQuadcopterOrigin(current_pos_of_drone, dest_pos_of_drone, ac_dest_pos_of_drone);
			designPathForDrone(current_pos_of_drone, ac_dest_pos_of_drone, path);
			moveDroneViaSetOfPoints(path);
			float step_distance = fabs(maxd-mind)/5.0;
			move = checkVisibility(planeParameters[planeIndex], continuousBoundingBoxPoints[planeIndex], 0);
			if(move==0) {planeTopBottomVisible = true;}
			else {planeTopBottomVisible = false;}
			while(!planeTopBottomVisible && mind < maxd)
			{
				getCurrentPositionOfDrone(current_pos_of_drone);
				dest_pos_of_drone.clear();
				dest_pos_of_drone.push_back(0.0);
				dest_pos_of_drone.push_back(move*step_distance);
				dest_pos_of_drone.push_back(0.0);
				dest_pos_of_drone.push_back(0.0);
				convertWRTQuadcopterOrigin(current_pos_of_drone, dest_pos_of_drone, ac_dest_pos_of_drone);
				designPathForDrone(current_pos_of_drone, ac_dest_pos_of_drone, path);
				moveDroneViaSetOfPoints(path);
				getMultiplePlanes3d (planeParameters, continuousBoundingBoxPoints, sorted_3d_points, percentageOfEachPlane);
				planeIndex = getCurrentPlaneIndex(plane_parameters, planeParameters, percentageOfEachPlane);
				move = checkVisibility(planeParameters[planeIndex], continuousBoundingBoxPoints[planeIndex], 0);
				if(move==0) {planeTopBottomVisible = true;}
				else
				{
					planeTopBottomVisible = false;
					if(move==-1)
					{
						mind = mind+(move*step_distance);
					}
					else
					{
						maxd = maxd+(move*step_distance);
					}
				}
			}
			move = checkVisibility(planeParameters[planeIndex], continuousBoundingBoxPoints[planeIndex], 1);
			if(move==0) {planeLeftVisible = true;}
			else {planeLeftVisible = false;}
			step_distance = 0.5; // @todo-me Fix this heuristic
			while(!planeLeftVisible)
			{
				getCurrentPositionOfDrone(current_pos_of_drone);
				dest_pos_of_drone.clear();
				dest_pos_of_drone.push_back(move*step_distance);
				dest_pos_of_drone.push_back(0.0);
				dest_pos_of_drone.push_back(0.0);
				dest_pos_of_drone.push_back(0.0);
				convertWRTQuadcopterOrigin(current_pos_of_drone, dest_pos_of_drone, ac_dest_pos_of_drone);
				designPathForDrone(current_pos_of_drone, ac_dest_pos_of_drone, path);
				moveDroneViaSetOfPoints(path);
				getMultiplePlanes3d (planeParameters, continuousBoundingBoxPoints, sorted_3d_points, percentageOfEachPlane);
				planeIndex = getCurrentPlaneIndex(plane_parameters, planeParameters, percentageOfEachPlane);
				move = checkVisibility(planeParameters[planeIndex], continuousBoundingBoxPoints[planeIndex], 1);
				if(move==0) {planeLeftVisible = true;}
				else {planeLeftVisible = false;}
			}
			if(planeIndex < planeParameters.size()-1)
			{
				flag = true; // Because I could see a new plane
			}
			else
			{
				flag = false;
			}
		}
		else
		{
			//while(!(image_gui->is_keyBoardActive())) {}
			cout << "[ DEBUG] [AdjustToSeeCurrentPlane] Control back to autonomous navigator\n";
		}
	}
	else
	{
		getMultiplePlanes3d (planeParameters, continuousBoundingBoxPoints, sorted_3d_points, percentageOfEachPlane);
		int planeIndex = getCurrentPlaneIndex(plane_parameters, planeParameters, percentageOfEachPlane);
		alignQuadcopterToCurrentPlane(current_pos_of_drone, planeParameters[planeIndex]);
		if(planeIndex < planeParameters.size()-1)
		{
			flag = true; // Because I could see a new plane
		}
		else
		{
			flag = false;
		}
	}
	dest_pos_of_drone.clear();
	ac_dest_pos_of_drone.clear();
	current_pos_of_drone.clear();
	percentageOfEachPlane.clear();
	cout << "[ DEBUG] [AdjustToSeeCurrentPlane] Completed\n";
	return flag;
}

int
ControlUINode::checkVisibility(const vector<float> &planeParameters, 
								const vector<Point3f> &continuous_bounding_box_points,
								bool which_side)
{
	cout << "[ DEBUG] [checkVisibility] Started\n";
	int move = 0;
	vector<Point2f> image_bounding_box_points;
	cout << "[ DEBUG] [checkVisibility] Plane Parameters: ";
	for (int i = 0; i < planeParameters.size(); ++i)
	{
		cout << planeParameters[i] << " ";
	}
	cout << "\n";
	cout << "[ DEBUG] [checkVisibility] CBB Points:";
	for (int i = 0; i < continuous_bounding_box_points.size(); ++i)
	{
		cout << continuous_bounding_box_points[i] << "\n";
	}
	cout << "\n";
	if(which_side == 0) // Top and Bottom edge
	{
		image_bounding_box_points.clear();
		project3DPointsOnImage(continuous_bounding_box_points, image_bounding_box_points);
		cout << "[ DEBUG] [checkVisibility] IBB Points:";
		for (int i = 0; i < image_bounding_box_points.size(); ++i)
		{
			cout << image_bounding_box_points[i] << "\n";
		}
		cout << "\n";
		Point2f top_mid = (image_bounding_box_points[0]+image_bounding_box_points[1]);
		top_mid.x = top_mid.x/(float)2.0;
		top_mid.y = top_mid.y/(float)2.0;
		Point2f bottom_mid = (image_bounding_box_points[2]+image_bounding_box_points[3]);
		bottom_mid.x = bottom_mid.x/(float)2.0;
		bottom_mid.y = bottom_mid.y/(float)2.0;
		Line2f tb_edge(top_mid, bottom_mid);
		cout << "Line2f line: " << tb_edge << "\n";
		// @todo-me Fix this heuristic
		if( //(tb_edge.start.x >= 256.0 && tb_edge.start.x <= 384.0) &&
			(tb_edge.start.y >= 72.0 && tb_edge.start.y <= 144.0) ||
			//(tb_edge.end.x >= 256.0 && tb_edge.end.x <= 384.0) &&
			(tb_edge.end.y >= 216.0 && tb_edge.end.y <= 288.0)  )
		{
			move = 0;
		}
		else if( //(tb_edge.start.x >= 256.0 && tb_edge.start.x <= 384.0) &&
				(tb_edge.start.y >= 0.0 && tb_edge.start.y <= 72.0) ||
				//(tb_edge.end.x >= 256.0 && tb_edge.end.x <= 384.0) &&
				(tb_edge.end.y >= 288.0 && tb_edge.end.y <= 360.0) )
		{
			move = -1;
		}
		else
		{
			move = 1;
		}
	}
	else if(which_side == 1) // Left Edge
	{
		image_bounding_box_points.clear();
		project3DPointsOnImage(continuous_bounding_box_points, image_bounding_box_points);
		Line2f left_edge(image_bounding_box_points[0], image_bounding_box_points[3]);
		// @todo-me Fix this heuristic
		if( (left_edge.start.x >= 128.0 && left_edge.start.x <= 256.0) &&
			//(left_edge.start.y >= 144.0 && left_edge.start.x <= 216.0) &&
			(left_edge.end.x >= 128.0 && left_edge.end.x <= 256.0) ) /*&&
			(left_edge.end.y >= 216.0 && left_edge.end.y <= 288.0)  )*/
		{
			move = 0;
		}
		else if( (left_edge.start.x >= 0.0 && left_edge.start.x <= 128.0) &&
				//(left_edge.start.y >= 0.0 && left_edge.start.x <= 72.0) &&
				(left_edge.end.x >= 0.0 && left_edge.end.x <= 128.0) )/*&&
				(left_edge.end.y >= 288.0 && left_edge.end.y <= 360.0) )*/
		{
			move = -1;
		}
		else
		{
			move = 1;
		}
	}
	else
	{
		cout << "[ DEBUG] Currently not dealing with it\n";
	}
	cout << "[ DEBUG] [checkVisibility] Completed\n";
	return move;
}

/**
 * @brief Move the quadcopter to the next plane such that it can see the left edge of the new plane
 * @param [RotateDirection] Rotation Direction Of Quadcopter: CLOCKWISE, COUNTERCLOCKWISE
 */
void
ControlUINode::MoveQuadcopterToNextPlane(RotateDirection dir, double dest_rotation,
											const vector< vector<float> > &plane_parameters)
{
	vector<double> current_pos_of_drone;
	vector<double> dest_pos_of_drone;
	vector<double> ac_dest_pos_of_drone;
	vector< vector<double> > xyz_yaw;
	vector<Point3f> _in_points;
	// Variables for understanding the current plane
	vector< vector<float> > planeParameters;
	vector< vector<Point3f> > continuousBoundingBoxPoints;
	vector< vector<Point3f> > sorted_3d_points;
	vector<float> percentageOfEachPlane;
	getCurrentPositionOfDrone(current_pos_of_drone);
	if(dir == CLOCKWISE)
	{
		// [MGP] -ve anti-clockwise , +ve clockwise
		/*double dest_yaw = current_pos_of_drone[3]+(fabs(dest_rotation));
		designPathToChangeYaw(current_pos_of_drone, dest_yaw, xyz_yaw);
		moveDroneViaSetOfPoints(xyz_yaw);*/
	}
	else if(dir == COUNTERCLOCKWISE)
	{
		/*designPathToChangeYaw(current_pos_of_drone, current_pos_of_drone[3]-(fabs(dest_rotation)), xyz_yaw);
		pthread_mutex_lock(&command_CS);
		moveDroneViaSetOfPoints(xyz_yaw);*/
		getMultiplePlanes3d (planeParameters, continuousBoundingBoxPoints, sorted_3d_points, percentageOfEachPlane);
		int last_plane_index = 0;
		float a = plane_parameters[plane_parameters.size()-1][0];
		float b = plane_parameters[plane_parameters.size()-1][1];
		float c = plane_parameters[plane_parameters.size()-1][2];
		for (int i = 0; i < planeParameters.size(); ++i)
		{
			float dot_p = planeParameters[i][0]*a + planeParameters[i][1]*b + planeParameters[i][2]*c;
			if(dot_p >= 0.984)
			{
				last_plane_index = i; break;
			}
		}
		Point3f bl = continuousBoundingBoxPoints[last_plane_index][3];
		Point3f br = continuousBoundingBoxPoints[last_plane_index][2];
		Point3f tr = continuousBoundingBoxPoints[last_plane_index][1];
		Point3f tl = continuousBoundingBoxPoints[last_plane_index][0];
		Point3f mid = (br+tr);
		mid.x = mid.x/(float)2.0;
		mid.y = mid.y/(float)2.0;
		mid.z = mid.z/(float)2.0;
		Point3f normal_old_plane(planeParameters[last_plane_index][0],
							planeParameters[last_plane_index][1], planeParameters[last_plane_index][2]);
		Point3f normal_new_plane(planeParameters[last_plane_index+1][0],
							planeParameters[last_plane_index+1][1], planeParameters[last_plane_index+1][2]);
		float mag_normal_new_plane = sqrt(normal_new_plane.x*normal_new_plane.x +
							normal_new_plane.y*normal_new_plane.y + normal_new_plane.z*normal_new_plane.z);
		getCurrentPositionOfDrone(current_pos_of_drone);
		float drone_distance = getPointPlaneDistance(current_pos_of_drone, planeParameters[last_plane_index]);
		float t = drone_distance/mag_normal_new_plane;
		//double angle = normal_old_plane.dot(normal_new_plane);
		ac_dest_pos_of_drone.clear();
		ac_dest_pos_of_drone.push_back(mid.x + t*normal_new_plane.x);
		ac_dest_pos_of_drone.push_back(mid.y + t*normal_new_plane.y);
		ac_dest_pos_of_drone.push_back(mid.z + t*normal_new_plane.z);
		Point3f pYAxis(ac_dest_pos_of_drone[0],ac_dest_pos_of_drone[1],ac_dest_pos_of_drone[2]);
		Point3f pOrigin(current_pos_of_drone[0],current_pos_of_drone[1],current_pos_of_drone[2]);
		Point3f projectedNormal(pYAxis-pOrigin);
		double angle = (double)findAngle(projectedNormal, normal_new_plane);
		ac_dest_pos_of_drone.push_back(current_pos_of_drone[3]-angle);
		designPathForDrone(current_pos_of_drone, ac_dest_pos_of_drone, xyz_yaw);
		moveDroneViaSetOfPoints(xyz_yaw);
		double width_of_3d_plane = sqrt(pow(bl.x-br.x,2)+pow(bl.y-br.y,2)+pow(bl.z-br.z,2));
		getCurrentPositionOfDrone(current_pos_of_drone);
		dest_pos_of_drone.clear();
		dest_pos_of_drone.push_back(0.0);
		dest_pos_of_drone.push_back(0.0);
		dest_pos_of_drone.push_back((width_of_3d_plane)/2);
		dest_pos_of_drone.push_back(0.0);
		convertWRTQuadcopterOrigin(current_pos_of_drone, dest_pos_of_drone, ac_dest_pos_of_drone);
		designPathForDrone(current_pos_of_drone, ac_dest_pos_of_drone, xyz_yaw);
		moveDroneViaSetOfPoints(xyz_yaw);
	}
	return ;
}

/**
 * @brief Function for testing stage 01 ideas
 * @details
 */
void
ControlUINode::testJLinkageOutput()
{
	cout << "[ DEBUG] [testJLinkageOutput] Started\n";
	int test_no = 3;
	if(test_no == 1)
	{
		cout << "[ DEBUG] [testJLinkageOutput] Conducting Test No. 1\n";
		vector< vector<float> > planeParameters;
		vector< vector<Point3f> > continuousBoundingBoxPoints;
		vector< vector<float> > plane_parameters;
		vector< vector<Point3f> > continuous_bounding_box_points;
		vector< vector<Point3f> > sorted_3d_points;
		vector<float> percentageOfEachPlane;
		getMultiplePlanes3d(planeParameters, continuousBoundingBoxPoints,
								sorted_3d_points, percentageOfEachPlane);
		cout << "[ DEBUG] Testing JLinkage Outout on key press f\n";
		for (int i = 0; i < planeParameters.size(); ++i)
		{
			cout << "[ INFO] Parameters (a, b, c, d) for plane " << i << ": ("
					<< percentageOfEachPlane[i] << ")\n";
			for (int j = 0; j < planeParameters[i].size(); ++j)
			{
				cout << planeParameters[i][j] << " ";
			}
			cout << "\n";
		}
		for (int i = 0; i < continuousBoundingBoxPoints.size(); ++i)
		{
			cout << "[ INFO] Bounding box points for plane " << i << "\n";
			for (int j = 0; j < continuousBoundingBoxPoints[i].size(); ++j)
			{
				cout << continuousBoundingBoxPoints[i][j] << " ";
			}
			cout << "\n";
		}
		image_gui->setContinuousBoundingBoxPoints(continuousBoundingBoxPoints);
		image_gui->setRender(false, true, false);
		cout << "Rendering frame\n";
		image_gui->renderFrame();
		cout << "Calculating plane parameters:\n";
		int planeIndex = getCurrentPlaneIndex(plane_parameters, planeParameters, percentageOfEachPlane);
		cout << "Plane Index:" << planeIndex << "\n";
		image_gui->setSigPlaneBoundingBoxPoints(continuousBoundingBoxPoints[planeIndex]);
		image_gui->setRender(false, false, true);
		cout << "Rendering frame\n";
		image_gui->renderFrame();
		vector<float> normal = bestFitPlane(sorted_3d_points[planeIndex]);
	}
	else if(test_no == 2)
	{
		just_navigation_commands.clear();
		targetPoints.clear();
		char buf[100];
		std_msgs::String s;
		vector<double> move_points;
		move_points.push_back(0.0);
		move_points.push_back(0.0);
		move_points.push_back(1.0);
		move_points.push_back(0.0);
		snprintf(buf, 100, "c goto %lf %lf %lf %lf",
				move_points[0], move_points[1], move_points[2], move_points[3]);
		s.data = buf;
		ROS_INFO("Message: ");
		ROS_INFO(buf);
		just_navigation_commands.push_back(s);
		targetPoints.push_back(move_points);
		move_points.clear();
		move_points.push_back(1.0);
		move_points.push_back(0.0);
		move_points.push_back(1.0);
		move_points.push_back(30.0);
		snprintf(buf, 100, "c goto %lf %lf %lf %lf",
				move_points[0], move_points[1], move_points[2], move_points[3]);
		s.data = buf;
		ROS_INFO("Message: ");
		ROS_INFO(buf);
		just_navigation_commands.push_back(s);
		targetPoints.push_back(move_points);
		move_points.clear();
		move_points.push_back(1.0);
		move_points.push_back(0.0);
		move_points.push_back(1.0);
		move_points.push_back(10.0);
		snprintf(buf, 100, "c goto %lf %lf %lf %lf",
				move_points[0], move_points[1], move_points[2], move_points[3]);
		s.data = buf;
		ROS_INFO("Message: ");
		ROS_INFO(buf);
		just_navigation_commands.push_back(s);
		targetPoints.push_back(move_points);
		pthread_mutex_lock(&command_CS);
		cout << "[ DEBUG] There are " << just_navigation_commands.size() << " commands for drone\n";
		if(just_navigation_commands.size() > 0)
		{
			while (!just_navigation_commands.empty())
			{
				pthread_mutex_lock(&tum_ardrone_CS);
				tum_ardrone_pub.publish(just_navigation_commands.front());
				pthread_mutex_unlock(&tum_ardrone_CS);
				targetPoint = targetPoints.front();
				printf("Moving to Current target: %lf %lf %lf\n", targetPoint[0], targetPoint[1] , targetPoint[2] );
				just_navigation_commands.pop_front();
				targetPoints.pop_front();
				ros::Duration(1.5).sleep();
			}
			justNavigation = false;
		}
		pthread_mutex_unlock(&command_CS);
		vector<double> current_pos_of_drone;
		getCurrentPositionOfDrone(current_pos_of_drone);
		cout << "[ DEBUG] Drone at: (" << current_pos_of_drone[0] << ", " 
			<< current_pos_of_drone[1] << ", " << current_pos_of_drone[2] << ", "
			<< current_pos_of_drone[3] << ")\n";
		cout << "[ DEBUG] Done\n";
	}
	else if(test_no == 3)
	{
		cout << "[ DEBUG] [testJLinkageOutput] Conducting Test No. 3\n";
		vector< vector<float> > planeParameters;
		vector< vector<Point3f> > continuousBoundingBoxPoints;
		vector< vector<float> > plane_parameters;
		vector< vector<Point3f> > continuous_bounding_box_points;
		vector< vector<Point3f> > sorted_3d_points;
		vector<float> percentageOfEachPlane;
		getMultiplePlanes3d(planeParameters, continuousBoundingBoxPoints,
								sorted_3d_points, percentageOfEachPlane);
		/*cout << "[ DEBUG] Testing JLinkage Outout on key press f\n";
		for (int i = 0; i < planeParameters.size(); ++i)
		{
			cout << "[ INFO] Parameters (a, b, c, d) for plane " << i << ": ("
					<< percentageOfEachPlane[i] << ")\n";
			for (int j = 0; j < planeParameters[i].size(); ++j)
			{
				cout << planeParameters[i][j] << " ";
			}
			cout << "\n";
		}
		for (int i = 0; i < continuousBoundingBoxPoints.size(); ++i)
		{
			cout << "[ INFO] Bounding box points for plane " << i << "\n";
			for (int j = 0; j < continuousBoundingBoxPoints[i].size(); ++j)
			{
				cout << continuousBoundingBoxPoints[i][j] << " ";
			}
			cout << "\n";
		}*/
		image_gui->setContinuousBoundingBoxPoints(continuousBoundingBoxPoints);
		image_gui->setRender(false, true, false);
		//cout << "Rendering frame\n";
		image_gui->renderFrame();
		//cout << "Calculating plane parameters:\n";
		int planeIndex = getCurrentPlaneIndex(plane_parameters, planeParameters, percentageOfEachPlane);
		//cout << "Plane Index:" << planeIndex << "\n";
		image_gui->setSigPlaneBoundingBoxPoints(continuousBoundingBoxPoints[planeIndex]);
		image_gui->setRender(false, false, true);
		//cout << "Rendering frame\n";
		image_gui->renderFrame();
		vector<double> current_pos_of_drone, dest_pos_of_drone, ac_dest_pos_of_drone;
		getCurrentPositionOfDrone(current_pos_of_drone);
		cout << "[ DEBUG] Curr Pos of Drone: (" << current_pos_of_drone[0]
			<< ", " << current_pos_of_drone[1] << ", " << current_pos_of_drone[2] << ", " 
			<< current_pos_of_drone[3] << ")\n";
		/*cout << "[ DEBUG] Plane parameters: ";
		for (int j = 0; j < planeParameters[planeIndex].size(); ++j)
		{
			cout << planeParameters[planeIndex][j] << " ";
		}
		cout << "\n";*/
		alignQuadcopterToCurrentPlane(current_pos_of_drone, planeParameters[planeIndex]);
		getCurrentPositionOfDrone(current_pos_of_drone);
		cout << "[ DEBUG] Curr Pos of Drone: (" << current_pos_of_drone[0]
			<< ", " << current_pos_of_drone[1] << ", " << current_pos_of_drone[2] << ", " 
			<< current_pos_of_drone[3] << ")\n";
		float distance = getPointPlaneDistance(current_pos_of_drone, planeParameters[planeIndex]);
		cout << "[ DEBUG] PP: (" << planeParameters[planeIndex][0]
			<< ", " << planeParameters[planeIndex][1] << ", "
			<< planeParameters[planeIndex][2] << ", " << planeParameters[planeIndex][3] << ")\n";
		cout << "[ DEBUG] Distance from the plane: " << distance << "\n";
		float mind = 3.0;
		float maxd = 5.0;
		bool flag = false;
		bool planeTopBottomVisible = false, planeLeftVisible = false;
		float mid_distance = mind + (maxd - mind) / 2.0;
		// Move the quadcopter to mid distance
		dest_pos_of_drone.push_back(0.0);
		dest_pos_of_drone.push_back(0.0);
		dest_pos_of_drone.push_back(0.0);
		dest_pos_of_drone.push_back(0.0);
		if(distance >= mid_distance)
		{
			dest_pos_of_drone[1] += fabs(distance - mid_distance);
		}
		else
		{
			dest_pos_of_drone[1] -= fabs(distance - mid_distance);
		}
		vector< vector<double> > path;
		convertWRTQuadcopterOrigin(current_pos_of_drone, dest_pos_of_drone, ac_dest_pos_of_drone);
		cout << "[ DEBUG] Actual Pos of Drone To move: (" << ac_dest_pos_of_drone[0]
			<< ", " << ac_dest_pos_of_drone[1] << ", " << ac_dest_pos_of_drone[2] << ", " 
			<< ac_dest_pos_of_drone[3] << ")\n";
		designPathForDrone(current_pos_of_drone, ac_dest_pos_of_drone, path);
		moveDroneViaSetOfPoints(path);
		float step_distance = fabs(maxd-mind)/3.0;
		double height = getHeightFromGround(planeParameters[planeIndex], continuousBoundingBoxPoints[planeIndex]);
		cout << "[DEBUG] Step Distance: " << step_distance << "\n";
		int move = checkVisibility(planeParameters[planeIndex], continuousBoundingBoxPoints[planeIndex], 0);
		if(move==0) {planeTopBottomVisible = true; cout << "[ DEBUG] Need not move\n";}
		else {planeTopBottomVisible = false;
			if(move==1)
			{cout << "[ DEBUG] Need to move forward\n";}
			else
			{cout << "[ DEBUG] Need to move backward\n";}
		}
		cout << "Move: " << move << ", Step Distance: " << step_distance << "\n";
		cout << "[ DEBUG] In while loop\n";
		while(!planeTopBottomVisible && mind < maxd)
		{
			getCurrentPositionOfDrone(current_pos_of_drone);
			distance = getPointPlaneDistance(current_pos_of_drone, planeParameters[planeIndex]);
			cout << "[ DEBUG] Distance from the plane: " << distance << "\n";
			cout << "[ DEBUG] Curr Pos of Drone: (" << current_pos_of_drone[0]
				<< ", " << current_pos_of_drone[1] << ", " << current_pos_of_drone[2] << ", " 
				<< current_pos_of_drone[3] << ")\n";
			double to_move = (double)move * (double)step_distance;
			cout << "Move: " << move << ", Step Distance: " << step_distance << "\n";
			dest_pos_of_drone.clear();
			dest_pos_of_drone.push_back(0.0);
			dest_pos_of_drone.push_back(to_move);
			dest_pos_of_drone.push_back(0.0);
			dest_pos_of_drone.push_back(0.0);
			cout << "[ DEBUG] Dest Pos of Drone: (" << dest_pos_of_drone[0]
				<< ", " << dest_pos_of_drone[1] << ", " << dest_pos_of_drone[2] << ", " 
				<< dest_pos_of_drone[3] << ")\n";
			convertWRTQuadcopterOrigin(current_pos_of_drone, dest_pos_of_drone, ac_dest_pos_of_drone);
			height = getHeightFromGround(planeParameters[planeIndex], continuousBoundingBoxPoints[planeIndex]);
			ac_dest_pos_of_drone[2] = height;
			cout << "[ DEBUG] Actual Pos of Drone To move: (" << ac_dest_pos_of_drone[0]
				<< ", " << ac_dest_pos_of_drone[1] << ", " << ac_dest_pos_of_drone[2] << ", " 
				<< ac_dest_pos_of_drone[3] << ")\n";
			designPathForDrone(current_pos_of_drone, ac_dest_pos_of_drone, path);
			moveDroneViaSetOfPoints(path);
			getMultiplePlanes3d (planeParameters, continuousBoundingBoxPoints, sorted_3d_points, percentageOfEachPlane);
			planeIndex = getCurrentPlaneIndex(plane_parameters, planeParameters, percentageOfEachPlane);
			distance = getPointPlaneDistance(current_pos_of_drone, planeParameters[planeIndex]);
			cout << "[ DEBUG] Distance from the plane: " << distance << "\n";
			move = checkVisibility(planeParameters[planeIndex], continuousBoundingBoxPoints[planeIndex], 0);
			to_move = (double)move * (double)step_distance;
			if(move==0) {planeTopBottomVisible = true; cout << "[ DEBUG] Need not move\n";}
			else
			{
				planeTopBottomVisible = false;
				if(move==-1)
				{
					cout << "Move: " << move << ", ToMove: " << to_move << "\n";
					mind = mind-(to_move); cout << "[ DEBUG] Need to move backward\n";
				}
				else
				{
					cout << "Move: " << move << ", ToMove: " << to_move << "\n";
					maxd = maxd-(to_move); cout << "[ DEBUG] Need to move forward\n";
				}
			}
			cout << "Mind: " << mind << ", Maxd: " << maxd << "\n";
		}
		cout << "[ DEBUG] [testJLinkageOutput] Adjusting top and bottom done.\n";
		move = checkVisibility(planeParameters[planeIndex], continuousBoundingBoxPoints[planeIndex], 1);
		if(move==0) {planeLeftVisible = true;}
		else {planeLeftVisible = false;}
		step_distance = 0.5; // @todo-me Fix this heuristic
		/*while(!planeLeftVisible)
		{
			getCurrentPositionOfDrone(current_pos_of_drone);
			dest_pos_of_drone.clear();
			dest_pos_of_drone.push_back(move*step_distance);
			dest_pos_of_drone.push_back(0.0);
			dest_pos_of_drone.push_back(0.0);
			dest_pos_of_drone.push_back(0.0);
			convertWRTQuadcopterOrigin(current_pos_of_drone, dest_pos_of_drone, ac_dest_pos_of_drone);
			designPathForDrone(current_pos_of_drone, ac_dest_pos_of_drone, path);
			moveDroneViaSetOfPoints(path);
			getMultiplePlanes3d (planeParameters, continuousBoundingBoxPoints, sorted_3d_points, percentageOfEachPlane);
			planeIndex = getCurrentPlaneIndex(plane_parameters, planeParameters, percentageOfEachPlane);
			move = checkVisibility(planeParameters[planeIndex], continuousBoundingBoxPoints[planeIndex], 1);
			if(move==0) {planeLeftVisible = true;}
			else {planeLeftVisible = false;}
		}
		if(planeIndex < planeParameters.size()-1)
		{
			flag = true; // Because I could see a new plane
		}
		else
		{
			flag = false;
		}*/
	}
	else
	{

	}
	cout << "[ DEBUG] [testJLinkageOutput] Completed\n";
}

double
ControlUINode::getHeightFromGround(const vector<float> &planeParameters, 
						const vector<Point3f> &continuousBoundingBoxPoints)
{
	double height;
	Point3f top_mid = (continuousBoundingBoxPoints[0]+continuousBoundingBoxPoints[1]);
	top_mid.x = top_mid.x/(float)2.0;
	top_mid.y = top_mid.y/(float)2.0;
	top_mid.z = top_mid.z/(float)2.0;
	Point3f bottom_mid = (continuousBoundingBoxPoints[2]+continuousBoundingBoxPoints[3]);
	bottom_mid.x = bottom_mid.x/(float)2.0;
	bottom_mid.y = bottom_mid.y/(float)2.0;
	bottom_mid.z = bottom_mid.z/(float)2.0;
	Point3f mid;
	mid.x = (top_mid.x+bottom_mid.x)/(float)2.0;
	mid.y = (top_mid.y+bottom_mid.y)/(float)2.0;
	mid.z = (top_mid.z+bottom_mid.z)/(float)2.0;
	Point3f normal_plane(planeParameters[0], planeParameters[1], planeParameters[2]);
	float mag_normal_plane = sqrt(normal_plane.x*normal_plane.x +
						normal_plane.y*normal_plane.y + normal_plane.z*normal_plane.z);
	vector<double> current_pos_of_drone;
	getCurrentPositionOfDrone(current_pos_of_drone);
	float drone_distance = getPointPlaneDistance(current_pos_of_drone, planeParameters);
	float t = drone_distance/mag_normal_plane;
	height = (double) mid.z + t*normal_plane.z;
	cout << "Height to adjust: " << height << "\n";
	cout << "Values x: " << mid.x + t*normal_plane.x << "\n";
	cout << "Values y: " << mid.y + t*normal_plane.y << "\n";
	cout << "Values z: " << mid.z + t*normal_plane.z << "\n";
	return height;
}

/************** UNNECESSARY FUNCTIONS ***************************************/

/**
 * @brief Gets all the 3d world points within the clicked 2d image points
 * @details
 */
void
ControlUINode::get3DPointsOfCapturedPlane(const vector<int> &ccPoints,
											const vector<vector<int> > &pointsClicked,
											vector< Point3f > &threed_points)
{
	vector< vector<int> > points;
	for(unsigned int i = 0; i < ccPoints.size(); i++)
	{
		points.push_back(pointsClicked[ccPoints[i]]);
	}
	pthread_mutex_lock(&keyPoint_CS);
	for(unsigned int i = 0; i < _2d_points.size(); i++)
	{
		if(liesInside(points, _2d_points[i]))
		{
			Point3f featurePt;
			featurePt.x = _3d_points[i][0];
			featurePt.y = _3d_points[i][1];
			featurePt.z = _3d_points[i][2];
			threed_points.push_back(featurePt);
		}
	}
	pthread_mutex_unlock(&keyPoint_CS);
}


/* CURRENTLY MIGHT NOT BE REQUIRED
// Gather the points for the marked plane by moving the quadcopter towards the left edge of the plane
// and towards the right edge of the plane from there
if(flag)
	GatherPoints(pointsClicked, dir);
else
	GatherPoints(pointsClicked, COUNTERCLOCKWISE);
// Write the 3D map to the file Plane_3D_Points.txt
string file_name = "Plane_3D_2D_Points.csv";
WritePointsToFile(_3d_points, _2d_points, file_name);
vector<Point3f> points_travel;
Point3f left_end_of_plane = ;
Point3f right_end_of_plane = ;
node->moveDrone(points_travel); */
