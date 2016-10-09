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
pthread_mutex_t ControlUINode::changeyaw_CS = PTHREAD_MUTEX_INITIALIZER;


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
	recordNow = false;
	notRecording = true;
	planeIndex = 0;

	timer_checkPos = nh_.createTimer(ros::Duration(pollingTime), &ControlUINode::checkPos, this);
	// timer_record = nh_.createTimer(ros::Duration(recordTime), &ControlUINode::recordVideo);

	/* PRANEETH's CODE */
	// Channel for controlling landing commands
	land_pub		= nh_.advertise<std_msgs::Empty>(land_channel, 1);
	// Whether it's seeing the plane for the first time
	_stage_of_plane_observation = true;
	// Is the plane big? requiring multiple attempts to cover ti
	_is_big_plane = false;
	// Check if the plane in consideration is  covered completely
	_is_plane_covered = false;
	// Number of planes covered completelt till now
	_node_completed_number_of_planes = 0;
	// Are you using moveDroneViaSetOfPoints. Indicating the drone is currently moving
	justNavigation = false;
	// Has the drone completed executing the command sent?
	traverseComplete = false;
	// Current command navigation number
	just_navigation_command_number = -1;
	// Total number of commands to be sent to the drone
	just_navigation_total_commands = -1;
	// Is it just changing the yaw? or also travelling
	linearTraversal = true;
	// Has my changeyaw_CS lock been released?
	changeyawLockReleased = 0;
	// Subscribing for pose channel
	new_pose_sub = nh_.subscribe(pose_channel, 10, &ControlUINode::newPoseCb, this);

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
			for(unsigned int j=0; j<xyz_yaw.size(); j++){
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

/* PRANEETH's CODE */

/**************************************************************************************
	New additions to the code
***************************************************************************************/

/**
 * @brief New pose callback for dealing with autonomous moving of quadcopter
 * @details
 */
void
ControlUINode::newPoseCb (const tum_ardrone::filter_stateConstPtr statePtr)
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
	// cout << "[ DEBUG] [poseCb] Acquired command_CS Lock\n";
	static int numCommands = 0;
	/*cout << "[ DEBUG] [poseCb] Checking for just navigation commands\n";
	cout << "[ DEBUG] [poseCb] Number of commands left: " << just_navigation_commands.size() << "\n";
	cout << "[ DEBUG] [poseCb] justNavigation: " << justNavigation << "\n";
	cout << "[ DEBUG] [poseCb] traverseComplete: " << traverseComplete << "\n";*/
	if(just_navigation_commands.size() == 0 && changeyawLockReleased==-1)
	{
		pthread_mutex_unlock(&changeyaw_CS);
		changeyawLockReleased = 0;
		cout << "[ DEBUG] [poseCb] Released first changeyaw_CS Lock\n";
	}
	else if(just_navigation_commands.size() > 0 && !justNavigation)
	{
		justNavigation = true;
		traverseComplete = false;
		just_navigation_command_number++;
		pthread_mutex_lock(&tum_ardrone_CS);
			tum_ardrone_pub.publish(just_navigation_commands.front());
		pthread_mutex_unlock(&tum_ardrone_CS);
		targetPoint.clear();
		targetPoint = targetPoints.front();
		printf("[ DEBUG] [poseCb] (%u) Just Navigation Current target %u: %lf %lf %lf %lf\n", 
			just_navigation_total_commands, just_navigation_command_number, targetPoint[0], 
			targetPoint[1] , targetPoint[2], targetPoint[3] );
	}
	else if(justNavigation && !traverseComplete)
	{
		double x = targetPoint[0];
		double y = targetPoint[1];
		double z = targetPoint[2];
		double ya = targetPoint[3];
		pthread_mutex_lock(&pose_CS);
		double ea = sqrt(pow(x - x_drone, 2) + pow(y - y_drone, 2) + pow(z - z_drone, 2) + pow(ya - yaw, 2));
		// cout << "[ DEBUG] [poseCb] Current Pose: " << x_drone << ", " << y_drone << ", " << z_drone << ", " << yaw << "\n";
		// printf("Error %lf\n", ea);
		pthread_mutex_unlock(&pose_CS);
		if(ea < error_threshold)
		{
			// cout << "[ DEBUG] [poseCb] Destination reached for command no. " << just_navigation_number << "\n";
			if(just_navigation_command_number <= just_navigation_total_commands)
			{
				ros::Duration(1).sleep();
				justNavigation = false;
				traverseComplete = true;
				just_navigation_commands.pop_front();
				targetPoints.pop_front();
				targetPoint.clear();
				pthread_mutex_unlock(&command_CS);
				cout << "[ DEBUG] [poseCb] Released elseif command_CS Lock\n";
				if(just_navigation_commands.size() == 0) {changeyawLockReleased = -1;}
				return;
			}
		}
		else
		{
			justNavigation = true; traverseComplete = false;
		}
	}
	else if(commands.size() > 0 && !currentCommand)
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
	// cout << "[ DEBUG] [poseCb] Released command_CS Lock\n";
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
 * @brief Sets the _node_main_angles vector from ImageView
 * @details
 */
void
ControlUINode::setMainAngles(const vector<double> &main_angles)
{
	_node_main_angles.clear();
	for (unsigned int i = 0; i < main_angles.size(); ++i)
	{
		_node_main_angles.push_back(main_angles[i]);
	}
	return ;
}

/**
 * @brief Sets the _node_main_directions vector from ImageView
 * @details
 */
void
ControlUINode::setMainDirections(const vector<RotateDirection> &main_directions)
{
	_node_main_directions.clear();
	for (unsigned int i = 0; i < main_directions.size(); ++i)
	{
		_node_main_directions.push_back(main_directions[i]);
	}
	return ;
}

void
ControlUINode::setValues(int number_of_planes, float min_height_of_plane, float min_distance, float max_height_of_plane, float max_distance)
{
	_node_min_distance = min_distance;
	_node_max_distance = max_distance;
	_node_min_height_of_plane = min_height_of_plane;
	_node_max_height_of_plane = max_height_of_plane;
	_node_number_of_planes = number_of_planes;
}

/*void
copyPlaneParameters(const vector<float> &copy_plane_parameters, 
						vector<float> &to_copy_plane_parameters)
{
	to_copy_plane_parameters.clear();
	for (unsigned int i = 0; i < copy_plane_parameters.size(); ++i)
	{
		to_copy_plane_parameters.push_back(copy_plane_parameters[i]);
	}
	return ;
}

void
copyBoundingBoxPoints(const vector<Point3f> &copy_continuous_bounding_box_points, 
						vector<Point3f> &to_copy_continuous_bounding_box_points)
{
	to_copy_continuous_bounding_box_points.clear();
	for (unsigned int i = 0; i < copy_continuous_bounding_box_points.size(); ++i)
	{
		to_copy_continuous_bounding_box_points.push_back(copy_continuous_bounding_box_points[i]);
	}
	return ;
}*/

/**
 * @brief Gets the current position of the drone in _node_current_pos_of_drone (See private variables)
 * @details Returns the position of the drone when the function is called
 */
void
ControlUINode::getCurrentPositionOfDrone()
{
	_node_current_pos_of_drone.clear();
	pthread_mutex_lock(&pose_CS);
		_node_current_pos_of_drone.push_back((double)x_drone);
		_node_current_pos_of_drone.push_back((double)y_drone);
		_node_current_pos_of_drone.push_back((double)z_drone);
		_node_current_pos_of_drone.push_back((double)yaw);
	pthread_mutex_unlock(&pose_CS);
}

/**
 * @brief Gets the current position of the drone in the varible pos
 * @details Returns the position of the drone when the function is called.
 * 			To be used when calling from other classes
 */
void
ControlUINode::getCurrentPositionOfDrone(vector<double> &current_drone_pos)
{
	current_drone_pos.clear();
	pthread_mutex_lock(&pose_CS);
		current_drone_pos.push_back((double)x_drone);
		current_drone_pos.push_back((double)y_drone);
		current_drone_pos.push_back((double)z_drone);
		current_drone_pos.push_back((double)yaw);
	pthread_mutex_unlock(&pose_CS);
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
 * @brief Gets all the data about the points in the clicked region using jlinkage
 * @details To be called after user clicks the 4 points on the DRONE CAMERA FEED Window
 */
void
ControlUINode::captureTheCurrentPlane()
{
	cout << "[ DEBUG] [captureTheCurrentPlane] Started\n";
	// 2d image points clicked on the DRONE CAMERA FEED Screen
	vector< vector<int> > points_clicked;
	// the 3d keypoints of control node for nearest keypoints
	vector< vector<float> > key_points_nearest;
	// corners of the convex hull
	vector<int> cc_points;
	// First param: Number of points clicked on the screen
	// Second param: Number of Key points detected
	image_gui->setNumberOfPoints(0, 0);
	// RendeRect: false, RenderPoly: false, RenderSignificantPlane: false
	image_gui->setRender(false, false, false);
	image_gui->getPointsClicked(points_clicked);
	cout << "[ INFO] [captureTheCurrentPlane] Extracting Bounding Poly\n";
	image_gui->extractBoundingPoly();
	int significantPlaneIndex = 0;
	image_gui->getCCPoints(cc_points);
	cout << "[ INFO] [captureTheCurrentPlane] Get multiple planes from the clicked points using JLinkage\n";
	// Calls JLinkage and finds all planes within the clicked region
	getMultiplePlanes3d(cc_points, points_clicked, jlink_all_plane_parameters, jlink_all_continuous_bounding_box_points,
						jlink_three_d_points, jlink_all_percentage_of_each_plane);
	significantPlaneIndex = getCurrentPlaneIndex(visited_plane_parameters, jlink_all_plane_parameters, jlink_all_percentage_of_each_plane);
	copyVector(jlink_all_plane_parameters[significantPlaneIndex], this_plane_parameters);
	copyVector(jlink_all_continuous_bounding_box_points[significantPlaneIndex], this_continuous_bounding_box_points);
	// Render significant plane
	image_gui->setRender(false, true, true);
	cout << "[ INFO] [captureTheCurrentPlane] Rendering the frames in the DRONE CAMERA FEED GUI\n";
	image_gui->setSigPlaneBoundingBoxPoints(jlink_all_continuous_bounding_box_points[significantPlaneIndex]);
	image_gui->renderFrame();
	// @todo-me Check if the plane is completed
	clear2dVector(this_visited_plane_parameters);
	for (unsigned int i = 0; i < visited_plane_parameters.size(); ++i)
	{
		this_visited_plane_parameters.push_back(visited_plane_parameters[i]);
	}
	this_visited_plane_parameters.push_back(this_plane_parameters);
	_is_plane_covered = isNewPlaneVisible(this_visited_plane_parameters,
									jlink_all_plane_parameters, jlink_all_percentage_of_each_plane, false);
	if(_is_plane_covered)
	{
		if(!_is_big_plane)
		{
			cout << "[ INFO] [captureTheCurrentPlane] Not a big plane. Covered in one go. Copying the necessary information\n";
			visited_plane_parameters.push_back(jlink_all_plane_parameters[significantPlaneIndex]);
			visited_continuous_bounding_box_points.push_back(jlink_all_continuous_bounding_box_points[significantPlaneIndex]);
		}
		else
		{
			cout << "[ INFO] [captureTheCurrentPlane] A big plane. Could not cover in one go. Estimating the best plane\n";
			vector<float> new_plane_params = bestFitPlane(aug_three_d_points);
			vector<Point3f> new_bounding_box_points;
			visited_plane_parameters.push_back(new_plane_params);
			new_bounding_box_points.clear();
			projectPointsOnPlane(aug_plane_bounding_box_points, new_plane_params, new_bounding_box_points);
			visited_continuous_bounding_box_points.push_back(aug_plane_bounding_box_points);
			aug_three_d_points.clear();
			aug_plane_bounding_box_points.clear();
		}
		_is_big_plane = false;
		_stage_of_plane_observation = true;
		_node_completed_number_of_planes++;
		// if(_node_completed_number_of_planes != _node_number_of_planes)
		// {
		// 	alignQuadcopterToNextPlane();
		// }
	}
	else
	{
		cout << "[ INFO] Adding the 3d points to be used later for best fit\n";
		for (unsigned int i = 0; i < jlink_three_d_points[significantPlaneIndex].size(); ++i)
		{
			aug_three_d_points.push_back(jlink_three_d_points[significantPlaneIndex][i]);
		}
		if(_stage_of_plane_observation)
		{
			for (unsigned int i = 0; i < jlink_all_continuous_bounding_box_points[significantPlaneIndex].size(); ++i)
			{
				aug_plane_bounding_box_points.push_back(jlink_all_continuous_bounding_box_points[significantPlaneIndex][i]);
			}
		}
		else
		{
			Point3f a0, a1, a2, a3;
			a0 = aug_plane_bounding_box_points[0];
			a1 = jlink_all_continuous_bounding_box_points[significantPlaneIndex][1];
			a2 = jlink_all_continuous_bounding_box_points[significantPlaneIndex][2];
			a3 = aug_plane_bounding_box_points[3];
			aug_plane_bounding_box_points.clear();
			aug_plane_bounding_box_points.push_back(a0);
			aug_plane_bounding_box_points.push_back(a1);
			aug_plane_bounding_box_points.push_back(a2);
			aug_plane_bounding_box_points.push_back(a3);
		}
		// adjustForNextCapture();
	}
	if(_node_completed_number_of_planes == _node_number_of_planes)
	{
		assert(visited_plane_parameters.size() == visited_continuous_bounding_box_points.size());
		string filename = "Plane_Info.txt";
		cout << "[ DEBUG] Writing info gathered to " << filename << "\n";
		for (unsigned int i = 0; i < visited_plane_parameters.size(); ++i)
		{
			image_gui->WriteInfoToFile(visited_continuous_bounding_box_points[i], visited_plane_parameters[i], i+1, filename);
		}
	}
	else
	{
		cout << "[ DEBUG] [captureTheCurrentPlane] All planes are not completed\n";
	}
	cout << "[ DEBUG] [captureTheCurrentPlane] Completed\n";
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
	pthread_mutex_lock(&changeyaw_CS);
	changeyawLockReleased = 1;
	cout << "[ DEBUG] [moveDroneViaSetOfPoints] Acquired first changeyaw_CS Lock\n";
	cout << "[ DEBUG] [moveDroneViaSetOfPoints] Started\n";
	char buf[100];
	just_navigation_commands.clear();
	targetPoints.clear();
	just_navigation_total_commands = dest_points.size();
	just_navigation_command_number = 0;
	cout << "[ DEBUG] [moveDroneViaSetOfPoints] Total Commands: " << just_navigation_total_commands << "\n";
	for (unsigned int i = 0; i < dest_points.size(); ++i)
	{
		cout << "(" << dest_points[i][0] << ", " << dest_points[i][1] << ", " 
			<< dest_points[i][2] << ", " << dest_points[i][3] << ")\n";
	}
	for (unsigned int i = 0; i < dest_points.size(); ++i)
	{
		snprintf(buf, 100, "c goto %lf %lf %lf %lf",
			dest_points[i][0], dest_points[i][1], dest_points[i][2], dest_points[i][3]);
		std_msgs::String s;
		s.data = buf;
		ROS_INFO("Message: ");
		ROS_INFO(buf);
		just_navigation_commands.push_back(s);
		targetPoints.push_back(dest_points[i]);
	}
	cout << "[ DEBUG] [moveDroneViaSetOfPoints] Commands Ready\n";
	cout << "[ DEBUG] [moveDroneViaSetOfPoints] Commands to execute: " << just_navigation_commands.size() << "\n";
	// just_navigation_number = just_navigation_commands.size();
	pthread_mutex_lock(&changeyaw_CS);
	cout << "[ DEBUG] [moveDroneViaSetOfPoints] Acquired second changeyaw_CS Lock\n";
	pthread_mutex_unlock(&changeyaw_CS);
	cout << "[ DEBUG] [moveDroneViaSetOfPoints] Released second changeyaw_CS Lock\n";
	just_navigation_total_commands = -1;
	just_navigation_command_number = -1;
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
				double dest_yaw)
{
	clear2dVector(_interm_path);
	vector<double> interm_point;
	double currentYaw = curr_point[3], desiredYaw = dest_yaw;
	double diff_angle = desiredYaw - currentYaw;
	double number = diff_angle/(double)6.0;
	cout << "[ DEBUG] [designPathToChangeYaw] Changing yaw by " << number << "\n";
	double move;
	if(currentYaw > desiredYaw) {move = -1.0;}
	else {move = 1.0;}
	cout << "[ DEBUG] [designPathToChangeYaw] Started\n";
	double prog_yaw = currentYaw;
	if(move == 1.0)
	{
		cout << "[ DEBUG] [designPathToChangeYaw] Move clockwise\n";
		while(prog_yaw < desiredYaw)
		{
			prog_yaw += (move * number);
			interm_point.clear();
			interm_point.push_back(curr_point[0]);
			interm_point.push_back(curr_point[1]);
			interm_point.push_back(curr_point[2]);
			// interm_point.push_back(prevYaw*(5-i)/5 + desiredYaw*i/5);
			interm_point.push_back(prog_yaw);
			_interm_path.push_back(interm_point);
		}
	}
	else
	{
		cout << "[ DEBUG] [designPathToChangeYaw] Move counter-clockwise\n";
		while(prog_yaw > desiredYaw)
		{
			prog_yaw += (move * number);
			interm_point.clear();
			interm_point.push_back(curr_point[0]);
			interm_point.push_back(curr_point[1]);
			interm_point.push_back(curr_point[2]);
			// interm_point.push_back(prevYaw*(5-i)/5 + desiredYaw*i/5);
			interm_point.push_back(prog_yaw);
			_interm_path.push_back(interm_point);
		}
	}
	for (unsigned int i = 0; i < _interm_path.size(); ++i)
	{
		cout << "(" << _interm_path[i][0] << ", " << _interm_path[i][1] << ", " 
			<< _interm_path[i][2] << ", " << _interm_path[i][3] << ")\n";
	}
	vector< vector<double> > test_interm_path;
	double d;
	for (unsigned int i = 0; i < _interm_path.size(); ++i)
	{
		if(i == 0)
		{
			d = _interm_path[i][3];
			test_interm_path.push_back(_interm_path[i]);
		}
		else
		{
			if(!(fabs(d-_interm_path[i][3])<=0.01))
			{
				test_interm_path.push_back(_interm_path[i]);
				d = _interm_path[i][3];
			}
			else
			{
				cout << "[ DEBUG] Non Linear: Not pushing this command\n";
			}
		}
	}
	clear2dVector(_interm_path);
	for (unsigned int i = 0; i < test_interm_path.size(); ++i)
	{
		_interm_path.push_back(test_interm_path[i]);
	}
	clear2dVector(test_interm_path);
	cout << "[ DEBUG] [designPathToChangeYaw] Completed\n";
}

/**
 * @brief Align the yaw of the quadcopter to the current plane's (the one which it is seeing) normal
 * @details
 */
void
ControlUINode::alignQuadcopterToCurrentPlane()
{
	if(_node_number_of_planes == 1)
	{
		_next_plane_dir = CLOCKWISE;
		_next_plane_angle = 90.0;
	}
	else
	{
		_next_plane_dir = _node_main_directions.front();
		_next_plane_angle = _node_main_angles.front();
	}
	cout << "[ DEBUG] [alignQuadcopterToCurrentPlane] Started\n";
	getCurrentPositionOfDrone();
	cout << "[ DEBUG] [alignQuadcopterToCurrentPlane] Current Pos of Drone: (" << _node_current_pos_of_drone[0]
		<< ", " << _node_current_pos_of_drone[1] << ", " << _node_current_pos_of_drone[2] << ", " 
		<< _node_current_pos_of_drone[3] << ")\n";
	_node_dest_pos_of_drone.push_back(0.0);
	_node_dest_pos_of_drone.push_back(1.0);
	_node_dest_pos_of_drone.push_back(0.0);
	_node_dest_pos_of_drone.push_back(0.0);
	convertWRTQuadcopterOrigin(_node_current_pos_of_drone, _node_dest_pos_of_drone, _node_ac_dest_pos_of_drone);
	Point3f pYAxis(_node_ac_dest_pos_of_drone[0], _node_ac_dest_pos_of_drone[1], _node_ac_dest_pos_of_drone[2]);
	Point3f pOrigin(_node_current_pos_of_drone[0], _node_current_pos_of_drone[1], _node_current_pos_of_drone[2]);
	Point3f projectedNormal(pYAxis-pOrigin);
	Point3f plane_params(this_plane_parameters[0], this_plane_parameters[1], 0.0);
	cout << "[ DEBUG] [alignQuadcopterToCurrentPlane] pYAxis: " << pYAxis << "\n";
	cout << "[ DEBUG] [alignQuadcopterToCurrentPlane] pOrigin: " << pOrigin << "\n";
	cout << "[ DEBUG] [alignQuadcopterToCurrentPlane] projectedNormal: " << projectedNormal << "\n";
	cout << "[ DEBUG] [alignQuadcopterToCurrentPlane] PP: " << plane_params << "\n";
	float angle = findAngle(projectedNormal, plane_params);
	cout << "[ DEBUG] Angle (radians): " << angle << "\n";
	angle = -angle*180/M_PI;
	cout << "[ DEBUG] Angle to rotate: " << angle << "\n";
	designPathToChangeYaw(_node_current_pos_of_drone, _node_current_pos_of_drone[3]+angle);
	moveDroneViaSetOfPoints(_interm_path);
	cout << "[ DEBUG] [alignQuadcopterToCurrentPlane] Completed\n";
	return ;
}

/**
 * @brief Code for performing module tests on functions written
 * @details Called by pressing a numeric key from DRONE CAMERA FEED Window
 * 	Key 0 - Get the current position of drone
 * 	Key 1 - Changing the yaw of drone
 * 	Key 2 - Translation and rotation of drone
 * 	Key 3 - Aligning the quadcopter to see the left edge of the plane
 * 	Key 4 - Aligning the quadcopter to see the top and bottom edge of the plane
 * 	Key 5 - Capture next part of the same plane
 * 	Key 6 - Move quadcopter to the next unvisited plane
 * 	Key 7 - 
 * 	Key 8 - 
 * 	Key 9 - Align the quadcopter to the current plane
 */
void
ControlUINode::testUtility(int test_no)
{
	cout << "[ INFO] [testUtility] Running test no.: " << test_no << "\n";
	if(test_no == 0)
	{
		getCurrentPositionOfDrone();
		cout << "Current Position of drone: ";
		cout << "(" << _node_current_pos_of_drone[0] << ", " << _node_current_pos_of_drone[1] 
				<< ", " << _node_current_pos_of_drone[2] << ", " << _node_current_pos_of_drone[3] << ")\n";
	}
	else if(test_no == 1)
	{
		cout << "[ INFO] Testing for changing yaw of drone\n";
		_node_number_of_planes = 1;
		_node_min_distance = 3.0;
		_node_max_distance = 5.0;
		getCurrentPositionOfDrone();
		cout << "Current Position of drone: ";
		cout << "(" << _node_current_pos_of_drone[0] << ", " << _node_current_pos_of_drone[1] 
				<< ", " << _node_current_pos_of_drone[2] << ", " << _node_current_pos_of_drone[3] << ")\n";
		designPathToChangeYaw(_node_current_pos_of_drone, 30.0);
		cout << "[ DEBUG] [testUtility] Acquired changeyaw_CS Lock\n";
		linearTraversal = false;
		moveDroneViaSetOfPoints(_interm_path);
		cout << "[ DEBUG] [testUtility] Moved drone changing yaw\n";
	}
	else if(test_no == 2)
	{
		cout << "[ INFO] [testUtility] Testing for linear translation and rotation of drone\n";
		_node_number_of_planes = 1;
		_node_min_distance = 3.0;
		_node_max_distance = 5.0;
		getCurrentPositionOfDrone();
		cout << "[ INFO] [testUtility] Current Position of drone: ";
		cout << "(" << _node_current_pos_of_drone[0] << ", " << _node_current_pos_of_drone[1] 
				<< ", " << _node_current_pos_of_drone[2] << ", " << _node_current_pos_of_drone[3] << ")\n";
		_node_dest_pos_of_drone.clear();
		_node_dest_pos_of_drone.push_back(-2.0);
		_node_dest_pos_of_drone.push_back(-2.5);
		_node_dest_pos_of_drone.push_back(2.0);
		_node_dest_pos_of_drone.push_back(0.0);
		convertWRTQuadcopterOrigin(_node_current_pos_of_drone, _node_dest_pos_of_drone, _node_ac_dest_pos_of_drone);
		cout << "[ INFO] [testUtility] Final Expected Position of drone: ";
		cout << "(" << _node_ac_dest_pos_of_drone[0] << ", " << _node_ac_dest_pos_of_drone[1] 
				<< ", " << _node_ac_dest_pos_of_drone[2] << ", " << _node_ac_dest_pos_of_drone[3] << ")\n";
		designPathForDrone(_node_current_pos_of_drone, _node_ac_dest_pos_of_drone);
		linearTraversal = true;
		moveDroneViaSetOfPoints(_interm_path);
		cout << "[ DEBUG] [testUtility] Moved drone linearly\n";
	}
	else if(test_no == 3)
	{
		cout << "[ INFO] [testUtility] Testing for linear translation to the left edge of the plane\n";
		_node_number_of_planes = 1;
		_node_min_distance = 3.0;
		_node_max_distance = 5.0;
		getCurrentPositionOfDrone();
		cout << "[ INFO] [testUtility] Current Position of drone: ";
		cout << "(" << _node_current_pos_of_drone[0] << ", " << _node_current_pos_of_drone[1] 
				<< ", " << _node_current_pos_of_drone[2] << ", " << _node_current_pos_of_drone[3] << ")\n";
		cout << "[ INFO] [testUtility] Calling JLinkage\n";
		clear2dVector(jlink_all_plane_parameters);
		clear2dVector(jlink_all_continuous_bounding_box_points);
		clear2dVector(jlink_three_d_points);
		jlink_all_percentage_of_each_plane.clear();
		getMultiplePlanes3d(jlink_all_plane_parameters, jlink_all_continuous_bounding_box_points,
						jlink_three_d_points, jlink_all_percentage_of_each_plane);
		int significantPlaneIndex = getCurrentPlaneIndex(visited_plane_parameters, jlink_all_plane_parameters, jlink_all_percentage_of_each_plane);
		copyVector(jlink_all_plane_parameters[significantPlaneIndex], this_plane_parameters);
		copyVector(jlink_all_continuous_bounding_box_points[significantPlaneIndex], this_continuous_bounding_box_points);
		// Render significant plane
		//image_gui->setRender(false, true, true);
		//cout << "[ INFO] [testUtility] Rendering the frames in the DRONE CAMERA FEED GUI\n";
		//image_gui->setSigPlaneBoundingBoxPoints(jlink_all_continuous_bounding_box_points[significantPlaneIndex]);
		//image_gui->renderFrame();
		adjustLeftEdge();
		cout << "[ INFO] [testUtility] Adjusting to left edge completed\n";
	}
	else if(test_no == 4)
	{
		cout << "[ INFO] [testUtility] Testing for linear translation to top and bottom edge of the plane\n";
		_node_number_of_planes = 1;
		_node_min_distance = 3.0;
		_node_max_distance = 5.0;
		getCurrentPositionOfDrone();
		cout << "[ INFO] [testUtility] Current Position of drone: ";
		cout << "(" << _node_current_pos_of_drone[0] << ", " << _node_current_pos_of_drone[1] 
				<< ", " << _node_current_pos_of_drone[2] << ", " << _node_current_pos_of_drone[3] << ")\n";
		cout << "[ INFO] [testUtility] Calling JLinkage\n";
		clear2dVector(jlink_all_plane_parameters);
		clear2dVector(jlink_all_continuous_bounding_box_points);
		clear2dVector(jlink_three_d_points);
		jlink_all_percentage_of_each_plane.clear();
		getMultiplePlanes3d(jlink_all_plane_parameters, jlink_all_continuous_bounding_box_points,
						jlink_three_d_points, jlink_all_percentage_of_each_plane);
		int significantPlaneIndex = getCurrentPlaneIndex(visited_plane_parameters, jlink_all_plane_parameters, jlink_all_percentage_of_each_plane);
		copyVector(jlink_all_plane_parameters[significantPlaneIndex], this_plane_parameters);
		copyVector(jlink_all_continuous_bounding_box_points[significantPlaneIndex], this_continuous_bounding_box_points);
		// Render significant plane
		//image_gui->setRender(false, true, true);
		//cout << "[ INFO] [testUtility] Rendering the frames in the DRONE CAMERA FEED GUI\n";
		//image_gui->setSigPlaneBoundingBoxPoints(jlink_all_continuous_bounding_box_points[significantPlaneIndex]);
		//image_gui->renderFrame();
		adjustTopBottomEdges();
		cout << "[ INFO] [testUtility] Adjusting to top edge completed\n";
	}
	else if(test_no == 5)
	{
		cout << "[ INFO] [testUtility] Testing for capture next part of the same plane\n";
		_node_number_of_planes = 1;
		_node_min_distance = 3.0;
		_node_max_distance = 5.0;
		getCurrentPositionOfDrone();
		cout << "[ INFO] [testUtility] Current Position of drone: ";
		cout << "(" << _node_current_pos_of_drone[0] << ", " << _node_current_pos_of_drone[1] 
				<< ", " << _node_current_pos_of_drone[2] << ", " << _node_current_pos_of_drone[3] << ")\n";
		cout << "[ INFO] [testUtility] Calling JLinkage\n";
		clear2dVector(jlink_all_plane_parameters);
		clear2dVector(jlink_all_continuous_bounding_box_points);
		clear2dVector(jlink_three_d_points);
		jlink_all_percentage_of_each_plane.clear();
		getMultiplePlanes3d(jlink_all_plane_parameters, jlink_all_continuous_bounding_box_points,
						jlink_three_d_points, jlink_all_percentage_of_each_plane);
		int significantPlaneIndex = getCurrentPlaneIndex(visited_plane_parameters, jlink_all_plane_parameters, jlink_all_percentage_of_each_plane);
		copyVector(jlink_all_plane_parameters[significantPlaneIndex], this_plane_parameters);
		copyVector(jlink_all_continuous_bounding_box_points[significantPlaneIndex], this_continuous_bounding_box_points);
		// Render significant plane
		//image_gui->setRender(false, true, true);
		//cout << "[ INFO] [testUtility] Rendering the frames in the DRONE CAMERA FEED GUI\n";
		//image_gui->setSigPlaneBoundingBoxPoints(jlink_all_continuous_bounding_box_points[significantPlaneIndex]);
		//image_gui->renderFrame();
		_next_plane_dir = CLOCKWISE;
		_next_plane_angle = 0.0;
		adjustForNextCapture();
		cout << "[ INFO] [testUtility] Adjusting to capture next part of same plane completed\n";
	}
	else if(test_no == 6)
	{
		cout << "[ INFO] [testUtility] Testing for translation to the next plane\n";
		_node_number_of_planes = 1;
		_node_min_distance = 3.0;
		_node_max_distance = 5.0;
		getCurrentPositionOfDrone();
		cout << "[ INFO] [testUtility] Current Position of drone: ";
		cout << "(" << _node_current_pos_of_drone[0] << ", " << _node_current_pos_of_drone[1] 
				<< ", " << _node_current_pos_of_drone[2] << ", " << _node_current_pos_of_drone[3] << ")\n";
		cout << "[ INFO] [testUtility] Calling JLinkage\n";
		clear2dVector(jlink_all_plane_parameters);
		clear2dVector(jlink_all_continuous_bounding_box_points);
		clear2dVector(jlink_three_d_points);
		jlink_all_percentage_of_each_plane.clear();
		getMultiplePlanes3d(jlink_all_plane_parameters, jlink_all_continuous_bounding_box_points,
						jlink_three_d_points, jlink_all_percentage_of_each_plane);
		int significantPlaneIndex = getCurrentPlaneIndex(visited_plane_parameters, jlink_all_plane_parameters, jlink_all_percentage_of_each_plane);
		copyVector(jlink_all_plane_parameters[significantPlaneIndex], this_plane_parameters);
		copyVector(jlink_all_continuous_bounding_box_points[significantPlaneIndex], this_continuous_bounding_box_points);
		alignQuadcopterToNextPlane();
	}
	else if(test_no == 9)
	{
		cout << "[ INFO] Testing for aligning the quadcopter to the current plane\n";
		_node_number_of_planes = 1;
		_node_min_distance = 3.0;
		_node_max_distance = 5.0;
		getMultiplePlanes3d(jlink_all_plane_parameters, jlink_all_continuous_bounding_box_points,
								jlink_three_d_points, jlink_all_percentage_of_each_plane);
		cout << "[ DEBUG] [testUtility] Get the current plane index: ";
		int planeIndex = getCurrentPlaneIndex(visited_plane_parameters, jlink_all_plane_parameters, jlink_all_percentage_of_each_plane);
		cout << planeIndex << "\n";
		copyVector(jlink_all_plane_parameters[planeIndex], this_plane_parameters);
		copyVector(jlink_all_continuous_bounding_box_points[planeIndex], this_continuous_bounding_box_points);
		alignQuadcopterToCurrentPlane();
		cout << "[ DEBUG] [testUtility] Alignment done\n";
	}
	else
	{
		cout << "[ ERROR] [testUtility] This test is not implemented\n";
	}
	cout << "[ DEBUG] [testUtility] Testing Completed\n";
}

/**
 * @brief Generates the set of points (smoothly distributed) which drone has to follow to move from start to end
 * @details The initial and end points are represented as (x, y, z, yaw)
 * @param [in] [vector< double >] start - Starting position of quadcopter (With yaw)
 * @param [in] [vector< double >] end - Ending position of quadcopter (With yaw)
 * @param [out] [vector< vector<double> >] _interm_path - Intermediate points from start to end (With yaw)
 * @return
 */
void
ControlUINode::designPathForDrone(const vector< double > &start,
					const vector< double > &end)
{
	cout << "[ DEBUG] [designPathForDrone] Started\n";
	clear2dVector(_interm_path);
	// See getInitialPath for help
	vector<double> start_3d_pos, end_3d_pos;
	for (int i = 0; i < 3; ++i)
	{
		start_3d_pos.push_back(start[i]);
		end_3d_pos.push_back(end[i]);
	}
	getInitialPath(start_3d_pos, end_3d_pos, start[3], end[3], _interm_path);
	double a, b, c, d;
	vector< vector<double> > test_interm_path;
	cout << "[ DEBUG] [designPathForDrone] Initial Path Points for drone:\n";
	for (unsigned int i = 0; i < _interm_path.size(); ++i)
	{
		for (unsigned int j = 0; j < _interm_path.size(); ++j)
		{
			cout << _interm_path[i][j] << " ";
		}
		cout << "\n";
	}
	for (unsigned int i = 0; i < _interm_path.size(); ++i)
	{
		if(i == 0)
		{
			a = _interm_path[i][0];
			b = _interm_path[i][1];
			c = _interm_path[i][2];
			d = _interm_path[i][3];
			test_interm_path.push_back(_interm_path[i]);
		}
		else
		{
			if( !( (a - _interm_path[i][0]<=0.0001) &&
					(b - _interm_path[i][1]<=0.0001) &&
					(c - _interm_path[i][2]<=0.0001) &&
					(d - _interm_path[i][3]<=0.0001) ))
			{
				test_interm_path.push_back(_interm_path[i]);
				a = _interm_path[i][0]; b = _interm_path[i][1];
				c = _interm_path[i][2]; d = _interm_path[i][3];
			}
			else
			{
				cout << "[ DEBUG] Linear: Not pushing this command\n";
			}
		}
	}
	clear2dVector(_interm_path);
	for (unsigned int i = 0; i < test_interm_path.size(); ++i)
	{
		_interm_path.push_back(test_interm_path[i]);
	}
	cout << "[ DEBUG] [designPathForDrone] Final Path Points for drone:\n";
	for (unsigned int i = 0; i < _interm_path.size(); ++i)
	{
		for (unsigned int j = 0; j < _interm_path.size(); ++j)
		{
			cout << _interm_path[i][j] << " ";
		}
		cout << "\n";
	}
	clear2dVector(test_interm_path);
	cout << "[ DEBUG] [designPathForDrone] Completed\n";
}

/**
 * @brief Adjust the quadcopter to capture next part of the same plane
 * @details
 */
void
ControlUINode::adjustForNextCapture()
{
	assert(this_plane_parameters.size() == 4);
	assert(this_continuous_bounding_box_points.size() == 5);
	cout << "[ INFO] [adjustForNextCapture] Started\n";
	vector< vector<float> > test_plane_parameters;
	int significantPlaneIndex;
	Point3f top_left = this_continuous_bounding_box_points[0];
	Point3f top_right = this_continuous_bounding_box_points[1];
	double width_of_3d_plane = (double)fabs(sqrt( (top_right.x - top_left.x)*(top_right.x - top_left.x) +
										(top_right.y - top_left.y)*(top_right.y - top_left.y) +
										(top_right.z - top_left.z)*(top_right.z - top_left.z) ));
	cout << "[ INFO] [adjustForNextCapture] Width of the plane is: " << width_of_3d_plane << "\n";
	getCurrentPositionOfDrone();
	_node_dest_pos_of_drone.clear();
	// Direction of next plane
	if(_next_plane_dir == CLOCKWISE)
	{
		cout << "[ INFO] [adjustForNextCapture] Next plane is CLOCKWISE wrt current plane\n";
		cout << "[ INFO] [adjustForNextCapture] Changing the yaw clockwise by " << (_next_plane_angle/2.0) << "\n";
		designPathToChangeYaw(_node_current_pos_of_drone, _node_current_pos_of_drone[3]+(_next_plane_angle/2.0));
		moveDroneViaSetOfPoints(_interm_path);
		cout << "[ INFO] [adjustForNextCapture] Estimating multiple planes -> call to JLinkage\n";
		clear2dVector(jlink_all_plane_parameters);
		clear2dVector(jlink_all_continuous_bounding_box_points);
		clear2dVector(jlink_three_d_points);
		jlink_all_percentage_of_each_plane.clear();
		cout << "[ DEBUG] [adjustForNextCapture] Calling Jlinkage\n";
		getMultiplePlanes3d (jlink_all_plane_parameters, jlink_all_continuous_bounding_box_points, jlink_three_d_points, jlink_all_percentage_of_each_plane);
		significantPlaneIndex = getCurrentPlaneIndex(visited_plane_parameters, jlink_all_plane_parameters, jlink_all_percentage_of_each_plane);
		copyVector(jlink_all_plane_parameters[significantPlaneIndex], this_plane_parameters);
		copyVector(jlink_all_continuous_bounding_box_points[significantPlaneIndex], this_continuous_bounding_box_points);
		clear2dVector(test_plane_parameters);
		// Adding the currently seeing plane to find out if a new plane another than the
		// current one is visible by rotating the drone
		for (unsigned int i = 0; i < jlink_all_plane_parameters.size(); ++i)
		{
			test_plane_parameters.push_back(jlink_all_plane_parameters[i]);
		}
		test_plane_parameters.push_back(this_plane_parameters);
		cout << "[ DEBUG] [adjustForNextCapture] Printing test_plane_parameters:\n";
		for (unsigned int i = 0; i < test_plane_parameters.size(); ++i)
		{
			cout << "[ ";
			for (unsigned int j = 0; j < test_plane_parameters[i].size(); ++j)
			{
				cout << test_plane_parameters[i][j] << " ";
			}
			cout << "]\n";
		}
		cout << "\n";
		_is_plane_covered = isNewPlaneVisible(test_plane_parameters, jlink_all_plane_parameters, jlink_all_percentage_of_each_plane, true, _next_plane_dir);
		cout << "[ DEBUG] [adjustForNextCapture] Is plane covered: " << _is_plane_covered << "\n";
		if(_is_plane_covered)
		{
			getCurrentPositionOfDrone();
			float check_dist = 
					getPointToPlaneDistance(jlink_all_plane_parameters[significantPlaneIndex+1], 
											_node_current_pos_of_drone);
			cout << "[ DEBUG] [adjustForNextCapture] Check_dist: " << check_dist << ", Max. Dist: " << _node_max_distance << "\n";
			if(check_dist > _node_max_distance)
			{
				_is_plane_covered = false;
			}
		}
		cout << "[ DEBUG] [adjustForNextCapture] Is plane covered: " << _is_plane_covered << "\n";
		if(_is_plane_covered)
		{
			_is_plane_covered = true;
			this_plane_parameters.clear();
			this_continuous_bounding_box_points.clear();
			if(!_is_big_plane)
			{
				cout << "[ INFO] [adjustForNextCapture] Not a big plane. Covered in one go. Copying the necessary information\n";
				visited_plane_parameters.push_back(this_plane_parameters);
				visited_continuous_bounding_box_points.push_back(this_continuous_bounding_box_points);
			}
			else
			{
				cout << "[ INFO] [adjustForNextCapture] A big plane. Could not cover in one go. Estimating the best plane\n";
				vector<float> new_plane_params = bestFitPlane(aug_three_d_points);
				projectPointsOnPlane(aug_plane_bounding_box_points, new_plane_params, this_continuous_bounding_box_points);
				visited_plane_parameters.push_back(new_plane_params);
				visited_continuous_bounding_box_points.push_back(this_continuous_bounding_box_points);
			}
		}
		else
		{
			// _is_plane_covered = false;
			cout << "[ INFO] [adjustForNextCapture] Restoring the yaw back and moving by width_of_plane by 2\n";
			_is_big_plane = true;
			getCurrentPositionOfDrone();
			_node_dest_pos_of_drone.clear();
			_node_dest_pos_of_drone.push_back(width_of_3d_plane/(double)2.0);
			_node_dest_pos_of_drone.push_back(0.0);
			_node_dest_pos_of_drone.push_back(0.0);
			_node_dest_pos_of_drone.push_back(-(_next_plane_angle/2.0));
			convertWRTQuadcopterOrigin(_node_current_pos_of_drone, _node_dest_pos_of_drone, _node_ac_dest_pos_of_drone);
			designPathForDrone(_node_current_pos_of_drone, _node_ac_dest_pos_of_drone);
			moveDroneViaSetOfPoints(_interm_path);
		}
	}
	else if(_next_plane_dir == COUNTERCLOCKWISE)
	{
		getCurrentPositionOfDrone();
		_node_dest_pos_of_drone.clear();
		_node_dest_pos_of_drone.push_back(width_of_3d_plane);
		_node_dest_pos_of_drone.push_back(0.0);
		_node_dest_pos_of_drone.push_back(0.0);
		_node_dest_pos_of_drone.push_back(0.0);
		convertWRTQuadcopterOrigin(_node_current_pos_of_drone, _node_dest_pos_of_drone, _node_ac_dest_pos_of_drone);
		designPathForDrone(_node_current_pos_of_drone, _node_ac_dest_pos_of_drone);
		moveDroneViaSetOfPoints(_interm_path);
		cout << "[ INFO] [adjustForNextCapture] Changing the yaw counterclockwise by " << (_next_plane_angle/2.0) << "\n";
		designPathToChangeYaw(_node_current_pos_of_drone, _node_current_pos_of_drone[3]-(_next_plane_angle/2.0));
		moveDroneViaSetOfPoints(_interm_path);
		cout << "[ DEBUG] [adjustForNextCapture] Calling Jlinkage\n";
		clear2dVector(jlink_all_plane_parameters);
		clear2dVector(jlink_all_continuous_bounding_box_points);
		clear2dVector(jlink_three_d_points);
		jlink_all_percentage_of_each_plane.clear();
		getMultiplePlanes3d (jlink_all_plane_parameters, jlink_all_continuous_bounding_box_points, jlink_three_d_points, jlink_all_percentage_of_each_plane);
		significantPlaneIndex = getCurrentPlaneIndex(visited_plane_parameters, jlink_all_plane_parameters, jlink_all_percentage_of_each_plane);
		copyVector(jlink_all_plane_parameters[significantPlaneIndex], this_plane_parameters);
		copyVector(jlink_all_continuous_bounding_box_points[significantPlaneIndex], this_continuous_bounding_box_points);
		clear2dVector(test_plane_parameters);
		for (unsigned int i = 0; i < jlink_all_plane_parameters.size(); ++i)
		{
			test_plane_parameters.push_back(jlink_all_plane_parameters[i]);
		}
		test_plane_parameters.push_back(this_plane_parameters);
		cout << "[ DEBUG] [adjustForNextCapture] Printing test_plane_parameters:\n";
		for (unsigned int i = 0; i < test_plane_parameters.size(); ++i)
		{
			cout << "[ ";
			for (unsigned int j = 0; j < test_plane_parameters[i].size(); ++j)
			{
				cout << test_plane_parameters[i][j] << " ";
			}
			cout << "]\n";
		}
		cout << "\n";
		_is_plane_covered = isNewPlaneVisible(test_plane_parameters, jlink_all_plane_parameters, jlink_all_percentage_of_each_plane, true, _next_plane_dir);
		if(_is_plane_covered)
		{
			_is_plane_covered = true;
			this_plane_parameters.clear();
			this_continuous_bounding_box_points.clear();
			if(!_is_big_plane)
			{
				cout << "[ INFO] [adjustForNextCapture] Not a big plane. Covered in one go. Copying the necessary information\n";
				visited_plane_parameters.push_back(this_plane_parameters);
				visited_continuous_bounding_box_points.push_back(this_continuous_bounding_box_points);
			}
			else
			{
				cout << "[ INFO] [adjustForNextCapture] A big plane. Could not cover in one go. Estimating the best plane\n";
				vector<float> new_plane_params = bestFitPlane(aug_three_d_points);
				projectPointsOnPlane(aug_plane_bounding_box_points, new_plane_params, this_continuous_bounding_box_points);
				visited_plane_parameters.push_back(new_plane_params);
				visited_continuous_bounding_box_points.push_back(this_continuous_bounding_box_points);
			}
		}
		else
		{
			// _is_plane_covered = false;
			cout << "[ INFO] [adjustForNextCapture] Restoring the yaw back\n";
			_is_big_plane = true;
			getCurrentPositionOfDrone();
			designPathToChangeYaw(_node_current_pos_of_drone, _node_current_pos_of_drone[3]+(_next_plane_angle/2.0));
			moveDroneViaSetOfPoints(_interm_path);
		}
	}
	cout << "[ INFO] [adjustForNextCapture] Completed\n";
}

/**
 * @brief Align the quadcopter to the next plane
 * @details
 */
void
ControlUINode::alignQuadcopterToNextPlane()
{
	getCurrentPositionOfDrone();
	if(_next_plane_dir == CLOCKWISE)
	{
		// [MGP] -ve anti-clockwise , +ve clockwise
	}
	else if(_next_plane_dir == COUNTERCLOCKWISE)
	{
		getMultiplePlanes3d (jlink_all_plane_parameters, jlink_all_continuous_bounding_box_points, jlink_three_d_points, jlink_all_percentage_of_each_plane);
		int planeIndex = getCurrentPlaneIndex(visited_plane_parameters, jlink_all_plane_parameters, jlink_all_percentage_of_each_plane);
		bool move_drone = false;
		if(planeIndex == -2)
		{
			planeIndex = 0;
			move_drone = true;
		}
		else if(planeIndex == -1)
		{
			cout << "[ DEBUG] [alignQuadcopterToNextPlane] Seems I can't see a new plane\n";
		}
		else
		{
			move_drone = true;
		}
		copyVector(jlink_all_plane_parameters[planeIndex], this_plane_parameters);
		copyVector(jlink_all_continuous_bounding_box_points[planeIndex], this_continuous_bounding_box_points);
		if(move_drone)
		{
			/* @todo-me Check do we require this or planeIndex would be OK?
			float a = jlink_all_plane_parameters[planeIndex+1][0];
			float b = jlink_all_plane_parameters[planeIndex+1][1];
			float c = jlink_all_plane_parameters[planeIndex+1][2];
			for (int i = 0; i < planeParameters.size(); ++i)
			{
				float dot_p = planeParameters[i][0]*a + planeParameters[i][1]*b + planeParameters[i][2]*c;
				if(dot_p >= 0.984)
				{
					last_plane_index = i; break;
				}
			}*/
			Point3f bl = jlink_all_continuous_bounding_box_points[planeIndex][3];
			Point3f br = jlink_all_continuous_bounding_box_points[planeIndex][2];
			Point3f tr = jlink_all_continuous_bounding_box_points[planeIndex][1];
			Point3f tl = jlink_all_continuous_bounding_box_points[planeIndex][0];
			Point3f mid = (br+tr);
			mid.x = mid.x/(float)2.0;
			mid.y = mid.y/(float)2.0;
			mid.z = mid.z/(float)2.0;
			// @todo-me Check whether to include d or not?
			Point3f normal_old_plane(jlink_all_plane_parameters[planeIndex-1][0],
										jlink_all_plane_parameters[planeIndex-1][1], 
										jlink_all_plane_parameters[planeIndex-1][2]);
			Point3f normal_new_plane(jlink_all_plane_parameters[planeIndex][0],
										jlink_all_plane_parameters[planeIndex][1], 
										jlink_all_plane_parameters[planeIndex][2]);
			float mag_normal_new_plane = sqrt(normal_new_plane.x*normal_new_plane.x +
								normal_new_plane.y*normal_new_plane.y + normal_new_plane.z*normal_new_plane.z);
			getCurrentPositionOfDrone();
			float drone_distance = getPointToPlaneDistance(jlink_all_plane_parameters[planeIndex-1], _node_current_pos_of_drone);
			float t = drone_distance/mag_normal_new_plane;
			//double angle = normal_old_plane.dot(normal_new_plane);
			_node_ac_dest_pos_of_drone.clear();
			_node_ac_dest_pos_of_drone.push_back(mid.x + t*normal_new_plane.x);
			_node_ac_dest_pos_of_drone.push_back(mid.y + t*normal_new_plane.y);
			_node_ac_dest_pos_of_drone.push_back(mid.z + t*normal_new_plane.z);
			Point3f pYAxis(_node_ac_dest_pos_of_drone[0], _node_ac_dest_pos_of_drone[1], _node_ac_dest_pos_of_drone[2]);
			Point3f pOrigin(_node_current_pos_of_drone[0], _node_current_pos_of_drone[1], _node_current_pos_of_drone[2]);
			Point3f projectedNormal(pYAxis-pOrigin);
			double angle = (double)findAngle(projectedNormal, normal_new_plane);
			_node_ac_dest_pos_of_drone.push_back(_node_current_pos_of_drone[3]-angle);
			designPathForDrone(_node_current_pos_of_drone, _node_ac_dest_pos_of_drone);
			moveDroneViaSetOfPoints(_interm_path);
			double width_of_3d_plane = sqrt(pow(bl.x-br.x,2)+pow(bl.y-br.y,2)+pow(bl.z-br.z,2));
			getCurrentPositionOfDrone();
			_node_dest_pos_of_drone.clear();
			_node_dest_pos_of_drone.push_back(0.0);
			_node_dest_pos_of_drone.push_back(0.0);
			_node_dest_pos_of_drone.push_back((width_of_3d_plane)/2);
			_node_dest_pos_of_drone.push_back(0.0);
			convertWRTQuadcopterOrigin(_node_current_pos_of_drone, _node_dest_pos_of_drone, _node_ac_dest_pos_of_drone);
			designPathForDrone(_node_current_pos_of_drone, _node_ac_dest_pos_of_drone);
			moveDroneViaSetOfPoints(_interm_path);
		}
	}
	_node_completed_number_of_planes++;
	_node_main_directions.pop_front();
	_node_main_angles.pop_front();
	return ;
}

/**
 * @brief
 * @details To be implemented if alignQuadcopterToNextPlane() doesnot work as expected
 */
void
ControlUINode::alignQuadcopterToNextPlaneAdvanced()
{
	getCurrentPositionOfDrone();
	if(_next_plane_dir == CLOCKWISE)
	{
		// [MGP] -ve anti-clockwise , +ve clockwise
	}
	else if(_next_plane_dir == COUNTERCLOCKWISE)
	{
		getMultiplePlanes3d (jlink_all_plane_parameters, jlink_all_continuous_bounding_box_points, jlink_three_d_points, jlink_all_percentage_of_each_plane);
		int planeIndex = getCurrentPlaneIndex(visited_plane_parameters, jlink_all_plane_parameters, jlink_all_percentage_of_each_plane);
	}
	return ;
}

/**
 * @brief Adjust the quadcopter to see the top and bottom edge of the current plane
 * @details
 */
void
ControlUINode::adjustTopBottomEdges()
{
	float mind = _node_min_distance;
	float maxd = _node_max_distance;
	float step_distance = fabs(_node_max_distance - _node_min_distance)/3.0;
	bool planeTopBottomVisible;
	float point_distance, height;
	height = getHeightFromGround(this_plane_parameters, this_continuous_bounding_box_points, _node_current_pos_of_drone);
	cout << "[DEBUG] [adjustTopBottomEdges] Step Distance: " << step_distance << "\n";
	int move = checkVisibility(this_plane_parameters, this_continuous_bounding_box_points, 0);
	if(move==0)
	{
		planeTopBottomVisible = true;
		cout << "[ DEBUG] [adjustTopBottomEdges] Need not move\n";
	}
	else 
	{
		planeTopBottomVisible = false;
		if(move==1)
		{cout << "[ DEBUG] [adjustTopBottomEdges] Need to move forward\n";}
		else
		{cout << "[ DEBUG] [adjustTopBottomEdges] Need to move backward\n";}
	}
	cout << "Move: " << move << ", Step Distance: " << step_distance << "\n";
	cout << "[ DEBUG] In while loop\n";
	while(!planeTopBottomVisible && mind < maxd)
	{
		getCurrentPositionOfDrone();
		point_distance = getPointToPlaneDistance(this_plane_parameters, _node_current_pos_of_drone);
		cout << "[ DEBUG] [adjustTopBottomEdges] Distance from the plane: " << point_distance << "\n";
		cout << "[ DEBUG] [adjustTopBottomEdges] Current Pos of Drone: (" << _node_current_pos_of_drone[0]
			<< ", " << _node_current_pos_of_drone[1] << ", " << _node_current_pos_of_drone[2] << ", " 
			<< _node_current_pos_of_drone[3] << ")\n";
		double to_move = (double)move * (double)step_distance;
		cout << "Move: " << move << ", Step Distance: " << step_distance << "\n";
		_node_dest_pos_of_drone.clear();
		_node_dest_pos_of_drone.push_back(0.0);
		_node_dest_pos_of_drone.push_back(to_move);
		_node_dest_pos_of_drone.push_back(0.0);
		_node_dest_pos_of_drone.push_back(0.0);
		cout << "[ DEBUG] Dest Pos of Drone: (" << _node_dest_pos_of_drone[0]
			<< ", " << _node_dest_pos_of_drone[1] << ", " << _node_dest_pos_of_drone[2] << ", " 
			<< _node_dest_pos_of_drone[3] << ")\n";
		convertWRTQuadcopterOrigin(_node_current_pos_of_drone, _node_dest_pos_of_drone, _node_ac_dest_pos_of_drone);
		height = getHeightFromGround(this_plane_parameters, this_continuous_bounding_box_points, _node_current_pos_of_drone);
		_node_ac_dest_pos_of_drone[2] = height;
		cout << "[ DEBUG] Actual Pos of Drone To move: (" << _node_ac_dest_pos_of_drone[0]
			<< ", " << _node_ac_dest_pos_of_drone[1] << ", " << _node_ac_dest_pos_of_drone[2] << ", " 
			<< _node_ac_dest_pos_of_drone[3] << ")\n";
		designPathForDrone(_node_current_pos_of_drone, _node_ac_dest_pos_of_drone);
		moveDroneViaSetOfPoints(_interm_path);
		clear2dVector(jlink_all_plane_parameters);
		clear2dVector(jlink_all_continuous_bounding_box_points);
		clear2dVector(jlink_three_d_points);
		jlink_all_percentage_of_each_plane.clear();
		getMultiplePlanes3d (jlink_all_plane_parameters, jlink_all_continuous_bounding_box_points, jlink_three_d_points, jlink_all_percentage_of_each_plane);
		planeIndex = getCurrentPlaneIndex(visited_plane_parameters, jlink_all_plane_parameters, jlink_all_percentage_of_each_plane);
		copyVector(jlink_all_plane_parameters[planeIndex], this_plane_parameters);
		copyVector(jlink_all_continuous_bounding_box_points[planeIndex], this_continuous_bounding_box_points);
		point_distance = getPointToPlaneDistance(this_plane_parameters, _node_current_pos_of_drone);
		cout << "[ DEBUG] Distance from the plane: " << point_distance << "\n";
		move = checkVisibility(this_plane_parameters, this_continuous_bounding_box_points, 0);
		to_move = (double)move * (double)step_distance;
		if(move==0)
		{
			planeTopBottomVisible = true; cout << "[ DEBUG] Need not move\n";
		}
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
	cout << "[ DEBUG] [adjustTopBottomEdges] Adjusting top and bottom done.\n";
	return ;
}

/**
 * @brief Adjust the quadcopter to see the left edge of the current plane
 * @details
 */
void
ControlUINode::adjustLeftEdge()
{
	cout << "[ DEBUG] [adjustLeftEdge] Started\n";
	bool planeLeftVisible;
	int planeIndex;
	int move = checkVisibility(this_plane_parameters, this_continuous_bounding_box_points, 1);
	if(move==0)
	{
		planeLeftVisible = true;
	}
	else
	{
		planeLeftVisible = false;
	}
	float step_distance = 0.5; // @todo-me Fix this heuristic
	while(!planeLeftVisible)
	{
		getCurrentPositionOfDrone();
		_node_dest_pos_of_drone.clear();
		_node_dest_pos_of_drone.push_back(move*step_distance);
		_node_dest_pos_of_drone.push_back(0.0);
		_node_dest_pos_of_drone.push_back(0.0);
		_node_dest_pos_of_drone.push_back(0.0);
		convertWRTQuadcopterOrigin(_node_current_pos_of_drone, _node_dest_pos_of_drone, _node_ac_dest_pos_of_drone);
		designPathForDrone(_node_current_pos_of_drone, _node_ac_dest_pos_of_drone);
		moveDroneViaSetOfPoints(_interm_path);
		clear2dVector(jlink_all_plane_parameters);
		clear2dVector(jlink_all_continuous_bounding_box_points);
		clear2dVector(jlink_three_d_points);
		jlink_all_percentage_of_each_plane.clear();
		getMultiplePlanes3d (jlink_all_plane_parameters, jlink_all_continuous_bounding_box_points, jlink_three_d_points, jlink_all_percentage_of_each_plane);
		planeIndex = getCurrentPlaneIndex(visited_plane_parameters, jlink_all_plane_parameters, jlink_all_percentage_of_each_plane);
		cout << "[ DEBUG] [adjustLeftEdge] Plane Index: " << planeIndex << "\n";
		copyVector(jlink_all_plane_parameters[planeIndex], this_plane_parameters);
		copyVector(jlink_all_continuous_bounding_box_points[planeIndex], this_continuous_bounding_box_points);
		move = checkVisibility(this_plane_parameters, this_continuous_bounding_box_points, 1);
		cout << "[ DEBUG] [adjustLeftEdge] Move: " << move << "\n";
		if(move==0) {planeLeftVisible = true;}
		else {planeLeftVisible = false;}
	}
	/*if(planeIndex < (int)jlink_all_plane_parameters.size()-1)
	{
		_is_able_to_see_new_plane = true; // Because I could see a new plane
	}
	else
	{
		_is_able_to_see_new_plane = false;
	}*/
	cout << "[ DEBUG] [adjustLeftEdge] Started\n";
}

/**
 * @brief Helper functions which derives if it can see the top, bottom, left edge
 *			depending on which_side variable
 * @details 
 * 		which_side 0 - Top and Bottom edge
 * 		which_side 1 - Left edge
 */
int
ControlUINode::checkVisibility(const vector<float> &plane_parameters, 
								const vector<Point3f> &continuous_bounding_box_points,
								bool which_side)
{
	cout << "[ DEBUG] [checkVisibility] Started\n";
	int move = 0;
	vector<Point2f> image_bounding_box_points;
	cout << "[ DEBUG] [checkVisibility] Plane Parameters: ";
	for (unsigned int i = 0; i < plane_parameters.size(); ++i)
	{
		cout << plane_parameters[i] << " ";
	}
	cout << "\n";
	cout << "[ DEBUG] [checkVisibility] CBB Points:";
	for (unsigned int i = 0; i < continuous_bounding_box_points.size(); ++i)
	{
		cout << continuous_bounding_box_points[i] << "\n";
	}
	cout << "\n";
	if(which_side == 0) // Top and Bottom edge
	{
		image_bounding_box_points.clear();
		project3DPointsOnImage(continuous_bounding_box_points, image_bounding_box_points);
		cout << "[ DEBUG] [checkVisibility] IBB Points:";
		for (unsigned int i = 0; i < image_bounding_box_points.size(); ++i)
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
		cout << "[ DEBUG] [checkVisibility] Checking for left edge\n";
		image_bounding_box_points.clear();
		project3DPointsOnImage(continuous_bounding_box_points, image_bounding_box_points);
		cout << "[ DEBUG] [checkVisibility] IBB Points:";
		for (unsigned int i = 0; i < image_bounding_box_points.size(); ++i)
		{
			cout << image_bounding_box_points[i] << "\n";
		}
		cout << "\n";
		Line2f left_edge(image_bounding_box_points[0], image_bounding_box_points[3]);
		cout << "Left Edge: " << left_edge << "\n";
		// @todo-me Fix this heuristic
		if( (left_edge.start.x >= 128.0 && left_edge.start.x <= 256.0) ||
			//(left_edge.start.y >= 144.0 && left_edge.start.x <= 216.0) &&
			(left_edge.end.x >= 128.0 && left_edge.end.x <= 256.0) ) /*&&
			(left_edge.end.y >= 216.0 && left_edge.end.y <= 288.0)  )*/
		{
			move = 0;
		}
		else if( (left_edge.start.x >= 0.0 && left_edge.start.x <= 128.0) ||
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
		cout << "[ DEBUG] [checkVisibility] Currently not dealing with it\n";
	}
	cout << "[ DEBUG] [checkVisibility] Completed\n";
	return move;
}
