/*****************************************************************************************
 * ControlUINode.cpp
 *
 *       Created on: 19-Feb-2015
 *    Last Modified: 13-Oct-2016
 *  Original Author: Anirudh Vemula
 *   Current Author: Meghshyam Govind Prasad
 *   Current Author: Sona Praneeth Akula
 *          Project: Quadcopter
 *      Description:
 *
 * Date				Author							Modification
 * 12-Sep-2016	Sona Praneeth Akula			* Added comments to the code
 * 21-Sep-2016	Sona Praneeth Akula			* Added code to land the quadcopter
 * 12-Oct-2016	Sona Praneeth Akula			* 
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
	//
	_node_number_of_planes = 0;
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
	_next_plane_dir = CLOCKWISE;
	_next_plane_angle = 0.0;
	_plane_d = 0.0;
	_step_distance = 0.5;
	_fixed_distance = _node_max_distance;
	_fixed_height = 0.7;
	_fixed_height_set = false;
	_is_adjusted = false;
	_move_heuristic = 0.5;
	_angle_heuristic = 4.0;
	_sig_plane_index = 0;
	_actual_plane_index = 0;
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
		cout << "[ DEBUG] [newPoseCb] Released first changeyaw_CS Lock\n";
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
		printf("[ INFO] [newPoseCb] (%u) Just Navigation Current target %u: %lf %lf %lf %lf\n", 
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
		//double ea = sqrt(pow(x - x_drone, 2) + pow(y - y_drone, 2) + pow(z - z_drone, 2) + pow(ya - yaw, 2));
		//double ea = fabs(fabs(x-x_drone)+fabs(y-y_drone)+fabs(z-z_drone)+fabs(ya-yaw));
		/*cout << "[ DEBUG] [poseCb] Current Pose: " << x_drone << ", " << y_drone << ", " << z_drone << ", " << yaw << "\n";
		cout << "[ DEBUG] [poseCb] " << fabs(x-x_drone) << ", " << fabs(y-y_drone) << ", "
					<< fabs(z-z_drone) << ", " << fabs(ya-yaw) << "\n";*/
		//printf("[ DEBUG] [poseCb] Error %lf\n", ea);
		getCurrentPositionOfDrone();
		/*print1dVector(_node_current_pos_of_drone, "[ DEBUG] [newPoseCb] Current position of drone");
		cout << " [DEBUG] [newPoseCb] x: " << x << ", y: " << y << ", z: " << z << "\n";
		cout << "[ DEBUG] [newPoseCb] Error: " << fabs(x-x_drone) << ", " << fabs(y-y_drone) << ", "
					<< fabs(z-z_drone) << ", " << fabs(ya-yaw) << "\n";*/
		pthread_mutex_unlock(&pose_CS);
		bool var1 = (fabs(x-x_drone) < 0.08);
		bool var2 = (fabs(y-y_drone) < 0.08);
		bool var3 = (fabs(z-z_drone) < 0.08);
		bool var4 = (fabs(ya-yaw) < 0.3);
		if( var1 & var2 & var3 & var4 )
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
				cout << "[ DEBUG] [newPoseCb] Released elseif command_CS Lock\n";
				getCurrentPositionOfDrone();
				print1dVector(_node_current_pos_of_drone, "[ DEBUG] [newPoseCb] Current position of drone after command");
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

void
ControlUINode::copyNecessaryInfo()
{
	if(!_is_big_plane)
	{
		cout << "[ INFO] [copyNecessaryInfo] Not a big plane. Covered in one go. Copying the necessary information\n";
		visited_plane_parameters.push_back(this_plane_parameters);
		visited_continuous_bounding_box_points.push_back(this_continuous_bounding_box_points);
	}
	else
	{
		cout << "[ INFO] [copyNecessaryInfo] A big plane. Could not cover in one go. Estimating the best plane\n";
		vector<float> new_plane_params;
		new_plane_params.clear();
		fitPlane3D(aug_three_d_points, new_plane_params);
		print1dVector(new_plane_params, "[ DEBUG] [copyNecessaryInfo] New plane parameters");
		getCurrentPositionOfDrone();
		checkPlaneParametersSign(_node_current_pos_of_drone, aug_three_d_points, new_plane_params);
		//new_plane_params[3] = _plane_d/(float)_plane_d_num;
		vector<Point3f> new_bounding_box_points;
		new_bounding_box_points.clear();
		print1dVector(aug_plane_bounding_box_points, "[ DEBUG] [copyNecessaryInfo] Aug CBB");
		cout << "[ DEBUG] [copyNecessaryInfo] Calculating the bounding box points\n";
		projectPointsOnPlane(aug_plane_bounding_box_points, new_plane_params, new_bounding_box_points);
		print1dVector(new_bounding_box_points, "[ DEBUG] [copyNecessaryInfo] New CBB");
		cout << "[ INFO] [copyNecessaryInfo] Copying the necessary information\n";
		visited_plane_parameters.push_back(new_plane_params);
		visited_continuous_bounding_box_points.push_back(new_bounding_box_points);
		aug_three_d_points.clear();
		aug_plane_bounding_box_points.clear();
		new_bounding_box_points.clear();
	}
	/*this_plane_parameters.clear();
	this_continuous_bounding_box_points.clear();*/
	print2dVector(visited_plane_parameters, "[ INFO] [copyNecessaryInfo] Visited Plane Parameters");
	print2dVector(visited_continuous_bounding_box_points, "[ INFO] [copyNecessaryInfo] Visited CBB");
	_is_big_plane = false;
	_stage_of_plane_observation = true;
	_node_completed_number_of_planes++;
	_plane_d = 0.0;
	_plane_d_num = 0;
	return ;
}

void
ControlUINode::augmentInfo()
{
	cout << "[ INFO] [augmentInfo] Adding the 3d points to be used later for best fit\n";
	int index = _actual_plane_index;
	cout << "[ DEBUG] [augmentInfo] Index: " << index << ", Actual Plane Index: " << _actual_plane_index 
				<< ", SigPlaneIndex: " << _sig_plane_index << "\n";
	//if(index == -1) {index  = 0;}
	for (unsigned int i = 0; i < jlink_three_d_points[index].size(); ++i)
	{
		aug_three_d_points.push_back(jlink_three_d_points[index][i]);
	}
	cout << "[ INFO] [augmentInfo] Stage of plane observation: " << _stage_of_plane_observation << "\n";
	if(_stage_of_plane_observation)
	{
		for (unsigned int i = 0; i < this_continuous_bounding_box_points.size(); ++i)
		{
			aug_plane_bounding_box_points.push_back(this_continuous_bounding_box_points[i]);
		}
		_stage_of_plane_observation = false;
	}
	else
	{
		Point3f a0, a1, a2, a3;
		a0 = aug_plane_bounding_box_points[0];
		a1 = this_continuous_bounding_box_points[1];
		a2 = this_continuous_bounding_box_points[2];
		a3 = aug_plane_bounding_box_points[3];
		aug_plane_bounding_box_points.clear();
		aug_plane_bounding_box_points.push_back(a0);
		aug_plane_bounding_box_points.push_back(a1);
		aug_plane_bounding_box_points.push_back(a2);
		aug_plane_bounding_box_points.push_back(a3);
		aug_plane_bounding_box_points.push_back(a0);
		print1dVector(aug_plane_bounding_box_points, "[ DEBUG] [augmentInfo] Aug. Plane BB");
	}
	return ;
}

/**
 * @brief Gets the current position of the drone in _node_current_pos_of_drone (See private variables)
 * @details Returns the position of the drone when the function is called
 */
void
ControlUINode::getCurrentPositionOfDrone()
{
	_node_current_pos_of_drone.clear();
	//pthread_mutex_lock(&pose_CS);
		_node_current_pos_of_drone.push_back((double)x_drone);
		_node_current_pos_of_drone.push_back((double)y_drone);
		_node_current_pos_of_drone.push_back((double)z_drone);
		_node_current_pos_of_drone.push_back((double)yaw);
	//pthread_mutex_unlock(&pose_CS);
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
	//pthread_mutex_lock(&pose_CS);
		current_drone_pos.push_back((double)x_drone);
		current_drone_pos.push_back((double)y_drone);
		current_drone_pos.push_back((double)z_drone);
		current_drone_pos.push_back((double)yaw);
	//pthread_mutex_unlock(&pose_CS);
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
	vector< vector<Point3f> > in_points;
	vector< vector<float> > in_pp;
	vector< vector<Point3f> > in_cbb;
	vector<float> in_p;
	findPercBoundEachPlane(_in_points, in_pp, in_cbb, in_points, in_p);
	getCurrentPositionOfDrone();
	cout << "[ DEBUG] [getMultiplePlanes3d] Fixing plane orientation and CBB\n";
	orderPlanesFromQuadcopterPosition(_node_current_pos_of_drone, 
									in_points, in_pp, in_cbb, in_p,
									sorted_3d_points, planeParameters, continuousBoundingBoxPoints, percentageOfEachPlane);
	clear2dVector(in_points);
	clear2dVector(in_pp);
	clear2dVector(in_cbb);
	in_p.clear();
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
	vector< vector<Point3f> > in_points;
	vector< vector<float> > in_pp;
	vector< vector<Point3f> > in_cbb;
	vector<float> in_p;
	findPercBoundEachPlane(_in_points, in_pp, in_cbb, in_points, in_p);
	getCurrentPositionOfDrone();
	cout << "[ DEBUG] [getMultiplePlanes3d] Fixing plane orientation and CBB\n";
	orderPlanesFromQuadcopterPosition(_node_current_pos_of_drone, 
									in_points, in_pp, in_cbb, in_p,
									sorted_3d_points, planeParameters, continuousBoundingBoxPoints, percentageOfEachPlane);
	clear2dVector(in_points);
	clear2dVector(in_pp);
	clear2dVector(in_cbb);
	in_p.clear();
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
	print2dVector(dest_points, "[ DEBUG] [moveDroneViaSetOfPoints] Moving points");
	for (unsigned int i = 0; i < dest_points.size(); ++i)
	{
		snprintf(buf, 100, "c goto %lf %lf %lf %lf",
			dest_points[i][0], dest_points[i][1], dest_points[i][2], dest_points[i][3]);
		visited_motion_points.push_back(dest_points[i]);
		std_msgs::String s;
		s.data = buf;
		just_navigation_commands.push_back(s);
		targetPoints.push_back(dest_points[i]);
	}
	cout << "[ DEBUG] [moveDroneViaSetOfPoints] Commands Ready\n";
	cout << "[ DEBUG] [moveDroneViaSetOfPoints] Commands to execute: " << just_navigation_commands.size() << "\n";
	if(just_navigation_commands.size() == 0)
	{
		changeyawLockReleased = -1;
	}
	pthread_mutex_lock(&changeyaw_CS);
	cout << "[ DEBUG] [moveDroneViaSetOfPoints] Acquired second changeyaw_CS Lock\n";
	pthread_mutex_unlock(&changeyaw_CS);
	getCurrentPositionOfDrone();
	print1dVector(_node_current_pos_of_drone, "[ DEBUG] [moveDroneViaSetOfPoints] Current position of drone after movement");
	cout << "[ DEBUG] [moveDroneViaSetOfPoints] Released second changeyaw_CS Lock\n";
	just_navigation_total_commands = -1;
	just_navigation_command_number = -1;
}

void
ControlUINode::rotateClockwise(double step_angle)
{
	cout << "[ DEBUG] [rotateClockwise] Started\n";
	getCurrentPositionOfDrone();
	designPathToChangeYaw(_node_current_pos_of_drone, _node_current_pos_of_drone[3]+step_angle);
	/*double current_angle = _node_current_pos_of_drone[3];
	double dest_angle = _node_current_pos_of_drone[3]+step_angle;
	vector<double> interm_point;
	bool to_rotate = true;
	double move = 1.0;
	while(to_rotate)
	{
		if(fabs(dest_angle - current_angle) < _angle_heuristic)
		{
			current_angle = dest_angle;
			to_rotate = false;
		}
		else
		{
			current_angle += (move * (double)_angle_heuristic);
			if(current_angle >= 180.0)
			{
				current_angle = -360.0+current_angle;
				dest_angle = -360.0+dest_angle;
			}
			to_rotate = true;
		}
		interm_point.clear();
		interm_point.push_back(_node_current_pos_of_drone[0]);
		interm_point.push_back(_node_current_pos_of_drone[1]);
		interm_point.push_back(_node_current_pos_of_drone[2]);
		interm_point.push_back(current_angle);
		_interm_path.push_back(interm_point);
	}*/
	moveDroneViaSetOfPoints(_interm_path);
	cout << "[ DEBUG] [rotateClockwise] Completed\n";
	return ;
}

void
ControlUINode::rotateCounterClockwise(double step_angle)
{
	cout << "[ DEBUG] [rotateCounterClockwise] Started\n";
	clear2dVector(_interm_path);
	getCurrentPositionOfDrone();
	designPathToChangeYaw(_node_current_pos_of_drone, _node_current_pos_of_drone[3]-step_angle);
	/*double current_angle = _node_current_pos_of_drone[3];
	double dest_angle = _node_current_pos_of_drone[3]-step_angle;
	vector<double> interm_point;
	bool to_rotate = true;
	double move = -1.0;
	while(to_rotate)
	{
		if(fabs(dest_angle - current_angle) < _angle_heuristic)
		{
			current_angle = dest_angle;
			to_rotate = false;
		}
		else
		{
			current_angle += (move * (double)_angle_heuristic);
			if(current_angle < -180.0)
			{
				current_angle = 360.0+current_angle;
				dest_angle = 360.0+dest_angle;
			}
			to_rotate = true;
		}
		interm_point.clear();
		interm_point.push_back(_node_current_pos_of_drone[0]);
		interm_point.push_back(_node_current_pos_of_drone[1]);
		interm_point.push_back(_node_current_pos_of_drone[2]);
		interm_point.push_back(current_angle);
		_interm_path.push_back(interm_point);
	}*/
	moveDroneViaSetOfPoints(_interm_path);
	cout << "[ DEBUG] [rotateCounterClockwise] Completed\n";
	return ;
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
	cout << "[ DEBUG] [designPathToChangeYaw] Changing yaw by " << _angle_heuristic << " step\n";
	cout << "[ DEBUG] [designPathToChangeYaw] Current Yaw: " << currentYaw << "\n";
	cout << "[ DEBUG] [designPathToChangeYaw] Destination Yaw: " << desiredYaw << "\n";
	double move; bool to_move = true;
	if(currentYaw > desiredYaw) {move = -1.0;}
	else {move = 1.0;}
	cout << "[ DEBUG] [designPathToChangeYaw] Started\n";
	double prog_yaw = currentYaw;
	if(move == 1.0)
	{
		cout << "[ DEBUG] [designPathToChangeYaw] Move clockwise\n";
	}
	else
	{
		cout << "[ DEBUG] [designPathToChangeYaw] Move counter-clockwise\n";
	}
	/*if(fabs(desiredYaw - prog_yaw) < _angle_heuristic)
	{
		interm_point.clear();
		interm_point.push_back(curr_point[0]); interm_point.push_back(curr_point[1]);
		interm_point.push_back(curr_point[2]); interm_point.push_back(desiredYaw);
		_interm_path.push_back(interm_point);
		to_move = false;
	}
	else
	{
		to_move = true;
	}*/
	while(to_move)
	{
		if(fabs(desiredYaw - prog_yaw) < _angle_heuristic)
		{
			//prog_yaw = desiredYaw;
			if(move == -1.0)
			{
				if(desiredYaw < -180.0)
				{
					desiredYaw = 360.0+desiredYaw;
					prog_yaw = desiredYaw;
				}
			}
			else if(move == 1.0)
			{
				if(desiredYaw >= 180.0)
				{
					desiredYaw = -360.0+desiredYaw;
					prog_yaw = desiredYaw;
				}
			}
			else
			{

			}
			to_move = false;
		}
		else
		{
			prog_yaw += (move * (double)_angle_heuristic);
			if(move == -1.0)
			{
				if(prog_yaw < -180.0)
				{
					prog_yaw = 360.0+prog_yaw;
					desiredYaw = 360.0+desiredYaw;
				}
			}
			else if(move == 1.0)
			{
				if(prog_yaw >= 180.0)
				{
					prog_yaw = -360.0+prog_yaw;
					desiredYaw = -360.0+desiredYaw;
				}
			}
			else
			{

			}
			to_move = true;
		}
		interm_point.clear();
		interm_point.push_back(curr_point[0]); interm_point.push_back(curr_point[1]);
		interm_point.push_back(curr_point[2]); interm_point.push_back(prog_yaw);
		_interm_path.push_back(interm_point);
	}
	print2dVector(_interm_path, "[ DEBUG] [designPathToChangeYaw] Final Path");
	cout << "[ DEBUG] [designPathToChangeYaw] Completed\n";
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
								int which_side)
{
	cout << "[ DEBUG] [checkVisibility] Started\n";
	int move = 0;
	vector<Point2f> image_bounding_box_points;
	print1dVector(plane_parameters, "[ DEBUG] [checkVisibility] Plane Parameters:");
	print1dVector(continuous_bounding_box_points, "[ DEBUG] [checkVisibility] CBB Points:");
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
		cout << "[ DEBUG] [checkVisibility] Top to Bottom edge: " << tb_edge << "\n";
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
		Point2f start, end;
		if(image_bounding_box_points[0].x >= image_bounding_box_points[1].x)
		{
			start = image_bounding_box_points[1];
			end = image_bounding_box_points[2];
		}
		else
		{
			start = image_bounding_box_points[0];
			end = image_bounding_box_points[3];
		}
		Line2f left_edge(start, end);
		cout << "[ DEBUG] [checkVisibility] Left Edge: " << left_edge << "\n";
		// @todo-me Fix this heuristic
		if( (left_edge.start.x < -40.0) ||
				(left_edge.end.x < -40.0) )
		{
			move = 0;
		}
		else if( (left_edge.start.x >= 128.0 && left_edge.start.x <= 256.0) ||
			//(left_edge.start.y >= 144.0 && left_edge.start.x <= 216.0) &&
			(left_edge.end.x >= 128.0 && left_edge.end.x <= 256.0) ) /*&&
			(left_edge.end.y >= 216.0 && left_edge.end.y <= 288.0)  )*/
		{
			move = 0;
		}
		else if( (left_edge.start.x >= -40.0 && left_edge.start.x <= 128.0) ||
				//(left_edge.start.y >= 0.0 && left_edge.start.x <= 72.0) &&
				(left_edge.end.x >= -40.0 && left_edge.end.x <= 128.0) )/*&&
				(left_edge.end.y >= 288.0 && left_edge.end.y <= 360.0) )*/
		{
			move = -1;
		}
		else
		{
			move = 1;
		}
	}
	else if(which_side == 2) // Right edge
	{
		cout << "[ DEBUG] [checkVisibility] Checking for right edge\n";
		image_bounding_box_points.clear();
		project3DPointsOnImage(continuous_bounding_box_points, image_bounding_box_points);
		cout << "[ DEBUG] [checkVisibility] IBB Points:";
		for (unsigned int i = 0; i < image_bounding_box_points.size(); ++i)
		{
			cout << image_bounding_box_points[i] << "\n";
		}
		cout << "\n";
		Point2f start, end;
		if(image_bounding_box_points[0].x >= image_bounding_box_points[1].x)
		{
			start = image_bounding_box_points[0];
			end = image_bounding_box_points[3];
		}
		else
		{
			start = image_bounding_box_points[1];
			end = image_bounding_box_points[2];
		}
		Line2f right_edge(start, end);
		cout << "[ DEBUG] [checkVisibility] Right Edge: " << right_edge << "\n";
		// @todo-me Fix this heuristic
		if( (right_edge.start.x >= 300.0 && right_edge.start.x <= 512.0) ||
			(right_edge.end.x >= 300.0 && right_edge.end.x <= 512.0) )
		{
			move = 0;
		}
		else if( (right_edge.start.x >= 512.0 && right_edge.start.x <= 640.0) ||
				(right_edge.end.x >= 512.0 && right_edge.end.x <= 640.0) )
		{
			move = 1;
		}
		/*if( (right_edge.start.x >= 240.0 && right_edge.start.x <= 580.0) ||
			(right_edge.end.x >= 240.0 && right_edge.end.x <= 580.0) )
		{
			move = 0;
		}
		else if( (right_edge.start.x >= 580.0 && right_edge.start.x <= 640.0) ||
				(right_edge.end.x >= 580.0 && right_edge.end.x <= 640.0) )
		{
			move = 1;
		}*/
		else
		{
			move = -1;
		}
	}
	else
	{
		cout << "[ DEBUG] [checkVisibility] Currently not dealing with it\n";
	}
	cout << "[ DEBUG] [checkVisibility] Completed\n";
	return move;
}

void
ControlUINode::doJLinkage()
{
	cout << "[ DEBUG] [doJLinkage] Started\n";
	clear2dVector(jlink_all_plane_parameters);
	clear2dVector(jlink_all_continuous_bounding_box_points);
	clear2dVector(jlink_three_d_points);
	jlink_all_percentage_of_each_plane.clear();
	cout << "[ DEBUG] [doJLinkage] Calling JLinkage\n";
	getMultiplePlanes3d (jlink_all_plane_parameters, jlink_all_continuous_bounding_box_points, jlink_three_d_points, jlink_all_percentage_of_each_plane);
	print2dVector(jlink_all_plane_parameters, "[ DEBUG] [doJLinkage] JLink Plane Params");
	print2dVector(jlink_all_continuous_bounding_box_points, "[ DEBUG] [doJLinkage] JLink CBB");
	print1dVector(jlink_all_percentage_of_each_plane, "[ DEBUG] [doJLinkage] JLink Pecentage");
	print2dVector(visited_plane_parameters, "[ DEBUG] [doJLinkage] Visited PP");
	print2dVector(visited_continuous_bounding_box_points, "[ DEBUG] [doJLinkage] Visited CBB");
	_sig_plane_index = getCurrentPlaneIndex(visited_plane_parameters, jlink_all_plane_parameters, jlink_all_percentage_of_each_plane);
	_actual_plane_index = _sig_plane_index;
	if(_actual_plane_index == -2) {_actual_plane_index = 0;}
	if(_actual_plane_index == -1) {_actual_plane_index = (int)jlink_all_plane_parameters.size()-1;}
	cout << "[ DEBUG] Sig Plane Index: " << _sig_plane_index << ", Actual Plane Index: " << _actual_plane_index << "\n";
	//assert(_sig_plane_index >= 0);
	getCompleteCurrentPlaneInfo(jlink_all_plane_parameters, jlink_all_continuous_bounding_box_points,
																jlink_three_d_points,
																jlink_all_percentage_of_each_plane, _actual_plane_index,
																this_plane_parameters, this_continuous_bounding_box_points,
																this_sorted_3d_points);
	print1dVector(this_plane_parameters, "[ DEBUG] [doJLinkage] Sig PP");
	print1dVector(this_continuous_bounding_box_points, "[ DEBUG] [doJLinkage] Sig CBB");
	cout << "[ DEBUG] [doJLinkage] Completed\n";
	return ;
}


void
ControlUINode::getCompleteCurrentPlaneInfo(const vector< vector<float> > &plane_parameters,
																					const vector< vector<Point3f> > &cbb,
																					const vector< vector<Point3f> > &points,
																					const vector<float> &percPlane,
																					int currPlaneIndex,
																					vector<float> &out_plane_parameters,
																					vector<Point3f> &out_cbb,
																					vector<Point3f> &out_3d_points)
{
	cout << "[ INFO] [getCompleteCurrentPlaneInfo] Started\n";
	if(currPlaneIndex < 0 && currPlaneIndex != -1)
	{
		cout << "[ ERROR] [getCompleteCurrentPlaneInfo] currPlaneIndex is negative: " << currPlaneIndex << "\n";
	}
	else
	{
		out_plane_parameters.clear();
		out_cbb.clear();
		out_3d_points.clear();
		vector<Point3f> three_d_points;
		vector<float> temp_pp;
		vector<Point3f> bbp;
		float temp_plane_d = 0.0;
		unsigned int destPlaneIndex = currPlaneIndex;
		Point3f normal_old;
		getCurrentPositionOfDrone();
		vector<float> plane_old, plane_new;
		plane_old.clear();
		plane_old.push_back(plane_parameters[currPlaneIndex][0]);
		plane_old.push_back(plane_parameters[currPlaneIndex][1]);
		plane_old.push_back(plane_parameters[currPlaneIndex][2]);
		plane_old.push_back(plane_parameters[currPlaneIndex][3]);
		checkPlaneParametersSign(_node_current_pos_of_drone, points[currPlaneIndex], plane_old);
		normal_old.x = plane_old[0];
		normal_old.y = plane_old[1];
		normal_old.z = plane_old[2];
		float plane_heuristic = 0.96592;
		for (unsigned int i = currPlaneIndex+1; i < plane_parameters.size(); ++i)
		{
			Point3f normal_new;
			plane_new.clear();
			plane_new.push_back(plane_parameters[i][0]);
			plane_new.push_back(plane_parameters[i][1]);
			plane_new.push_back(plane_parameters[i][2]);
			plane_new.push_back(plane_parameters[i][3]);
			getCurrentPositionOfDrone();
			checkPlaneParametersSign(_node_current_pos_of_drone, points[i], plane_new);
			normal_new.x = plane_new[0];
			normal_new.y = plane_new[1];
			normal_new.z = plane_new[2];
			float dot_p = ((normal_old.x * normal_new.x)+(normal_old.y * normal_new.y)+(normal_old.z * normal_new.z));
			if(dot_p >= plane_heuristic)
			{
				destPlaneIndex++;
			}
			else
			{
				break;
			}
		}
		float avg_a = 0.0, avg_b = 0.0, avg_c = 0.0, avg_d = 0.0;
		cout << "[ DEBUG] [getCompleteCurrentPlaneInfo] CurrPlaneIndex: " << currPlaneIndex << ", destPlaneIndex: "
					<< destPlaneIndex << "\n";
		for (unsigned int i = currPlaneIndex; i <= destPlaneIndex; ++i)
		{
			avg_a += plane_parameters[i][0];
			avg_b += plane_parameters[i][1];
			avg_c += plane_parameters[i][2];
			avg_d += plane_parameters[i][3];
			for (unsigned int j = 0; j < points[i].size(); ++j)
			{
				three_d_points.push_back(points[i][j]);
				out_3d_points.push_back(points[i][j]);
			}
			temp_plane_d += plane_parameters[i][3];
		}
		avg_a /= (destPlaneIndex - currPlaneIndex + 1);
		avg_b /= (destPlaneIndex - currPlaneIndex + 1);
		avg_c /= (destPlaneIndex - currPlaneIndex + 1);
		avg_d /= (destPlaneIndex - currPlaneIndex + 1);
		/*out_plane_parameters.push_back(avg_a);
		out_plane_parameters.push_back(avg_b);
		out_plane_parameters.push_back(avg_c);
		out_plane_parameters.push_back(avg_d);*/
		temp_pp.push_back(avg_a);
		temp_pp.push_back(avg_b);
		temp_pp.push_back(avg_c);
		if((int)destPlaneIndex > (int)currPlaneIndex)
		{
			out_plane_parameters.clear();
			fitPlane3D(three_d_points, out_plane_parameters);
		}
		else
		{
			out_plane_parameters.clear();
			out_plane_parameters.push_back(avg_a);
			out_plane_parameters.push_back(avg_b);
			out_plane_parameters.push_back(avg_c);
			out_plane_parameters.push_back(avg_d);
		}
		getCurrentPositionOfDrone();
		checkPlaneParametersSign(_node_current_pos_of_drone, three_d_points, out_plane_parameters);
		if(signbit(avg_a) == signbit(out_plane_parameters[0]))
		{
			cout << "[ DEBUG] [getCompleteCurrentPlaneInfo] Plane parameters sign not reversed\n";
			bbp.push_back(cbb[currPlaneIndex][0]);
			bbp.push_back(cbb[destPlaneIndex][1]);
			bbp.push_back(cbb[destPlaneIndex][2]);
			bbp.push_back(cbb[currPlaneIndex][3]);
			bbp.push_back(cbb[currPlaneIndex][0]);
		}
		else
		{
			cout << "[ DEBUG] [getCompleteCurrentPlaneInfo] Plane parameters sign reversed\n";
			bbp.push_back(cbb[currPlaneIndex][1]);
			bbp.push_back(cbb[destPlaneIndex][0]);
			bbp.push_back(cbb[destPlaneIndex][3]);
			bbp.push_back(cbb[currPlaneIndex][2]);
			bbp.push_back(cbb[currPlaneIndex][1]);
		}
		cout << "[ DEBUG] [getCompleteCurrentPlaneInfo] Calculating the bounding box points\n";
		projectPointsOnPlane(bbp, out_plane_parameters, out_cbb);
		three_d_points.clear();
		bbp.clear();
		print1dVector(out_plane_parameters, "[ INFO] [getCompleteCurrentPlaneInfo] Current visible plane parameters");
		print1dVector(out_cbb, "[ INFO] [getCompleteCurrentPlaneInfo] Current visible cbb");
	}
	cout << "[ INFO] [getCompleteCurrentPlaneInfo] Completed\n";
	return ;
}

void
ControlUINode::checkPlaneParametersSign(const vector<double> &position, 
																				const vector<Point3f> &points,
																				vector<float> &plane_parameters)
{
	assert(points.size() >= 3);
	cout << "[ INFO] [checkPlaneParametersSign] Started\n";
	// Create a matrix out of the vector of points: Dimension: numberOfPoints*3
	float x_c = 0.0, y_c = 0.0, z_c = 0.0;
	for (unsigned int i = 0; i < points.size(); ++i)
	{
		x_c += points[i].x;
		y_c += points[i].y;
		z_c += points[i].z;
	}
	x_c /= points.size();
	y_c /= points.size();
	z_c /= points.size();
	// Calculate the centroid of the points
	float centroidX = x_c;
	float centroidY = y_c;
	float centroidZ = z_c;
	Point3f p(centroidX, centroidY, centroidZ), qp;
	cout << "[ DEBUG] [checkPlaneParametersSign] Centroid: " << p << "\n";
	print1dVector(position, "[ DEBUG] [checkPlaneParametersSign] Position of drone");
	float a = plane_parameters[0];
	float b = plane_parameters[1];
	float c = plane_parameters[2];
	float mag = ((a*a)+(b*b)+(c*c));
	print1dVector(plane_parameters, "[ DEBUG] [checkPlaneParametersSign] Old Plane Parameters");
	Point3f pos((float)position[0], (float)position[1], (float)position[2]);
	qp = pos - p;
	cout << "[ DEBUG] [checkPlaneParametersSign] qp: " << qp << "\n";
	float t = (qp.x * (a/mag)) + (qp.y * (b/mag)) + ((qp.z * (c/mag)));
	cout << "[ DEBUG] [checkPlaneParametersSign] t: " << t << "\n";
	if(!signbit(t))
	{
		cout << "[ DEBUG] [checkPlaneParametersSign] Sign change required\n";
		plane_parameters[0] = -plane_parameters[0];
		plane_parameters[1] = -plane_parameters[1];
		plane_parameters[2] = -plane_parameters[2];
		plane_parameters[3] = -plane_parameters[3];
	}
	print1dVector(plane_parameters, "[ DEBUG] [checkPlaneParametersSign] New Plane Parameters");
	cout << "[ INFO] [checkPlaneParametersSign] Completed\n";
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
		doJLinkage();
		// Render significant plane
		//image_gui->setRender(false, true, true, false);
		//cout << "[ INFO] [testUtility] Rendering the frames in the DRONE CAMERA FEED GUI\n";
		//image_gui->setSigPlaneBoundingBoxPoints(jlink_all_continuous_bounding_box_points[_sig_plane_index]);
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
		doJLinkage();
		// Render significant plane
		//image_gui->setRender(false, true, true, false);
		//cout << "[ INFO] [testUtility] Rendering the frames in the DRONE CAMERA FEED GUI\n";
		//image_gui->setSigPlaneBoundingBoxPoints(jlink_all_continuous_bounding_box_points[_sig_plane_index]);
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
		doJLinkage();
		// Render significant plane
		//image_gui->setRender(false, true, true, false);
		//cout << "[ INFO] [testUtility] Rendering the frames in the DRONE CAMERA FEED GUI\n";
		//image_gui->setSigPlaneBoundingBoxPoints(jlink_all_continuous_bounding_box_points[_sig_plane_index]);
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
		doJLinkage();
		/*visited_plane_parameters.clear();
		vector<float> pp;
		pp.push_back(0.0208796);
		pp.push_back(0.999466);
		pp.push_back(0.0251287);
		pp.push_back(-1.85729);
		visited_plane_parameters.push_back(pp);*/
		_next_plane_dir = COUNTERCLOCKWISE;
		_next_plane_angle = 30.0;
		adjustForNextCapture();
		//pp.clear();
		alignQuadcopterToNextPlaneAdvanced();
	}
	else if(test_no == 7)
	{
		cout << "[ INFO] [testUtility] Testing for translation to the next plane\n";
		_node_number_of_planes = 1;
		_node_min_distance = 3.0;
		_node_max_distance = 5.0;
		getCurrentPositionOfDrone();
		cout << "[ INFO] [testUtility] Current Position of drone: ";
		cout << "(" << _node_current_pos_of_drone[0] << ", " << _node_current_pos_of_drone[1] 
				<< ", " << _node_current_pos_of_drone[2] << ", " << _node_current_pos_of_drone[3] << ")\n";
		doJLinkage();
	}
	else if(test_no == 8)
	{
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
		vector<float> plane_1, plane_2;
		cout << "[ INFO] [testUtility] bestFitPlane\n";
		plane_1 = bestFitPlane(_in_points);
		cout << "[ INFO] [testUtility] fitPlane3D\n";
		fitPlane3D(_in_points, plane_2);
		print1dVector(plane_1, "[ INFO] [testUtility] 4 matrix");
		print1dVector(plane_2, "[ INFO] [testUtility] 3 matrix");
	}
	else if(test_no == 9)
	{
		cout << "[ INFO] Testing for aligning the quadcopter to the current plane\n";
		_node_number_of_planes = 1;
		_node_min_distance = 3.0;
		_node_max_distance = 5.0;
		doJLinkage();
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
	assert(start.size() == end.size());
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
	//print2dVector(_interm_path, "[ DEBUG] [designPathForDrone] Initial Path Points for drone:");
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
			/*cout << "a: " << a << ", b: " << b << ", c: " << c << ", d: " << d << "\n";
			cout << "0: " << _interm_path[i][0] << ", 1: " << _interm_path[i][1] 
					<< ", 2: " << _interm_path[i][2] <<  ", 3: " << _interm_path[i][3] << "\n";*/
			if( !( (fabs(a - _interm_path[i][0])<=0.0001) &&
					(fabs(b - _interm_path[i][1])<=0.0001) &&
					(fabs(c - _interm_path[i][2])<=0.0001) &&
					(fabs(d - _interm_path[i][3])<=0.0001) ))
			{
				test_interm_path.push_back(_interm_path[i]);
				a = _interm_path[i][0]; b = _interm_path[i][1];
				c = _interm_path[i][2]; d = _interm_path[i][3];
			}
			else
			{
				//cout << "[ DEBUG] [designPathForDrone] Linear: Not pushing this command\n";
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
		for (unsigned int j = 0; j < _interm_path[i].size(); ++j)
		{
			cout << _interm_path[i][j] << " ";
		}
		cout << "\n";
	}
	clear2dVector(test_interm_path);
	cout << "[ DEBUG] [designPathForDrone] Completed\n";
}

/**
 * @brief Align the yaw of the quadcopter to the current plane's (the one which it is seeing) normal
 * @details
 */
void
ControlUINode::alignQuadcopterToCurrentPlane()
{
	cout << "[ INFO] [alignQuadcopterToCurrentPlane] Started\n";
	if(_node_number_of_planes == 1)
	{
		_next_plane_dir = CLOCKWISE;
		_next_plane_angle = 0.0;
	}
	else
	{
		_next_plane_dir = _node_main_directions.front();
		_next_plane_angle = _node_main_angles.front();
	}
	cout << "[ INFO] [alignQuadcopterToCurrentPlane] Number of planes: " << _node_number_of_planes <<
				", Completed Number of planes: " << _node_completed_number_of_planes << 
				", Next plane dir: " << _next_plane_dir << ", Next plane angle: " << _next_plane_angle << "\n";
	adjustYawToCurrentPlane();
	cout << "[ INFO] [alignQuadcopterToCurrentPlane] Observing the plane: " << _stage_of_plane_observation << "\n";
	if(_stage_of_plane_observation)
	{
		cout << "[ DEBUG] [alignQuadcopterToCurrentPlane] Observing plane for the first time\n";
		cout << "[ DEBUG] [alignQuadcopterToCurrentPlane] Plane Parameters Size: " << this_plane_parameters.size() << "\n";
		cout << "[ INFO] [alignQuadcopterToCurrentPlane] Moving back by max. distance\n";
		adjustTopBottomEdges();
		adjustLeftEdge();
		adjustYawToCurrentPlane();
	}
	cout << "[ INFO] [alignQuadcopterToCurrentPlane] Completed\n";
	cout << "[ INFO] [alignQuadcopterToCurrentPlane] Please click 4 points on the DRONE CAMERA FEED Screen\n";
	return ;
}

void
ControlUINode::adjustYawToCurrentPlane()
{
	cout << "[ INFO] [adjustYawToCurrentPlane] Started\n";
	getCurrentPositionOfDrone();
	print1dVector(_node_current_pos_of_drone, "[ INFO] [adjustYawToCurrentPlane] Current position of drone");
	_node_dest_pos_of_drone.clear();
	_node_dest_pos_of_drone.push_back(0.0);
	_node_dest_pos_of_drone.push_back(1.0);
	_node_dest_pos_of_drone.push_back(0.0);
	_node_dest_pos_of_drone.push_back(0.0);
	cout << "[ DEBUG] [adjustYawToCurrentPlane] Converting destination position wrt world quadcopter origin\n";
	convertWRTQuadcopterOrigin(_node_current_pos_of_drone, _node_dest_pos_of_drone, _node_ac_dest_pos_of_drone);
	Point3f pYAxis(_node_ac_dest_pos_of_drone[0], _node_ac_dest_pos_of_drone[1], _node_ac_dest_pos_of_drone[2]);
	Point3f pOrigin(_node_current_pos_of_drone[0], _node_current_pos_of_drone[1], _node_current_pos_of_drone[2]);
	Point3f projectedNormal(pYAxis-pOrigin);
	cout << "[ INFO] [adjustYawToCurrentPlane] Estimating multiple planes -> call to JLinkage\n";
	doJLinkage();
	Point3f plane_params(this_plane_parameters[0], this_plane_parameters[1], 0.0);
	cout << "[ DEBUG] [adjustYawToCurrentPlane] pYAxis: " << pYAxis << "\n";
	cout << "[ DEBUG] [adjustYawToCurrentPlane] pOrigin: " << pOrigin << "\n";
	cout << "[ DEBUG] [adjustYawToCurrentPlane] projectedNormal: " << projectedNormal << "\n";
	cout << "[ DEBUG] [adjustYawToCurrentPlane] PP: " << plane_params << "\n";
	float angle = findAngle(projectedNormal, plane_params);
	cout << "[ DEBUG] [adjustYawToCurrentPlane] Angle (radians): " << angle << "\n";
	angle = -angle*180/M_PI;
	cout << "[ INFO] [adjustYawToCurrentPlane] Change the yaw of quadcopter\n";
	cout << "[ INFO] [adjustYawToCurrentPlane] Angle to rotate: " << angle << "\n";
	designPathToChangeYaw(_node_current_pos_of_drone, _node_current_pos_of_drone[3]+angle);
	moveDroneViaSetOfPoints(_interm_path);
	cout << "[ INFO] [adjustYawToCurrentPlane] Completed\n";
}

/**
 * @brief Adjust the quadcopter to see the top and bottom edge of the current plane
 * @details
 */
void
ControlUINode::adjustTopBottomEdges()
{
	cout << "[ INFO] [adjustTopBottomEdges] Started\n";
	float step_distance;
	float point_distance, height;
	print1dVector(this_plane_parameters, "[ DEBUG] [adjustTopBottomEdges] Sig PP");
	print1dVector(this_continuous_bounding_box_points, "[ DEBUG] [adjustTopBottomEdges] Sig CBB");
	getCurrentPositionOfDrone();
	print1dVector(_node_current_pos_of_drone, "[ DEBUG] [adjustTopBottomEdges] Current position of drone");
	point_distance = getPointToPlaneDistance(this_plane_parameters, _node_current_pos_of_drone);
	cout << "[ DEBUG] [adjustTopBottomEdges] Distance from the plane: " << point_distance << "\n";
	step_distance = fabs(_node_max_distance - point_distance);
	int move = (_node_max_distance >= point_distance) ? -1: 1;
	cout << "[ DEBUG] [adjustTopBottomEdges] Move: " << move << ", Step Distance: " << step_distance << "\n";
	if(move == -1)
	{
		cout << "[ DEBUG] [adjustTopBottomEdges] Moving backwards\n";
		designPathForDroneRelative(step_distance, MOVE_DIRECTIONS::BACKWARD);
	}
	else if(move == 1)
	{
		cout << "[ DEBUG] [adjustTopBottomEdges] Moving forwards\n";
		designPathForDroneRelative(step_distance, MOVE_DIRECTIONS::FORWARD);
	}
	getCurrentPositionOfDrone();
	print1dVector(_node_current_pos_of_drone, "[ DEBUG] [adjustTopBottomEdges] Current position of drone");
	if(!_fixed_height_set)
	{
		cout << "[ DEBUG] [adjustTopBottomEdges] Fixed height not set\n";
		height = getHeightFromGround(this_plane_parameters, this_continuous_bounding_box_points, _node_current_pos_of_drone);
		cout << "[ DEBUG] [adjustTopBottomEdges] Height: " << height << "\n";
		move = (_node_current_pos_of_drone[2] >= height) ? -1: 1;
		step_distance = fabs(_node_current_pos_of_drone[2] - height);
	}
	else
	{
		cout << "[ DEBUG] [adjustTopBottomEdges] Fixed height set. Fixed Height: " << _fixed_height << "\n";
		move = (_node_current_pos_of_drone[2] >= _fixed_height) ? -1: 1;
		step_distance = fabs(_node_current_pos_of_drone[2] - _fixed_height);
	}
	if(move == -1)
	{
		cout << "[ DEBUG] [adjustTopBottomEdges] Moving down\n";
		designPathForDroneRelative(step_distance, MOVE_DIRECTIONS::DOWN);
	}
	else if(move == 1)
	{
		cout << "[ DEBUG] [adjustTopBottomEdges] Moving up\n";
		designPathForDroneRelative(step_distance, MOVE_DIRECTIONS::UP);
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
	cout << "[ DEBUG] [adjustLeftEdge] Call Jlinkage\n";
	doJLinkage();
	bool planeLeftVisible;
	int move = checkVisibility(this_plane_parameters, this_continuous_bounding_box_points, 1);
	if(move==0)
	{
		planeLeftVisible = true;
	}
	else
	{
		planeLeftVisible = false;
	}
	while(!planeLeftVisible)
	{
		getCurrentPositionOfDrone();
		cout << "[ DEBUG] [adjustLeftEdge] Move: " << move << ", Step Distance: " << _move_heuristic << "\n";
		if(move == -1)
		{
			cout << "[ DEBUG] [adjustLeftEdge] Moving left\n";
			designPathForDroneRelative(_move_heuristic, MOVE_DIRECTIONS::LEFT);
		}
		else if(move == 1)
		{
			cout << "[ DEBUG] [adjustLeftEdge] Moving right\n";
			designPathForDroneRelative(_move_heuristic, MOVE_DIRECTIONS::RIGHT);
		}
		doJLinkage();
		move = checkVisibility(this_plane_parameters, this_continuous_bounding_box_points, 1);
		cout << "[ DEBUG] [adjustLeftEdge] Move: " << move << "\n";
		if(move==0) {planeLeftVisible = true;}
		else {planeLeftVisible = false;}
	}
	cout << "[ DEBUG] [adjustLeftEdge] Completed\n";
}

/**
 *
 */
void
ControlUINode::designPathForDroneRelative(double dest, int direction)
{
	// 1 - Left
	// 2 - Right
	// 3 - Front
	// 4 - Back
	// 5 - Top
	// 6 - Back
	cout << "[ DEBUG] [designPathForDroneRelative] Started\n";
	cout << "[ DEBUG] [designPathForDroneRelative] Direction: " << direction << "\n";
	if(direction == MOVE_DIRECTIONS::LEFT) // Left
	{
		moveLeft(dest);
	}
	else if(direction == MOVE_DIRECTIONS::RIGHT) // Right
	{
		moveRight(dest);
	}
	else if(direction == MOVE_DIRECTIONS::FORWARD) // Front
	{
		moveForward(dest);
	}
	else if(direction == MOVE_DIRECTIONS::BACKWARD) // Back
	{
		moveBackward(dest);
	}
	else if(direction == MOVE_DIRECTIONS::UP) // Up
	{
		moveUp(dest);
	}
	else if(direction == MOVE_DIRECTIONS::DOWN) // Down
	{
		moveDown(dest);
	}
	else if(direction == MOVE_DIRECTIONS::CLOCK) // Clockwise
	{
		rotateClockwise(dest);
	}
	else if(direction == MOVE_DIRECTIONS::COUNTERCLOCK) // CounterClockwise
	{
		rotateCounterClockwise(dest);
	}
	else
	{
		cout << "[ DEBUG] [designPathForDroneRelative] I don't understand this direction\n";
	}
	cout << "[ DEBUG] [designPathForDroneRelative] Completed\n";
	return ;
}

void
ControlUINode::move(double distance, int i)
{
	cout << "[ DEBUG] [move] Started\n";
	clear2dVector(_interm_path);
	getCurrentPositionOfDrone();
	_node_dest_pos_of_drone.clear();
	_node_dest_pos_of_drone.push_back(0.0);
	_node_dest_pos_of_drone.push_back(0.0);
	_node_dest_pos_of_drone.push_back(0.0);
	_node_dest_pos_of_drone.push_back(0.0);
	print1dVector(_node_current_pos_of_drone, "[ DEBUG] [move] Current position of drone");
	cout << "[ DEBUG] [move] Requested movement: " << distance << ", Direction: " << i << "\n";
	double start = 0.0;
	double move;
	if(signbit(distance))
	{
		move = -1.0;
	}
	else
	{
		move = 1.0;
	}
	cout << "[ DEBUG] [move] Move: " << move << "\n";
	bool to_move = false;
	if(fabs(start - distance) < _move_heuristic)
	{
		start = distance;
		_node_dest_pos_of_drone[i] = start;
		print1dVector(_node_dest_pos_of_drone, "[ DEBUG] [move] Dest position of drone (relative)");
		convertWRTQuadcopterOrigin(_node_current_pos_of_drone, _node_dest_pos_of_drone, _node_ac_dest_pos_of_drone);
		_interm_path.push_back(_node_ac_dest_pos_of_drone);
	}
	else
	{
		to_move = true;
	}
	while(to_move)
	{
		if(fabs(start - distance) < _move_heuristic)
		{
			start = distance;
			to_move = false;
		}
		else
		{
			start += (move * _move_heuristic);
			to_move = true;
		}
		_node_dest_pos_of_drone[i] = start;
		print1dVector(_node_dest_pos_of_drone, "[ DEBUG] [move] Dest position of drone (relative)");
		convertWRTQuadcopterOrigin(_node_current_pos_of_drone, _node_dest_pos_of_drone, _node_ac_dest_pos_of_drone);
		_interm_path.push_back(_node_ac_dest_pos_of_drone);
	}
	if(fabs(start - distance) < _move_heuristic)
	{
		start = distance;
		_node_dest_pos_of_drone[i] = start;
		convertWRTQuadcopterOrigin(_node_current_pos_of_drone, _node_dest_pos_of_drone, _node_ac_dest_pos_of_drone);
		_interm_path.push_back(_node_ac_dest_pos_of_drone);
	}
	moveDroneViaSetOfPoints(_interm_path);
	cout << "[ DEBUG] [move] Completed\n";
	return ;
}

void
ControlUINode::moveUp(double up_distance)
{
	cout << "[ DEBUG] [moveUp] Started\n";
	move(up_distance, 2);
	cout << "[ DEBUG] [moveUp] Completed\n";
	return ;
}

void
ControlUINode::moveDown(double down_distance)
{
	cout << "[ DEBUG] [moveDown] Started\n";
	move(-down_distance, 2);
	cout << "[ DEBUG] [moveDown] Completed\n";
	return ;
}

void
ControlUINode::moveLeft(double left_distance)
{
	cout << "[ DEBUG] [moveLeft] Started\n";
	move(-left_distance, 0);
	cout << "[ DEBUG] [moveLeft] Completed\n";
	return ;
}

void
ControlUINode::moveRight(double right_distance)
{
	cout << "[ DEBUG] [moveRight] Started\n";
	move(right_distance, 0);
	cout << "[ DEBUG] [moveRight] Completed\n";
	return ;
}

void
ControlUINode::moveForward(double forward_distance)
{
	cout << "[ DEBUG] [moveForward] Started\n";
	move(forward_distance, 1);
	cout << "[ DEBUG] [moveForward] Completed\n";
	return ;
}

void
ControlUINode::moveBackward(double backward_distance)
{
	cout << "[ DEBUG] [moveBackward] Started\n";
	move(-backward_distance, 1);
	cout << "[ DEBUG] [moveBackward] Completed\n";
	return ;
}

void
ControlUINode::moveInDirection(const vector<float> &dir, 
										const vector<double> &position,
										const vector<Point3f> &points)
{
	cout << "[ DEBUG] [moveInDirection] Started\n";
	float point_distance = getPointToPlaneDistance(dir, position);
	int move = (_fixed_distance >= point_distance) ? -1: 1;
	float step_distance = fabs(_fixed_distance - point_distance);
	cout << "[ DEBUG] [moveInDirection] Drone Distance: " << point_distance
				<< ", Fixed Distance: " << _fixed_distance << "\n";
	cout << "[ DEBUG] [moveInDirection] Move: " << move << ", Step Distance: " << step_distance << "\n";
	if(move == -1)
	{
		cout << "[ DEBUG] [moveInDirection] Moving backwards\n";
	}
	else if(move == 1)
	{
		cout << "[ DEBUG] [moveInDirection] Moving forwards\n";
	}
	Point3f pp, pos((float)position[0], (float)position[1], (float)position[2]);
	float t, avg_a = 0.0, avg_b = 0.0, avg_c = 0.0;
	float a = dir[0];
	float b = dir[1];
	float c = dir[2];
	print1dVector(dir, "[ DEBUG] [moveInDirection] Plane Parameters for distance adjustment");
	print1dVector(position, "[ DEBUG] [moveInDirection] Current position of drone");
	unsigned int numberOfPointsInThisPlane = points.size();
	for (unsigned int j = 0; j < numberOfPointsInThisPlane; ++j)
	{
		avg_a += points[j].x;
		avg_b += points[j].y;
		avg_c += points[j].z;
	}
	avg_a /= (float)points.size();
	avg_b /= (float)points.size();
	avg_c /= (float)points.size();
	Point3f p(avg_a, avg_b, avg_c), qp;
	float mag = ((a*a)+(b*b)+(c*c));
	qp = pos - p;
	t = (qp.x * (a/mag)) + (qp.y * (b/mag)) + ((qp.z * (c/mag)));
	Point3f proj(pos.x - a*t, pos.y - b*t, pos.z - c*t);
	Point3f dest;
	cout << "[ DEBUG] [moveInDirection] t: " << t << "\n";
	if(signbit(t)) {t = -1.0;}
	else {t = 1.0;}
	dest.x = proj.x + a*t*(_fixed_distance);
	dest.y = proj.y + b*t*(_fixed_distance);
	dest.z = proj.z + c*t*(_fixed_distance);
	cout << "[ DEBUG] [moveInDirection] t: " << t*(_fixed_distance) << "\n";
	cout << "[ DEBUG] [moveInDirection] Point on plane: " << proj << ", Point to move: " << dest << "\n";
	vector<double> init_pos, dest_pos;
	init_pos.push_back(position[0]);
	init_pos.push_back(position[1]);
	init_pos.push_back(position[2]);
	dest_pos.push_back((double)dest.x);
	dest_pos.push_back((double)dest.y);
	dest_pos.push_back((double)dest.z);
	clear2dVector(_interm_path);
	getInitialPath(init_pos, dest_pos, position[3], position[3], _interm_path);
	moveDroneViaSetOfPoints(_interm_path);
	cout << "[ DEBUG] [moveInDirection] Completed\n";
	return ;
}

/**
 * @brief Gets all the data about the points in the clicked region using jlinkage
 * @details To be called after user clicks the 4 points on the DRONE CAMERA FEED Window
 */
void
ControlUINode::captureTheCurrentPlane()
{
	cout << "[ INFO] [captureTheCurrentPlane] Started\n";
	cout << "[ INFO] [captureTheCurrentPlane] Number of planes: " << _node_number_of_planes 
			<< ", Number of planes covered: " << _node_completed_number_of_planes << "\n";
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
	cout << "[ DEBUG] [captureTheCurrentPlane] Stopped rendering of frame\n";
	image_gui->setRender(false, false, true, true);
	image_gui->getPointsClicked(points_clicked);
	cout << "[ INFO] [captureTheCurrentPlane] Extracting Bounding Poly\n";
	image_gui->extractBoundingPoly();
	_sig_plane_index = 0;
	image_gui->getCCPoints(cc_points);
	cout << "[ INFO] [captureTheCurrentPlane] Get multiple planes from the clicked points using JLinkage\n";
	// Calls JLinkage and finds all planes within the clicked region
	vector< vector<float> > test_plane_parameters;
	doJLinkage();
	// Render significant plane
	image_gui->setContinuousBoundingBoxPoints(jlink_all_continuous_bounding_box_points);
	image_gui->setSigPlaneBoundingBoxPoints(this_continuous_bounding_box_points);
	image_gui->setVisitedBoundingBoxPoints(visited_continuous_bounding_box_points);
	cout << "[ INFO] [captureTheCurrentPlane] Rendering the frames in the DRONE CAMERA FEED GUI\n";
	image_gui->setRender(false, false, true, true);
	cout << "[ DEBUG] [captureTheCurrentPlane] SigPlaneIndex: " << _sig_plane_index << "\n";
	cout << "[ DEBUG] [captureTheCurrentPlane] ActualPlaneIndex: " << _actual_plane_index << "\n";
	cout << "[ DEBUG] [captureTheCurrentPlane] Rendering poly and sig plane and visited planes\n";
	image_gui->renderFrame();
	// Check if the plane is completed by finding if a new plane is visible or not other than current one
	clear2dVector(test_plane_parameters);
	for (unsigned int i = 0; i < visited_plane_parameters.size(); ++i)
	{
		test_plane_parameters.push_back(visited_plane_parameters[i]);
	}
	test_plane_parameters.push_back(this_plane_parameters);
	print2dVector(visited_plane_parameters, "[ DEBUG] [captureTheCurrentPlane] All planes visited completely");
	print2dVector(test_plane_parameters, "[ DEBUG] [captureTheCurrentPlane] All planes visited completely including the current one");
	if(_node_completed_number_of_planes == 0 && _stage_of_plane_observation)
	{
		cout << "[ INFO] [captureTheCurrentPlane] Checking if adjustment is required\n";
		getCurrentPositionOfDrone();
		print1dVector(_node_current_pos_of_drone, "[ DEBUG] [captureTheCurrentPlane] Current position of drone");
		Point3f top_mid = (this_continuous_bounding_box_points[0]+this_continuous_bounding_box_points[1]);
		top_mid.x = top_mid.x/(float)2.0;
		top_mid.y = top_mid.y/(float)2.0;
		top_mid.z = top_mid.z/(float)2.0;
		float distance = getDistanceToSeePlane((int)ceil(top_mid.z));
		_fixed_distance = distance;
		float point_distance = getPointToPlaneDistance(this_plane_parameters, _node_current_pos_of_drone);
		int move = (distance >= point_distance) ? -1: 1;
		float step_distance = fabs(distance - point_distance);
		if(move == -1)
		{
			cout << "[ DEBUG] [captureTheCurrentPlane] Moving backwards\n";
			moveBackward(step_distance);
		}
		else if(move == 1)
		{
			cout << "[ DEBUG] [captureTheCurrentPlane] Moving forwards\n";
			moveForward(step_distance);
		}
		getCurrentPositionOfDrone();
		float height = getHeightFromGround(this_plane_parameters, this_continuous_bounding_box_points, _node_current_pos_of_drone);
		_fixed_height = height;
		_fixed_height_set = true;
		move = (_node_current_pos_of_drone[2] >= height) ? -1: 1;
		step_distance = fabs(_node_current_pos_of_drone[2] - height);
		if(move == -1)
		{
			cout << "[ DEBUG] [captureTheCurrentPlane] Moving down\n";
			moveDown(step_distance);
		}
		else if(move == 1)
		{
			cout << "[ DEBUG] [captureTheCurrentPlane] Moving up\n";
			moveUp(step_distance);
		}
		doJLinkage();
	}
	cout << "[ DEBUG] [captureTheCurrentPlane] Observe the plane without rotation\n";
	_is_plane_covered = isNewPlaneVisible(test_plane_parameters,
										jlink_all_plane_parameters, jlink_all_percentage_of_each_plane, false);
	cout << "[ DEBUG] [captureTheCurrentPlane] Is plane covered: " << _is_plane_covered << "\n";
	if(!_is_plane_covered)
	{
		cout << "[ DEBUG] [captureTheCurrentPlane] Checking if right edge is within the frame to reach a conclusion\n";
		int move = checkVisibility(this_plane_parameters, this_continuous_bounding_box_points, 2);
		if(move==0)
		{
			_is_plane_covered = true;
		}
		else
		{
			_is_plane_covered = false;
		}
		cout << "[ DEBUG] [captureTheCurrentPlane] Is plane covered: " << _is_plane_covered << ", Move: " << move << "\n";
	}
	if(_is_plane_covered)
	{
		copyNecessaryInfo();
		if(_node_completed_number_of_planes != _node_number_of_planes)
		{
			cout << "[ INFO] [captureTheCurrentPlane] Completed plane no.: " << _node_completed_number_of_planes << "\n";
			cout << "[ INFO] [captureTheCurrentPlane] Aligning the quadcopter to the next plane\n";
			alignQuadcopterToNextPlaneAdvanced();
		}
		else
		{
			cout << "[ INFO] [captureTheCurrentPlane] All planes covered\n";
			cout << "[ INFO] [captureTheCurrentPlane] Landing the quadcopter\n";
			sendLand();
		}
	}
	else
	{
		augmentInfo();
		cout << "[ INFO] [captureTheCurrentPlane] Adjusting quadcopter for next capture of the same plane\n";
		adjustForNextCapture();
		cout << "[ INFO] [captureTheCurrentPlane] Adjusted for next capture. Please click the 4 points on the DRONE CAMERA FEED\n";
	}
	if(_node_completed_number_of_planes == _node_number_of_planes)
	{
		assert(visited_plane_parameters.size() == visited_continuous_bounding_box_points.size());
		cout << "[ INFO] [captureTheCurrentPlane] All planes are completed\n";
		string filename = "Plane_Info.txt";
		cout << "[ INFO] [captureTheCurrentPlane] Writing info gathered to " << filename << "\n";
		for (unsigned int i = 0; i < visited_plane_parameters.size(); ++i)
		{
			image_gui->WriteInfoToFile(visited_continuous_bounding_box_points[i], visited_plane_parameters[i], i+1, filename);
		}
		print2dVector(visited_plane_parameters, "[ INFO] [captureTheCurrentPlane] All Plane Parameters");
		print2dVector(visited_continuous_bounding_box_points, "[ INFO] [captureTheCurrentPlane] All Plane BBP");
		print2dVector(visited_motion_points, "[ INFO] [captureTheCurrentPlane] All motion points");
	}
	else
	{
		cout << "[ INFO] [captureTheCurrentPlane] All planes are not completed\n";
	}
	cout << "[ INFO] [captureTheCurrentPlane] Completed\n";
}

/**
 * @brief Adjust the quadcopter to capture next part of the same plane
 * @details
 */
void
ControlUINode::adjustForNextCapture()
{
	cout << "[ INFO] [adjustForNextCapture] Started\n";
	vector< vector<float> > test_plane_parameters;
	Point3f top_left = this_continuous_bounding_box_points[0];
	Point3f top_right = this_continuous_bounding_box_points[1];
	double width_of_3d_plane = (double)fabs(sqrt( (top_right.x - top_left.x)*(top_right.x - top_left.x) +
										(top_right.y - top_left.y)*(top_right.y - top_left.y) +
										(top_right.z - top_left.z)*(top_right.z - top_left.z) ));
	cout << "[ INFO] [adjustForNextCapture] Width of the plane is: " << width_of_3d_plane << "\n";
	// Direction of next plane
	if(_next_plane_dir == CLOCKWISE)
	{
		cout << "[ INFO] [adjustForNextCapture] Next plane is CLOCKWISE wrt current plane\n";
		getCurrentPositionOfDrone();
		/*cout << "[ INFO] [adjustForNextCapture] Moving by width of plane by 4\n";
		moveRight(width_of_3d_plane/(double)4.0);*/
		if(_node_completed_number_of_planes != _node_number_of_planes-1)
		{
			cout << "[ INFO] [adjustForNextCapture] Changing the yaw clockwise by " << (3*fabs(_next_plane_angle)/4.0) << "\n";
			rotateClockwise(3*fabs(_next_plane_angle)/4.0);
			cout << "[ INFO] [adjustForNextCapture] Moving backwards by 0.5\n";
			moveBackward(0.5);
			cout << "[ INFO] [adjustForNextCapture] Moving forwards by 0.5\n";
			moveForward(0.5);
		}
		else
		{
			cout << "[ DEBUG] [adjustForNextCapture] Last plane to cover\n";
		}
		cout << "[ INFO] [adjustForNextCapture] Estimating multiple planes -> call to JLinkage\n";
		doJLinkage();
		// Adding the currently seeing plane to find out if a new plane another than the
		// current one is visible by rotating the drone
		clear2dVector(test_plane_parameters);
		for (unsigned int i = 0; i < visited_plane_parameters.size(); ++i)
		{
			test_plane_parameters.push_back(visited_plane_parameters[i]);
		}
		test_plane_parameters.push_back(this_plane_parameters);
		print2dVector(visited_plane_parameters, "[ INFO] [adjustForNextCapture] All planes visited completely");
		print2dVector(test_plane_parameters, "[ INFO] [adjustForNextCapture] All planes visited completely including the current one");
		if(_node_completed_number_of_planes == _node_number_of_planes-1)
		{
			cout << "[ INFO] [adjustForNextCapture] Observe the plane without rotation\n";
			_is_plane_covered = isNewPlaneVisible(test_plane_parameters,
											jlink_all_plane_parameters, jlink_all_percentage_of_each_plane, false);
		}
		else
		{
			cout << "[ INFO] [adjustForNextCapture] Observe the plane by rotation: " 
						<< _next_plane_dir << "\n";
			_is_plane_covered = isNewPlaneVisible(test_plane_parameters,
											jlink_all_plane_parameters, jlink_all_percentage_of_each_plane, true, _next_plane_dir);
		}
		cout << "[ DEBUG] [adjustForNextCapture] Is plane covered: " << _is_plane_covered << "\n";
		if(_is_plane_covered && 
			(_node_completed_number_of_planes != _node_number_of_planes-1) )
		{
			cout << "[ INFO] [adjustForNextCapture] Checking if the plane is covered\n";
			getCurrentPositionOfDrone();
			if(_actual_plane_index+1 < jlink_all_plane_parameters.size() && _actual_plane_index+1 >= 0)
			{
				float check_dist = 
						getPointToPlaneDistance(jlink_all_plane_parameters[_actual_plane_index+1], 
												_node_current_pos_of_drone);
				cout << "[ DEBUG] [adjustForNextCapture] Check_dist: " << check_dist << ", Max. Dist: " << _node_max_distance << "\n";
				if(check_dist > _node_max_distance+1.0)
				{
					_is_plane_covered = false;
				}
			}
			else
			{
				cout << "[ INFO] [adjustForNextCapture] Current plane is the right most plane visible\n";
			}
		}
		if(!_is_plane_covered)
		{
			cout << "[ DEBUG] [adjustForNextCapture] Checking if right edge is within the frame to reach a conclusion\n";
			int move = checkVisibility(this_plane_parameters, this_continuous_bounding_box_points, 2);
			if(move==0)
			{
				_is_plane_covered = true;
			}
			else
			{
				_is_plane_covered = false;
			}
			cout << "[ DEBUG] [adjustForNextCapture] Is plane covered: " << _is_plane_covered << ", Move: " << move << "\n";
		}
		cout << "[ DEBUG] [adjustForNextCapture] Is plane covered: " << _is_plane_covered << "\n";
		if(_is_plane_covered)
		{
			copyNecessaryInfo();
		}
		else
		{
			// _is_plane_covered = false;
			cout << "[ INFO] [adjustForNextCapture] Restoring the yaw back and moving right by width_of_plane by 2\n";
			_is_big_plane = true;
			if(_node_completed_number_of_planes != _node_number_of_planes-1)
			{
				cout << "[ DEBUG] [adjustForNextCapture] Restoring the yaw. Rotating CounterClockwise by " << 3*fabs(_next_plane_angle)/4.0 << "\n";
				rotateCounterClockwise(3*fabs(_next_plane_angle)/4.0);
			}
			else
			{
				cout << "[ DEBUG] [adjustForNextCapture] Last plane to cover. Moving right\n";
			}
			cout << "[ DEBUG] [adjustForNextCapture] Moving right by " << width_of_3d_plane/(double)2.0 << "\n";
			moveRight(width_of_3d_plane/(double)2.0);
		}
	}
	else if(_next_plane_dir == COUNTERCLOCKWISE)
	{
		cout << "[ INFO] [adjustForNextCapture] Next plane is COUNTERCLOCKWISE wrt current plane\n";
		getCurrentPositionOfDrone();
		cout << "[ INFO] [adjustForNextCapture] Moving the drone horizontally by " << width_of_3d_plane << "\n";
		moveRight(width_of_3d_plane);
		cout << "[ INFO] [adjustForNextCapture] Changing the yaw counterclockwise by " << (_next_plane_angle/5.0) << "\n";
		getCurrentPositionOfDrone();
		designPathToChangeYaw(_node_current_pos_of_drone, _node_current_pos_of_drone[3]-(_next_plane_angle/5.0));
		moveDroneViaSetOfPoints(_interm_path);
		cout << "[ INFO] [adjustForNextCapture] Calling Jlinkage\n";
		doJLinkage();
		clear2dVector(test_plane_parameters);
		for (unsigned int i = 0; i < visited_plane_parameters.size(); ++i)
		{
			test_plane_parameters.push_back(visited_plane_parameters[i]);
		}
		test_plane_parameters.push_back(this_plane_parameters);
		print2dVector(visited_plane_parameters, "[ INFO] [adjustForNextCapture] All planes visited completely");
		print2dVector(test_plane_parameters, "[ INFO] [adjustForNextCapture] All planes visited completely including the current one");
		if(_node_completed_number_of_planes == _node_number_of_planes-1)
		{
			cout << "[ DEBUG] [adjustForNextCapture] Observe the plane without rotation\n";
			_is_plane_covered = isNewPlaneVisible(test_plane_parameters,
											jlink_all_plane_parameters, jlink_all_percentage_of_each_plane, false);
		}
		else
		{
			cout << "[ DEBUG] [adjustForNextCapture] Observe the plane by rotation: " 
						<< _next_plane_dir << "\n";
			_is_plane_covered = isNewPlaneVisible(test_plane_parameters,
											jlink_all_plane_parameters, jlink_all_percentage_of_each_plane, true, _next_plane_dir);
		}
		cout << "[ DEBUG] [adjustForNextCapture] Is plane covered: " << _is_plane_covered << "\n";
		if(!_is_plane_covered)
		{
			cout << "[ DEBUG] [adjustForNextCapture] Checking if the right edge is visible within heuristics\n";
			int move = checkVisibility(this_plane_parameters, this_continuous_bounding_box_points, 2);
			if(move==0)
			{
				_is_plane_covered = true;
			}
			else
			{
				_is_plane_covered = false;
			}
			cout << "[ DEBUG] [adjustForNextCapture] Is plane covered: " << _is_plane_covered << ", Move: " << move << "\n";
		}
		if(_is_plane_covered)
		{
			copyNecessaryInfo();
		}
		else
		{
			// _is_plane_covered = false;
			_is_big_plane = true;
			getCurrentPositionOfDrone();
			print1dVector(_node_current_pos_of_drone, "[ INFO] [adjustForNextCapture] Current position of drone");
			cout << "[ INFO] The plane in inspection is a big one. Restoring the yaw back to complete this plane\n";
			designPathToChangeYaw(_node_current_pos_of_drone, _node_current_pos_of_drone[3]+(_next_plane_angle/5.0));
			moveDroneViaSetOfPoints(_interm_path);
		}
	}
	if(_node_completed_number_of_planes == _node_number_of_planes)
	{
		cout << "[ DEBUG] [adjustForNextCapture] VPP size: " << visited_plane_parameters.size() << "\n";
		assert(visited_plane_parameters.size() == visited_continuous_bounding_box_points.size());
		print2dVector(visited_plane_parameters, "[ DEBUG] [adjustForNextCapture] VPP");
		print2dVector(visited_continuous_bounding_box_points, "[ DEBUG] [adjustForNextCapture] VCBB");
		string filename = "Plane_Info.txt";
		cout << "[ DEBUG] [adjustForNextCapture] Writing info gathered to " << filename << "\n";
		for (unsigned int i = 0; i < visited_plane_parameters.size(); ++i)
		{
			image_gui->WriteInfoToFile(visited_continuous_bounding_box_points[i], visited_plane_parameters[i], i+1, filename);
		}
		print2dVector(visited_plane_parameters, "[ INFO] [adjustForNextCapture] All Plane Parameters");
		print2dVector(visited_continuous_bounding_box_points, "[ INFO] [adjustForNextCapture] All Plane BBP");
		print2dVector(visited_motion_points, "[ INFO] [adjustForNextCapture] All motion points");
		cout << "[ INFO] [adjustForNextCapture] All planes are covered!\n";
		cout << "[ INFO] [adjustForNextCapture] Landing the quadcopter\n";
		sendLand();
	}
	if((_is_plane_covered) &&
			(_node_completed_number_of_planes != _node_number_of_planes))
	{
			cout << "[ INFO] [adjustForNextCapture] Current plane is covered. All planes are covered\n";
			cout << "[ DEBUG] [adjustForNextCapture] VPP: \n";
			print2dVector(visited_plane_parameters, "[ DEBUG] [adjustForNextCapture] Visited plane parameters");
			cout << "[ INFO] [adjustForNextCapture] Aligning quadcoter to next plane\n";
			_is_adjusted = true;
			alignQuadcopterToNextPlaneAdvanced();
	}
	else
	{
		cout << "[ INFO] [adjustForNextCapture] Please click 4 points on the DRONE CAMERA FEED window\n";
	}
	cout << "[ INFO] [adjustForNextCapture] Completed\n";
}

/**
 * @brief
 * @details To be implemented if alignQuadcopterToNextPlane() doesnot work as expected
 */
void
ControlUINode::alignQuadcopterToNextPlaneAdvanced()
{
	cout << "[ INFO] [alignQuadcopterToNextPlaneAdvanced] Started\n";
	cout << "[ INFO] [alignQuadcopterToNextPlaneAdvanced] Completed no. of planes: " 
			<< _node_completed_number_of_planes
			<< ", Total number of planes: " << _node_number_of_planes << "\n";
	float angle;
	if(_next_plane_dir == COUNTERCLOCKWISE)
	{
		bool yaw_change = false;
		if(!_is_adjusted)
		{
			yaw_change = true;
			cout << "[ INFO] [alignQuadcopterToNextPlaneAdvanced] Not adjusted by width of plane\n";
			Point3f top_left = visited_continuous_bounding_box_points.back()[0];
			Point3f top_right = visited_continuous_bounding_box_points.back()[1];
			double width_of_3d_plane = (double)fabs(sqrt( (top_right.x - top_left.x)*(top_right.x - top_left.x) +
												(top_right.y - top_left.y)*(top_right.y - top_left.y) +
												(top_right.z - top_left.z)*(top_right.z - top_left.z) ));
			cout << "[ INFO] [alignQuadcopterToNextPlaneAdvanced] Width of the plane is: " << width_of_3d_plane << "\n";
			cout << "[ INFO] [alignQuadcopterToNextPlaneAdvanced] Moving the drone horizontally by " << width_of_3d_plane << "\n";
			moveRight(width_of_3d_plane);
			float m = 1.2;
			if(_next_plane_angle >= 70.0)
			{
				do
				{
					cout << "[ DEBUG] [alignQuadcopterToNextPlaneAdvanced] Not adjusted. Can't see a new plane\n";
					moveRight(m);
					doJLinkage();
					m = m-0.2;
				}while(_sig_plane_index == -1);
			}
		}
		else
		{
			float m = 1.2;
			if(_next_plane_angle >= 70.0)
			{
				do
				{
					cout << "[ DEBUG] [alignQuadcopterToNextPlaneAdvanced] Not adjusted. Can't see a new plane\n";
					moveRight(m);
					doJLinkage();
					m = m-0.2;
				}while(_sig_plane_index == -1);
			}
		}
		_is_adjusted = false;
		doJLinkage();
		cout << "[ INFO] [alignQuadcopterToNextPlaneAdvanced] Next plane is counter clockwise to current plane\n";
		image_gui->setContinuousBoundingBoxPoints(jlink_all_continuous_bounding_box_points);
		image_gui->setSigPlaneBoundingBoxPoints(this_continuous_bounding_box_points);
		cout << "[ DEBUG] [alignQuadcopterToNextPlaneAdvanced] Rendering poly and sig plane\n";
		image_gui->setRender(false, false, true, true);
		image_gui->renderFrame();
		cout << "[ DEBUG] [alignQuadcopterToNextPlaneAdvanced] _sig_plane_index: " << _sig_plane_index << "\n";
		cout << "[ DEBUG] [alignQuadcopterToNextPlaneAdvanced] _actual_plane_index: " << _actual_plane_index << "\n";
		bool move_drone = false;
		/*bool new_plane_visible = isNewPlaneVisible(visited_plane_parameters, jlink_all_plane_parameters,
					jlink_all_percentage_of_each_plane, true, _next_plane_dir);*/
		if(_sig_plane_index == -2)
		{
			cout << "[ DEBUG] [alignQuadcopterToNextPlaneAdvanced] Seems I can see only new planes\n";
			_actual_plane_index = 0;
			move_drone = true;
		}
		else if(_sig_plane_index == -1)
		{
			cout << "[ DEBUG] [alignQuadcopterToNextPlaneAdvanced] Seems I can't see a new plane\n";
			cout << "[ DEBUG] [alignQuadcopterToNextPlaneAdvanced] Moving more right by 1.2\n";
			float m = 1.2;
			do
			{
				cout << "[ DEBUG] [alignQuadcopterToNextPlaneAdvanced] Not adjusted. Can't see a new plane\n";
				moveRight(m);
				doJLinkage();
				m = m-0.2;
			}while(_sig_plane_index == -1);
		}
		else
		{
			cout << "[ DEBUG] [alignQuadcopterToNextPlaneAdvanced] Seems I can see both new and old planes\n";
			move_drone = true;
		}
		if(move_drone)
		{
			// bool aligned = (_sig_plane_index == 0)? true: false;
			bool aligned = false;
			double denom = 3.0;
			double initial_move = 0.8;
			if(_next_plane_angle <= 50.0)
			{
				initial_move = 0.8;
			}
			else if(_next_plane_angle > 50.0 && _next_plane_angle <= 70.0)
			{
				initial_move = 1.0;
			}
			else if(_next_plane_angle > 70.0 && _next_plane_angle <= 90.0)
			{
				initial_move = 1.3;
				denom = 3.0;
			}
			else
			{
				initial_move = 1.6;
				denom = 2.0;
			}
			int rounds = 0;
			double angle_to_rotate;
			do
			{
				rounds++;
				if(rounds >= 6)
				{
					angle_to_rotate = (double)fabs(angle);
				}
				else
				{
					angle_to_rotate = _next_plane_angle/denom;
				}
				cout << "[ DEBUG] [alignQuadcopterToNextPlaneAdvanced] Round no.: " << rounds <<
							", Next plane angle: " << _next_plane_angle << "\n";
				if(yaw_change)
				{
					cout << "[ DEBUG] [alignQuadcopterToNextPlaneAdvanced] Changing yaw cc by: " << angle_to_rotate << "\n";
					rotateCounterClockwise(angle_to_rotate);
				}
				yaw_change = true;
				doJLinkage();
				cout << "[ DEBUG] [alignQuadcopterToNextPlaneAdvanced] Linearly translating along X by " << initial_move << "\n";
				getCurrentPositionOfDrone();
				/*float point_distance = getPointToPlaneDistance(this_plane_parameters, _node_current_pos_of_drone);
				int move = (_fixed_distance >= point_distance) ? -1: 1;
				float step_distance = fabs(_fixed_distance - point_distance);
				cout << "[ DEBUG] [alignQuadcopterToNextPlaneAdvanced] Drone Distance: " << point_distance
							<< ", Fixed Distance: " << _fixed_distance << "\n";
				cout << "[ DEBUG] [alignQuadcopterToNextPlaneAdvanced] Move: " << move << ", Step Distance: " << step_distance << "\n";
				if(move == -1)
				{
					cout << "[ DEBUG] [alignQuadcopterToNextPlaneAdvanced] Moving backwards\n";
					moveBackward(step_distance);
				}
				else if(move == 1)
				{
					cout << "[ DEBUG] [alignQuadcopterToNextPlaneAdvanced] Moving forwards\n";
					moveForward(step_distance);
				}*/
				moveInDirection(this_plane_parameters, _node_current_pos_of_drone, this_sorted_3d_points);
				getCurrentPositionOfDrone();
				int move = (_node_current_pos_of_drone[2] >= _fixed_height) ? -1:1;
				float step_distance = fabs(_node_current_pos_of_drone[2] - _fixed_height);
				cout << "[ DEBUG] [alignQuadcopterToNextPlaneAdvanced] Drone Height: " << _node_current_pos_of_drone[2]
							<< ", Fixed Height: " << _fixed_height << "\n";
				cout << "[ DEBUG] [alignQuadcopterToNextPlaneAdvanced] Move: " << move << ", Step Distance: " << step_distance << "\n";
				if(move == -1)
				{
					cout << "[ DEBUG] [alignQuadcopterToNextPlaneAdvanced] Moving down\n";
					moveDown(step_distance);
				}
				else if(move == 1)
				{
					cout << "[ DEBUG] [alignQuadcopterToNextPlaneAdvanced] Moving up\n";
					moveUp(step_distance);
				}
				moveRight(initial_move);
				doJLinkage();
				Point3f normal_new_plane(this_plane_parameters[0],
											this_plane_parameters[1], this_plane_parameters[2]);
				cout << "[ DEBUG] [alignQuadcopterToNextPlaneAdvanced] Normal new plane: " << normal_new_plane << "\n";
				getCurrentPositionOfDrone();
				print1dVector(_node_current_pos_of_drone, "[ DEBUG] [alignQuadcopterToNextPlaneAdvanced] Current position of drone");
				_node_dest_pos_of_drone.clear();
				_node_dest_pos_of_drone.push_back(0.0);
				_node_dest_pos_of_drone.push_back(1.0);
				_node_dest_pos_of_drone.push_back(0.0);
				_node_dest_pos_of_drone.push_back(0.0);
				convertWRTQuadcopterOrigin(_node_current_pos_of_drone, _node_dest_pos_of_drone, _node_ac_dest_pos_of_drone);
				print1dVector(_node_current_pos_of_drone, "[ INFO] [alignQuadcopterToNextPlaneAdvanced] Current position of drone");
				print1dVector(_node_dest_pos_of_drone, "[ DEBUG] [alignQuadcopterToNextPlaneAdvanced] Dest position of drone");
				print1dVector(_node_ac_dest_pos_of_drone, "[ INFO] [alignQuadcopterToNextPlaneAdvanced] Actual dest position of drone");
				Point3f pYAxis(_node_ac_dest_pos_of_drone[0], _node_ac_dest_pos_of_drone[1], _node_ac_dest_pos_of_drone[2]);
				Point3f pOrigin(_node_current_pos_of_drone[0], _node_current_pos_of_drone[1], _node_current_pos_of_drone[2]);
				Point3f projectedNormal(pYAxis-pOrigin);
				Point3f plane_params(normal_new_plane.x, normal_new_plane.y, 0.0);
				projectedNormal.z = 0.0;
				angle = findAngle(projectedNormal, plane_params);
				cout << "[ DEBUG] [alignQuadcopterToNextPlaneAdvanced] Normal new plane: " << plane_params << "\n";
				cout << "[ DEBUG] [alignQuadcopterToNextPlaneAdvanced] Normal of quadcopter: " << projectedNormal << "\n";
				cout << "[ DEBUG] [alignQuadcopterToNextPlaneAdvanced] Opposite Normal of quadcopter: " << (-1.0)*projectedNormal << "\n";
				cout << "[ DEBUG] [alignQuadcopterToNextPlaneAdvanced] Angle (radians): " << angle << "\n";
				angle = -angle*180/M_PI;
				//float angle_diff = fabs(fabs(angle) - fabs(_node_current_pos_of_drone[3]));
				cout << "[ DEBUG] [alignQuadcopterToNextPlaneAdvanced] angle: " << angle << "\n";
				/*cout << "[ DEBUG] [alignQuadcopterToNextPlaneAdvanced] drone_angle: " << _node_current_pos_of_drone[3] << "\n";
				cout << "[ DEBUG] [alignQuadcopterToNextPlaneAdvanced] angle_diff: " << angle_diff << "\n";*/
				if(fabs(angle) > 20.0)
				{
					aligned = false;
					if(fabs(angle) > 20.0 && fabs(angle) <= 30.0)
					{
						initial_move = initial_move - 0.1;
					}
					else if(fabs(angle) > 30.0 && fabs(angle) <= 50.0)
					{
						//initial_move = initial_move - 0.05;
						initial_move = 0.7;
					}
					else if(fabs(angle) > 50.0 && fabs(angle) <= 70.0)
					{
						//initial_move = initial_move - 0.025;
						initial_move = 0.8;
					}
					else
					{
						initial_move = 1.0;
					}
					cout << "[ DEBUG] [alignQuadcopterToNextPlaneAdvanced] Next Move distance: " << initial_move << "\n";
				}
				else
				{
					aligned = true;
				}
				denom += 1.0;
			} while(!aligned);
		}
		cout << "[ DEBUG] [alignQuadcopterToNextPlaneAdvanced] Aligning quadcopter to the new plane\n";
		alignQuadcopterToCurrentPlane();
		doJLinkage();
		adjustLeftEdge();
		cout << "[ DEBUG] [alignQuadcopterToNextPlaneAdvanced] Aligned quadcopter to the new plane\n";
	}
	else if(_next_plane_dir == CLOCKWISE)
	{
		double denom = 3.0;
		if(!_is_adjusted)
		{
			/*cout << "[ INFO] [alignQuadcopterToNextPlaneAdvanced] Not adjusted by width of plane\n";
			Point3f top_left = visited_continuous_bounding_box_points.back()[0];
			Point3f top_right = visited_continuous_bounding_box_points.back()[1];
			double width_of_3d_plane = (double)fabs(sqrt( (top_right.x - top_left.x)*(top_right.x - top_left.x) +
												(top_right.y - top_left.y)*(top_right.y - top_left.y) +
												(top_right.z - top_left.z)*(top_right.z - top_left.z) ));
			cout << "[ INFO] [alignQuadcopterToNextPlaneAdvanced] Width of the plane is: " << width_of_3d_plane << "\n";
			cout << "[ INFO] [alignQuadcopterToNextPlaneAdvanced] Moving the drone horizontally by " << width_of_3d_plane/3.0 << "\n";
			moveRight(width_of_3d_plane/(double)4.0);*/
			cout << "[ DEBUG] [alignQuadcopterToNextPlaneAdvanced] Not adjusted. Can't see a new plane\n";
			cout << "[ DEBUG] [alignQuadcopterToNextPlaneAdvanced] Therefore, rotating clockwise by: " << 
					3*fabs(_next_plane_angle)/(double)4.0 << "\n";
			rotateClockwise(3*fabs(_next_plane_angle)/(double)4.0);
			cout << "[ INFO] [adjustForNextCapture] Moving backwards by 0.5\n";
			moveBackward(0.5);
			cout << "[ INFO] [adjustForNextCapture] Moving forwards by 0.5\n";
			moveForward(0.5);
			while(_sig_plane_index == -1)
			{
				rotateClockwise(fabs(_next_plane_angle)/denom);
				doJLinkage();
				denom = denom + 1.0;
			}
		}
		else
		{
			while(_sig_plane_index == -1)
			{
				cout << "[ DEBUG] [alignQuadcopterToNextPlaneAdvanced] Adjusted. Can't see a new plane\n";
				rotateClockwise(fabs(_next_plane_angle)/denom);
				doJLinkage();
				denom = denom + 1.0;
			}
		}
		_is_adjusted = false;
		rotateClockwise(fabs(_next_plane_angle)/denom);
		denom = denom + 1.0;
		rotateClockwise(fabs(_next_plane_angle)/denom);
		doJLinkage();
		cout << "[ INFO] [alignQuadcopterToNextPlaneAdvanced] Next plane is clockwise to current plane\n";
		image_gui->setContinuousBoundingBoxPoints(jlink_all_continuous_bounding_box_points);
		image_gui->setSigPlaneBoundingBoxPoints(this_continuous_bounding_box_points);
		cout << "[ DEBUG] [alignQuadcopterToNextPlaneAdvanced] Rendering poly and sig plane\n";
		image_gui->setRender(false, false, true, true);
		image_gui->renderFrame();
		cout << "[ DEBUG] [alignQuadcopterToNextPlaneAdvanced] _sig_plane_index: " << _sig_plane_index << "\n";
		cout << "[ DEBUG] [alignQuadcopterToNextPlaneAdvanced] _actual_plane_index: " << _actual_plane_index << "\n";
		cout << "[ DEBUG] [alignQuadcopterToNextPlaneAdvanced] Moving backwards by 0.4\n";
		moveBackward(0.4);
		cout << "[ DEBUG] [alignQuadcopterToNextPlaneAdvanced] Moving forwards by 0.4\n";
		moveForward(0.4);
		cout << "[ DEBUG] [alignQuadcopterToNextPlaneAdvanced] Aligning the quadcoper to the new plane\n";
		alignQuadcopterToCurrentPlane();
		adjustLeftEdge();
	}
	_stage_of_plane_observation = true;
	_node_main_directions.pop_front();
	_node_main_angles.pop_front();
	if(_node_main_angles.size() == 0)
	{
		_next_plane_dir = CLOCKWISE;
		_next_plane_angle = 0.0;
	}
	else
	{
		_next_plane_dir = _node_main_directions.front();
		_next_plane_angle = _node_main_angles.front();
	}
	cout << "[ DEBUG] [alignQuadcopterToNextPlaneAdvanced] Completed\n";
	return ;
}
