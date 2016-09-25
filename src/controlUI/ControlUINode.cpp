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
	write3DPointsToCSV(_3d_points);
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

void
ControlUINode::sendLand()
{
	//if(isControlling)
	land_pub.publish(std_msgs::Empty());
}

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


float
ControlUINode::getDistanceToSeePlane(int height)
{
	// Read points
	vector<Point2f> imagePoints = GenerateMy2DPoints();
	float width = (16.0/9.0)*height;
	float drone_length = 0.6;
	vector<Point3f> objectPoints = GenerateMy3DPoints(width, height);

	//cout << "There are " << imagePoints.size() << " imagePoints and " 
	//	<< objectPoints.size() << " objectPoints." << endl;
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
	//cout << "Initial cameraMatrix:\n" << cameraMatrix << endl;

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

	cout << "Expected Quadcopter Location: (" << tvec.at<double>(0) << ", " 
		<< tvec.at<double>(2) << ", " << -tvec.at<double>(1) << ")\n";
	double dist = tvec.at<double>(2);

	return dist;
}

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
	/*for(unsigned int i = 0; i < points.size(); ++i)
	{
		cout << points[i] << ", ";
	}
	cout << "\n";*/
	return points;
}

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
	/*for(unsigned int i = 0; i < points.size(); ++i)
	{
		cout << points[i] << ", ";
	}
	cout << "\n";*/
	return points;
}

/**
 * @brief Move the drone to the destination point
 * @details
 * @param [vector<double>] dest_point - Final Destination of quadcopter
 * 									Includes yaw in the vector
 * @return
 */
void
ControlUINode::moveDroneToPosition(const vector<double> &dest_point)
{
	char buf[100];
	commands.clear();
	targetPoints.clear();
	snprintf(buf, 100, "c goto %lf %lf %lf %lf", 
		dest_point[0], dest_point[1], dest_point[2], dest_point[3]);
	std_msgs::String s;
	s.data = buf;
	ROS_INFO("Message: ");
	ROS_INFO(buf);
	commands.push_back(s);
	targetPoints.push_back(dest_point);
}

/**
 * @brief Move the drone to the destination point via a set of points
 * @details
 * @param [vector< vector<double> >] dest_points - Points to where quadcopter has to travel
 * 									Includes yaw in the vector
 * @return
 */
void
ControlUINode::moveDroneViaSetOfPoints(const vector< vector<double> > &dest_points)
{
	char buf[100];
	commands.clear();
	targetPoints.clear();
	for (int i = 0; i < dest_points.size(); ++i)
	{
		snprintf(buf, 100, "c goto %lf %lf %lf %lf", 
			dest_points[i][0], dest_points[i][1], dest_points[i][2], dest_points[i][3]);
		std_msgs::String s;
		s.data = buf;
		ROS_INFO("Message: ");
		ROS_INFO(buf);
		commands.push_back(s);
		targetPoints.push_back(dest_points[i]);
	}
}

/**
 * @brief Generates the set of points (smoothly distributed) which drone has to follow to move from start to end
 * @details
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
	// See getInitialPath for help
	vector<double> start_3d_pos, end_3d_pos;
	for (int i = 0; i < 3; ++i)
	{
		start_3d_pos.push_back(start[i]);
		end_3d_pos.push_back(end[i]);
	}
	getInitialPath(start_3d_pos, end_3d_pos, 
								start[3], end[3], path);
}

/**
 * @brief Move the drone to the destination point via a set of points
 * @details
 * @param [vector< vector<double> >] dest_points - Points to where quadcopter has to travel
 * 									Includes yaw in the vector
 * @return
 */
void
ControlUINode::designPathToChangeYaw(const vector<double> &curr_point, 
				double dest_yaw, 
				vector< vector<double> > &xyz_yaw)
{
	char buf[100];
	commands.clear();
	targetPoints.clear();
	double yaw_diff = dest_yaw - curr_point[3];
	vector<double> interm_point;
	clearDoubleVector(xyz_yaw);
	double prevYaw = curr_point[3], desiredYaw = dest_yaw;
	for(int i = 0; i < 6; i++)
	{
		interm_point.push_back(curr_point[0]);
		interm_point.push_back(curr_point[1]);
		interm_point.push_back(curr_point[2]);
		interm_point.push_back(prevYaw*(5-i)/5 + desiredYaw*i/5);
		xyz_yaw.push_back(interm_point);
	}
}

/**
 * @brief Get the bounding box points of each plane by autonomously navigating the quadcopter
 *
 */
void
ControlUINode::getMeTheMap(const vector< double > &angles,
							const vector< RotateDirection > directions,
							int min_height_of_plane,
							int max_height_of_plane)
{
	vector<float> current_pos_of_drone, start_pos, end_pos;
	int plane_num = 01;
	float min_distance = getDistanceToSeePlane(min_height_of_plane);
	float max_distance = getDistanceToSeePlane(max_height_of_plane);
	// At the end of CoverTheCurrentPlane(), the quadcopter is expected at the center of the
	// plane (both wrt width and height) it is currently seeing
	if(directions.size()==0)
	{
		// CLOCKWISE - to prevent is crashing it with nearby adjacent plane (if it exists)
		CoverTheCurrentPlane(plane_num, min_distance, max_distance, CLOCKWISE);
	}
	else
	{
		CoverTheCurrentPlane(plane_num, min_distance, max_distance, directions[0]);
		/* directions*/
		int number_of_rotations = directions.size();
		for(int i = 0; i < number_of_rotations; i++)
		{
			current_pos_of_drone.clear();
			double angle = angles[i];
			MoveQuadcopterToNextPlane(directions[i], angle);
			plane_num++;
			if(i == number_of_rotations-1)
			{
				CoverTheCurrentPlane(plane_num, min_distance, max_distance, CLOCKWISE);
			}
			else
			{
				CoverTheCurrentPlane(plane_num, min_distance, max_distance, directions[i+1]);
			}
		}
	}
	sendLand();
}


/**
 * @brief Cover the current plane visible to the quadcopter camera
 * 			At the end of CoverTheCurrentPlane(), the quadcopter is expected at 
 * 			the center of the plane (both wrt width and height) it is currently seeing
 * @details 
 * @param [int] plane_num - Which plane you're covering
 * @param [float] max_height - Maximum height of the plane
  * @return
 */
void
ControlUINode::CoverTheCurrentPlane (int plane_num, float min_distance, float max_distance, 
									RotateDirection dir)
{
	/* Push that position to vector */
	vector<double> dest_pos_of_drone;
	vector<double> ac_dest_pos_of_drone;
	// To check if the plane is covered completely
	bool planeCovered = false;
	/* As obtained from ControlUINode.cpp Line 429 */
	bool stage = true; // Variable to indicate whether it's the first time seeing the plane
	while (!planeCovered)
	{
		// First param: Number of points clicked on the screen 
		// Second param: Number of Key points detected
		image_gui->setNumberOfPoints(0, 0);
		// RendeRect: false, RenderPoly: false, RenderSignificantPlane: false
		image_gui->setRender(false, false, false);
		// Clear all Vectors
		image_gui->clearInputVectors();
		// 2d image points clicked
		vector< vector<int> > points_clicked;
		// the 3d keypoints of control node for nearest keypoints
		vector< vector<float> > key_points_nearest;
		// corners of the convex hull
		vector<int> cc_points;
		// corners of multiple planes bounding boxes in 3D
		vector< vector<Point3f> > continuous_bounding_box_points;
		// Plane parameters
		vector< vector<float> > plane_parameters;
		/* Adjust the quadcopter to see the top and bottom edge of current plane (plane in view) */
		/* Returns TRUE if the plane in view is covered completely, else FALSE*/
		bool flag = AdjustToSeeCurrentPlane(min_distance,
											max_distance, stage );
		stage = false;
		/* Mark the bounding points for the plane to be scanned */
		/* Output: vector<Point2f> pointsClicked */
		/* Loop until 4 points are clicked on the image */
		// Stop the system until all 4 points are clicked
		cout << "[ INFO] Start Clicking the bounding box points\n";
		image_gui->getPointsClicked(points_clicked);
		while(points_clicked.size()!=4) {image_gui->getPointsClicked(points_clicked);}
		// @todo-meghshyam Should I ask user to press a key to indicate he has finished clicking the points
		// @todo-sir Get the bounding box points of the leftmost-plane/significant-plane from the marked points
		image_gui->extractBoundingPoly();
		// image_gui->renderSignificantPlane = true; // @todo-me See 288-328 for implementation
		image_gui->setRender(false, false, true);
		// @todo-me Only render rectangle of significant plane
		image_gui->renderFrame();
		//image_gui->clearOutputVectors();
		// Calls JLinkage and finds all planes within the clicked region
		// @todo-me @todo-sir Also return the most significant plane found
		// @todo-me fix the renderFrame for significant plane
		int significantPlaneIndex = 0;
		image_gui->getCCPoints(cc_points);
		image_gui->getPointsClicked(points_clicked);
		/*image_gui->getPlaneParameters(plane_parameters);
		image_gui->getContinuousBoundingBoxPoints(continuous_bounding_box_points);*/
		int size = continuous_bounding_box_points.size();
		assert(size == plane_parameters.size());
		for (int i = 0; i < size; ++i)
		{
			continuous_bounding_box_points[i].clear();
			plane_parameters[i].clear();
		}
		continuous_bounding_box_points.clear();
		plane_parameters.clear();
		fitMultiplePlanes3d(cc_points, points_clicked, plane_parameters, continuous_bounding_box_points);
		/* Write the bounding box points to file Plane_Info.txt */
		/* Write the plane parameters to the same file */
		string file_name = "Plane_Info.txt";
		// bounding_box_points[0] - corresponds to the points of the leftmost plane
		// Write points of significant plane to file
		// @todo-me So indices might change form 0 to something else. Please NOTE
		image_gui->WriteInfoToFile(continuous_bounding_box_points[significantPlaneIndex], 
			plane_parameters[significantPlaneIndex], plane_num, file_name);
		if (flag)
		{
			planeCovered = true;
		}
		else
		{
			/* @todo-me Shift by some amount to see the next part of the same plane */
			Point3f top_right = continuous_bounding_box_points[significantPlaneIndex][2];
			Point3f top_left = continuous_bounding_box_points[significantPlaneIndex][3];
			int width_of_3d_plane = fabs(sqrt( (top_right.x - top_left.x)*(top_right.x - top_left.x) +
												(top_right.y - top_left.y)*(top_right.y - top_left.y) + 
												(top_right.z - top_left.z)*(top_right.z - top_left.z) ));
			/* I think 1/4 of width of the plane is better */
			// @todo-sir @todo-meghshyam Currently the plane is not yet covered so second plane is not in the picture
			// Get the final destination of quadcopter and use getInitialPath() for moving quadcopter
			vector<double> current_pos_of_drone;
			pthread_mutex_lock(&pose_CS);
				getCurrentPositionOfDrone(current_pos_of_drone);
			pthread_mutex_unlock(&pose_CS);
			dest_pos_of_drone.clear();
			// @todo-me Ensure that next plane is not being hit in CLOCKWISE
			if(dir == CLOCKWISE)
			{
				dest_pos_of_drone.push_back(current_pos_of_drone[0] + width_of_3d_plane/2); // heuristic
			}
			else
			{
				dest_pos_of_drone.push_back(current_pos_of_drone[0] + width_of_3d_plane); // heuristic
			}
			dest_pos_of_drone.push_back(current_pos_of_drone[1]);
			dest_pos_of_drone.push_back(current_pos_of_drone[2]);
			dest_pos_of_drone.push_back(current_pos_of_drone[3]);
			vector< vector<double> > path;
			convertWRTQuadcopterOrigin(current_pos_of_drone, dest_pos_of_drone, ac_dest_pos_drone);
			pthread_mutex_lock(&command_CS);
				designPathForDrone(current_pos_of_drone, ac_dest_pos_of_drone, path);
				moveDroneViaSetOfPoints(path);
			pthread_mutex_unlock(&command_CS);
			planeCovered = false;
		}
	}
}

/**
 * @brief Calculate the optimal position of quadcopter such that it can see the top and bottom
 * 			edge of the plane
 * @details
 * @param
 * @return
 */
bool
ControlUINode::AdjustToSeeCurrentPlane(float min_distance, float max_distance, bool stage)
{
	vector< Point2f > image_bounding_box_points;
	bool flag = true; // flag indicates whether or not the plane is covered completely
	// @todo-sir Using focal length and max_height to calculate position
	bool move = true;
	vector<double> dest_pos_of_drone;
	vector<double> ac_dest_pos_drone;
	vector<float> current_plane_parameters;
	vector<double> current_pos_of_drone;
	if(stage)
	{
		float max_ac_dis_plane = max_distance;
		float min_ac_dis_plane = min_distance;
		float ac_dis_plane = max_ac_dis_plane;
		current_plane_parameters = fitPlane3dForTheCurrentPlane(); // NOTE: Not fitMultiplePlane3D() because we are currently seeing only 1 plane
		float a = current_plane_parameters[0];
		float b = current_plane_parameters[1];
		float c = current_plane_parameters[2];
		float d = current_plane_parameters[3];
		pthread_mutex_lock(&pose_CS);
			getCurrentPositionOfDrone(current_pos_of_drone);
		pthread_mutex_unlock(&pose_CS);
		float x0 = current_pos_of_drone[0];
		float y0 = current_pos_of_drone[1];
		float z0 = current_pos_of_drone[2];
		float quad_dis_plane = fabs(a*x0+b*y0+c*z0+d)/sqrt(a*a+b*b+c*c); // Current distance of quadcopter from the plane
		dest_pos_of_drone.clear();
		float distance_to_move = fabs(quad_dis_plane - ac_dis_plane);
		if(quad_dis_plane > ac_dis_plane)
		{
			// @todo-sir @todo-meghshyam What distance should the quadcopter move?
			dest_pos_of_drone.push_back(current_pos_of_drone[0]);
			dest_pos_of_drone.push_back(current_pos_of_drone[1] + distance_to_move);
			dest_pos_of_drone.push_back(current_pos_of_drone[2]);
		}
		else if(quad_dis_plane < ac_dis_plane)
		{
			dest_pos_of_drone.push_back(current_pos_of_drone[0]);
			dest_pos_of_drone.push_back(current_pos_of_drone[1] - distance_to_move);
			dest_pos_of_drone.push_back(current_pos_of_drone[2]);
		}
		else
		{
			move = false;
		}
	}
	else
	{
	}
	vector< vector<double> > xyz_yaw;
	if(move)
	{
		pthread_mutex_lock(&pose_CS);
			getCurrentPositionOfDrone(current_pos_of_drone);
		pthread_mutex_unlock(&pose_CS);
		dest_pos_of_drone.push_back(current_pos_of_drone[3]);
		clearDoubleVector(xyz_yaw);
		convertWRTQuadcopterOrigin(current_pos_of_drone, dest_pos_of_drone, ac_dest_pos_drone);
		pthread_mutex_lock(&command_CS);
			designPathForDrone(current_pos_of_drone, ac_dest_pos_of_drone, xyz_yaw);
			moveDroneViaSetOfPoints(xyz_yaw);
		pthread_mutex_unlock(&command_CS);
	}
	// corners of multiple planes bounding boxes in 3D
	vector< vector<Point3f> > continuous_bounding_box_points;
	// Plane parameters
	vector< vector<float> > plane_parameters;
	vector<Point3f> _in_points;
	pthread_mutex_lock(&keyPoint_CS);
		_in_points.clear();
		for (int i = 0; i < _3d_points.size(); ++i)
		{
			float x = _3d_points[i][0];
			float y = _3d_points[i][1];
			float z = _3d_points[i][2];
			_in_points.push_back(Point3f(x, y, z));
		}
	pthread_mutex_unlock(&keyPoint_CS);
	image_gui->getPlaneParameters(plane_parameters);
	image_gui->getContinuousBoundingBoxPoints(continuous_bounding_box_points);
	findMultiplePlanes(_in_points, plane_parameters, continuous_bounding_box_points);
	image_bounding_box_points.clear();
	project3DPointsOnImage(continuous_bounding_box_points[0], image_bounding_box_points);
	// According to makeBoundingRects.cpp (202-205) UV are pushed in 
	// bottom-left, bottom-right, top-left, top-right
	// So I assume XYZ are stored in the same manner
	// Right edge start is towards the bottom of the screen
	// Right edge end is towards the top of the screen
	Line2f right_edge(image_bounding_box_points[1], image_bounding_box_points[2]); 
	// @todo-sir @todo-meghshyam check this
	if( (right_edge.start.x >= 620.0 && right_edge.start.y >= 340.0) &&
		(right_edge.end.x >= 620.0 && right_edge.end.y <= 10.0) )
	{
		flag = true;
	}
	else
	{
		flag = false;
	}
	return flag;
}

/**
 * @brief Move the quadcopter to the next plane such that it can see the left edge of the new plane
 * @param [RotateDirection] Rotation Direction Of Quadcopter: CLOCKWISE, COUNTERCLOCKWISE
 */
void
ControlUINode::MoveQuadcopterToNextPlane(RotateDirection dir, double dest_rotation)
{
	vector<double> current_pos_of_drone;
	vector<double> dest_pos_of_drone;
	vector<double> ac_dest_pos_drone;
	vector< vector<double> > xyz_yaw;
	vector<Point3f> _in_points;
	vector< Point2f > image_bounding_box_points;
	pthread_mutex_lock(&pose_CS);
		getCurrentPositionOfDrone(current_pos_of_drone);
	pthread_mutex_unlock(&pose_CS);
	if(dir == CLOCKWISE)
	{
		// [MGP] -ve anti-clockwise , +ve clockwise
		double dest_yaw = yaw+(fabs(dest_rotation)); // heuristic
		designPathToChangeYaw(current_pos_of_drone, dest_yaw, xyz_yaw);
		pthread_mutex_lock(&command_CS);
			moveDroneViaSetOfPoints(xyz_yaw);
		pthread_mutex_unlock(&command_CS);
	}
	else if(dir == COUNTERCLOCKWISE)
	{
		// @todo-me Check for forward/backward
		// @todo-meghshyam Because the quadcopter is at the center of plane i-1
		// we need to move it by width of plane i-1
		double some_value = 5.0; // @todo-me Fix this heuristic
		dest_pos_of_drone.clear();
		dest_pos_of_drone.push_back(current_pos_of_drone[0]+some_value);
		dest_pos_of_drone.push_back(current_pos_of_drone[1]);
		dest_pos_of_drone.push_back(current_pos_of_drone[2]);
		dest_pos_of_drone.push_back(current_pos_of_drone[3]-(fabs(dest_rotation/2)));
		convertWRTQuadcopterOrigin(current_pos_of_drone, dest_pos_of_drone, ac_dest_pos_drone);
		pthread_mutex_lock(&command_CS);
			designPathForDrone(current_pos_of_drone, ac_dest_pos_of_drone, xyz_yaw);
			moveDroneViaSetOfPoints(xyz_yaw);
		pthread_mutex_unlock(&command_CS);
		pthread_mutex_lock(&pose_CS);
			getCurrentPositionOfDrone(current_pos_of_drone);
		pthread_mutex_unlock(&pose_CS);
		// @todo-me Fix these values in if-else block
		if(dest_rotation >= 75.0)
		{
			//some_value = max_ac_dis_plane;
			some_value = 5.0;
		}
		else
		{
			// @todo-me Calculate this value
			//some_value = width_of_plane;
			some_value = 5.0;
		}
		dest_rotation /= 2;
		dest_pos_of_drone.clear();
		dest_pos_of_drone.push_back(current_pos_of_drone[0]);
		dest_pos_of_drone.push_back(current_pos_of_drone[1]+some_value);
		dest_pos_of_drone.push_back(current_pos_of_drone[2]);
		dest_pos_of_drone.push_back(current_pos_of_drone[3]-(fabs(dest_rotation/4)));
		convertWRTQuadcopterOrigin(current_pos_of_drone, dest_pos_of_drone, ac_dest_pos_drone);
		pthread_mutex_lock(&command_CS);
			designPathForDrone(current_pos_of_drone, ac_dest_pos_of_drone, xyz_yaw);
			moveDroneViaSetOfPoints(xyz_yaw);
		pthread_mutex_unlock(&command_CS);
	}
	// By this time I expect the quadcopter to see the complete plane i
	pthread_mutex_lock(&keyPoint_CS);
		_in_points.clear();
		for (int i = 0; i < _3d_points.size(); ++i)
		{
			float x = _3d_points[i][0];
			float y = _3d_points[i][1];
			float z = _3d_points[i][2];
			_in_points.push_back(Point3f(x, y, z));
		}
	pthread_mutex_unlock(&keyPoint_CS);
	// corners of multiple planes bounding boxes in 3D
	vector< vector<Point3f> > continuous_bounding_box_points;
	// Plane parameters
	vector< vector<float> > plane_parameters;
	int size = continuous_bounding_box_points.size();
	assert(size == plane_parameters.size());
	for (int i = 0; i < size; ++i)
	{
		continuous_bounding_box_points[i].clear();
		plane_parameters[i].clear();
	}
	continuous_bounding_box_points.clear();
	plane_parameters.clear();
	findMultiplePlanes(_in_points, plane_parameters, continuous_bounding_box_points);
	image_bounding_box_points.clear();
	project3DPointsOnImage(continuous_bounding_box_points[0], image_bounding_box_points);
	vector<float> pos; pos.clear();
	double desiredYaw = 0;
	int newPlaneIndex = 1; // @todo-me Check this value
	Point3f projectedNormal(plane_parameters[newPlaneIndex][0], plane_parameters[newPlaneIndex][1], 0);
	Point3f yAxis(0,1,0);
	desiredYaw = findAngle(projectedNormal, yAxis);
	desiredYaw= desiredYaw*180/M_PI;
	float theta = desiredYaw;
	float dest_yaw;
	pthread_mutex_lock(&pose_CS);
		getCurrentPositionOfDrone(current_pos_of_drone);
	pthread_mutex_unlock(&pose_CS);
	dest_yaw = current_pos_of_drone[3];
	dest_pos_of_drone.clear();
	dest_pos_of_drone.push_back(x_drone);
	dest_pos_of_drone.push_back(y_drone);
	dest_pos_of_drone.push_back(z_drone);
	if(dir == CLOCKWISE)
		dest_yaw = yaw+theta;
	else
		dest_yaw = yaw-theta;
	dest_pos_of_drone.push_back(dest_yaw);
	convertWRTQuadcopterOrigin(current_pos_of_drone, dest_pos_of_drone, ac_dest_pos_drone);
	// Align the quadcoper to plane i
	pthread_mutex_lock(&command_CS);
		designPathForDrone(current_pos_of_drone, ac_dest_pos_of_drone, xyz_yaw);
		moveDroneViaSetOfPoints(xyz_yaw);
	pthread_mutex_unlock(&command_CS);
	// Adjust the quadcopter to see the left edge of the plane
	bool leftEdgeOfPlaneVisible = false;
	do
	{
		pthread_mutex_lock(&keyPoint_CS);
			_in_points.clear();
			for (int i = 0; i < _3d_points.size(); ++i)
			{
				float x = _3d_points[i][0];
				float y = _3d_points[i][1];
				float z = _3d_points[i][2];
				_in_points.push_back(Point3f(x, y, z));
			}
		pthread_mutex_unlock(&keyPoint_CS);
		int size = continuous_bounding_box_points.size();
		assert(size == plane_parameters.size());
		for (int i = 0; i < size; ++i)
		{
			continuous_bounding_box_points[i].clear();
			plane_parameters[i].clear();
		}
		continuous_bounding_box_points.clear();
		plane_parameters.clear();
		findMultiplePlanes(_in_points, plane_parameters, continuous_bounding_box_points);
		image_bounding_box_points.clear();
		project3DPointsOnImage(continuous_bounding_box_points[0], image_bounding_box_points);
		Line2f left_edge(image_bounding_box_points[0], image_bounding_box_points[3]);
		leftEdgeOfPlaneVisible = false;
		// @todo-me @todo-meghshyam Fix this heuristic
		if( (left_edge.start.x <= 10.0 && left_edge.start.y >= 340.0) &&
			(left_edge.end.x <= 10.0 && left_edge.end.y <= 10.0) )
		{
			leftEdgeOfPlaneVisible = true;
		}
		else
		{
			leftEdgeOfPlaneVisible = false;
			float distance = 0.25; // @todo-me heuristic. Kept it at 0.25m move leftwards
			pthread_mutex_lock(&pose_CS);
				getCurrentPositionOfDrone(current_pos_of_drone);
			pthread_mutex_unlock(&pose_CS);
			dest_pos_of_drone.clear();
			// @todo-me @todo-meghshyam Fix this heuristic value
			dest_pos_of_drone.push_back(current_pos_of_drone[0] - distance);
			dest_pos_of_drone.push_back(current_pos_of_drone[1]);
			dest_pos_of_drone.push_back(current_pos_of_drone[2]);
			dest_pos_of_drone.push_back(current_pos_of_drone[3]);
			clearDoubleVector(xyz_yaw);
			convertWRTQuadcopterOrigin(current_pos_of_drone, dest_pos_of_drone, ac_dest_pos_drone);
			pthread_mutex_lock(&command_CS);
				designPathForDrone(current_pos_of_drone, ac_dest_pos_of_drone, xyz_yaw);
				moveDroneViaSetOfPoints(xyz_yaw);
			pthread_mutex_unlock(&command_CS);
		}
	} while(!leftEdgeOfPlaneVisible);
}

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

void
ControlUINode::convertWRTQuadcopterOrigin(const vector<double> &current_pos_of_drone, 
											const vector<double> &dest_pos_of_drone, 
											vector<double> &ac_dest_pos_drone)
{
	ac_dest_pos_drone.clear();
	Mat rotationMatrix = Mat::eye(3, 3, CV_64F);
	double angle = current_pos_of_drone[3];
	rotationMatrix.at<double>(0, 0) = cos(angle);
	rotationMatrix.at<double>(0, 1) = -sin(angle);
	rotationMatrix.at<double>(1, 0) = sin(angle);
	rotationMatrix.at<double>(1, 1) = cos(angle);
	Mat translationVector(3, 1, DataType<double>::type);
	translationVector.at<double>(0, 0) = current_pos_of_drone[0];
	translationVector.at<double>(1, 0) = current_pos_of_drone[1];
	translationVector.at<double>(2, 0) = current_pos_of_drone[2];
	Mat dest_point_drone_origin_mat(3, 1, DataType<double>::type);
	dest_point_drone_origin_mat.at<double>(0, 0) = dest_pos_of_drone[0];
	dest_point_drone_origin_mat.at<double>(1, 0) = dest_pos_of_drone[1];
	dest_point_drone_origin_mat.at<double>(2, 0) = dest_pos_of_drone[2];
	Mat sub = rotationMatrix*translationVector;
	Mat b = dest_point_drone_origin_mat + sub;
	// How do I solve Ax = b?
	// Will this always be solvable?
	Mat x = rotationMatrix.inv() * b;
	ac_dest_pos_drone.push_back(x.at<double>(0, 0));
	ac_dest_pos_drone.push_back(x.at<double>(1, 0));
	ac_dest_pos_drone.push_back(x.at<double>(2, 0));
	ac_dest_pos_drone.push_back(angle+dest_pos_of_drone[3]);
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
