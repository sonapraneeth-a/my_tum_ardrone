#pragma once
/*****************************************************************************************
 * helperFunctions.h
 *
 *       Created on: 12-Mar-2015
 *    Last Modified: 12-Sep-2016
 *  Original Author: Anirudh Vemula
 *   Current Author: Meghshyam Govind Prasad
 *   Current Author: Sona Praneeth Akula
 *          Project: Quadcopter
 *      Description: 
 *
 * Date				Author							Modification
 * 12-Sep-2016	Sona Praneeth Akula			Added comments to the code
 * 08-Oct-2016	Sona Praneeth Akula 		Moved certain helper functions from ControlUINode to 
 *											this file
 *****************************************************************************************/

#ifndef _HELPER_FUNCTIONS_H
#define _HELPER_FUNCTIONS_H

#include <cmath>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <vector>

#include "Line2.hpp"


using namespace std;
using namespace cv;

/* A simple sign function */
/**
 * @brief @todo Why R-z is being returned which is identity
 * @details
 * @param [float] roll - 
 * @param [float] pitch - 
 * @param [float] yaw - 
 * @return
 */
inline static cv::Mat
getRotationMatrix(float roll, float pitch, float yaw)
{
	roll  = roll*M_PI/180;
	pitch = pitch*M_PI/180;
	yaw   = (-1.0)*yaw*M_PI/180;
	cv::Mat R_x = cv::Mat::eye(3,3, CV_32F);
	cv::Mat R_y = cv::Mat::eye(3,3, CV_32F);
	cv::Mat R_z = cv::Mat::eye(3,3, CV_32F);
	/*
	R_z.at<float>(0,0) = cos(yaw);
	R_z.at<float>(0,2) = -sin(yaw);
	R_z.at<float>(2,0) = sin(yaw);
	R_z.at<float>(2,2) = cos(yaw);
	*/
	return R_z; // Roll & Pitch are not reliable, also most of the time we will have yaw only
}

/**
 * @brief Get the sign of the number
 * @details
 * @param [float] x - Floating Number
 * @return [int] - Sign og the number: 1 is +ve, -1 is -ve 
 */
inline static int
signD(float x)
{
	return (x>0)-(x<0);
}

/**
 * @brief 
 * @details
 * @param
 * @return
 */
inline static bool
onLeft(std::vector<int> p1, std::vector<int> p2, std::vector<int> p)
{
	if(signD( (p2[0]-p1[0])*(p[1]-p1[1]) - (p2[1]-p1[1])*(p[0]-p1[0]) ) == 1)
		return true;
	else 
		return false;
}

/**
 * @brief 
 * @details
 * @param
 * @return
 */
template<typename K>
inline static int
position(std::vector<int> p1, std::vector<int> p2, std::vector<K> p)
{
	return signD( (p2[0]-p1[0])*(p[1]-p1[1]) - (p2[1]-p1[1])*(p[0]-p1[0]) );
}

/**
 * @brief 
 * @details
 * @param
 * @return
 */
inline static bool
onSameSide(std::vector<int> p1, std::vector<int> p2, 
			std::vector<int> vertex, 
			std::vector<float> point)
{
	if(position(p1, p2, vertex) * position(p1, p2, point) > 0)
		return true;
	else
		return false;
}

/**
 * @brief 
 * @details
 * @param
 * @return
 */
inline static bool
liesInside(std::vector< std::vector<int> > ccPoints, std::vector<float> p)
{
	for(unsigned int i=0; i<ccPoints.size()-1; i++)
	{
		if(i==ccPoints.size()-2)
		{
			if(onSameSide(ccPoints[i], ccPoints[i+1], ccPoints[i-1], p))
				continue;
			else 
				return false;
		}
		else
		{
			if(onSameSide(ccPoints[i], ccPoints[i+1], ccPoints[i+2], p))
				continue;
			else
				return false;
		}
	}
	return true;
}

/**
 * @brief Dot product of two vectors
 * @details
 * @param [vector<float>] v1 - Vector 1
 * @param [vector<float>] v2 - Vector 2
 * @return [float] - Dot product of two vectors
 */
inline static float
innerProduct(std::vector<float> v1, std::vector<float> v2)
{
	assert(v1.size() == v2.size());
	float sum = 0;
	for(unsigned int i = 0; i < v1.size(); i++)
	{
		sum += v1[i]*v2[i];
	}
	return sum;
}

/**
 * @brief Get the normal of the plane
 * @details
 * @param [vector<float>] plane - Plane parameters (a, b, c, d)
 * @return [vector<float>] - Normal of the plane (a/norm, b/norm, c/norm)
 */
inline static std::vector<float>
getNormal(std::vector<float> plane)
{
	float a = plane[0];
	float b = plane[1];
	float c = plane[2];
	assert( (a != 0) || (b != 0) || (c != 0) );

	float norm = sqrt(a*a + b*b + c*c);

	std::vector<float> unitNorm;
	unitNorm.push_back(a/norm);
	unitNorm.push_back(b/norm);
	unitNorm.push_back(c/norm);

	return unitNorm;
}

/**
 * @brief Get the co-ordinates of the point projected onto a plane
 * @details
 * @param [vector<float>] plane - Plane parameters (a, b, c, d)
 * @param [vector<float>] pt - Point
 * @return [vector<float>] - Projected point onto the plane (x, y, z)
 */
inline static std::vector<float>
projectPoint(std::vector<float> plane, std::vector<float> pt)
{
	std::vector<float> normal = getNormal(plane);
	float dist = innerProduct(normal, pt) + plane[3];
	std::vector<float> pPoint;
	for(int i = 0; i < 3; i++)
	{
		float p = pt[i] - dist*normal[i];
		pPoint.push_back(p);
	}
	return pPoint;
}

/**
 * @brief Get the width and height of the polygon (convex hull) determined by
 * 			pPoints vector
 * @details Here X and Z have been considered because Y corresponds to depth in quadcopter
 * 			@todo How are the X and Z axis considered here?
 * @param [in] [vector< vector<float> >] - Set of points
 * @param [in] [float] - Left Upper corner
 * @param [out] [float] - Width of the plane/polygon
 * @param [out] [float] - Height of the plane/polygon
 * @return
 */
inline static void
getDimensions(std::vector< std::vector<float> > pPoints , 
				std::vector<float> &lu, float &width, float &height)
{
	float minX = pPoints[0][0];
	float minZ = pPoints[0][2];
	float maxX = pPoints[0][0];
	float maxZ = pPoints[0][2];

	for (unsigned int i = 0; i < pPoints.size(); ++i)
	{
		float x = pPoints[i][0];
		float z = pPoints[i][2];
		if(x < minX) minX = x;
		if(x > maxX) maxX = x;
		if(z < minZ) minZ = z;
		if(z > maxZ) maxZ = z;
	}

	lu.push_back(minX);
	lu.push_back(maxZ);

	width = maxX - minX;
	height = maxZ - minZ;
}

/**
 * @brief 
 * @details
 * @param
 * @return
 */
inline static void
getPDimensions (std::vector< std::vector<float> > pPoints, 
				std::vector<float> &lu, std::vector<float> &rd, 
				std::vector<float> &dd, float &maxD, float &maxR)
{

}

/**
 * @brief Given plane equation and point(x, __, z) on the plane, get the
 * 			unknown quantity
 * @details
 * @param [float] x - X co-ordinate of the point
 * @param [float] z - Z co-ordinate of the point
 * @param [vector<float>] plane - Plane equation given by (a, b, c, d)
 * @return [float] - Y co-ordinate of the point
 */
inline static float
getY(float x, float z, std::vector<float> plane)
{
	assert(plane[1]!=0);
	return (-plane[3]-plane[0]*x-plane[2]*z)/plane[1];
	// return 0;
}

/**
 * @brief Printing a set of 3d points
 * @details
 * @param [vector< vector<double> >] points - Vector containing the 3d points where
 * 									the interior vector is a point
 * @return
 */
inline static void
print3dPoints(std::vector< std::vector<double> > points)
{
	for(unsigned int i = 0; i < points.size(); i++)
	{
		printf("(x = %lf, y = %lf, z = %lf)\n", points[i][0], points[i][1], points[i][2]);
	}
}




/***********************************************************************************************
NEW FUNCITONS ADDED ON 08-OCT-2016
************************************************************************************************/

/**
 * @brief 
 * @details 
 * 
 */
template<typename T>
inline static void
normalize(vector<T> &vec)
{
	T magnitude = 0.0;
	for(unsigned int i = 0; i < vec.size(); i++)
	{
		magnitude += (vec[i]*vec[i]);
	}
	for(unsigned int i = 0; i < vec.size(); i++)
	{
		vec[i] /= magnitude;
	}
}

template<typename T>
inline static T
normalized_dot_product(const vector<T> &one, const vector<T> &two)
{
	assert(one.size() == two.size());
	vector<T> copy_one, copy_two;
	copy_one.clear(); copy_two.clear();
	for (unsigned int i = 0; i < one.size(); ++i)
	{
		copy_one.push_back(one[i]);
	}
	for (unsigned int i = 0; i < two.size(); ++i)
	{
		copy_two.push_back(two[i]);
	}
	normalize(copy_one);
	normalize(copy_two);
	T product = 0.0;
	for (unsigned int i = 0; i < copy_one.size(); ++i)
	{
		product += (copy_one[i] * copy_two[i]);
	}
	copy_one.clear(); copy_two.clear();
	return product;
}

/**
 * @brief Get index of the plane for which the quadcopter has to capture information at that instant
 * @details Uses the information from previously visited plane normlas
 * @todo Please test this extensively
 */
inline static int
getCurrentPlaneIndex(const vector< vector<float> > &plane_parameters,
						const vector< vector<float> > &temp_plane_parameters,
						const vector<float> &percentagePlane)
{
	// planeIndex = -2 means the parameter I'm seeing is not there in plane_parameters
	// planeIndex = -1 means the parameter I'm seeing are all there in plane_parameters
	int planeIndex = -2;
	float plane_heuristic = 0.984; // +-10 degrees variation
	float dot_p;
	vector<float> plane_normal_1, plane_normal_2;
	if(plane_parameters.size() == 0)
	{
		planeIndex = -1;
		plane_normal_1.clear();
		plane_normal_1.push_back(0.0);
		plane_normal_1.push_back(1.0); //@todo-me Check if it is +1.0 or -1.0
		plane_normal_1.push_back(0.0);
		for (unsigned int i = 0; i < temp_plane_parameters.size(); ++i)
		{
			plane_normal_2.clear();
			for(unsigned int j = 0; j < temp_plane_parameters[i].size()-1; j++)
			{
				plane_normal_2.push_back(temp_plane_parameters[i][j]);
			}
			dot_p = normalized_dot_product(plane_normal_1, plane_normal_2); // yaw_axis[0]*a + yaw_axis[1]*b + yaw_axis[2]*c;
			if(dot_p >= plane_heuristic)
			{
				planeIndex = (unsigned int)i; break;
			}
		}
	}
	else
	{
		for (unsigned int i = 0; i < temp_plane_parameters.size(); ++i)
		{
			bool found = false;
			plane_normal_1.clear();
			for(unsigned int j = 0; j < temp_plane_parameters[i].size()-1; j++)
			{
				plane_normal_1.push_back(temp_plane_parameters[i][j]);
			}
			for (unsigned int j = 0; j < plane_parameters.size(); ++j)
			{
				plane_normal_2.clear();
				for(unsigned int k = 0; k < plane_parameters[j].size()-1; k++)
				{
					plane_normal_2.push_back(plane_parameters[j][k]);
				}
				dot_p = normalized_dot_product(plane_normal_1, plane_normal_2); //in_a*out_a + in_b*out_b + in_c*out_c;
				if(dot_p >= plane_heuristic)
				{
					found = true; break;
				}
			}
			if(!found)
			{
				planeIndex = (unsigned int)i;
				break;
			}
			else
			{
				planeIndex = -1;
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
 * @brief
 * @details
 */
inline static bool
isNewPlaneVisible(const vector< vector<float> > &already_visited_plane_parameters,
					const vector< vector<float> > &current_visible_plane_parameters,
					const vector<float> &current_visible_percentage,
					bool observe_by_rotation,
					RotateDirection dir = CLOCKWISE)
{
	bool flag = true;
	// planeIndex = -2 means the parameter I'm seeing is not there in already_visited_plane_parameters
	// planeIndex = -1 means the parameter I'm seeing are all there in already_visited_plane_parameters
	int plane_index = getCurrentPlaneIndex(already_visited_plane_parameters, current_visible_plane_parameters,
											current_visible_percentage);
	if(!observe_by_rotation)
	{
		if(plane_index == -1)
		{
			flag = false;
		}
		else if(plane_index == -2 || plane_index <= (int)current_visible_plane_parameters.size()-1)
		{
			flag = true;
		}
		else if(plane_index <= (int)current_visible_plane_parameters.size()-1)
		{
			flag = true;
		}
		else
		{
			flag = false;
		}
	}
	else
	{
		if(dir == CLOCKWISE)
		{
			if(plane_index == -1)
			{
				flag = false;
			}
			else if(plane_index == -2 || plane_index <= (int)current_visible_plane_parameters.size()-1)
			{
				flag = true;
			}
			else
			{
				flag = false;
			}
		}
		else if(dir == COUNTERCLOCKWISE)
		{
			if(plane_index == -1)
			{
				flag = false;
			}
			else if(plane_index == -2 || plane_index <= (int)current_visible_plane_parameters.size()-1)
			{
				flag = true;
			}
			else if(plane_index <= (int)current_visible_plane_parameters.size()-1)
			{
				flag = true;
			}
			else
			{
				flag = false;
			}
		}
		else
		{

		}
	}
	return flag;
}

inline static float
getPointToPlaneDistance(const vector<float> &planeParameters, const vector<double> &point_position)
{
	float a = planeParameters[0];
	float b = planeParameters[1];
	float c = planeParameters[2];
	float d = planeParameters[3];
	float mag = sqrt(a*a+b*b+c*c);
	float x0 = point_position[0];
	float y0 = point_position[1];
	float z0 = point_position[2];
	float dist = abs(a*x0 + b*y0 + c*z0 + d)/mag;
	return dist;
}

/**
 * @brief Clears a double vector
 * @details
 */
template<typename T>
inline static void
clear2dVector(vector< vector<T> > &vec)
{
	unsigned int size = vec.size();
	for (unsigned int i = 0; i < size; ++i)
	{
		vec[i].clear();
	}
	vec.clear();
}

/**
 * @brief Converts a point given from quadcopter's current position as origin to quadcopter's actual origin
 * @details This is required for generating points for quadcopter's autonomous movement
 */
inline static void
convertWRTQuadcopterOrigin(const vector<double> &current_pos_of_drone,
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
	// cout << "[ DEBUG] Rotation Matrix: " << rotationMatrix << "\n";
	Mat translationVector(3, 1, DataType<double>::type);
	translationVector.at<double>(0, 0) = current_pos_of_drone[0];
	translationVector.at<double>(1, 0) = current_pos_of_drone[1];
	translationVector.at<double>(2, 0) = current_pos_of_drone[2];
	// cout << "[ DEBUG] Translation Vector: " << translationVector << "\n";
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
inline static vector<float>
bestFitPlane(const vector<Point3f> &threed_points)
{
	// http://stackoverflow.com/questions/1400213/3d-least-squares-plane
	float x_c = 0.0, y_c = 0.0, z_c = 0.0;
	float a, b, c, d;
	Mat X(3, threed_points.size(), DataType<float>::type);
	for (unsigned int i = 0; i < threed_points.size(); ++i)
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
	for (unsigned int i = 0; i < threed_points.size(); ++i)
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
	d = (float)0.0;
	normal.push_back(a);
	normal.push_back(b);
	normal.push_back(c);
	normal.push_back(d);
	return normal;
}

/**
 * @brief Generates 2d points on the image on the left, middle and right edge of plane
 * @details
 */
inline static vector<Point2f>
GenerateMy2DPoints()
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
inline static vector<Point3f>
GenerateMy3DPoints(float width, float height)
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
 * @brief Calculate the distance to see the plane completely from top to bottom
 * @details From tyhis position the leftmost and rightmost edges of the plane may not be visible
 */
inline static float
getDistanceToSeePlane(int height)
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

inline static void
projectPointsOnPlane (const vector<Point3f> &points, const vector<float> &planeParameters,
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

inline static double
getHeightFromGround(const vector<float> &planeParameters, 
						const vector<Point3f> &continuousBoundingBoxPoints,
						const vector<double> &current_pos_of_drone)
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
	float drone_distance = getPointToPlaneDistance(planeParameters, current_pos_of_drone);
	float t = drone_distance/mag_normal_plane;
	height = (double) mid.z + t*normal_plane.z;
	cout << "Height to adjust: " << height << "\n";
	cout << "Values x: " << mid.x + t*normal_plane.x << "\n";
	cout << "Values y: " << mid.y + t*normal_plane.y << "\n";
	cout << "Values z: " << mid.z + t*normal_plane.z << "\n";
	return height;
}

template<typename T>
inline static void
copyVector(const vector<T> vec1, vector<T> &vec2)
{
	vec2.clear();
	for (unsigned int i = 0; i < vec1.size(); ++i)
	{
		vec2.push_back(vec1[i]);
	}
	return ;
}



#endif
