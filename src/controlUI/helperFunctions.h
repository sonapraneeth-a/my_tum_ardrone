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
 * 12-Sep-2016	Sona Praneeth Akula	Added		Added comments to the code
 *****************************************************************************************/

#ifndef _HELPER_FUNCTIONS_H
#define _HELPER_FUNCTIONS_H

#include <cmath>
#include <opencv2/core/core.hpp>


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

#endif
