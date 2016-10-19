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
#include "Multiple-Plane-JLinkage/utilities.hpp"


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
	// BUG: It can be +- dist
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
 */
inline static void
print2dVector(const vector< vector<Point3f> > &two_d_vector, string vec_name = "vector")
{
	//cout << "Printing the vector: " << vec_name << "\n";
	cout << vec_name << "\n";
	cout << "[...\n";
	for (unsigned int i = 0; i < two_d_vector.size(); ++i)
	{
		for (unsigned int j = 0; j < two_d_vector[i].size(); ++j)
		{
			cout << "\t[ ";
			cout << two_d_vector[i][j].x << "; "
					<< two_d_vector[i][j].y << "; ";
			if(j!=two_d_vector[i].size()-1)
				cout << two_d_vector[i][j].z << "], ...\n";
			else
				cout << two_d_vector[i][j].z << "]...\n";
		}
	}
	cout << "]\n";
	return ;
}

/**
 * @brief
 * @details
 */
template<typename T>
inline static void
print2dVector(const vector< vector<T> > &two_d_vector, string vec_name = "vector")
{
	//cout << "Printing the vector: " << vec_name << "\n";
	cout << vec_name << "\n";
	cout << "[...\n";
	for (unsigned int i = 0; i < two_d_vector.size(); ++i)
	{
		cout << "\t[ ";
		for (unsigned int j = 0; j < two_d_vector[i].size(); ++j)
		{
			if(j!=two_d_vector[i].size()-1)
			{
				cout << two_d_vector[i][j] << "; ";
			}
			else
			{
				cout << two_d_vector[i][j] << "";
			}
		}
		if(i!=two_d_vector.size()-1)
		{
			cout << "], ...\n";
		}
		else
		{
			cout << "] ...\n";
		}
	}
	cout << "]\n";
	return ;
}

/**
 * @brief
 * @details
 */
inline static void
print1dVector(const vector<Point3f> &one_d_vector, string vec_name = "vector")
{
	//cout << "Printing the vector: " << vec_name << "\n";
	cout << vec_name << "\n";
	cout << "[...\n";
	for (unsigned int i = 0; i < one_d_vector.size(); ++i)
	{
		cout << "\t[ ";
		cout << one_d_vector[i].x << "; "
				<< one_d_vector[i].y << "; ";
		if(i!=one_d_vector.size()-1)
			cout << one_d_vector[i].z << "], ...\n";
		else
			cout << one_d_vector[i].z << "] ...\n";
	}
	cout << "]\n";
	return ;
}

/**
 * @brief
 * @details
 */
template<typename T>
inline static void
print1dVector(const vector<T> &one_d_vector, string vec_name = "vector")
{
	//cout << "Printing the vector: " << vec_name << "\n";
	cout << vec_name << "\n";
	cout << "[...\n\t[ ";
	for (unsigned int i = 0; i < one_d_vector.size(); ++i)
	{
		if(i!=one_d_vector.size()-1)
			cout << one_d_vector[i] << "; ";
		else
			cout << one_d_vector[i] << "";
	}
	cout << "]...\n]\n";
	return ;
}


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
	float plane_heuristic = 0.9848; // +-15 degrees variation
	float dot_p;
	vector<float> plane_normal_1, plane_normal_2;
	vector<float> dot_p_vec;
	dot_p_vec.clear();
	if(plane_parameters.size() == 0)
	{
		cout << "[ DEBUG] [getCurrentPlaneIndex] No planes visited till now\n";
		planeIndex = -1;
		plane_normal_1.clear();
		plane_normal_1.push_back(0.0);
		plane_normal_1.push_back(1.0);
		plane_normal_1.push_back(0.0);
		for (unsigned int i = 0; i < temp_plane_parameters.size(); ++i)
		{
			plane_normal_2.clear();
			for(unsigned int j = 0; j < temp_plane_parameters[i].size()-1; j++)
			{
				plane_normal_2.push_back(temp_plane_parameters[i][j]);
			}
			dot_p = normalized_dot_product(plane_normal_1, plane_normal_2); // yaw_axis[0]*a + yaw_axis[1]*b + yaw_axis[2]*c;
			dot_p_vec.push_back(dot_p);
			print1dVector(plane_normal_1, "[ DEBUG] [getCurrentPlaneIndex] Expected yaw of drone");
			print1dVector(plane_normal_2, "[ DEBUG] [getCurrentPlaneIndex] Current plane visible");
			cout << "[ DEBUG] [getCurrentPlaneIndex] dot_p: " << dot_p << "\n";
			if(dot_p >= plane_heuristic)
			{
				planeIndex = (unsigned int)i; break;
			}
			else
			{
				/*float magnitude = ( pow((plane_normal_1[0] - plane_normal_2[0]), 2) + 
									pow((plane_normal_1[1] - plane_normal_2[1]), 2) + 
									pow((plane_normal_1[2] - plane_normal_2[2]), 2) );
				cout << "[ DEBUG] [getCurrentPlaneIndex] magnitude: " << magnitude << "\n";
				if(magnitude <= 0.3)
				{
					planeIndex = (unsigned int)i; break;
				}*/
				if(fabs(plane_normal_2[0]-plane_normal_1[0]) < 0.08 &&
					  fabs(plane_normal_2[1]-plane_normal_1[1]) < 0.08 &&
					  fabs(plane_normal_2[2]-plane_normal_1[2]) < 0.08 )
				{
					planeIndex = (unsigned int)i; break;
				}
			}
		}
		if(planeIndex < 0)
		{
			cout << "[ DEBUG] [getCurrentPlaneIndex] planeIndex: " << planeIndex << "\n";
			print1dVector(dot_p_vec, "[ DEBUG] [getCurrentPlaneIndex] Dot Product Values");
			planeIndex = distance(dot_p_vec.begin(),
					 std::max_element(dot_p_vec.begin(), dot_p_vec.end()));
			cout << "[ DEBUG] [getCurrentPlaneIndex] planeIndex: " << planeIndex << "\n";
		}
	}
	else
	{
		print2dVector(temp_plane_parameters, "[ DEBUG] [getCurrentPlaneIndex] Current visible planes");
		print2dVector(plane_parameters, "[ DEBUG] [getCurrentPlaneIndex] Already visited planes");
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
				print1dVector(plane_normal_2, "[ DEBUG] [getCurrentPlaneIndex] Visited plane");
				print1dVector(plane_normal_1, "[ DEBUG] [getCurrentPlaneIndex] Current plane visible");
				cout << "[ DEBUG] [getCurrentPlaneIndex] dot_p: " << dot_p << "\n";
				if(dot_p >= plane_heuristic)
				{
					if(fabs(temp_plane_parameters[i][3]-plane_parameters[j][3]) < 0.8)
						found = true; break;
				}
				else
				{
					/*float magnitude = ( pow((plane_normal_1[0] - plane_normal_2[0]), 2) + 
										pow((plane_normal_1[1] - plane_normal_2[1]), 2) + 
										pow((plane_normal_1[2] - plane_normal_2[2]), 2) );
					cout << "[ DEBUG] [getCurrentPlaneIndex] magnitude: " << magnitude << "\n";
					if(magnitude <= 0.3)
					{
						found = true; break;
					}*/
					bool var1 = (fabs(plane_normal_2[0]-plane_normal_1[0]) < 0.15);
					bool var2 = (fabs(plane_normal_2[1]-plane_normal_1[1]) < 0.15);
					bool var3 = (fabs(plane_normal_2[2]-plane_normal_1[2]) < 0.15);
					bool var4 = (fabs(temp_plane_parameters[i][3]-plane_parameters[j][3]) < 0.8);
					cout << "[ DEBUG] [getCurrentPlaneIndex] Var1: " << var1 << ", Var2: " << var2 << ", Var3: " << var3 << ", Var4: " << var4 << "\n";
					if(var1 & var2 & var3 & var4)
					{
						found = true; break;
					}
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
	/*if(planeIndex == -1)
	{
		planeIndex = (int)temp_plane_parameters.size()-1;
	}
	cout << "[ DEBUG] [getCurrentPlaneIndex] Changed Current Plane Index: " << planeIndex << "\n";*/
	cout << "[ DEBUG] [getCurrentPlaneIndex] Completed\n";
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
	assert(current_pos_of_drone.size() == dest_pos_of_drone.size());
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
	ac_dest_pos_of_drone.push_back(current_pos_of_drone[3]+dest_pos_of_drone[3]);
}

/**
 * @brief Converts points given from quadcopter's actual origin to quadcopter's current position as origin
 * @details This is required for sorting points based on x centroid;
 */
inline static void
convertWRTCurrentQuadcopterOrigin(const vector<double> &current_pos_of_drone,
									const vector< vector<Point3f> > &points,
									vector< vector<Point3f> > &output_points)
{
	assert(current_pos_of_drone.size() == 4);
	vector<Point3f> current_points;
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
	Mat x = rotationMatrix*translationVector;
	Mat c(3, 1, DataType<double>::type);
	for (unsigned int i = 0; i < points.size(); ++i)
	{
		current_points.clear();
		for (unsigned int j = 0; j < points[i].size(); ++j)
		{
			c.at<double>(0, 0) = (double)points[i][j].x;
			c.at<double>(0, 1) = (double)points[i][j].y;
			c.at<double>(0, 2) = (double)points[i][j].z;
			Mat output = rotationMatrix*c - x;
			/*cout << rotationMatrix << "\n";
			cout << c << "\n";
			cout << translationVector << "\n";
			cout << x << "\n";
			cout << output << "\n";*/
			Point3f out;
			out.x = (float)output.at<double>(0, 0);
			out.y = (float)output.at<double>(0, 1);
			out.z = (float)output.at<double>(0, 2);
			current_points.push_back(out);
		}
		output_points.push_back(current_points);
	}
	return ;
}

inline static void
fixPlaneOrientation(const vector<double> &position, 
						const vector<Point3f> &points,
						vector<float> &plane_parameters,
						vector<Point3f> &continuous_bounding_box_points)
{
	assert(points.size() >= 3);
	cout << "[ INFO] [fixPlaneOrientation] Started\n";
	/*print1dVector(plane_parameters, "[ DEBUG] [fixPlaneOrientation] Old PP");
	print1dVector(continuous_bounding_box_points, "[ DEBUG] [fixPlaneOrientation] Old CBB");*/
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
	cout << "[ DEBUG] [fixPlaneOrientation] Centroid: " << p << "\n";
	print1dVector(position, "[ DEBUG] [fixPlaneOrientation] Position of drone");
	float a = plane_parameters[0];
	float b = plane_parameters[1];
	float c = plane_parameters[2];
	float mag = ((a*a)+(b*b)+(c*c));
	print1dVector(plane_parameters, "[ DEBUG] [fixPlaneOrientation] Old Plane Parameters");
	Point3f pos((float)position[0], (float)position[1], (float)position[2]);
	qp = pos - p;
	cout << "[ DEBUG] [fixPlaneOrientation] qp: " << qp << "\n";
	float t = (qp.x * (a/mag)) + (qp.y * (b/mag)) + ((qp.z * (c/mag)));
	cout << "[ DEBUG] [fixPlaneOrientation] t: " << t << "\n";
	if(!signbit(t))
	{
		cout << "[ DEBUG] [fixPlaneOrientation] Sign change required\n";
		plane_parameters[0] = -plane_parameters[0];
		plane_parameters[1] = -plane_parameters[1];
		plane_parameters[2] = -plane_parameters[2];
		plane_parameters[3] = -plane_parameters[3];
		Point3f top_left = continuous_bounding_box_points[1];
		Point3f top_right = continuous_bounding_box_points[0];
		Point3f bottom_right = continuous_bounding_box_points[3];
		Point3f bottom_left = continuous_bounding_box_points[2];
		continuous_bounding_box_points.clear();
		continuous_bounding_box_points.push_back(top_left);
		continuous_bounding_box_points.push_back(top_right);
		continuous_bounding_box_points.push_back(bottom_right);
		continuous_bounding_box_points.push_back(bottom_left);
		continuous_bounding_box_points.push_back(top_left);
	}
	/*print1dVector(plane_parameters, "[ DEBUG] [fixPlaneOrientation] New Plane Parameters");
	print1dVector(continuous_bounding_box_points, "[ DEBUG] [fixPlaneOrientation] New CBB");*/
	cout << "[ INFO] [fixPlaneOrientation] Completed\n";
	return ;
}

inline static void
orderPlanesFromQuadcopterPosition(const vector<double> &current_pos_of_drone,
									const vector< vector<Point3f> > &in_points,
									const vector< vector<float> > &in_pp,
									const vector< vector<Point3f> > &in_cbb,
									const vector<float> &in_p,
									vector< vector<Point3f> > &out_points,
									vector< vector<float> > &out_pp,
									vector< vector<Point3f> > &out_cbb,
									vector<float> &out_p)
{
	cout << "[ DEBUG] [orderPlanesFromQuadcopterPosition] Started\n";
	clear2dVector(out_points);
	clear2dVector(out_pp);
	clear2dVector(out_cbb);
	out_p.clear();
	float x_c = 0.0;
	vector<Point3f> dummy_points, dummy_cbb;
	vector<float> dummy_pp;
	// Vector for x co-ordinate of centroids for each plane
	vector<float> xCentroidPoints;
	// Vector for sorting x co-ordinate of centroids for each plane
	vector<float> sortedXCentroidPoints;
	vector<int> indices;
	vector< vector<Point3f> > points;
	convertWRTCurrentQuadcopterOrigin(current_pos_of_drone, in_points, points);
	for (unsigned int i = 0; i < points.size(); ++i)
	{
		x_c = 0.0;
		for (unsigned int j = 0; j < points[i].size(); ++j)
		{
			x_c += points[i][j].x;
		}
		x_c /= (float)points[i].size();
		xCentroidPoints.push_back(x_c);
	}
	// Sort the x centroids
	sortData( xCentroidPoints, sortedXCentroidPoints, indices, true);
	print1dVector(indices, "[ DEBUG] [orderPlanesFromQuadcopterPosition] New Plane Indices");
	for (unsigned int i = 0; i < in_points.size(); ++i)
	{
		int index = indices[i];
		dummy_points.clear();
		dummy_pp.clear();
		dummy_cbb.clear();
		for (unsigned int j = 0; j < in_points[index].size(); ++j)
		{
			dummy_points.push_back(in_points[index][j]);
		}
		for (unsigned int j = 0; j < in_pp[index].size(); ++j)
		{
			dummy_pp.push_back(in_pp[index][j]);
		}
		for (unsigned int j = 0; j < in_cbb[index].size(); ++j)
		{
			dummy_cbb.push_back(in_cbb[index][j]);
		}
		out_cbb.push_back(dummy_cbb);
		out_pp.push_back(dummy_pp);
		out_points.push_back(dummy_points);
		out_p.push_back(in_p[index]);
	}
	print2dVector(in_pp, "[ DEBUG] [orderPlanesFromQuadcopterPosition] Input Plane Parameters");
	print2dVector(out_pp, "[ DEBUG] [orderPlanesFromQuadcopterPosition] Output Plane Parameters");
	print2dVector(in_cbb, "[ DEBUG] [orderPlanesFromQuadcopterPosition] Input Plane CBB");
	print2dVector(out_cbb, "[ DEBUG] [orderPlanesFromQuadcopterPosition] Output Plane CBB");
	print1dVector(in_p, "[ DEBUG] [orderPlanesFromQuadcopterPosition] Input Plane Percentage");
	print1dVector(out_p, "[ DEBUG] [orderPlanesFromQuadcopterPosition] Output Plane Percentage");
	cout << "[ DEBUG] [orderPlanesFromQuadcopterPosition] Fixing orientations\n";
	for (unsigned int i = 0; i < out_pp.size(); ++i)
	{
		fixPlaneOrientation(current_pos_of_drone, out_points[i], out_pp[i], out_cbb[i]);
	}
	print2dVector(out_pp, "[ DEBUG] [orderPlanesFromQuadcopterPosition] Fixed Output Plane Parameters");
	print2dVector(out_cbb, "[ DEBUG] [orderPlanesFromQuadcopterPosition] Fixed Output Plane CBB");
	cout << "[ DEBUG] [orderPlanesFromQuadcopterPosition] Fixing Done\n";
	cout << "[ DEBUG] [orderPlanesFromQuadcopterPosition] Completed\n";
	dummy_points.clear();
	dummy_pp.clear();
	dummy_cbb.clear();
	clear2dVector(points);
	return ;
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
	Mat Y(4, threed_points.size(), DataType<float>::type);
	for (unsigned int i = 0; i < threed_points.size(); ++i)
	{
		x_c += threed_points[i].x;
		y_c += threed_points[i].y;
		z_c += threed_points[i].z;
		X.at<float>(0, i) = threed_points[i].x;
		X.at<float>(1, i) = threed_points[i].y;
		X.at<float>(2, i) = threed_points[i].z;
		Y.at<float>(0, i) = threed_points[i].x;
		Y.at<float>(1, i) = threed_points[i].y;
		Y.at<float>(2, i) = threed_points[i].z;
		Y.at<float>(3, i) = 1.0;
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
	a = ((val2*val12) - (val1*val22))/D;
	b = ((val12*val1) - (val11*val2))/D;
	c = 1.0;
	d = 0.0;
	float mag = sqrt(a*a + b*b + c*c);
	cout << "Method 1: (" << a/mag << ", " << b/mag << ", " << c/mag << ")\n";
	normal.push_back(a/mag);
	normal.push_back(b/mag);
	normal.push_back(c/mag);
	normal.push_back(d);*/
	vector<float> normal;
	// Method 2: SVD Method
	// http://math.stackexchange.com/questions/99299/best-fitting-plane-given-a-set-of-points
	// http://www.ltu.se/cms_fs/1.51590!/svd-fitting.pdf
	/*Mat w(3, threed_points.size(), DataType<float>::type);
	Mat vt(threed_points.size(), threed_points.size(), DataType<float>::type);
	Mat u(3, 3, DataType<float>::type);
	SVD::compute(X, w, u, vt);
	cout << "U:\n" << u << "\n";
	cout << "S:\n" << w << "\n";
	cout << "Method 2: (" << u.at<float>(0, 2) << ", " << u.at<float>(1, 2) << ", " << u.at<float>(2, 2) << ")\n";
	a = u.at<float>(0, 2);
	b = u.at<float>(1, 2);
	c = u.at<float>(2, 2);
	d = (float)0.0;
	normal.push_back(a);
	normal.push_back(b);
	normal.push_back(c);
	normal.push_back(d);*/
	// Method 3: Complete
	Mat w(3, threed_points.size(), DataType<float>::type);
	Mat vt(threed_points.size(), threed_points.size(), DataType<float>::type);
	Mat u(3, 3, DataType<float>::type);
	SVD::compute(Y, w, u, vt);
	cout << "U:\n" << u << "\n";
	cout << "S:\n" << w << "\n";
	cout << "Vt:\n" << vt << "\n";
	cout << "Method 3: (" << u.at<float>(0, 3) << ", " << u.at<float>(1, 3) << ", " 
			<< u.at<float>(2, 3) << ", " << u.at<float>(3, 3) << ")\n";
	a = u.at<float>(0, 3);
	b = u.at<float>(1, 3);
	c = u.at<float>(2, 3);
	d = u.at<float>(3, 3);
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
	vector<float> v;
	vector<double> point;
	Point3f pp;
	float distance, t;
	float a = planeParameters[0];
	float b = planeParameters[1];
	float c = planeParameters[2];
	float d = planeParameters[3];
	/*if( !(fabs(a-0.0) <= 0.001 ) )
	{
		p.x = -d/a;
		p.y = 0.0;
		p.z = 0.0;
	}
	else if( !(fabs(b-0.0) <= 0.001 ) )
	{
		p.x = 0.0;
		p.y = -d/b;
		p.z = 0.0;
	}
	else if( !(fabs(c-0.0) <= 0.001 ) )
	{
		p.x = 0.0;
		p.y = 0.0;
		p.z = -d/c;
	}
	else
	{
		 cout << "[ ERROR] [projectPointsOnPlane] None of a, b, c are non-zero\n";
	}*/
	int numberOfPointsInThisPlane = points.size();
	// Create a matrix out of the vector of points: Dimension: numberOfPoints*3
	Mat pointsMatrixTemp(numberOfPointsInThisPlane, 3, CV_32F);
	for (unsigned int j = 0; j < numberOfPointsInThisPlane; ++j) {
		pointsMatrixTemp.at<float>(j, 0) = points[j].x;
		pointsMatrixTemp.at<float>(j, 1) = points[j].y;
		pointsMatrixTemp.at<float>(j, 2) = points[j].z;
	}
	// Calculate the centroid of the points
	float centroidX = (mean(pointsMatrixTemp.col(0)))[0];
	float centroidY = (mean(pointsMatrixTemp.col(1)))[0];
	float centroidZ = (mean(pointsMatrixTemp.col(2)))[0];
	Point3f p(centroidX, centroidY, centroidZ), qp;
	float mag = ((a*a)+(b*b)+(c*c));
	for(unsigned int i = 0; i < points.size(); i++)
	{
		point.clear();
		point.push_back((double)points[i].x);
		point.push_back((double)points[i].y);
		point.push_back((double)points[i].z);
		// v = projectPoint(planeParameters, point);
		// distance = getPointToPlaneDistance(planeParameters, point);
		qp = points[i] - p;
		t = (qp.x * (a/mag)) + (qp.y * (b/mag)) + ((qp.z * (c/mag)));
		v.clear();
		v.push_back(points[i].x - a*t);
		v.push_back(points[i].y - b*t);
		v.push_back(points[i].z - c*t);
		pp.x = v[0]; pp.y = v[1]; pp.z = v[2];
		projectedPoints.push_back(pp);
		v.clear();
	}
	return ;
}

inline static void
genPointAlongNormalAtDistance()
{
	return ;
}

inline static double
getHeightFromGround(const vector<float> &planeParameters, 
						const vector<Point3f> &continuousBoundingBoxPoints,
						const vector<double> &current_pos_of_drone)
{
	cout << "[ DEBUG] [getHeightFromGround] Started\n";
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
	/*mid.x = top_mid.x;
	mid.y = top_mid.y;
	mid.z = top_mid.z/(float)2.0;*/
	cout << "[ DEBUG] [getHeightFromGround] Top mid: " << top_mid << "\n";
	cout << "[ DEBUG] [getHeightFromGround] Mid: " << mid << "\n";
	cout << "[ DEBUG] [getHeightFromGround] Bottom mid: " << bottom_mid << "\n";
	cout << "[ DEBUG] [getHeightFromGround] Height of plane's mid point: " << mid << "\n";
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
	cout << "[ DEBUG] [getHeightFromGround] Completed\n";
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
