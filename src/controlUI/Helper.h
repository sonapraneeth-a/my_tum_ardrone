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

#include "Headers.h"


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
					float a_1, a_2, a_3, a_4;
					float b_1, b_2, b_3, b_4;
					a_1 = plane_normal_2[0];
					a_2 = plane_normal_2[1];
					a_3 = plane_normal_2[2];
					a_4 = temp_plane_parameters[i][3];
					b_1 = plane_normal_1[0];
					b_2 = plane_normal_1[1];
					b_3 = plane_normal_1[2];
					b_4 = plane_parameters[j][3];
					bool var1 = (fabs(a_1-b_1) < 0.15);
					bool var2 = (fabs(a_2-b_2) < 0.15);
					bool var3 = (fabs(a_3-b_3) < 0.15);
					bool var4 = (fabs(a_4-b_4) < 1.5);
					float mag = ( pow((a_1-b_1),2) + pow((a_2-b_2),2) + pow((a_3-b_3),2) );
					bool var5 = (mag <= 0.25);
					cout << "[ DEBUG] [getCurrentPlaneIndex] Magnitude: " << mag << ", D_Diff: " << fabs(a_4-b_4) << "\n";
					cout << "[ DEBUG] [getCurrentPlaneIndex] Var1: " << var1 << ", Var2: " << var2 << ", Var3: " << var3 << ", Var4: " << var4 << ", Var5: " << var5 << "\n";
					/*if(var1 & var2 & var3 & var4)
					{
						found = true; break;
					}*/
					if(var4 & var5)
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
	for (unsigned int j = 0; j < numberOfPointsInThisPlane; ++j)
	{
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
