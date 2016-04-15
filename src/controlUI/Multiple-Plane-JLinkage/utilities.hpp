/**
 * @defgroup utilities
 * @file utilities.hpp
 * @ingroup utilities
 */

/*
 *   File Name: utilities.hpp
 *     Project: Multiple Plane JLinkage
 *  Created on: 29-Mar-2016
 *      Author: Sona Praneeth Akula
 *     Details:
 */


#include "allHeaders.hpp"

/**
 * @details Print the vector
 *
 * @param [in] data - Vector of ints
 *
 */
//template<typename T>
void printVector(
	const vector<int> &data) ;

void printVectorOfVectors(
	const vector< vector<float> > &data) ;

/**
 * @details Get the plane parameters for each plane given the points in that plane
 *
 * @param [in] planePoints - Points in the plane
 * @param [in] planeIndices - PointIndex -> planeIndex mapping. Gives information
 * 				regarding which point belongs to which plane
 * @param [out] planeParameters - Parameters for the plane. planeParameters[i] corresponds
 * 						to parameters for plane i
 * @param [out] planeOrderedPoints - Vector where planeOrderedPoints[i] contains all the points
 * 						belonging to a particular plane
 */
void getPlaneParameters(
	const vector<Point3f> &planePoints,
	const vector<int> &planeIndices,
	vector< vector<float> >  &planeParameters,
	vector< vector<Point3f> > &planeOrderedPoints);

/**
 * @details Get the number of unique planes from planeIndices
 *
 * @param [in] planeIndices - Details regarding point[i] belongs to which plane
 *
 * @return Number of unique planes present in the plane
 *
 */
int numberOfUniquePlanes(
		const vector<int> &planeIndices );

/**
 * @details Get the plane parameters for a plane based on the given points
 *
 * @param [in] planePoints - Points belonging to a particular plane
 * @param [out] planeParameters - Plane Parameters(a, b, c, d) for that particular plane
 *
 */
void fitPlane3D(
	const vector<Point3f> &planePoints,
	vector<float>  &planeParameters);


/**
 * @details Write the 3D points obtained to a CSV file
 *
 * @param [in] data - data containing 3D points
 * @param [in] percentile - The threshold required
 *
 * @return The threshold according to percentile
 */
float getKthPercentile(
		const vector<float> &data,
		const float percentile);

/**
 * @details Sort the data and give which indices was previously present at the current position
 *
 * @param [in] data - data containing floats
 * @param [out] sortedData - data obtained after sorting data
 * @param [out] indices - Indices of the sorted point in original data
 */
void sortData(
		const vector<float> &data,
		vector<float> &sortedData,
		vector<int> &indices, bool isAscending);

void clearVectorOfVectors (
		vector< vector<float> > &data);

/**
 * @details Write the 3D points obtained to a CSV file
 *
 * @param [in] data - data containing 3D points
 * @param [in] filename - Filename to which the points re to be written
 *
 */
void writePointsToCSV(
		const vector<Point3f> &data,
		string filename);


int writePointsToCSVForGPlot(
		const vector<Point3f> &data,
		const vector<int> &planeIndices,
		const string &filename) ;

int writePointsToCSVForGPlot(
		const vector<Point3f> &data,
		const map<int, pair<int,int> > &planeIndexBounds,
		const string &filename) ;

int writePointsToCSVForGPlot(
		const vector< vector<Point3f> > &data,
		const string &filename) ;


//find angle between two vectors
double findAngle(Point3f vec1, Point3f vec2);

