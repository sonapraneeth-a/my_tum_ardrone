/**
 * @defgroup additionalSteps
 */

/**
 * @file additionalSteps.hpp
 * @ingroup additionalSteps
 *
 *   File Name: additionalSteps.hpp
 *     Project: Multiple Plane JLinkage
 *  Created on: 22-Mar-2016
 *      Author: Sona Praneeth Akula
 *     Details:
 *   TodoNotes: TODO
 */

#ifndef ADDITIONALSTEPS_HPP_
#define ADDITIONALSTEPS_HPP_


#include "allHeaders.hpp"

/**
 * @details This function remove planes which have less than
 * 			'minimumNumberOfPointsPerPlane' in them
 *
 * @param [in] oldData - Initial data containing points from all planes
 * @param [in] planeIndices - Vector which determines which point belongs to which plane
 * @param [in] minimumNumberOfPointsPerPlane - Minimum number of points required for
 * 										any plane to be considered
 * @param [in] numberOfPointsPerPlane - Frequency map of planeIndex and number of points
 * 								present in the plane
 * @param [out] newData -	Data after removing points belonging to planes having less than
 * 				'minimumNumberOfPointsPerPlane' in them
 * @param [out] newPlaneIndices - Vector which determines which point belongs to which plane
 * 						after removal of unnecessary planes
 * @param [out] numberOfPlanes - Number of planes we're considering after removing unnecessary planes
 */
void removeUnnecessaryPlanes(
		const vector<Point3d> &oldData,
		const vector<int> &planeIndices,
		const int &minimumNumberOfPointsPerPlane,
		map<int, int> &numberOfPointsPerPlane,
		vector<Point3d> &newData,
		vector<int> &newPlaneIndices,
		int &numberOfPlanes);

/**
 * @brief This functions calculates the distances of each point to its respective plane
 e
 * @param [in] [vector<Point3d>] data - Set of all points
 * @param [in] [vector< vector<double> >] planeParameters - Set of plane parameters for the planes
 * 								planeParamters[i] corresponds to parameters of plane <i>i</i>
 * @param [in] [vector<int>] planeIndices - Indices revealing which point belongs
 * 										to which plane
 * @param [out] [vector<double>] distanceMatrix - Distance of each point from its respective plane
 * @param [out] [vector< vector<int> >] planePointsIndexMapping - This contains
 * 					which plane contains which points.
 * 					Example: planePointsIndexMapping[i] contains points corresponding to plane <i>i</i>
 * @return Nothing
 */
void calculateDistanceFromPlane(
		const vector<Point3d> &data,
		const vector< vector<double> > &planeParameters,
		const vector<int> &planeIndices,
		vector<double> &distanceMatrix,
		vector< vector<int> > &planePointsIndexMapping);

/**
 *
 * @param [in] [vector<double>] distanceMatrix - Distance of each point from its respective plane
 * @param [in] [vector< vector<int> >] planePointsIndexMapping - This contains
 * 					which plane contains which points.
 * 					Example: planePointsIndexMapping[i] contains points corresponding to plane <i>i</i>
 * @param [out] [vector<Point3d>] newSortedData - Sorted planePoints according to planeIndex
 * @param [out] [vector< vector<double> >] newPlaneParameters - Plane parameters corresponding to index i.
 * 					newPlaneParameters[i] contains parameters for plane i
 * @param [out] [map<int, pair<int, int> >] planeIndexBounds - The bounds which tells the plane(i) points are from index k1 to k2
 */
void removePointsFarFromPlane(
		const vector<Point3d> &data,
		const vector< vector<double> > &planeParameters,
		const vector<double> &distanceMatrix,
		const vector< vector<int> > &planePointsIndexMapping,
		vector<Point3d> &newSortedData,
		vector< vector<double> > &newPlaneParameters,
		map<int, pair<int, int> > &planeIndexBounds);

void removePointsFarFromPlane1(
		const vector<Point3d> &data,
		const vector< vector<double> > &planeParameters,
		const vector<double> &distanceMatrix,
		const vector< vector<int> > &planePointsIndexMapping,
		vector< vector<Point3d> > &newSortedData,
		vector< vector<double> > &newPlaneParameters );

void get3DPlaneProjectionsOfPoints (
		const vector<Point3d> &data,
		const vector< vector<double> > &planeParameters,
		const map<int, pair<int, int> > &planeIndexBounds,
		vector<Point3d> &projectionsOf3DPoints	);

void get3DPlaneProjectionsOfPoints1 (
		const vector< vector<Point3d> > &data,
		const vector< vector<double> > &planeParameters,
		vector<Point3d> &projectionsOf3DPoints	);

double getKthPercentile(
		const vector<double> &data,
		const int k);

#endif /* ADDITIONALSTEPS_HPP_ */
