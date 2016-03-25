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
		const vector<long long int> &planeIndices,
		const long long int minimumNumberOfPointsPerPlane,
		map<long long int, long long int> &numberOfPointsPerPlane,
		vector<Point3d> &newData,
		vector<long long int> &newPlaneIndices,
		long long int &numberOfPlanes);

/**
 * @brief This functions calculates the distances of each point to its respective plane
 e
 * @param [in] [vector<Point3d>] data - Set of all points
 * @param [in] [vector< vector<double> >] planeParameters - Set of plane parameters for the planes
 * 								planeParamters[i] corresponds to parameters of plane <i>i</i>
 * @param [in] [vector<long long int>] planeIndices - Indices revealing which point belongs
 * 										to which plane
 * @param [out] [vector<double>] distanceMatrix - Distance of each point from its respective plane
 * @param [out] [vector< vector<long long int> >] planePointsIndexMapping - This contains
 * 					which plane contains which points.
 * 					Example: planePointsIndexMapping[i] contains points corresponding to plane <i>i</i>
 * @return Nothing
 */
void calculateDistanceFromPlane(
		const vector<Point3d> &data,
		const vector< vector<double> > &planeParameters,
		const vector<long long int> &planeIndices,
		vector<double> &distanceMatrix,
		vector< vector<long long int> > &planePointsIndexMapping);

/**
 *
 * @param [in] [vector<double>] distanceMatrix - Distance of each point from its respective plane
 * @param [in] [vector< vector<long long int> >] planePointsIndexMapping - This contains
 * 					which plane contains which points.
 * 					Example: planePointsIndexMapping[i] contains points corresponding to plane <i>i</i>
 * @param [out] [vector<Point3d>] newSortedData - Sorted planePoints according to planeIndex
 * @param [out] [vector< vector<double> >] newPlaneParameters - Plane parameters corresponding to index i.
 * 					newPlaneParameters[i] contains parameters for plane i
 * @param [out] [map<LLI, pair<LLI, LLI> >] planeIndexBounds - The bounds which tells the plane(i) points are from index k1 to k2
 */
void removePointsFarFromPlane(
		const vector<double> &distanceMatrix,
		const vector< vector<LLI> > &planePointsIndexMapping,
		vector<Point3d> &newSortedData,
		vector< vector<double> > &newPlaneParameters,
		map<LLI, pair<LLI, LLI> > &planeIndexBounds);


double getKthPercentile(
		const vector<double> &data,
		const LLI k);

#endif /* ADDITIONALSTEPS_HPP_ */
