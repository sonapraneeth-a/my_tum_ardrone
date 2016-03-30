/**
 * @defgroup additionalSteps
 * @file additionalSteps.hpp
 * @ingroup additionalSteps
 */

/*
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
 * @param [in] oldPlaneIndices - Vector which determines which point belongs to which plane
 * @param [in] minimumNumberOfPointsPerPlane - Minimum number of points required for
 * 										any plane to be considered
 * @param [in] numberOfPointsPerPlane - Frequency map of planeIndex and number of points
 * 								present in the plane
 * @param [out] newData -	Data after removing points belonging to planes having less than
 * 				'minimumNumberOfPointsPerPlane' in them
 * @param [out] newPlaneIndices - Vector which determines which point belongs to which plane
 * 						after removal of unnecessary planes. Indices of the planes are in sorted order
 * @param [out] numberOfPlanes - Number of planes we're considering after removing unnecessary planes
 */
void removeUnnecessaryPlanes(
		const vector<Point3f> &oldData,
		const vector<int> &oldPlaneIndices,
		const int &minimumNumberOfPointsPerPlane,
		map<int, int> &numberOfPointsPerPlane,
		vector<Point3f> &newData,
		vector<int> &newPlaneIndices,
		int &numberOfPlanes);

/**
 * @brief This functions calculates the distances of each point to its respective plane
 *
 * @param [in] data - Set of all points
 * @param [in] planeParameters - Set of plane parameters for the planes
 * 								planeParamters[i] corresponds to parameters of plane <i>i</i>
 * @param [in] planeIndices - Indices revealing which point belongs to which plane
 * @param [out] distanceMatrix - Distance of each point from its respective plane
 * @param [out] planePointsIndexMapping - This contains which plane contains which points.
 * 					Example: planePointsIndexMapping[i] contains points corresponding to plane <i>i</i>
 *
 */
void calculateDistanceFromPlane(
		const vector<Point3f> &data,
		const vector< vector<float> > &planeParameters,
		const vector<int> &planeIndices,
		vector<float> &distanceMatrix,
		vector< vector<int> > &planePointsIndexMapping);

/**
 * @brief This function removes points far from the plane by distance threshold
 * 			 and calculates new plane parameters
 *
 * @param [in] data - Set of all points
 * @param [in] planeParameters - Set of plane parameters for the planes
 * 								planeParamters[i] corresponds to parameters of plane <i>i</i>
 * @param [in] distanceMatrix - Distance of each point from its respective plane
 * @param [in] planePointsIndexMapping - This contains which plane contains which points.
 * 					Example: planePointsIndexMapping[i] contains points corresponding to plane <i>i</i>
 * @param [out] newSortedData - Data sorted by plane index after removing points far from the plane
 * @param [out] newPlaneParameters - Recalculated new parameters from new points.
 * 										newPlaneParameters[i] has plane parameters for plane i
 * @param [out] planeIndexBounds - Start and end of plane Points for plane i
 *
 */
void removePointsFarFromPlane(
		const vector<Point3f> &data,
		const vector< vector<float> > &planeParameters,
		const vector<float> &distanceMatrix,
		const vector< vector<int> > &planePointsIndexMapping,
		vector<Point3f> &newSortedData,
		vector< vector<float> > &newPlaneParameters,
		map<int, pair<int, int> > &planeIndexBounds);

/**
 * @brief This function makes the projection of 3D points onto their planes
 *
 * @param [in] data - Set of all points
 * @param [in] planeParameters - Set of plane parameters for the planes
 * 								planeParamters[i] corresponds to parameters of plane <i>i</i>
 * @param [in] planeIndexBounds - Start and end of plane Points for plane i
 * @param [out] projectionsOf3DPoints - 3D Projections of data[i] onto their respective plane
 *
 */
void get3DPlaneProjectionsOfPoints (
		const vector<Point3f> &data,
		const vector< vector<float> > &planeParameters,
		const map<int, pair<int, int> > &planeIndexBounds,
		vector<Point3f> &projectionsOf3DPoints	);


/******************** EXTRA FUNCTIONS *****************************************/

/**
 * @brief This functions calculates the distances of each point to its respective plane
 *
 * @param [in] data - Set of all points. data[i] contains points corresponding to plane i
 * @param [in] planeParameters - Set of plane parameters for the planes
 * 								planeParamters[i] corresponds to parameters of plane <i>i</i>
 * @param [out] distanceMatrix - Distance of each point from its respective plane.
 * 								distanceMatrix[i] contains distances of point to that plane i
 *
 */
void calculateDistanceFromPlane1(
		const vector< vector<Point3f> > &data,
		const vector< vector<float> > &planeParameters,
		vector< vector<float> > &distanceMatrix );


/**
 * @brief This function removes points far from the plane by distance threshold
 * 			 and calculates new plane parameters
 *
 * @param [in] data - Set of all points. data[i] contains points corresponding to plane i
 * @param [in] planeParameters - Set of plane parameters for the planes
 * 								planeParamters[i] corresponds to parameters of plane <i>i</i>
 * @param [in] distanceMatrix - Distance of each point from its respective plane.
 * 								distanceMatrix[i] contains distances of point to that plane i
 * @param [out] newData - Data after removing points far from the plane
 * @param [out] newPlaneParameters - Recalculated new parameters from new points.
 * 										newPlaneParameters[i] has plane parameters for plane i
 *
 */
void removePointsFarFromPlane1(
		const vector< vector<Point3f> > &data,
		const vector< vector<float> > &planeParameters,
		const vector< vector<float> > &distanceMatrix,
		vector< vector<Point3f> > &newData,
		vector< vector<float> > &newPlaneParameters ) ;


/**
 * @brief This function makes the projection of 3D points onto their planes
 *
 * @param [in] data - Set of all points
 * @param [in] planeParameters - Set of plane parameters for the planes
 * 								planeParamters[i] corresponds to parameters of plane <i>i</i>
 * @param [out] projectionsOf3DPoints - 3D Projections of data[i] onto their respective plane
 * 										in projectionsOf3DPoints[i]
 *
 */
void get3DPlaneProjectionsOfPoints1 (
		const vector< vector<Point3f> > &data,
		const vector< vector<float> > &planeParameters,
		vector< vector<Point3f> > &projectionsOf3DPoints	);


#endif /* ADDITIONALSTEPS_HPP_ */
