/**
 * @defgroup makeBoundingRects
 * @file makeBoundingRects.hpp
 * @ingroup makeBoundingRects
 */

/*
 *   File Name: makeBoundingRects.hpp
 *     Project: Multiple Plane JLinkage
 *  Created on: 29-Mar-2016
 *      Author: Sona Praneeth Akula
 *     Details:
 */

#ifndef MAKEBOUNDINGRECTS_HPP_
#define MAKEBOUNDINGRECTS_HPP_

#include "conversion.hpp"
#include "utilities.hpp"
#include "calculateIntersections.hpp"


/**
 * @brief This function order the planes by the position from the x axis
 *
 * @param [in] projectionOf3DPoints - Set of all 3D projected points
 * @param [in] planeParameters - Set of plane parameters for the planes
 * 								planeParamters[i] corresponds to parameters of plane <i>i</i>
 * @param [in] planeIndexBounds - Start and end of plane Points for plane i
 * @param [out] sortedProjectionOf3DPoints - Set of all projected 3D points sorted by x centroids
 * @param [out] sortedPlaneParameters - Set of plane parameters for the planes sorted according to sortedData
 * 								planeParamters[i] corresponds to parameters of plane <i>i</i>
 * @param [out] sortedPlaneIndexBounds - Start and end of plane Points for plane i according to sortedData
 *
 */
void orderPlanePointsByCentroids(
		const vector<Point3f> &projectionOf3DPoints,
		const vector< vector<float> > &planeParameters,
		const map<int, pair<int, int> > &planeIndexBounds,
		vector<Point3f> &sortedProjectionOf3DPoints,
		vector< vector<float> > &sortedPlaneParameters,
		map<int, pair<int, int> > &sortedPlaneIndexBounds );


/**
 * @brief This function generates the bounding box for the points in a plane
 *
 * @param [in] sortedProjectionOf3DPoints - Set of all projected 3D points sorted by x centroids
 * @param [in] sortedPlaneParameters - Set of plane parameters for the planes sorted according to sortedData
 * 								planeParamters[i] corresponds to parameters of plane <i>i</i>
 * @param [in] sortedPlaneIndexBounds - Start and end of plane Points for plane i according to sortedData
 * @param [out] boundingBoxPoints - Bounding box co-ordinates for plane i in boundingBoxPoints[i]
 */
void getBoundingBoxCoordinates (
		const vector<Point3f> &sortedProjectionOf3DPoints,
		const vector< vector<float> > &sortedPlaneParameters,
		const map<int, pair<int, int> > &sortedPlaneIndexBounds,
		vector< vector<Point3f> > &boundingBoxPoints );

void getBoundingBoxPointsOfPlane(
		const vector<Point3f> &pointsInThePlane, 
		const vector<float> &sortedPlaneParameters, 
		vector<Point3f> &sortedPlaneXYZBoundingPoints);

void getPercentageOfEachPlane (
		const vector<Point3f> &sortedProjectionOf3DPoints,
		const map<int, pair<int, int> > &sortedPlaneIndexBounds,
		vector< vector<Point3f> > &sorted_3d_points,
		vector<float> &percentageOfEachPlane);

/**
 * @brief This function makes the bounding boxes generated before continuous
 *
 * @param [in] boundingBoxPoints - Bounding box co-ordinates for plane i in boundingBoxPoints[i]
 * @param [in] sortedPlaneParameters - Set of plane parameters for the planes sorted according to sortedData
 * 								planeParamters[i] corresponds to parameters of plane <i>i</i>
 * @param [out] continuousBoundingBoxPoints - Bounding box co-ordinates for plane i in boundingBoxPoints[i]
 * 										such that the boxes appear to be continuous
 */
void getContinuousBoundingBox (
		const vector< vector<Point3f> > &boundingBoxPoints,
		const vector< vector<float> > &sortedPlaneParameters,
		vector< vector<Point3f> > &continuousBoundingBoxPoints);


/*************** EXTRA FUNCTIONS ***********************************************/

/**
 * @brief This function order the planes by the position from the x axis
 *
 * @param [in] projectionOf3DPoints, - Set of all projected 3D points for plane i in projectionOf3DPoints[i]
 * @param [in] planeParameters - Set of plane parameters for the planes
 * 								planeParamters[i] corresponds to parameters of plane <i>i</i>
 * @param [out] sortedProjectionsOf3DPoints - Set of all 3D projected points sorted by x centroids
 * 											 for plane i in sortedProjectionOf3DPoints[i]
 * @param [out] sortedPlaneParameters - Set of plane parameters for the planes sorted according to sortedData
 * 								planeParamters[i] corresponds to parameters of plane <i>i</i>
 *
 */
void orderPlanePointsByCentroids1(
		const vector< vector<Point3f> > &projectionOf3DPoints,
		const vector< vector<float> > &planeParameters,
		vector< vector<Point3f> > &sortedProjectionOf3DPoints,
		vector< vector<float> > &sortedPlaneParameters );

/**
 * @brief This function generates the bounding box for the points in a plane
 *
 * @param [in] sortedProjectionOf3DPoints - Set of all projected 3D points sorted by x centroids
 * @param [in] sortedPlaneParameters - Set of plane parameters for the planes sorted according to sortedData
 * 								planeParamters[i] corresponds to parameters of plane <i>i</i>
 * @param [out] boundingBoxPoints - Bounding box co-ordinates for plane i in boundingBoxPoints[i]
 */
void getBoundingBoxCoordinates1 (
		const vector< vector<Point3f> > &sortedProjectionOf3DPoints,
		const vector< vector<float> > &sortedPlaneParameters,
		vector< vector<Point3f> > &boundingBoxPoints );


void sortXYZCorners(const vector<Point3f> &corners, vector<Point3f> &sortedPlaneXYZBoundingPoints);
void sortUVCorners(const vector<Point2f> &planeUVBoundingPoints, vector<Point2f> &sortedPlaneUVBoundingPoints);

#endif /* MAKEBOUNDINGRECTS_HPP_ */
