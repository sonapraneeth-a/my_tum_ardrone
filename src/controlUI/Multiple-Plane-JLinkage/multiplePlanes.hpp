/**
 * @defgroup multiplePlanes
 * @file multiplePlanes.hpp
 * @ingroup multiplePlanes
 */

/*
 *   File Name: multiplePlanes.hpp
 *     Project: Multiple Plane JLinkage
 *  Created on: 29-Mar-2016
 *      Author: Sona Praneeth Akula
 *     Details:
 */
#ifndef MULTIPLEPLANES_HPP
#define MULTIPLEPLANES_HPP

#include "allHeaders.hpp"
#include "readingData.hpp"
#include "additionalSteps.hpp"
#include "utilities.hpp"
#include "makeBoundingRects.hpp"
#include "JLinkage/JLinkage.h"
#include "JLinkage/RandomSampler.h"


/**
 * @brief This function performs JLinkage algorithm
 *
 * @param [in] locations - Set of 3D points
 * @param [out] labels - Labels assigned to each point by JLinkage
 *
 */
void callJLinkage(
		const vector<Point3f> &locations,
		vector<int> &labels);

/**
 * @brief This function finds multiple planes in the set of 3D points
 *
 * @param [in] points - Set of 3D points
 * @param [out] sortedPlaneParameters - plane parameters corresponding to plane i after arranging the points by
 * 								their distance from X axis
 * @param [out] continuousBoundingBoxPoints - Bounding box co-ordinates for plane i in boundingBoxPoints[i]
 * 										such that the boxes appear to be continuous
 */
int findMultiplePlanes(
		const vector<Point3f> &points,
		vector< vector<float> > &sortedPlaneParameters,
		vector< vector<Point3f> > &continuousBoundingBoxPoints);


void
findPercBoundEachPlane(
		const vector<Point3f> &points,
		vector< vector<float> > &sortedPlaneParameters,
		vector< vector<Point3f> > &continuousBoundingBoxPoints,
		vector< vector<Point3f> > &sorted_3d_points,
		vector<float> &percentageOfEachPlane);

int findMultiplePlanes1(
		const vector<Point3f> &points,
		vector< vector<float> > &sortedPlaneParameters,
		vector< vector<Point3f> > &continuousBoundingBoxPoints);



#endif
