/*
 *   File Name: makeBoundingRects.hpp
 *     Project: Multiple Plane JLinkage
 *  Created on: 22-Mar-2016
 *      Author: Sona Praneeth Akula
 *     Details:
 *   TodoNotes: TODO
 */

#ifndef MAKEBOUNDINGRECTS_HPP_
#define MAKEBOUNDINGRECTS_HPP_

#include "conversion.hpp"
#include "utilities.hpp"
#include "calculateIntersections.hpp"


void orderPlanePointsByCentroids(
		const vector<Point3f> &projectionOf3DPoints,
		const vector< vector<float> > &planeParameters,
		const map<int, pair<int, int> > &planeIndexBounds,
		vector<Point3f> &sortedProjectionsOf3DPoints,
		vector< vector<float> > &sortedPlaneParameters,
		map<int, pair<int, int> > &sortedPlaneIndexBounds );


void orderPlanePointsByCentroids1(
		const vector< vector<Point3f> > &projectionOf3DPoints,
		const vector< vector<float> > &planeParameters,
		vector< vector<Point3f> > &sortedProjectionsOf3DPoints,
		vector< vector<float> > &sortedPlaneParameters );

void getBoundingBoxCoordinates (
		const vector<Point3f> &sortedProjectionOf3DPoints,
		const vector< vector<float> > &sortedPlaneParameters,
		const map<int, pair<int, int> > &sortedPlaneIndexBounds,
		vector< vector<Point3f> > &boundingBoxPoints );

void getBoundingBoxCoordinates1 (
		const vector< vector<Point3f> > &sortedProjectionOf3DPoints,
		const vector< vector<float> > &sortedPlaneParameters,
		vector< vector<Point3f> > &boundingBoxPoints );

void getContinuousBoundingBox (
		const vector< vector<Point3f> > &boundingBoxPoints,
		const vector< vector<float> > &sortedPlaneParameters,
		vector< vector<Point3f> > &continuousBoundingBoxPoints);

#endif /* MAKEBOUNDINGRECTS_HPP_ */
