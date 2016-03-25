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
		const vector<Point3d> &projectionOf3DPoints,
		const vector< vector<double> > &planeParameters,
		const map<int, pair<int, int> > &planeIndexBounds,
		vector<Point3d> &sortedProjectionsOf3DPoints,
		vector< vector<double> > &sortedPlaneParameters,
		map<int, pair<int, int> > &sortedPlaneIndexBounds );


void orderPlanePointsByCentroids1(
		const vector< vector<Point3d> > &projectionOf3DPoints,
		const vector< vector<double> > &planeParameters,
		vector< vector<Point3d> > &sortedProjectionsOf3DPoints,
		vector< vector<double> > &sortedPlaneParameters );

void getBoundingBoxCoordinates (
		const vector<Point3d> &sortedProjectionOf3DPoints,
		const vector< vector<double> > &sortedPlaneParameters,
		const map<int, pair<int, int> > &sortedPlaneIndexBounds,
		vector< vector<Point3d> > &boundingBoxPoints );

void getBoundingBoxCoordinates1 (
		const vector< vector<Point3d> > &sortedProjectionOf3DPoints,
		const vector< vector<double> > &sortedPlaneParameters,
		vector< vector<Point3d> > &boundingBoxPoints );

void getContinuousBoundingBox (
		const vector< vector<Point3d> > &boundingBoxPoints,
		const vector< vector<double> > &sortedPlaneParameters,
		vector< vector<Point3d> > &continuousBoundingBoxPoints);

#endif /* MAKEBOUNDINGRECTS_HPP_ */
