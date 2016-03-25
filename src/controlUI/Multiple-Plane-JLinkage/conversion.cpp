/**
 * @file conversion.cpp
 * @ingroup conversion
 *
 *   File Name: conversion.cpp
 *     Project: Multiple Plane JLinkage
 *  Created on: 22-Mar-2016
 *      Author: Sona Praneeth Akula
 *     Details: The file contains code for to and fro conversion from XYZ to UV form
 *   TodoNotes: Test the code
 */

#include "conversion.hpp"

void XYZToUVCoordinates(
		const Point3d &xyzCoordinates,
		const vector<double> &planeParameters,
		Point2d &uvCoordinates,
		vector<Point3d> &uvAxes) {

	// Extract the plane parameters to variables
	double a = planeParameters[0];
	double b = planeParameters[1];
	double c = planeParameters[2];
	double d = planeParameters[3];
	// Make the normal vector of the plane
	Point3d n(a, b, c);
	// Make the u, v vector for the transformation
	// \f$u = [b -a 0]\f$, \f$v = n \times u$\f
	Point3d u(b, -a, 0);
	Point3d v = n.cross(u);
	// Make the u and v co-ordinates
	double uCoord = u.dot(xyzCoordinates);
	double vCoord = v.dot(xyzCoordinates);
	// Copy it to Point2d structure
	uvCoordinates.x = uCoord;
	uvCoordinates.y = vCoord;
	uvAxes.clear();
	// Copy the UV Axes
	uvAxes.push_back(u);
	uvAxes.push_back(v);
	uvAxes.push_back(n);

	return;

}

void AllXYZToUVCoordinates(
		const vector<Point3d> &xyzCoordinates,
		const vector<double> &planeParameters,
		vector<Point2d> &uvCoordinates,
		vector<Point3d> &uvAxes) {

	LLI numberOfPoints = xyzCoordinates.size();
	LLI i, j;
	Point2d uvCoord;
	vector<Point3d> uvAxis;

	for (i = 0; i < numberOfPoints; ++i) {
		XYZToUVCoordinates( xyzCoordinates[i], planeParameters,
							uvCoord, uvAxis);
		uvCoordinates.push_back(uvCoord);
		uvAxes = uvAxis;
	}

	return ;

}


void UVToXYZCoordinates(
		const Point2d &uvCoordinates,
		const vector<Point3d> &uvAxes,
		const double d,
		Point3d &xyzCoordinates) {

	// Converting uvCoordinates to Mat format
	Mat uvPoint = Mat::zeros(3, 1, CV_64FC1);
	// Converting uvAxes to to Mat format
	Mat uvAxis = Mat::zeros(3, 3, CV_64FC1);
	// Initializing the output Matrix
	Mat xyzPoint = Mat::zeros(3, 1, CV_64FC1);

	uvPoint.at<double>(1, 1) = uvCoordinates.x;
	uvPoint.at<double>(2, 1) = uvCoordinates.y;
	uvPoint.at<double>(3, 1) = -d;

	int i, j;
	// Copying uvAxes to Matrix uvAxis
	for (i = 0; i < 3; i++) {
		j = 0;
		uvAxis.at<double>(i, j) = uvAxes[i].x; j++;
		uvAxis.at<double>(i, j) = uvAxes[i].y; j++;
		uvAxis.at<double>(i, j) = uvAxes[i].z; j++;
	}
	xyzPoint = uvAxis * uvPoint;
	// Generating the XYZ co-ordinate
	xyzCoordinates.x = xyzPoint.at<double>(1, 1);
	xyzCoordinates.y = xyzPoint.at<double>(2, 1);
	xyzCoordinates.z = xyzPoint.at<double>(3, 1);

	return;

}

void AllUVToXYZCoordinates(
		const vector<Point2d> &uvCoordinates,
		const vector<Point3d> &uvAxes,
		const double d,
		vector<Point3d> &xyzCoordinates) {

	LLI numberOfPoints = uvCoordinates.size();
	LLI i, j;
	Point3d xyzCoord;

	for (i = 0; i < numberOfPoints; ++i) {
		UVToXYZCoordinates( uvCoordinates[i], uvAxes, d,
							xyzCoord);
		xyzCoordinates.push_back(xyzCoord);
	}

	return ;

}
