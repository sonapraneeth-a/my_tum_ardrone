/**
 * @file conversion.cpp
 * @ingroup conversion
 *
 */

/*
 *   File Name: conversion.cpp
 *     Project: Multiple Plane JLinkage
 *  Created on: 29-Mar-2016
 *      Author: Sona Praneeth Akula
 *     Details: The file contains code for to and fro conversion from XYZ to UV form
 *   TodoNotes: Test the code
 */

#include "conversion.hpp"

void XYZToUVCoordinates(
		const vector<float> &planeParameters,
		vector<Point3f> &uvAxes) {

	// Extract the plane parameters to variables
	float a = planeParameters[0];
	float b = planeParameters[1];
	float c = planeParameters[2];

	// Make the normal vector of the plane
	Point3f n(a, b, c);
	// Make the u, v vector for the transformation
	// \f$u = [b -a 0]\f$, \f$v = n \times u$\f
	Point3f u(b, -a, 0);
	Point3f v = n.cross(u);

	// Copy it to Point2d structure
	uvAxes.clear();

	// Copy the UV Axes
	uvAxes.push_back(u);
	uvAxes.push_back(v);
	uvAxes.push_back(n);

	return;

}

void AllXYZToUVCoordinates(
		const vector<Point3f> &xyzCoordinates,
		const vector<float> &planeParameters,
		vector<Point2f> &uvCoordinates,
		vector<Point3f> &uvAxes) {

	// Get the number of points
	int numberOfPoints = xyzCoordinates.size();
	int i, j;
	Point2f uvCoord;
	vector<Point3f> uvAxis;

	// Convert each XYZ point to UV and make the uvAxis used in making the transformation
	cout << "[ DEBUG ] All XYZ -> UV Conversion Started for " << numberOfPoints << "points\n";
	XYZToUVCoordinates( planeParameters,uvAxes);
	cout << "UV Axes\n";
	cout << uvAxes;
	for (i = 0; i < numberOfPoints; ++i) {
		uvCoord.x = uvAxes[0].dot(xyzCoordinates[i]);
		uvCoord.y = uvAxes[1].dot(xyzCoordinates[i]);
		uvCoordinates.push_back(uvCoord);
	}

	cout << "[ DEBUG ] All XYZ -> UV Conversion Completed\n";
	return ;

}


void UVToXYZCoordinates(
		const Point2f &uvCoordinates,
		const vector<Point3f> &uvAxes,
		const float d,
		Point3f &xyzCoordinates) {

	// Converting uvCoordinates to Mat format
	Mat uvPoint = Mat::zeros(3, 1, CV_32F);
	// Converting uvAxes to to Mat format
	Mat uvAxis = Mat::zeros(3, 3, CV_32F);
	// Initializing the output Matrix
	Mat xyzPoint = Mat::zeros(3, 1, CV_32F);

	uvPoint.at<float>(0, 0) = uvCoordinates.x;
	uvPoint.at<float>(1, 0) = uvCoordinates.y;
	uvPoint.at<float>(2, 0) = -d;

	int i, j;
	// Copying uvAxes to Matrix uvAxis
	for (i = 0; i < 3; i++) {
		j = 0;
		uvAxis.at<float>(j, i) = uvAxes[i].x; j++;
		uvAxis.at<float>(j, i) = uvAxes[i].y; j++;
		uvAxis.at<float>(j, i) = uvAxes[i].z; j++;
	}
	xyzPoint = uvAxis * uvPoint;
	// Generating the XYZ co-ordinate
	xyzCoordinates.x = xyzPoint.at<float>(0, 0);
	xyzCoordinates.y = xyzPoint.at<float>(1, 0);
	xyzCoordinates.z = xyzPoint.at<float>(2, 0);

	return;

}

void AllUVToXYZCoordinates(
		const vector<Point2f> &uvCoordinates,
		const vector<Point3f> &uvAxes,
		const float d,
		vector<Point3f> &xyzCoordinates) {

	// Get the number of points
	int numberOfPoints = uvCoordinates.size();
	int i;
	Point3f xyzCoord;

	xyzCoordinates.clear();
	// Transform each UV point to XYZ point
	cout << "[ DEBUG ] All UV -> XYZ Conversion Started for " << numberOfPoints << "\n";
	for (i = 0; i < numberOfPoints; ++i) {
		UVToXYZCoordinates( uvCoordinates[i], uvAxes, d,
							xyzCoord);
		xyzCoordinates.push_back(xyzCoord);
	}

	cout << "[ DEBUG ] All UV -> XYZ Conversion Completed\n";

	return ;

}
