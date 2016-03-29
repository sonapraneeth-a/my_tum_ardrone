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
		const Point3f &xyzCoordinates,
		const vector<float> &planeParameters,
		Point2f &uvCoordinates,
		vector<Point3f> &uvAxes) {

	// Extract the plane parameters to variables
	float a = planeParameters[0];
	float b = planeParameters[1];
	float c = planeParameters[2];
	float d = planeParameters[3];
	// Make the normal vector of the plane
	Point3f n(a, b, c);
	// Make the u, v vector for the transformation
	// \f$u = [b -a 0]\f$, \f$v = n \times u$\f
	Point3f u(b, -a, 0);
	Point3f v = n.cross(u);
	// Make the u and v co-ordinates
	float uCoord = u.dot(xyzCoordinates);
	float vCoord = v.dot(xyzCoordinates);
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
		const vector<Point3f> &xyzCoordinates,
		const vector<float> &planeParameters,
		vector<Point2f> &uvCoordinates,
		vector<Point3f> &uvAxes) {

	int numberOfPoints = xyzCoordinates.size();
	int i, j;
	Point2f uvCoord;
	vector<Point3f> uvAxis;

	for (i = 0; i < numberOfPoints; ++i) {
		uvAxis.clear();
		XYZToUVCoordinates( xyzCoordinates[i], planeParameters,
							uvCoord, uvAxis);
		uvCoordinates.push_back(uvCoord);
		int uvAxisLength = uvAxis.size();
		uvAxes.clear();
		for (j = 0; j < uvAxisLength; ++j) {
			uvAxes.push_back(uvAxis[j]);
		}
	}

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
		uvAxis.at<float>(i, j) = uvAxes[i].x; j++;
		uvAxis.at<float>(i, j) = uvAxes[i].y; j++;
		uvAxis.at<float>(i, j) = uvAxes[i].z; j++;
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

	int numberOfPoints = uvCoordinates.size();
	int i, j;
	Point3f xyzCoord;

	xyzCoordinates.clear();
	for (i = 0; i < numberOfPoints; ++i) {
		UVToXYZCoordinates( uvCoordinates[i], uvAxes, d,
							xyzCoord);
		xyzCoordinates.push_back(xyzCoord);
	}

	return ;

}
