/**
 * @file calculateIntersections.cpp
 * @ingroup calculateIntersection
 *
 */

/*
 *   File Name: calculateIntersections.cpp
 *     Project: Multiple Plane JLinkage
 *  Created on: 22-Mar-2016
 *      Author: Sona Praneeth Akula
 *     Details:
 *   TodoNotes: TODO
 */

#include "calculateIntersections.hpp"

void makeLineFromPoints(
		const Point3f &point1,
		const Point3f &point2,
		vector< vector<float> > &lineParameters) {

	// Determine the normal of the line
	float a = point2.x-point1.x;
	float b = point2.y-point1.y;
	float c = point2.z-point1.z;

	// Get the co-ordinates of the point1
	float x0 = point1.x;
	float y0 = point1.y;
	float z0 = point1.z;

	// Make the vector containing the lineParameters
	vector<float> parameters;
	parameters.push_back(x0);
	parameters.push_back(y0);
	parameters.push_back(z0);
	int lineParametersSize = lineParameters.size();
	clearVectorOfVectors(lineParameters);
	if( lineParametersSize >= 2) {
		lineParameters[0][1] = x0;
		lineParameters[0][2] = y0;
		lineParameters[0][3] = z0;
	}
	else {
		lineParameters.push_back(parameters);
	}
	parameters.clear();

	parameters.push_back(a);
	parameters.push_back(b);
	parameters.push_back(c);
	if( lineParametersSize >= 2) {
		lineParameters[1][1] = a;
		lineParameters[1][2] = b;
		lineParameters[1][3] = c;
	}
	else {
		lineParameters.push_back(parameters);
	}
	parameters.clear();

	return ;

}

void calculateIntersectionOfPlanes(
		const vector<float> &plane1,
		const vector<float> &plane2,
		vector< vector<float> > &lineParameters) {

	// Parameters for plane 1: \fa1*x+b1*y+c1*z+d1=0$\f
	float a1 = plane1[0];
	float b1 = plane1[1];
	float c1 = plane1[2];
	float d1 = plane1[3];

	// Parameters for plane 2: \fa2*x+b2*y+c2*z+d2=0$\f
	float a2 = plane2[0];
	float b2 = plane2[1];
	float c2 = plane2[2];
	float d2 = plane2[3];

	Point3f plane1Normal(a1, b1, c1);
	Point3f plane2Normal(a2, b2, c2);
	Point3f dir = plane1Normal.cross(plane2Normal);

	// The direction of we're finding (a, b, c)
	float a = dir.x;
	float b = dir.y;
	float c = dir.z;

	// Defining a point on the line (x0, y0, z0)
	float x0 = (d2*b1-d1*b2)/(a1*b2-a2*b1);
	float y0 = (-d1 - a1*x0)/b1;
	float z0 = 0;

	// Definition of line: \f$x = x0 + a*t; y = y0 + b*t; z = z0 + c*t;$\f
	vector<float> parameters;
	parameters.push_back(x0);
	parameters.push_back(y0);
	parameters.push_back(z0);
	int lineParametersSize = lineParameters.size();
	clearVectorOfVectors(lineParameters);
	if( lineParametersSize >= 2) {
		lineParameters[0][1] = x0;
		lineParameters[0][2] = y0;
		lineParameters[0][3] = z0;
	}
	else {
		lineParameters.push_back(parameters);
	}
	parameters.clear();

	parameters.push_back(a);
	parameters.push_back(b);
	parameters.push_back(c);
	if( lineParametersSize >= 2) {
		lineParameters[1][1] = a;
		lineParameters[1][2] = b;
		lineParameters[1][3] = c;
	}
	else {
		lineParameters.push_back(parameters);
	}
	parameters.clear();

	return ;

}

void calculateIntersectionOfLines(
		const vector< vector<float> > &line1,
		const vector< vector<float> > &line2,
		Point3f &intersectionOfLines) {

	// Get the parameters of the line1
	// x = x01 + a1*t; y = y01 + b1*t; z = z01 + c1*t;
	float a1 = line1[0][0];
	float b1 = line1[0][1];
	float c1 = line1[0][2];

	float x01 = line1[1][0];
	float y01 = line1[1][1];
	float z01 = line1[1][2];

	// Get the parameters of the line2
	// x = x02 + a2*d; y = y02 + b2*d; z = z02 + c2*d;
	float a2 = line2[0][0];
	float b2 = line2[0][1];
	float c2 = line2[0][2];

	float x02 = line2[1][0];
	float y02 = line2[1][1];
	float z02 = line2[1][2];

	// Solve for d in line2
	float d = ((x01*b1-x02*b1)-(y01*a1-y02*a1))/(a2*b1-a1*b2);

	// Substitute to get the point
	float x0 = x02+(a2*d);
	float y0 = y02+(b2*d);
	float z0 = z02+(c2*d);

	// Make the 3D Point
	intersectionOfLines.x = x0;
	intersectionOfLines.y = y0;
	intersectionOfLines.z = z0;

	return ;

}
