/**
 * @file calculateIntersections.cpp
 * @ingroup calculateIntersection
 *
 *   File Name: calculateIntersections.cpp
 *     Project: Multiple Plane JLinkage
 *  Created on: 22-Mar-2016
 *      Author: Sona Praneeth Akula
 *     Details:
 *   TodoNotes: TODO
 */

#include "calculateIntersections.hpp"

void makeLineFromPoints(
		const Point3d &point1,
		const Point3d &point2,
		vector< vector<double> > &lineParameters) {

	// Determine the normal of the line
	double a = point2.x-point1.x;
	double b = point2.y-point1.y;
	double c = point2.z-point1.z;

	// Get the co-ordinates of the point1
	double x0 = point1.x;
	double y0 = point1.y;
	double z0 = point1.z;

	// Make the vector containing the lineParameters
	vector<double> parameters;
	parameters.push_back(x0);
	parameters.push_back(y0);
	parameters.push_back(z0);
	lineParameters.push_back(parameters);
	parameters.clear();
	parameters.push_back(a);
	parameters.push_back(b);
	parameters.push_back(c);
	lineParameters.push_back(parameters);
	parameters.clear();

	return ;

}

void calculateIntersectionOfPlanes(
		const vector<double> &plane1,
		const vector<double> &plane2,
		vector< vector<double> > &lineParameters) {

	// Parameters for plane 1: \fa1*x+b1*y+c1*z+d1=0$\f
	double a1 = plane1[0];
	double b1 = plane1[1];
	double c1 = plane1[2];
	double d1 = plane1[3];

	// Parameters for plane 2: \fa2*x+b2*y+c2*z+d2=0$\f
	double a2 = plane2[0];
	double b2 = plane2[1];
	double c2 = plane2[2];
	double d2 = plane2[3];

	Point3d plane1Normal(a1, b1, c1);
	Point3d plane2Normal(a2, b2, c2);
	Point3d dir = plane1Normal.cross(plane2Normal);

	// The direction of we're finding (a, b, c)
	double a = dir.x;
	double b = dir.y;
	double c = dir.z;

	// Defining a point on the line (x0, y0, z0)
	double x0 = (d2*b1-d1*b2)/(a1*b2-a2*b1);
	double y0 = (-d1 - a1*x0)/b1;
	double z0 = 0;

	// Definition of line: \f$x = x0 + a*t; y = y0 + b*t; z = z0 + c*t;$\f
	vector<double> parameters;
	parameters.push_back(x0);
	parameters.push_back(y0);
	parameters.push_back(z0);
	lineParameters.push_back(parameters);
	parameters.clear();
	parameters.push_back(a);
	parameters.push_back(b);
	parameters.push_back(c);
	lineParameters.push_back(parameters);
	parameters.clear();

	return ;

}

Point3d calculateIntersectionOfLines(
		vector< vector<double> > line1,
		vector< vector<double> > line2) {

	Point3d intersectionOfLines;

	// Get the parameters of the line1
	// x = x01 + a1*t; y = y01 + b1*t; z = z01 + c1*t;
	double a1 = line1[0][0];
	double b1 = line1[0][1];
	double c1 = line1[0][2];

	double x01 = line1[1][0];
	double y01 = line1[1][1];
	double z01 = line1[1][2];

	// Get the parameters of the line2
	// x = x02 + a2*d; y = y02 + b2*d; z = z02 + c2*d;
	double a2 = line2[0][0];
	double b2 = line2[0][1];
	double c2 = line2[0][2];

	double x02 = line2[1][0];
	double y02 = line2[1][1];
	double z02 = line2[1][2];

	// Solve for d in line2
	double d = ((x01*b1-x02*b1)-(y01*a1-y02*a1))/(a2*b1-a1*b2);

	// Substitute to get the point
	double x0 = x02+(a2*d);
	double y0 = y02+(b2*d);
	double z0 = z02+(c2*d);

	// Make the 3D Point
	intersectionOfLines.x = x0;
	intersectionOfLines.y = y0;
	intersectionOfLines.z = z0;

	return intersectionOfLines;
}
