/*
 *   File Name: utilities.cpp
 *     Project: Multiple Plane JLinkage
 *  Created on: 23-Mar-2016
 *      Author: Sona Praneeth Akula
 *     Details:
 *   TodoNotes: TODO
 */

#include "utilities.hpp"

void sortData(
		const vector<double> &data,
		vector<double> &sortedData,
		vector<int> &indices) {

	sortedData = data;
	int dataSize = data.size();

	map<double, int> oldIndices;
	int i;
	for (i = 0; i < dataSize; ++i) {
		oldIndices[data[i]] = i;
	}

	sort(sortedData.begin(), sortedData.end());
	for (i = 0; i < dataSize; ++i) {
		indices[i] = oldIndices[sortedData[i]];
	}

	return ;

}

int numberOfUniquePlanes(
		const vector<int> &planeIndices ) {

	int ans = 0;
	int numberOfPlanes = planeIndices.size();
	set<int> uniquePlaneIndices(planeIndices.begin(), planeIndices.end());
	ans = uniquePlaneIndices.size();
	return ans;

}

void fitFunctionPlane(
	const vector<Point3d> &planePoints,
	const vector<int> planeIndices,
	vector< vector<double> >  &planeParameters) {

	vector< vector<Point3d> > planeOrderedPoints;
	int numberOfPlanes = numberOfUniquePlanes(planeIndices);
	int numberOfPointsInThePlane = planePoints.size();
	vector<Point3d> pointsOfAPlane;
	int i, j;

	for (i = 0; i < numberOfPlanes; ++i) {
		planeOrderedPoints.push_back(pointsOfAPlane);
	}

	for (i = 0; i < numberOfPointsInThePlane; ++i) {
		planeOrderedPoints[planeIndices[i]].push_back(planePoints[i]);
	}

	double x1, x2, y1, y2, z1, z2, normN;
	double a, b, c, d;
	for (i = 0; i < numberOfPlanes; ++i) {

		// Normal found as cross product of X2 - X1, X3 - X1
		//for (j = 0; j < 3; ++j) {
			x1 = planeOrderedPoints[i][1].x - planeOrderedPoints[i][0].x;
			x2 = planeOrderedPoints[i][2].x - planeOrderedPoints[i][0].x;
			y1 = planeOrderedPoints[i][1].y - planeOrderedPoints[i][0].y;
			y2 = planeOrderedPoints[i][2].y - planeOrderedPoints[i][0].y;
			z1 = planeOrderedPoints[i][1].z - planeOrderedPoints[i][0].z;
			z2 = planeOrderedPoints[i][2].z - planeOrderedPoints[i][0].z;

			// Compute a, b, c, d in a*x+b*y+c*z+d=0
			a = y1 * z2 - z1 * y2;
			b = z1 * x2 - x1 * z2;
			c = x1 * y2 - y1 * x2;
			d =  - (a * planeOrderedPoints[i][0].x ) - (b * planeOrderedPoints[i][0].y ) - (c * planeOrderedPoints[i][0].z ); 

			// Normalize
			normN = sqrt(a*a + b*b + c*c);
			a = a/normN;
			b = b/normN;
			c = c/normN;
			d = d/normN;
		//}

	}

	return ;

}

void fitFunctionPlane1(
	const vector<Point3d> &planePoints,
	const vector<int> planeIndices,
	vector< vector<double> >  &planeParameters) {

	vector< vector<Point3d> > planeOrderedPoints;
	int numberOfPlanes = numberOfUniquePlanes(planeIndices);
	int numberOfPointsInThePlane = planePoints.size();
	vector<Point3d> pointsOfAPlane;
	int i, j;

	for (i = 0; i < numberOfPlanes; ++i) {
		planeOrderedPoints.push_back(pointsOfAPlane);
	}

	for (i = 0; i < numberOfPointsInThePlane; ++i) {
		planeOrderedPoints[planeIndices[i]].push_back(planePoints[i]);
	}

	double x1, x2, y1, y2, z1, z2, normN;
	double a, b, c, d;
	for (i = 0; i < numberOfPlanes; ++i) {


	}

	return ;

}
