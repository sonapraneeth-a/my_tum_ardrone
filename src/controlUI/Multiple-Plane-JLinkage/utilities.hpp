/*
 *   File Name: utilities.hpp
 *     Project: Multiple Plane JLinkage
 *  Created on: 23-Mar-2016
 *      Author: Sona Praneeth Akula
 *     Details:
 *   TodoNotes: TODO
 */




#include "allHeaders.hpp"


void sortData(
		const vector<double> &data,
		vector<double> &sortedData,
		vector<int> &indices);

int numberOfUniquePlanes(
		const vector<double> &planeIndices );

void fitFunctionPlane(
	const vector<Point3d> &planePoints,
	const vector<int> planeIndices,
	vector< vector<double> >  &planeParameters);

void fitFunctionPlane1(
	const vector<Point3d> &planePoints,
	const vector<int> planeIndices,
	vector< vector<double> >  &planeParameters);
