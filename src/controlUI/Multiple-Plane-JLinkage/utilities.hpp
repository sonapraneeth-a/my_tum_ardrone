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
		const vector<float> &data,
		vector<float> &sortedData,
		vector<int> &indices);

int numberOfUniquePlanes(
		const vector<float> &planeIndices );

void fitFunctionPlane(
	const vector<Point3f> &planePoints,
	const vector<int> planeIndices,
	vector< vector<float> >  &planeParameters);

void getPlaneParameters(
	const vector<Point3f> &planePoints,
	const vector<int> planeIndices,
	vector< vector<float> >  &planeParameters);

void fitPlane3D(
	const vector<Point3f> &planePoints,
	vector<float>  &planeParameters);

//template<typename T>
void printVector(
			vector<int> data) ;