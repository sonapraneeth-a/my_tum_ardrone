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
		const vector<double> &planeIndices ) {

	int ans = 0;
	int numberOfPlanes = planeIndices.size();
	set<int> uniquePlaneIndices(planeIndices.begin(), planeIndices.end());
	ans = uniquePlaneIndices.size();
	return ans;

}

