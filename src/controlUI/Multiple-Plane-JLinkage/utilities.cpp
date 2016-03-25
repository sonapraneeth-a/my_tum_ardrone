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
		const vector<double> &sortedData,
		const vector<LLI> &indices) {

	data = sortedData;
	LLI dataSize = data.size();

	map<double, LLI> oldIndices;
	LLI i;
	for (i = 0; i < dataSize; ++i) {
		oldIndices[data[i]] = i;
	}

	sort(sortedData.begin(), sortedData.end());
	for (i = 0; i < dataSize; ++i) {
		indices[i] = oldIndices[sortedData[i]];
	}

	return ;

}

LLI numberOfUniquePlanes(
		vector<double> planeIndices ) {

	LLI ans = 0;
	LLI numberOfPlanes = planeIndices.size();
	set<LLI> uniquePlaneIndices(planeIndices.begin(), planeIndices.end());
	ans = uniquePlaneIndices.size();
	return ans;

}

