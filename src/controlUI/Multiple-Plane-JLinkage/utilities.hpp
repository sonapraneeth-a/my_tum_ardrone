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
		vector<LLI> &indices);

LLI numberOfUniquePlanes(
		const vector<double> &planeIndices );
