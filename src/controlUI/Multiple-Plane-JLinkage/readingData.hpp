/*
 *   File Name: readingData.hpp
 *     Project: Multiple Plane JLinkage
 *  Created on: 21-Mar-2016
 *      Author: Sona Praneeth Akula
 *     Details:
 *   TodoNotes: TODO
 */

#ifndef READINGDATA_HPP_
#define READINGDATA_HPP_

#include "allHeaders.hpp"

int readPointsFromCSV(
		const string filename,
		vector<Point3f> &points);

void readPointsFromTXT(
		const string filename,
		vector<Point3f> &points);

void printPointsVector(
		const vector<Point3f> &points,
		const long long int numberOfPoints);

#endif /* READINGDATA_HPP_ */
