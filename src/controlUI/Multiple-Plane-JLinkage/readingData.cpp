/*
 *   File Name: readingData.cpp
 *     Project: Multiple Plane JLinkage
 *  Created on: 21-Mar-2016
 *      Author: Sona Praneeth Akula
 *     Details:
 *   TodoNotes: TODO
 */


#include "readingData.hpp"

int readPointsFromCSV(
		const string filename,
		vector<Point3f> &points) {

	cout << "Reading CSV file: " << filename << "\n";
	const char* inFilename = filename.c_str();
	ifstream inFile;
	inFile.open(inFilename, ios::in);
	if (!inFile.is_open()) {
		cerr << "\nFile " << filename << " cannot be opened for reading.\n";
		cerr << "Please check if the file is existing and has required permissions";
		cerr << " for reading.\n";
		return -1;
	}

	string readALineFromFile;
	Point3f toInsert;
	std::string line;
	int i = 1, j = 1;
	while(std::getline(inFile, line)) {
		std::stringstream  ss(line);
		float val;
		while (ss >> val) {
			cout << val << "\n";
			if (ss.peek() == ',') { ss.ignore(); }
			if(i==1) { toInsert.x = val; i++; }
			else if(i==2) { toInsert.y = val; i++; }
			else if(i==3) { toInsert.z = val; i=1; }
		}
//		std::string        cell;
//		while(std::getline(lineStream, cell, ',')) {
//			const char* cellChar = cell.c_str();
//			stringstream ss(cell);
//			float val;
//			ss >> val;
//			if(i==1) { toInsert.x = val; i++; }
//			else if(i==2) { toInsert.y = val; i++; }
//			else if(i==3) { toInsert.z = val; i=1; }
//		}
		points.push_back(toInsert);
	}

	return 0;

}

void printPointsVector(
		const vector<Point3f> &points,
		const long long int numberOfPoints) {

	long long int i, j, size;
	size = points.size();
	if( numberOfPoints == -1 ) {
		size = numberOfPoints;
	}
	else if( size > numberOfPoints ) {
		size = numberOfPoints;
	}
	cout << "\nWriting " << size << " points from \'points\' to stdout.\n";
	for(i=0; i<size; i++) {
		cout << points[i].x << " " << points[i].y << " " << points[i].z << "\n";
	}
	cout << "\n";
}
