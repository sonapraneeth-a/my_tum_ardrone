/**
 * @file utilities.cpp
 * @ingroup utilities
*/

/*
 *   File Name: utilities.cpp
 *     Project: Multiple Plane JLinkage
 *  Created on: 29-Mar-2016
 *      Author: Sona Praneeth Akula
 *     Details:
 */

#include "utilities.hpp"


//template<typename T>
void printVector(
			const vector<int> &data) {

	cout << "[ DEBUG ] Printing the vector: ";
	int i;
	for (i = 0; i < data.size(); ++i) {
		cout << data[i] << " ";
	}
	cout << "\n";

	return ;

}

void printVectorOfVectors(
			const vector< vector<float> > &data) {

	cout << "[ DEBUG ] Printing the vector of vectors: \n";
	int i, j;
	int dataSize = data.size();
	for (i = 0; i < dataSize; ++i) {
		int size= data[i].size();
		for(j = 0; j < size; j++) {
			cout << data[i][j] << " ";
		}
		cout << "\n";
	}
	cout << "\n";

	return ;

}

void getPlaneParameters(
	const vector<Point3f> &planePoints,
	const vector<int> &planeIndices,
	vector< vector<float> >  &planeParameters,
	vector< vector<Point3f> > &planeOrderedPoints) {

	cout << "[ DEBUG ] getPlaneParametes Started\n";
	// vector< vector<Point3f> > planeOrderedPoints;
	// Get the number of unique planes
	int numberOfPlanes = numberOfUniquePlanes(planeIndices);
	// Get the total number of points
	int numberOfPointsInThePlane = planePoints.size();
	// Vector for points belonging to particular plane
	vector<Point3f> pointsOfAPlane;
	// Vector for parameters of particular plane
	vector<float> planeParametersForThisPlane;

	int i;

	// Put all points belonging to a particular plane i in planeOrderedPoints[i]
	cout << "[ DEBUG ] Creating planeOrderPoints\n";
	for (i = 0; i < numberOfPlanes; ++i) {
		planeOrderedPoints.push_back(pointsOfAPlane);
	}
	for (i = 0; i < numberOfPointsInThePlane; ++i) {
		planeOrderedPoints[planeIndices[i]].push_back(planePoints[i]);
	}

	writePointsToCSVForGPlot(planeOrderedPoints, "output/planeOrderedPoints.txt");
	float a, b, c, d;
	// Find the plane parameters for each plane
	for (i = 0; i < numberOfPlanes; ++i) {

		// Clear old plane parameters
		planeParametersForThisPlane.clear();
		// Get the plane parameters
		cout << "[ DEBUG ] Fitting a plane to set of points for plane " << i << "\n";
		fitPlane3D(planeOrderedPoints[i], planeParametersForThisPlane);
		planeParameters.push_back(planeParametersForThisPlane);

	}

	printVectorOfVectors(planeParameters);
	cout << "[ DEBUG ] getPlaneParametes Completed\n";
	return ;

}

int numberOfUniquePlanes(
		const vector<int> &planeIndices ) {

	cout << "[ DEBUG ] Calculating the number of unique planes\n";
	int ans = 0;
	int numberOfPlanes = planeIndices.size();
	// Convert the vector set to contain only the unique elements
	set<int> uniquePlaneIndices(planeIndices.begin(), planeIndices.end());
	ans = uniquePlaneIndices.size();
	cout << "[ DEBUG ] There are " << ans << " number of planes.\n";
	return ans;

}

void fitPlane3D(
	const vector<Point3f> &planePoints,
	vector<float>  &planeParameters) {

	float a, b, c, d;
	int j;
	// Get the number of points in the plane
	int numberOfPointsInThisPlane = planePoints.size();
	cout << "[ DEBUG ] fitPlane3D started for " << numberOfPointsInThisPlane << " points\n";
	// Create a matrix out of the vector of points: Dimension: numberOfPoints*3
	Mat pointsMatrixTemp(numberOfPointsInThisPlane, 3, CV_32F);
	for (j = 0; j < numberOfPointsInThisPlane; ++j) {
		pointsMatrixTemp.at<float>(j, 0) = planePoints[j].x;
		pointsMatrixTemp.at<float>(j, 1) = planePoints[j].y;
		pointsMatrixTemp.at<float>(j, 2) = planePoints[j].z;
	}
	// Calculate the centroid of the points
	float centroidX = (mean(pointsMatrixTemp.col(0)))[0];
	float centroidY = (mean(pointsMatrixTemp.col(1)))[0];
	float centroidZ = (mean(pointsMatrixTemp.col(2)))[0];
	cout << centroidX << " " << centroidY << " " << centroidZ << "\n";
	Mat eigenvalues, eigenvectors;
	// Make the points mean centered
	cout << "[ DEBUG ] Making the points mean-centric\n";
	for (j = 0; j < numberOfPointsInThisPlane; ++j) {
		pointsMatrixTemp.at<float>(j, 0) = pointsMatrixTemp.at<float>(j, 0) - centroidX;
		pointsMatrixTemp.at<float>(j, 1) = pointsMatrixTemp.at<float>(j, 1) - centroidY;
		pointsMatrixTemp.at<float>(j, 2) = pointsMatrixTemp.at<float>(j, 2) - centroidZ;
	}
	// Calculate the eigenvector of pointsMatrixTemp'*pointsMatrixTemp
	eigen(pointsMatrixTemp.t()*pointsMatrixTemp, eigenvalues, eigenvectors);
	// Pick the eigenvector corresponding to least eigenvalue
	Mat minEigVector = eigenvectors.row(2);
	cout << "[ DEBUG ] Obtaining the plane parameters corresponding to minimum eigenvalue\n";
	float normSum = 0.0;
	// Normalise the eigenvector
	for(j = 0; j < 3; ++j) {
		normSum += (minEigVector.at<float>(0, j))*(minEigVector.at<float>(0, j));
	}
	for(j = 0; j < 3; ++j) {
		minEigVector.at<float>(0, j) = minEigVector.at<float>(0, j)/sqrt(normSum);
	}
	// Extract the plane parameters from the minimum eigenvector
	Mat abc = minEigVector;
	a = abc.at<float>(0, 0); b = abc.at<float>(0, 1); c = abc.at<float>(0, 2); d = 0.0;
	d += centroidX*a; d += centroidY*b; d += centroidZ*c;
	d = (-1.0)*d;
	// Copy the parameters to the output
	planeParameters.push_back(a);
	planeParameters.push_back(b);
	planeParameters.push_back(c);
	planeParameters.push_back(d);

	cout << "[ DEBUG ] fitPlane3D Completed\n";
	return ;

}


float getKthPercentile(
		const vector<float> &data,
		const float k) {

	// Reference:
	// http://web.stanford.edu/class/archive/anthsci/anthsci192/anthsci192.1064/handouts/calculating%20percentiles.pdf
	float threshold;
	cout << "[ DEBUG ] getKthPercentile Started\n";
	vector<float> copiedData;
	int sizeOfData = data.size();
	int i;
	// Create a copy of data for sorting
	for( i = 0; i < sizeOfData; i++) {
		copiedData.push_back(data[i]);
	}

	// Sort the data
	sort(copiedData.begin(), copiedData.end());

	// Get the threshold
	cout << "[ DEBUG ] Get the Kth threshold\n";
	int size = copiedData.size();
	float percentile;
	for (i = 0; i < size; ++i) {
		percentile = (100.0/float(size))*(i+1.0-0.5);
		if ( percentile >= k) { // TODO: Check if the comparison is working properly
			threshold = copiedData[i];
		}
	}
	// Clear the copy of data
	copiedData.clear();

	cout << "[ DEBUG ] getKthPercentile Completed. Threshold is " << threshold << "\n";
	return threshold;

}

typedef std::pair<float,int> mypair;
	bool asc_comparator ( const mypair& l, const mypair& r)
	   { return l.first < r.first; }
	bool dsc_comparator ( const mypair& l, const mypair& r)
		   { return l.first >= r.first; }

void sortData(
		const vector<float> &data,
		vector<float> &sortedData,
		vector<int> &indices,
		bool isAscending) {

	//cout << "[ DEBUG ] sortData Started\n";
	// Get number of points in data
	int dataSize = data.size();
	//cout << "[ DEBUG ] Data for sorting has size: " << dataSize << "\n";

	vector< mypair> oldDataWithIndex;
	int i;

	sortedData.clear();
	// Make a map of data point to its index in data
	// Create a copy of data in sortedData
	for (i = 0; i < dataSize; ++i) {
		oldDataWithIndex.push_back(make_pair(data[i],i));
	}


	// Sort the data
	//cout << "[ DEBUG ] Sorting Data ...\n";
	if(isAscending)
		sort(oldDataWithIndex.begin(), oldDataWithIndex.end(), asc_comparator);
	else
		sort(oldDataWithIndex.begin(), oldDataWithIndex.end(), dsc_comparator);
	indices.clear();
	//cout << "[ DEBUG ] Sorting Completed ...\n";

	// Make new indices based on oldIndices map
	//cout << "[ DEBUG ] Making indices ...\n";
	for (i = 0; i < dataSize; ++i) {
		indices.push_back(oldDataWithIndex[i].second);
		sortedData.push_back(oldDataWithIndex[i].first);
	}
	//cout << "[ DEBUG ] Making indices Completed ...\n";

	//cout << "[ DEBUG ] sortData Completed\n";
	return ;

}

void clearVectorOfVectors (
		vector< vector<float> > &data) {

	int dataSize = data.size();
	int i, j;

	for(i = 0; i < dataSize; i++) {
		data[i].clear();
	}

	return ;

}

int writePointsToCSV(
		const vector<Point3f> &data,
		const string &filename) {

	int dataSize = data.size();
	int i;

	cout << "[ DEBUG ] Writing the points to CSV Started\n";
	// Initiate a ofstream object
	const char* outFilename = filename.c_str();
	ofstream outFile;
	// Open the object in writing mode
	outFile.open(outFilename, ios::out);
	// Check if the file is open
	if (!outFile.is_open()) {
		cerr << "\nFile " << filename << " cannot be opened for writing.\n";
		cerr << "Please check if the file is existing and has required permissions ";
		cerr << " for writing.\n";
		return -1;
	}
	// Write the data to file
	for( i = 0; i < dataSize; i++) {
		outFile << data[i].x << ", " << data[i].y << ", " << data[i].z << "\n";
	}

	// Close the file
	outFile.close();

	cout << "[ DEBUG ] Writing the points to CSV Completed\n";
	return 0;

}

int writePointsToCSVForGPlot(
		const vector<Point3f> &data,
		const vector<int> &planeIndices,
		const string &filename) {

	int dataSize = data.size();
	int i;

	cout << "[ DEBUG ] Writing the points to CSV for GNUPlot Started\n";
	// Initiate a ofstream object
	const char* outFilename = filename.c_str();
	ofstream outFile;
	// Open the object in writing mode
	outFile.open(outFilename, ios::out);
	// Check if the file is open
	if (!outFile.is_open()) {
		cerr << "\nFile " << filename << " cannot be opened for writing.\n";
		cerr << "Please check if the file is existing and has required permissions ";
		cerr << " for writing.\n";
		return -1;
	}
	// Write the data to file
	cout << "[ DEBUG ] There are " << planeIndices.size() << "(" << dataSize <<
			") points in the plane.\n";
	for( i = 0; i < dataSize; i++) {
		outFile << data[i].x << " " << data[i].y << " " << data[i].z << " " << planeIndices[i] << "\n";
	}

	// Close the file
	outFile.close();

	cout << "[ DEBUG ] Writing the points to CSV for GNUPlot Completed\n";
	return 0;

}

int writePointsToCSVForGPlot(
		const vector< vector<Point3f> > &data,
		const string &filename) {

	int numberOfPlanes = data.size();
	int i, j;

	cout << "[ DEBUG ] Writing the points(vector of vector) to CSV for GNUPlot Started\n";
	// Initiate a ofstream object
	const char* outFilename = filename.c_str();
	ofstream outFile;
	// Open the object in writing mode
	outFile.open(outFilename, ios::out);
	// Check if the file is open
	if (!outFile.is_open()) {
		cerr << "\nFile " << filename << " cannot be opened for writing.\n";
		cerr << "Please check if the file is existing and has required permissions ";
		cerr << " for writing.\n";
		return -1;
	}

	// Write the data to file
	for( i = 0; i < numberOfPlanes; i++) {
		int numberOfPoints = data[i].size();
		for( j = 0; j < numberOfPoints; j++) {
			outFile << data[i][j].x << " " << data[i][j].y << " " << data[i][j].z << " " << i << "\n";
		}
	}

	// Close the file
	outFile.close();

	cout << "[ DEBUG ] Writing the points(vector of vector) to CSV for GNUPlot Completed\n";
	return 0;

}

int writePointsToCSVForGPlot(
		const vector<Point3f> &data,
		const map<int, pair<int,int> > &planeIndexBounds,
		const string &filename) {

	int numberOfPlanes = planeIndexBounds.size();
	int i, j;

	cout << "[ DEBUG ] Writing the points(map) to CSV for GNUPlot Started\n";
	// Initiate a ofstream object
	const char* outFilename = filename.c_str();
	ofstream outFile;
	// Open the object in writing mode
	outFile.open(outFilename, ios::out);
	// Check if the file is open
	if (!outFile.is_open()) {
		cerr << "\nFile " << filename << " cannot be opened for writing.\n";
		cerr << "Please check if the file is existing and has required permissions ";
		cerr << " for writing.\n";
		return -1;
	}

	// Write the data to file
	cout << "[ DEBUG ] There are " << numberOfPlanes << " planes.\n";
	for( i = 0; i < numberOfPlanes; i++) {
		int firstBound = planeIndexBounds.at(i).first;
		int secondBound = planeIndexBounds.at(i).second;
		for( j = firstBound; j < secondBound; j++) {
			outFile << data[j].x << " " << data[j].y << " " << data[j].z << " " << i << "\n";
		}
	}

	// Close the file
	outFile.close();

	cout << "[ DEBUG ] Writing the points(map) to CSV for GNUPlot Completed\n";
	return 0;

}
