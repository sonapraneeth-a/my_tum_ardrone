/*
 *   File Name: utilities.cpp
 *     Project: Multiple Plane JLinkage
 *  Created on: 26-Mar-2016
 *      Author: Sona Praneeth Akula
 *     Details:
 *   TodoNotes: TODO
 */

#include "utilities.hpp"

void sortData(
		const vector<float> &data,
		vector<float> &sortedData,
		vector<int> &indices) {

	int dataSize = data.size();
	cout << "[ DEBUG ] Data for sorting has size: " << dataSize << "\n";
	map<float, int> oldIndices;
	int i;
	//cout << "[ DEBUG ] Data: ";
	sortedData.clear();
	for (i = 0; i < dataSize; ++i) {
		oldIndices[data[i]] = i;
		sortedData.push_back(data[i]);
		//cout << data[i] << " ";
	}
	//cout << "\n";
	/*for (map<float,int>::iterator it=oldIndices.begin(); it!=oldIndices.end(); ++it)
		cout << it->first << " => " << it->second << '\n';*/
	cout << "[ DEBUG ] Sorting Data ...\n";
	sort(sortedData.begin(), sortedData.end());
	indices.clear();
	cout << "[ DEBUG ] Sorting Completed ...\n";
	cout << "[ DEBUG ] Making indices ...\n";
	for (i = 0; i < dataSize; ++i) {
		//cout << "[ DEBUG ] indices: " << oldIndices.at(sortedData[i]) << "\n";
		indices.push_back(oldIndices.at(sortedData[i]));
	}
	cout << "[ DEBUG ] Making indices Completed ...\n";

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

void getPlaneParameters(
	const vector<Point3f> &planePoints,
	const vector<int> planeIndices,
	vector< vector<float> >  &planeParameters) {

	vector< vector<Point3f> > planeOrderedPoints;
	int numberOfPlanes = numberOfUniquePlanes(planeIndices);
	int numberOfPointsInThePlane = planePoints.size();
	vector<Point3f> pointsOfAPlane;
	vector<float> planeParametersForThisPlane;
	int i;

	for (i = 0; i < numberOfPlanes; ++i) {
		planeOrderedPoints.push_back(pointsOfAPlane);
	}

	for (i = 0; i < numberOfPointsInThePlane; ++i) {
		planeOrderedPoints[planeIndices[i]].push_back(planePoints[i]);
	}

	float a, b, c, d;
	for (i = 0; i < numberOfPlanes; ++i) {

		fitPlane3D(planeOrderedPoints[i], planeParametersForThisPlane);
		planeParameters.push_back(planeParametersForThisPlane);
		planeParametersForThisPlane.clear();

	}

	return ;

}


void fitPlane3D(
	const vector<Point3f> &planePoints,
	vector<float>  &planeParameters) {

	float a, b, c, d;
	int j;
	int numberOfPointsInThisPlane = planePoints.size();
	Mat pointsMatrixTTemp(numberOfPointsInThisPlane, 3, CV_32F);
	for (j = 0; j < numberOfPointsInThisPlane; ++j) {
		pointsMatrixTTemp.at<float>(j, 0) = planePoints[j].x;
		pointsMatrixTTemp.at<float>(j, 1) = planePoints[j].y;
		pointsMatrixTTemp.at<float>(j, 2) = planePoints[j].z;
	}
	float centroidX = (mean(pointsMatrixTTemp.col(0)))[0];
	float centroidY = (mean(pointsMatrixTTemp.col(1)))[0];
	float centroidZ = (mean(pointsMatrixTTemp.col(2)))[0];
	Mat eigenvalues, eigenvectors;
	for (j = 0; j < numberOfPointsInThisPlane; ++j) {
		pointsMatrixTTemp.at<float>(0, j) = pointsMatrixTTemp.at<float>(0, j) - centroidX;
		pointsMatrixTTemp.at<float>(1, j) = pointsMatrixTTemp.at<float>(1, j) - centroidY;
		pointsMatrixTTemp.at<float>(2, j) = pointsMatrixTTemp.at<float>(2, j) - centroidZ;
	}
	eigen(pointsMatrixTTemp.t()*pointsMatrixTTemp, eigenvalues, eigenvectors);
	Mat minEigVector = eigenvectors.row(2);
	float normSum = 0.0;
	for(j = 0; j < 3; ++j) {
		normSum += (minEigVector.at<float>(0, j))*(minEigVector.at<float>(0, j));
	}
	for(j = 0; j < 3; ++j) {
		minEigVector.at<float>(0, j) = minEigVector.at<float>(0, j)/normSum;
	}
	Mat abc = minEigVector;
	a = abc.at<float>(0, 0); b = abc.at<float>(0, 1); c = abc.at<float>(0, 2); d = 0.0;
	d += centroidX*a; d += centroidY*b; d += centroidZ*c;
	d = (-1.0)*d; 
	planeParameters.push_back(a);
	planeParameters.push_back(b);
	planeParameters.push_back(c);
	planeParameters.push_back(d);

	return ;

}

//template<typename T>
void printVector(
			vector<int> data) {

	int i;
	for (i = 0; i < data.size(); ++i) {
		cout << data[i] << " ";
	}
	cout << "\n";

	return ;

}
