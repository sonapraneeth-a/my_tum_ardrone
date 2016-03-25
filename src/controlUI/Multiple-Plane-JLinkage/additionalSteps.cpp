/**
 * @file additionalSteps.cpp
 * @ingroup additionalSteps
 *
 *   File Name: additionalSteps.cpp
 *     Project: Multiple Plane JLinkage
 *  Created on: 22-Mar-2016
 *      Author: Sona Praneeth Akula
 *     Details:
 *   TodoNotes: TODO
 */

#include "additionalSteps.hpp"

void removeUnnecessaryPlanes(
		const vector<Point3d> &oldData,
		const vector<long long int> &planeIndices,
		const long long int &minimumNumberOfPointsPerPlane,
		map<long long int, long long int> &numberOfPointsPerPlane,
		vector<Point3d> &newData,
		vector<long long int> &newPlaneIndices,
		long long int &numberOfPlanes) {

	// Get the number of points present in the plane
	long long int numberOfPoints = planeIndices.size();
	long long int i, numberOfUniquePlanes=0;

	// Calculate the number of unique planes and also how many points are there
	// in each plane
	for(i = 0; i < numberOfPoints; i++) {
		if ( numberOfPointsPerPlane.find(planeIndices[i]) == numberOfPointsPerPlane.end() ) {
			numberOfPointsPerPlane[planeIndices[i]] = 1;
			numberOfUniquePlanes++;
		} else {
			numberOfPointsPerPlane[planeIndices[i]]++;
		}
	}

	// Remove those planes which having less than minimum number of points in them
	for (i = 1; i < numberOfUniquePlanes; i++) {
		if( numberOfPointsPerPlane[i] <= minimumNumberOfPointsPerPlane ) {
			map<long long int, long long int>::iterator it = numberOfPointsPerPlane.find (i);
			numberOfPointsPerPlane.erase(it);
		}
	}

	// Create the new data after removing unnecessary planes and also new indices
	// corresponding to the newPlaneDataPoints
	for(i = 0; i < numberOfPoints; i++) {
		if ( numberOfPointsPerPlane.count(planeIndices[i])>0 ) {
			newData.push_back(oldData[i]);
			newPlaneIndices.push_back(planeIndices[i]);
		}
	}

	// Number of planes found after removal of unnecessary planes
	numberOfPlanes = numberOfPointsPerPlane.size();

	return ;

}

void calculateDistanceFromPlane(
		const vector<Point3d> &data,
		const vector< vector<double> > &planeParameters,
		const vector<long long int> &planeIndices,
		vector<double> &distanceMatrix,
		vector< vector<long long int> > &planePointsIndexMapping) {

	// Get the number of points in the plane
	long long int numberOfPoints = data.size();
	long long int i;

	for(i = 0; i < numberOfPoints; i++) {
		// Get the plane parameters into variables a, b, c, d
		double a = planeParameters[planeIndices[i]][0];
		double b = planeParameters[planeIndices[i]][1];
		double c = planeParameters[planeIndices[i]][2];
		double d = planeParameters[planeIndices[i]][3];

		double value = (a*data[i].x)+(b*data[i].y)+(c*data[i].z)+d;
		double distance;

		// Calculate the distance of the point from the plane
		if (value>=0) {
			// Towards +ve normal
			distance = abs(((a*data[i].x)+(b*data[i].y)+(c*data[i].z)+d))/sqrt(pow(a,2)+pow(b,2)+pow(c,2));
		}
		else {
			// If it is on other side of plane. Towards -ve normal
			distance = abs(((-a*data[i].x)+(-b*data[i].y)+(-c*data[i].z)-d))/sqrt(pow(a,2)+pow(b,2)+pow(c,2));
		}

		// Push it into distance matrix
		distanceMatrix.push_back(distance);
		// Add point i to plane planePointsIndexMapping[i]
		// This means include this particular point with index i in plane j determined
		// by planeIndices[i]
		planePointsIndexMapping[planeIndices[i]].push_back(i);

	}

	return ;

}

void removePointsFarFromPlane(
		const vector<Point3d> &data,
		const vector< vector<double> > &planeParameters,
		const vector<double> &distanceMatrix,
		const vector< vector<LLI> > &planePointsIndexMapping,
		vector<Point3d> &newSortedData,
		vector< vector<double> > &newPlaneParameters,
		map<LLI, pair<LLI, LLI> > &planeIndexBounds) {

	LLI i, j;
	LLI numberOfPlanes = planePointsIndexMapping.size();

	vector<double> distanceOfPointsFromPlane;
	for(i = 0; i < numberOfPlanes; i++) {
		LLI numberOfPointsInThePlane = planePointsIndexMapping[i].size();
		for(j = 0; j < numberOfPointsInThePlane; j++) {
			distanceOfPointsFromPlane.push_back(distanceMatrix[planePointsIndexMapping[i][j]]);
		}
		double distanceThreshold = getKthPercentile(distanceOfPointsFromPlane, 95);
		LLI newDataSize = newSortedData.size(), numberOfPointsInTheCurrentPlane=0;
		for(j = 0; j < numberOfPointsInThePlane; j++) {
			if (distanceOfPointsFromPlane[j] <= distanceThreshold ) {
				newSortedData.push_back(data[planePointsIndexMapping[i][j]]);
				numberOfPointsInTheCurrentPlane++;
			}
		}
		// Get plane index bounds after making new data for plane i
		planeIndexBounds[i] = make_pair(newDataSize, newDataSize+numberOfPointsInThePlane);
		// Get the new plane parameters
		newPlaneParameters[i] = planeParameters[planeIndexBounds[i].first];
		distanceOfPointsFromPlane.clear();
	}

	return ;

}

void removePointsFarFromPlane1(
		const vector<Point3d> &data,
		const vector< vector<double> > &planeParameters,
		const vector<double> &distanceMatrix,
		const vector< vector<LLI> > &planePointsIndexMapping,
		vector< vector<Point3d> > &newSortedData,
		vector< vector<double> > &newPlaneParameters ) {

	LLI i, j;
	LLI numberOfPlanes = planePointsIndexMapping.size();

	vector<double> distanceOfPointsFromPlane;
	vector<Point3d> pointsInThePlane;
	for(i = 0; i < numberOfPlanes; i++) {
		LLI numberOfPointsInThePlane = planePointsIndexMapping[i].size();
		for(j = 0; j < numberOfPointsInThePlane; j++) {
			distanceOfPointsFromPlane.push_back(distanceMatrix[planePointsIndexMapping[i][j]]);
		}
		double distanceThreshold = getKthPercentile(distanceOfPointsFromPlane, 95);
		LLI newDataSize = newSortedData.size(), numberOfPointsInTheCurrentPlane=0;
		for(j = 0; j < numberOfPointsInThePlane; j++) {
			if (distanceOfPointsFromPlane[j] <= distanceThreshold ) {
				pointsInThePlane.push_back(data[planePointsIndexMapping[i][j]]);
				numberOfPointsInTheCurrentPlane++;
			}
		}
		newSortedData.push_back(pointsInThePlane);
		pointsInThePlane.clear();
		// Get the new plane parameters
		newPlaneParameters[i] = planeParameters[i];
		distanceOfPointsFromPlane.clear();
	}

	return ;

}

void get3DPlaneProjectionsOfPoints (
		const vector<Point3d> &data,
		const vector< vector<double> > &planeParameters,
		const map<LLI, pair<LLI, LLI> > &planeIndexBounds,
		vector<Point3d> &projectionsOf3DPoints	) {

	LLI numberOfPlanes = planeIndexBounds.size();
	LLI startOfPlanePoints, endOfPlanePoints;

	LLI i, j;
	for (i = 0; i < numberOfPlanes; ++i) {
		startOfPlanePoints = planeIndexBounds[i].first;
		endOfPlanePoints = planeIndexBounds[i].second;
		// Get the plane parameters into variables a, b, c, d
		double a = planeParameters[i][0];
		double b = planeParameters[i][1];
		double c = planeParameters[i][2];
		double d = planeParameters[i][3];
		// Get the projected points as
		// x = x0 + a*t;
		// y = y0 + b*t
		// z = z0 + c*t
		// Put x, y, z in ax+by+cz+d=0 to get t
		for (j = startOfPlanePoints; j <= endOfPlanePoints; ++j) {
			double x0 = data[j].x;
			double y0 = data[j].y;
			double z0 = data[j].z;
			double t = ((-1)*(a*x0+b*y0+c*z0+d))/(a*a+b*b+c*c);
			double projX0 = x0 + a*t;
			double projY0 = y0 + b*t;
			double projZ0 = z0 + c*t;
			projectionsOf3DPoints.push_back(Point3d(projX0, projY0, projZ0));
		}
	}
	return ;

}

void get3DPlaneProjectionsOfPoints1 (
		const vector< vector<Point3d> > &data,
		const vector< vector<double> > &planeParameters,
		vector<Point3d> &projectionsOf3DPoints	) {

	LLI numberOfPlanes = data.size();
	LLI numberOfPointsInThisPlane;

	LLI i, j;
	for (i = 0; i < numberOfPlanes; ++i) {
		// Get the plane parameters into variables a, b, c, d
		double a = planeParameters[i][0];
		double b = planeParameters[i][1];
		double c = planeParameters[i][2];
		double d = planeParameters[i][3];
		numberOfPointsInThisPlane = data[i].size();
		// Get the projected points as
		// x = x0 + a*t;
		// y = y0 + b*t
		// z = z0 + c*t
		// Put x, y, z in ax+by+cz+d=0 to get t
		for (j = 0; j < numberOfPointsInThisPlane; ++j) {
			double x0 = data[i][j].x;
			double y0 = data[i][j].y;
			double z0 = data[i][j].z;
			double t = ((-1)*(a*x0+b*y0+c*z0+d))/(a*a+b*b+c*c);
			double projX0 = x0 + a*t;
			double projY0 = y0 + b*t;
			double projZ0 = z0 + c*t;
			projectionsOf3DPoints.push_back(Point3d(projX0, projY0, projZ0));
		}
	}
	return ;

}

double getKthPercentile(
		const vector<double> &data,
		const LLI k) {

	// Reference:
	// http://web.stanford.edu/class/archive/anthsci/anthsci192/anthsci192.1064/handouts/calculating%20percentiles.pdf
	double threshold;
	vector<double> copiedData;
	copiedData = data;

	sort(copiedData.begin(), copiedData.end());

	LLI i, size = copiedData.size();
	double percentile;
	for (i = 0; i < size; ++i) {
		percentile = (100.0/double(size))*(i+1.0-0.5);
		if ( (LLI)(percentile) >= k) {
			threshold = copiedData[i];
		}
	}
	return threshold;

}


