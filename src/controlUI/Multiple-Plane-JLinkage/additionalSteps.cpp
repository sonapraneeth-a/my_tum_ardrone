/**
 * @file additionalSteps.cpp
 * @ingroup additionalSteps
 *
 *   File Name: additionalSteps.cpp
 *     Project: Multiple Plane JLinkage
 *  Created on: 28-Mar-2016
 *      Author: Sona Praneeth Akula
 *     Details: Additional Steps to be performed after JLinkage
 * 					1. Removing unncessary planes
 * 					2. Calculating distance of points from the respective planes
 * 					3. Get the 3D projections of pointsonto their respective planes
 *   TodoNotes: TODO
 */

#include "additionalSteps.hpp"

void removeUnnecessaryPlanes(
		const vector<Point3f> &oldData,
		const vector<int> &planeIndices,
		const int &minimumNumberOfPointsPerPlane,
		map<int, int> &numberOfPointsPerPlane,
		vector<Point3f> &newData,
		vector<int> &newPlaneIndices,
		int &numberOfPlanes) {

	// Get the number of points present in the plane
	int numberOfPoints = planeIndices.size();
	int i, numberOfUniquePlanes=0;
	map<int, int> oldToNewPlaneIndices;
	// Actual indices of planes selected after removing unneccesary planes
	vector<int> selectedPlaneIndices;

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

	cout << "[ DEBUG ] Detected number of planes initially: " << numberOfUniquePlanes << "\n";
	for (map<int,int>::iterator it=numberOfPointsPerPlane.begin(); it!=numberOfPointsPerPlane.end(); ++it)
		cout << "[ DEBUG ] " << it->first << " => " << it->second << "\n";

	// Remove those planes which having less than minimum number of points in them
	for (i = 0; i < numberOfUniquePlanes; i++) {
		if( numberOfPointsPerPlane[i] <= minimumNumberOfPointsPerPlane ) {
			map<int, int>::iterator it = numberOfPointsPerPlane.find (i);
			numberOfPointsPerPlane.erase(it);
		}
		else {
			selectedPlaneIndices.push_back(i);
		}
	}
	int numberOfSelectedPlanes = selectedPlaneIndices.size();
	for (i = 0; i < numberOfSelectedPlanes; ++i) {
		oldToNewPlaneIndices[selectedPlaneIndices[i]] = i;
	}

	// Number of planes found after removal of unnecessary planes
	numberOfPlanes = numberOfPointsPerPlane.size();
	cout << "[ DEBUG ] Detected number of planes after removing unnecessary planes: " << numberOfPlanes << "\n";
	for (map<int,int>::iterator it=numberOfPointsPerPlane.begin(); it!=numberOfPointsPerPlane.end(); ++it)
		cout << "[ DEBUG ] " << it->first << " => " << it->second << "\n";
	cout << "[ DEBUG ] Old indices mapped to new indices" << endl;
	for (map<int,int>::iterator it=oldToNewPlaneIndices.begin(); it!=oldToNewPlaneIndices.end(); ++it)
		cout << "[ DEBUG ] " << it->first << " => " << it->second << "\n";

	// Create the new data after removing unnecessary planes and also new indices
	// corresponding to the newPlaneDataPoints
	for(i = 0; i < numberOfPoints; i++) {
		if ( numberOfPointsPerPlane.count(planeIndices[i])>0 ) {
			Point3f pt3d = oldData[i];
			Point3f pt3f;
			pt3f.x = float(pt3d.x);
			pt3f.y = float(pt3d.y);
			pt3f.z = float(pt3d.z);
			newData.push_back(pt3f);
			newPlaneIndices.push_back(oldToNewPlaneIndices[planeIndices[i]]);
		}
	}


	return ;

}

void calculateDistanceFromPlane(
		const vector<Point3f> &data,
		const vector< vector<float> > &planeParameters,
		const vector<int> &planeIndices,
		vector<float> &distanceMatrix,
		vector< vector<int> > &planePointsIndexMapping) {

	// Get the number of points in the plane
	int numberOfPoints = data.size();
	int numberOfPlanes = planeParameters.size();
	int i;
	vector<int> pointsIndex;

	for (i = 0; i < numberOfPlanes; ++i) {
		planePointsIndexMapping.push_back(pointsIndex);
	}
	for(i = 0; i < numberOfPoints; i++) {
		// Get the plane parameters into variables a, b, c, d
		float a = planeParameters[planeIndices[i]][0];
		float b = planeParameters[planeIndices[i]][1];
		float c = planeParameters[planeIndices[i]][2];
		float d = planeParameters[planeIndices[i]][3];

		float value = (a*data[i].x)+(b*data[i].y)+(c*data[i].z)+d;
		float distance;

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
		const vector<Point3f> &data,
		const vector< vector<float> > &planeParameters,
		const vector<float> &distanceMatrix,
		const vector< vector<int> > &planePointsIndexMapping,
		vector<Point3f> &newSortedData,
		vector< vector<float> > &newPlaneParameters,
		map<int, pair<int, int> > &planeIndexBounds) {

	int i, j;
	int numberOfPlanes = planePointsIndexMapping.size();
	vector<float> abcd;
	for (i = 0; i < 4; ++i) {
		abcd.push_back(0.0);
	}
	for (i = 0; i < numberOfPlanes; ++i) {
		cout << "[ DEBUG ] Plane Parameters for plane " << i << " are: ";
		for (j = 0; j < 4; ++j) {
			cout << planeParameters[i][j] << " ";
		}
	}
	cout << "\n";
	for (i = 0; i < numberOfPlanes; ++i) {
		newPlaneParameters.push_back(abcd);
	}
	cout << "[ DEBUG ] Initialised newPlaneParameters.\n";

	vector<float> distanceOfPointsFromPlane;
	for(i = 0; i < numberOfPlanes; i++) {
		cout << "[ DEBUG ] Plane " << i << endl;
		int numberOfPointsInThePlane = planePointsIndexMapping[i].size();
		for(j = 0; j < numberOfPointsInThePlane; j++) {
			distanceOfPointsFromPlane.push_back(distanceMatrix[planePointsIndexMapping[i][j]]);
		}
		cout << "[ DEBUG ] Calculating Kth Percentile.\n";
		float distanceThreshold = getKthPercentile(distanceOfPointsFromPlane, 95);
		cout << "[ DEBUG ] Calculating Kth Percentile Success!!!\n";
		int newDataSize = newSortedData.size(), numberOfPointsInTheCurrentPlane=0;
		for(j = 0; j < numberOfPointsInThePlane; j++) {
			if (distanceOfPointsFromPlane[j] <= distanceThreshold ) {
				newSortedData.push_back(data[planePointsIndexMapping[i][j]]);
				numberOfPointsInTheCurrentPlane++;
			}
		}
		cout << "[ DEBUG ] Added points of plane " << i << " to newSortedData\n";
		// Get plane index bounds after making new data for plane i
		planeIndexBounds[i] = make_pair(newDataSize, newDataSize+numberOfPointsInThePlane);
		// Get the new plane parameters
		cout << "[ DEBUG ] Plane Index Bounds for plane " << i << " are  " << newDataSize << ", " << newDataSize+numberOfPointsInThePlane << "\n";
		cout << "Plane Parameters for this plane are: ";
		for (j = 0; j < 4; ++j) {
			cout << planeParameters[i][j] << " ";
		}
		cout << "\n";
		newPlaneParameters[i] = planeParameters[i];
		distanceOfPointsFromPlane.clear();
	}

	cout << "[ DEBUG ] End of removePointsFarFromPlane.\n";
	return ;

}

void removePointsFarFromPlane1(
		const vector<Point3f> &data,
		const vector< vector<float> > &planeParameters,
		const vector<float> &distanceMatrix,
		const vector< vector<int> > &planePointsIndexMapping,
		vector< vector<Point3f> > &newSortedData,
		vector< vector<float> > &newPlaneParameters ) {

	int i, j;
	int numberOfPlanes = planePointsIndexMapping.size();

	vector<float> distanceOfPointsFromPlane;
	vector<Point3f> pointsInThePlane;
	for(i = 0; i < numberOfPlanes; i++) {
		int numberOfPointsInThePlane = planePointsIndexMapping[i].size();
		for(j = 0; j < numberOfPointsInThePlane; j++) {
			distanceOfPointsFromPlane.push_back(distanceMatrix[planePointsIndexMapping[i][j]]);
		}
		float distanceThreshold = getKthPercentile(distanceOfPointsFromPlane, 95);
		int newDataSize = newSortedData.size(), numberOfPointsInTheCurrentPlane=0;
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
		const vector<Point3f> &data,
		const vector< vector<float> > &planeParameters,
		const map<int, pair<int, int> > &planeIndexBounds,
		vector<Point3f> &projectionsOf3DPoints	) {

	int numberOfPlanes = planeIndexBounds.size();
	int startOfPlanePoints, endOfPlanePoints;

	int i, j;
	for (i = 0; i < numberOfPlanes; ++i) {
		cout << "[ DEBUG ] Plane " << i << endl;
		startOfPlanePoints = planeIndexBounds.at(i).first;
		endOfPlanePoints = planeIndexBounds.at(i).second;
		// Get the plane parameters into variables a, b, c, d
		float a = planeParameters[i][0];
		float b = planeParameters[i][1];
		float c = planeParameters[i][2];
		float d = planeParameters[i][3];
		// Get the projected points as
		// x = x0 + a*t;
		// y = y0 + b*t
		// z = z0 + c*t
		// Put x, y, z in ax+by+cz+d=0 to get t
		for (j = startOfPlanePoints; j <= endOfPlanePoints; ++j) {
			float x0 = data[j].x;
			float y0 = data[j].y;
			float z0 = data[j].z;
			float t = ((-1)*(a*x0+b*y0+c*z0+d))/(a*a+b*b+c*c);
			float projX0 = x0 + a*t;
			float projY0 = y0 + b*t;
			float projZ0 = z0 + c*t;
			projectionsOf3DPoints.push_back(Point3f(projX0, projY0, projZ0));
		}
	}
	cout << "[ DEBUG ] get3DPlaneProjectionsOfPoints completed!!!!\n";
	return ;

}

void get3DPlaneProjectionsOfPoints1 (
		const vector< vector<Point3f> > &data,
		const vector< vector<float> > &planeParameters,
		vector<Point3f> &projectionsOf3DPoints	) {

	int numberOfPlanes = data.size();
	int numberOfPointsInThisPlane;

	int i, j;
	for (i = 0; i < numberOfPlanes; ++i) {
		// Get the plane parameters into variables a, b, c, d
		float a = planeParameters[i][0];
		float b = planeParameters[i][1];
		float c = planeParameters[i][2];
		float d = planeParameters[i][3];
		numberOfPointsInThisPlane = data[i].size();
		// Get the projected points as
		// x = x0 + a*t;
		// y = y0 + b*t
		// z = z0 + c*t
		// Put x, y, z in ax+by+cz+d=0 to get t
		for (j = 0; j < numberOfPointsInThisPlane; ++j) {
			float x0 = data[i][j].x;
			float y0 = data[i][j].y;
			float z0 = data[i][j].z;
			float t = ((-1)*(a*x0+b*y0+c*z0+d))/(a*a+b*b+c*c);
			float projX0 = x0 + a*t;
			float projY0 = y0 + b*t;
			float projZ0 = z0 + c*t;
			projectionsOf3DPoints.push_back(Point3f(projX0, projY0, projZ0));
		}
	}
	return ;

}

float getKthPercentile(
		const vector<float> &data,
		const int k) {

	// Reference:
	// http://web.stanford.edu/class/archive/anthsci/anthsci192/anthsci192.1064/handouts/calculating%20percentiles.pdf
	float threshold;
	vector<float> copiedData;
	int sizeOfData = data.size();
	int i;
	for( i = 0; i < sizeOfData; i++) {
		copiedData.push_back(data[i]);
	}

	sort(copiedData.begin(), copiedData.end());

	int size = copiedData.size();
	float percentile;
	for (i = 0; i < size; ++i) {
		percentile = (100.0/float(size))*(i+1.0-0.5);
		if ( (int)(percentile) >= k) {
			threshold = copiedData[i];
		}
	}
	copiedData.clear();

	return threshold;

}


