/**
 * @file additionalSteps.cpp
 * @ingroup additionalSteps
*/

/*
 *   File Name: additionalSteps.cpp
 *     Project: Multiple Plane JLinkage
 *  Created on: 29-Mar-2016
 *      Author: Sona Praneeth Akula
 *     Details: Additional Steps to be performed after JLinkage
 * 					1. Removing unncessary planes
 * 					2. Calculating distance of points from the respective planes
 * 					3. Get the 3D projections of pointsonto their respective planes
 */

#include "additionalSteps.hpp"
#include "utilities.hpp"

void removeUnnecessaryPlanes(
		const vector<Point3f> &oldData,
		const vector<int> &oldPlaneIndices,
		const int &minimumNumberOfPointsPerPlane,
		map<int, int> &numberOfPointsPerPlane,
		vector<Point3f> &newData,
		vector<int> &newPlaneIndices,
		int &numberOfPlanes) {

	cout << "[ DEBUG ] removeUnnecessaryPlanes Started\n";

	// Get the number of points present in the plane
	int numberOfPoints = oldData.size();
	// These are the number of planes before removal of unnecessary planes
	int i, numberOfUniquePlanes=0;
	// Mapping of old plane indices to new plane indices -> Sorted order
	map<int, int> oldToNewPlaneIndices;
	// Actual indices of planes selected after removing unnecessary planes
	vector<int> selectedPlaneIndices;

	// Calculate the number of unique planes before removal of unnecessary planes
	// and also how many points are there in each plane
	for(i = 0; i < numberOfPoints; i++) {
		if ( numberOfPointsPerPlane.find(oldPlaneIndices[i]) == numberOfPointsPerPlane.end() ) {
			numberOfPointsPerPlane[oldPlaneIndices[i]] = 1;
			numberOfUniquePlanes++;
		}
		else {
			numberOfPointsPerPlane[oldPlaneIndices[i]]++;
		}
	}

	cout << "[ DEBUG ] Detected number of planes before removing unnecessary planes : " << numberOfUniquePlanes << "\n";
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

	// Creating a mapping of old to new plane indices in sorted order
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
		if ( numberOfPointsPerPlane.count(oldPlaneIndices[i])>0 ) {
			Point3f pt3d = oldData[i];
			Point3f pt3f;
			pt3f.x = float(pt3d.x);
			pt3f.y = float(pt3d.y);
			pt3f.z = float(pt3d.z);
			newData.push_back(pt3f);
			newPlaneIndices.push_back(oldToNewPlaneIndices[oldPlaneIndices[i]]);
		}
	}


	cout << "[ DEBUG ] removeUnnecessaryPlanes Completed\n";
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
	// Get the number of planes
	int numberOfPlanes = planeParameters.size();
	cout << "[ DEBUG ] calculateDistanceFromPlane Started with " << numberOfPlanes << " planes.\n";
	int i;
	// Dummy vector to initialize planePointsIndexMapping
	vector<int> pointsIndex;

	// Generates a list of points belonging to plane i in planePointsIndexMapping[i]
	for (i = 0; i < numberOfPlanes; ++i) {
		planePointsIndexMapping.push_back(pointsIndex);
	}

	cout << "[ DEBUG ] Calculating the distances of each point to its respective plane\n";
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

	cout << "[ DEBUG ] calculateDistanceFromPlane Completed\n";
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
	// Get the number of planes
	int numberOfPlanes = planeParameters.size();
	// Create a dummy vector for initializing newPlaneParameters
	vector<float> abcd;

	for (i = 0; i < numberOfPlanes; ++i) {
		cout << "[ DEBUG ] Initial Plane Parameters for plane " << i << " are: ";
		for (j = 0; j < 4; ++j) {
			cout << planeParameters[i][j] << " ";
		}
		cout << "\n";
	}
	cout << "\n";

	vector<float> distanceOfPointsFromPlane;
	for(i = 0; i < numberOfPlanes; i++) {

		cout << "[ DEBUG ] Plane " << i << endl;
		int numberOfPointsInThePlane = planePointsIndexMapping[i].size();

		for(j = 0; j < numberOfPointsInThePlane; j++) {
			distanceOfPointsFromPlane.push_back(distanceMatrix[planePointsIndexMapping[i][j]]);
		}

		cout << "[ DEBUG ] Calculating Kth Percentile for distance threshold.\n";
		float distanceThreshold = getKthPercentile(distanceOfPointsFromPlane, 95.0);
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
		planeIndexBounds.insert(make_pair(i,make_pair(newDataSize, newDataSize+numberOfPointsInThePlane)));
		// Get the new plane parameters
		cout << "[ DEBUG ] Plane Index Bounds for plane " << i << " are  " << newDataSize << ", " << newDataSize+numberOfPointsInThePlane << "\n";

		cout << "Plane Parameters for this plane are: ";
		for (j = 0; j < 4; ++j) {
			cout << planeParameters[i][j] << " ";
		}
		cout << "\n";

		for(j = 0; j < 4; j++) {
			abcd.push_back(planeParameters[i][j]);
		}
		newPlaneParameters.push_back(abcd);
		abcd.clear();
		distanceOfPointsFromPlane.clear();
	}

	cout << "[ DEBUG ] removePointsFarFromPlane Completed.\n";
	return ;

}

void get3DPlaneProjectionsOfPoints (
		const vector<Point3f> &data,
		const vector< vector<float> > &planeParameters,
		const map<int, pair<int, int> > &planeIndexBounds,
		vector<Point3f> &projectionsOf3DPoints	) {

	cout << "[ DEBUG ] get3DPlaneProjectionsOfPoints Started\n";
	// Get the number of planes
	int numberOfPlanes = planeIndexBounds.size();
	int startOfPlanePoints, endOfPlanePoints;

	int i, j;
	for (i = 0; i < numberOfPlanes; ++i) {

		cout << "[ DEBUG ] Calculating 3D Projections for Plane " << i << "\n";
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
		// TODO: Check if the condition is j < endOfPlanePoints (or) j <= endOfPlanePoints
		for (j = startOfPlanePoints; j < endOfPlanePoints; ++j) {

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

/******************** EXTRA FUNCTIONS *****************************************/
void calculateDistanceFromPlane1(
		const vector< vector<Point3f> > &data,
		const vector< vector<float> > &planeParameters,
		vector< vector<float> > &distanceMatrix ) {

	// Get the number of planes
	int numberOfPlanes = planeParameters.size();
	cout << "[ DEBUG ] calculateDistanceFromPlane1 Started with " << numberOfPlanes << " planes.\n";
	int i, j;
	cout << "[ DEBUG ] Calculating the distances of each point to its respective plane\n";
	vector<float> distances;

	for(i = 0; i < numberOfPlanes; i++) {

		int size = data[i].size();
		distanceMatrix.push_back(distances);
		cout << "[ DEBUG ] Plane " << i << "\n";

		for (j = 0; j < size; j++) {

			// Get the plane parameters into variables a, b, c, d
			float a = planeParameters[i][0];
			float b = planeParameters[i][1];
			float c = planeParameters[i][2];
			float d = planeParameters[i][3];

			float value = (a*data[i][j].x)+(b*data[i][j].y)+(c*data[i][j].z)+d;
			float distance;

			// Calculate the distance of the point from the plane
			if (value>=0) {
				// Towards +ve normal
				distance = abs(((a*data[i][j].x)+(b*data[i][j].y)+(c*data[i][j].z)+d))/sqrt(pow(a,2)+pow(b,2)+pow(c,2));
			}
			else {
				// If it is on other side of plane. Towards -ve normal
				distance = abs(((-a*data[i][j].x)+(-b*data[i][j].y)+(-c*data[i][j].z)-d))/sqrt(pow(a,2)+pow(b,2)+pow(c,2));
			}

			// Push it into distance matrix
			distanceMatrix[i].push_back(distance);

		}

	}

	cout << "[ DEBUG ] calculateDistanceFromPlane1 Completed\n";
	return ;

}

void removePointsFarFromPlane1(
		const vector< vector<Point3f> > &data,
		const vector< vector<float> > &planeParameters,
		const vector< vector<float> > &distanceMatrix,
		vector< vector<Point3f> > &newData,
		vector< vector<float> > &newPlaneParameters ) {

	int i, j;
	cout << "[ DEBUG ] removePointsFarFromPlane1 Started\n";
	// Get the number of planes
	int numberOfPlanes = data.size(), size;
	// Create a dummy vector for initializing newPlaneParameters, newData
	vector<float> abcd; vector<Point3f> newPoints;

	for (i = 0; i < numberOfPlanes; ++i) {
		cout << "[ DEBUG ] Plane Parameters for plane " << i << " are: ";
		for (j = 0; j < 4; ++j) {
			cout << planeParameters[i][j] << " ";
		}
		cout << "\n";
	}
	cout << "\n";

	// Initialize newData
	cout << "[ DEBUG] Initialize newData\n";
	for (i = 0; i < numberOfPlanes; ++i) {
		size = data[i].size();
		newData.push_back(newPoints);
		for (j = 0; j < size; ++j) {
			newData[i].push_back(data[i][j]);
		}
	}

	for(i = 0; i < numberOfPlanes; i++) {

		cout << "[ DEBUG ] Plane " << i << endl;
		int numberOfPointsInThePlane = data[i].size();

		// Calculating the distance threshold using kth percentile
		cout << "[ DEBUG ] Calculating Kth Percentile for distance threshold.\n";
		float distanceThreshold = getKthPercentile(distanceMatrix[i], 95.0);
		cout << "[ DEBUG ] Calculating Kth Percentile Success!!!\n";

		int newDataSize = newData.size();
		// Remove points greater than distance threshold
		vector<Point3f>::iterator it = newData[i].begin();
		size = data[i].size();
		for(j = 0; j < size; j++) {
			if (distanceMatrix[i][j] > distanceThreshold ) {
				newData[i].erase(it);
			}
			++it; // TODO: Please verify this if it's not deleting needed points
		}
		cout << "[ DEBUG ] Added points of plane " << i << " to newSortedData\n";

		fitPlane3D(newData[i], abcd);
		newPlaneParameters.push_back(abcd);
		cout << "[ DEBUG ] New Plane Parameters for plane " << i << " are: ";
		for (j = 0; j < 4; ++j) {
			cout << newPlaneParameters[i][j] << " ";
		}
		cout << "\n";
		abcd.clear();

	}

	cout << "[ DEBUG ] removePointsFarFromPlane1 Completed.\n";
	return ;

}

void get3DPlaneProjectionsOfPoints1 (
		const vector< vector<Point3f> > &data,
		const vector< vector<float> > &planeParameters,
		vector< vector<Point3f> > &projectionsOf3DPoints	) {

	cout << "[ DEBUG ] get3DPlaneProjectionsOfPoints1 Started\n";
	// Get the number of planes
	int numberOfPlanes = data.size();
	vector<Point3f> new3DProjectedPoints;

	int i, j;
	for (i = 0; i < numberOfPlanes; ++i) {

		cout << "[ DEBUG ] Calculating 3D Projections for Plane " << i << "\n";

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
		// TODO: Check if the condition is j < endOfPlanePoints (or) j <= endOfPlanePoints
		int size = data[i].size();
		projectionsOf3DPoints.push_back(new3DProjectedPoints);
		for (j = 0; j < size; ++j) {

			float x0 = data[i][j].x;
			float y0 = data[i][j].y;
			float z0 = data[i][j].z;
			float t = ((-1)*(a*x0+b*y0+c*z0+d))/(a*a+b*b+c*c);
			float projX0 = x0 + a*t;
			float projY0 = y0 + b*t;
			float projZ0 = z0 + c*t;
			projectionsOf3DPoints[i].push_back(Point3f(projX0, projY0, projZ0));

		}

	}

	cout << "[ DEBUG ] get3DPlaneProjectionsOfPoints1 completed!!!!\n";
	return ;

}
