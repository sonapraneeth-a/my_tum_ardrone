/*
 *   File Name: makeBoundingRects.cpp
 *     Project: Multiple Plane JLinkage
 *  Created on: 22-Mar-2016
 *      Author: Sona Praneeth Akula
 *     Details:
 *   TodoNotes: TODO
 */


#include "allHeaders.hpp"
#include "makeBoundingRects.hpp"

void orderPlanePointsByCentroids(
		const vector<Point3f> &projectionOf3DPoints,
		const vector< vector<float> > &planeParameters,
		const map<int, pair<int, int> > &planeIndexBounds,
		vector<Point3f> &sortedProjectionsOf3DPoints,
		vector< vector<float> > &sortedPlaneParameters,
		map<int, pair<int, int> > &sortedPlaneIndexBounds ) {

	cout << "[ DEBUG ] orderPlanePointsByCentroids Started.\n";
	int numberOfPlanes = planeParameters.size();
	int i, j;
	int startIndex, endIndex;
	vector<float> xCentroidPoints;
	vector<float> sortedXCentroidPoints;
	vector<int> indices;
	float xCentroid = 0.0;
	vector<float> abcd;
	for( j = 0; j < 4; j++) {
		abcd.push_back(0.0);
	}
	for(i = 0; i < numberOfPlanes; i++) {
		sortedPlaneParameters.push_back(abcd);
	}

	for(i = 0; i < numberOfPlanes; i++) {
		cout << "[ DEBUG ] Finding XCentroid Plane " << i << ".\n";
		xCentroid = 0.0;
		startIndex = planeIndexBounds.at(i).first;
		endIndex = planeIndexBounds.at(i).second;
		for (j = startIndex; j <= endIndex; ++j) {
			xCentroid += projectionOf3DPoints[i].x;
		}
		xCentroid /= (endIndex-startIndex+1);
		xCentroidPoints.push_back(xCentroid);
	}
	cout << "[ DEBUG ] Sorting X Centroid Data Started.\n";
	sortData( xCentroidPoints, sortedXCentroidPoints, indices);
	cout << "[ DEBUG ] Sorting X Centroid Data Completed.\n";
	for(i = 0; i < numberOfPlanes; i++) {
		cout << "[ DEBUG ] Sorted Projections for Plane " << i << ".\n";
		startIndex = planeIndexBounds.at(indices[i]).first;
		endIndex = planeIndexBounds.at(indices[i]).second;
		for (j = startIndex; j <= endIndex; ++j) {
			sortedProjectionsOf3DPoints.push_back(projectionOf3DPoints[j]);
		}
		for( j = 0; j < 4; j++) {
			sortedPlaneParameters[i][j] = planeParameters[indices[i]][j];
		}
		sortedPlaneIndexBounds[i] = make_pair(startIndex, endIndex);
	}

	return ;

}

void orderPlanePointsByCentroids1(
		const vector< vector<Point3f> > &projectionOf3DPoints,
		const vector< vector<float> > &planeParameters,
		vector< vector<Point3f> > &sortedProjectionsOf3DPoints,
		vector< vector<float> > &sortedPlaneParameters ) {


	int numberOfPlanes = planeParameters.size();
	int i, j;
	int numberOfPointsInThisPlane;
	vector<float> xPoints;
	vector<float> sortedXPoints;
	vector<int> indices;
	float xCentroid = 0.0;
	vector<Point3f> points;
	//sortedProjectionsOf3DPoints(vector< vector<Point3f> >(numberOfPlanes));
	for (i = 0; i < numberOfPlanes; ++i) {
		sortedProjectionsOf3DPoints.push_back(points);
	}

	for(i = 0; i < numberOfPlanes; i++) {
		numberOfPointsInThisPlane = projectionOf3DPoints[i].size();
		for (j = 0; j < numberOfPointsInThisPlane; ++j) {
			xPoints.push_back(projectionOf3DPoints[i][j].x);
			xCentroid += projectionOf3DPoints[i][j].x;
		}
		xCentroid /= numberOfPointsInThisPlane;
		sortData( xPoints, sortedXPoints, indices);
		sortedProjectionsOf3DPoints[i] = projectionOf3DPoints[indices[i]];
		sortedPlaneParameters[i] = planeParameters[indices[i]];
	}

	return ;

}

void getBoundingBoxCoordinates (
		const vector<Point3f> &sortedProjectionOf3DPoints,
		const vector< vector<float> > &sortedPlaneParameters,
		const map<int, pair<int, int> > &sortedPlaneIndexBounds,
		vector< vector<Point3f> > &boundingBoxPoints ) {

	cout << "[ DEBUG ] getBoundingBoxCoordinates Started\n";
	int numberOfPlanes = sortedPlaneIndexBounds.size();
	int i, j;
	int numberOfPointsInThePlane, indexOne, indexTwo;
	vector<Point3f> pointsInThePlane;
	vector<Point2f> uvCoord;
	vector<float> uCoord, vCoord;
	vector<Point3f> uvAxes;
	vector<Point2f> planeUVBoundingPoints;
	vector<Point3f> planeXYZBoundingPoints;

	for (i = 0; i < numberOfPlanes; ++i) {
		cout << "[ DEBUG ] Plane " << i << "\n";
		indexOne = sortedPlaneIndexBounds.at(i).first;
		indexTwo = sortedPlaneIndexBounds.at(i).second;
		numberOfPointsInThePlane = indexTwo - indexOne;
		cout << "[ DEBUG ] Points are starting from " << indexOne << " " << indexTwo << "\n";
		cout << "[ DEBUG ] There are " << numberOfPointsInThePlane << " in the plane\n";
		pointsInThePlane.clear();
		uvAxes.clear();
		uvCoord.clear();
		for (j = indexOne; j < indexTwo; ++j) {
			pointsInThePlane.push_back(sortedProjectionOf3DPoints[j]);
		}

		cout << "[ DEBUG ] Converting XYZ to UV\n";
		AllXYZToUVCoordinates( pointsInThePlane, sortedPlaneParameters[i],
								uvCoord, uvAxes);

		uCoord.clear();
		for (j = 0; j < numberOfPointsInThePlane; ++j) {
			uCoord.push_back(uvCoord[j].x);
		}
		vCoord.clear();
		for (j = 0; j < numberOfPointsInThePlane; ++j) {
			vCoord.push_back(uvCoord[j].y);
		}

		float minU = *min_element(uCoord.begin(), uCoord.end());
		float maxU = *max_element(uCoord.begin(), uCoord.end());
		float minV = *min_element(vCoord.begin(), vCoord.end());
		float maxV = *max_element(vCoord.begin(), vCoord.end());
		Point2f bottomLeft = Point2f(minU, minV);
		Point2f bottomRight = Point2f(maxU, minV);
		Point2f topLeft = Point2f(minU, maxV);
		Point2f topRight = Point2f(maxU, maxV);

		planeUVBoundingPoints.clear();
		planeXYZBoundingPoints.clear();
		cout << "[ DEBUG ] Making planeUVBoundingPoints\n";
		planeUVBoundingPoints.push_back(bottomLeft);
		planeUVBoundingPoints.push_back(bottomRight);
		planeUVBoundingPoints.push_back(topLeft);
		planeUVBoundingPoints.push_back(topRight);

		cout << "[ DEBUG ] Converting UV to XYZ\n";
		AllUVToXYZCoordinates( planeUVBoundingPoints, uvAxes, sortedPlaneParameters[i][3],
				planeXYZBoundingPoints);

		vector<float> zCoord;
		vector<float> xCoord;
		vector<float> sortedZCoord;
		vector<int> sortedZCoordOriginalIndices;

		zCoord.clear();
		xCoord.clear();
		sortedZCoord.clear();
		sortedZCoordOriginalIndices.clear();
		for (j = 0; j < numberOfPointsInThePlane; ++j) {
			zCoord.push_back(planeXYZBoundingPoints[j].z);
			xCoord.push_back(planeXYZBoundingPoints[j].x);
		}
		cout << "[ DEBUG ] Sorting by Z Coordinates\n";
		sortData(zCoord, sortedZCoord, sortedZCoordOriginalIndices);
		cout << "[ DEBUG ] Swapping by X Coordinates\n";
		if(xCoord[sortedZCoordOriginalIndices[0]] >= xCoord[sortedZCoordOriginalIndices[1]]) {
			swap(planeXYZBoundingPoints[sortedZCoordOriginalIndices[0]],
					planeXYZBoundingPoints[sortedZCoordOriginalIndices[1]]);
		}
		if(xCoord[sortedZCoordOriginalIndices[2]] < xCoord[sortedZCoordOriginalIndices[3]]) {
			swap(planeXYZBoundingPoints[sortedZCoordOriginalIndices[2]],
					planeXYZBoundingPoints[sortedZCoordOriginalIndices[3]]);
		}

		planeXYZBoundingPoints.push_back(planeXYZBoundingPoints[0]);
		cout << "[ DEBUG ] Created Bounding Box Points for Plane " << i << "\n";
		boundingBoxPoints.push_back(planeXYZBoundingPoints);


	}
	cout << "[ DEBUG ] getBoundingBoxCoordinates Completed.\n";

	return ;

}

void getBoundingBoxCoordinates1 (
		const vector< vector<Point3f> > &sortedProjectionOf3DPoints,
		const vector< vector<float> > &sortedPlaneParameters,
		vector< vector<Point3f> > &boundingBoxPoints ) {


	int numberOfPlanes = sortedProjectionOf3DPoints.size();
	int i, j;
	int numberOfPointsInThePlane, indexOne, indexTwo;
	vector<Point3f> pointsInThePlane;
	vector<Point2f> uvCoord;
	vector<float> uCoord, vCoord;
	vector<Point3f> uvAxes;
	vector<Point2f> planeUVBoundingPoints;
	vector<Point3f> planeXYZBoundingPoints;

	for (i = 0; i < numberOfPlanes; ++i) {
		numberOfPointsInThePlane = sortedProjectionOf3DPoints[i].size();
		pointsInThePlane = sortedProjectionOf3DPoints[i];

		AllXYZToUVCoordinates( pointsInThePlane, sortedPlaneParameters[i],
								uvCoord, uvAxes);

		for (j = 0; j < numberOfPointsInThePlane; ++j) {
			uCoord.push_back(uvCoord[j].x);
		}
		for (j = 0; j < numberOfPointsInThePlane; ++j) {
			vCoord.push_back(uvCoord[j].y);
		}

		float minU = *min_element(uCoord.begin(), uCoord.end());
		float maxU = *max_element(uCoord.begin(), uCoord.end());
		float minV = *min_element(vCoord.begin(), vCoord.end());
		float maxV = *max_element(vCoord.begin(), vCoord.end());
		Point2f bottomLeft = Point2f(minU, minV);
		Point2f bottomRight = Point2f(maxU, minV);
		Point2f topLeft = Point2f(minU, maxV);
		Point2f topRight = Point2f(maxU, maxV);

		planeUVBoundingPoints.push_back(bottomLeft);
		planeUVBoundingPoints.push_back(bottomRight);
		planeUVBoundingPoints.push_back(topLeft);
		planeUVBoundingPoints.push_back(topRight);

		AllUVToXYZCoordinates( planeUVBoundingPoints, uvAxes, sortedPlaneParameters[i][3],
				planeXYZBoundingPoints);

		vector<float> zCoord;
		vector<float> xCoord;
		vector<float> sortedZCoord;
		vector<int> sortedZCoordOriginalIndices;


		for (j = 0; j < numberOfPointsInThePlane; ++j) {
			zCoord.push_back(planeXYZBoundingPoints[j].z);
			xCoord.push_back(planeXYZBoundingPoints[j].x);
		}
		sortData(zCoord, sortedZCoord, sortedZCoordOriginalIndices);

		if(xCoord[sortedZCoordOriginalIndices[0]] >= xCoord[sortedZCoordOriginalIndices[1]]) {
			swap(planeXYZBoundingPoints[sortedZCoordOriginalIndices[0]],
					planeXYZBoundingPoints[sortedZCoordOriginalIndices[1]]);
		}
		if(xCoord[sortedZCoordOriginalIndices[2]] < xCoord[sortedZCoordOriginalIndices[3]]) {
			swap(planeXYZBoundingPoints[sortedZCoordOriginalIndices[2]],
					planeXYZBoundingPoints[sortedZCoordOriginalIndices[3]]);
		}

		planeXYZBoundingPoints.push_back(planeXYZBoundingPoints[0]);
		boundingBoxPoints.push_back(planeXYZBoundingPoints);


	}


	return ;

}

void getContinuousBoundingBox (
		const vector< vector<Point3f> > &boundingBoxPoints,
		const vector< vector<float> > &sortedPlaneParameters,
		vector< vector<Point3f> > &continuousBoundingBoxPoints) {

	cout << "[ DEBUG ] getContinuousBoundingBox Started\n";
	int numberOfPlanes = sortedPlaneParameters.size();
	int i, j;
	vector< vector<float> > lineParameters1;
	vector< vector<float> > lineParameters2;
	vector< vector<float> > lineParameters3;
	vector< vector<float> > lineParameters4;
	vector< vector<float> > lineIntersectionOfPlanes;
	int firstPlaneBoundingBoxStart, secondPlaneBoundingBoxStart;
	Point3f firstPoint, secondPoint;
	Point3f point1, point2, point3, point4;

	int boundingBoxPointsSize = boundingBoxPoints.size();
	cout << "[ DEBUG ] Initializing continuousBoundingBoxPoints\n";
	for (i = 0; i < boundingBoxPointsSize; ++i) {
		int size = boundingBoxPoints[i].size();
		for(j = 0; j < size; ++j) {
			continuousBoundingBoxPoints.push_back(boundingBoxPoints[i]);
		}
	}
	cout << "[ DEBUG ] continuousBoundingBoxPoints Initialized\n";
	for (i = 0; i < numberOfPlanes-1; ++i) {
		/*lineIntersectionOfPlanes.swap(vector< <vector<float> >());
		lineParameters1.swap(vector< <vector<float> >());
		lineParameters2.swap(vector< <vector<float> >());
		lineParameters3.swap(vector< <vector<float> >());
		lineParameters4.swap(vector< <vector<float> >());*/
		cout << "[ DEBUG ] Calculating intersections for plane " << i << " and plane " << i+1 << "\n";
		calculateIntersectionOfPlanes( sortedPlaneParameters[i],
				sortedPlaneParameters[i+1], lineIntersectionOfPlanes);

		firstPlaneBoundingBoxStart = 5*i;
		secondPlaneBoundingBoxStart = 5*(i+1);

		firstPoint = boundingBoxPoints[i][firstPlaneBoundingBoxStart];
		secondPoint = boundingBoxPoints[i][firstPlaneBoundingBoxStart+1];
		cout << "[ DEBUG ] Calculate Line 1 for plane " << i << "\n";
		makeLineFromPoints( firstPoint, secondPoint, lineParameters1);
		firstPoint = boundingBoxPoints[i][firstPlaneBoundingBoxStart+2];
		secondPoint = boundingBoxPoints[i][firstPlaneBoundingBoxStart+3];
		cout << "[ DEBUG ] Calculate Line 2 for plane " << i << "\n";
		makeLineFromPoints( firstPoint, secondPoint, lineParameters2);

		firstPoint = boundingBoxPoints[i][secondPlaneBoundingBoxStart];
		secondPoint = boundingBoxPoints[i][secondPlaneBoundingBoxStart+1];
		cout << "[ DEBUG ] Calculate Line 3 for plane " << i << "\n";
		makeLineFromPoints( firstPoint, secondPoint, lineParameters3);
		firstPoint = boundingBoxPoints[i][secondPlaneBoundingBoxStart+2];
		secondPoint = boundingBoxPoints[i][secondPlaneBoundingBoxStart+3];
		cout << "[ DEBUG ] Calculate Line 4 for plane " << i << "\n";
		makeLineFromPoints( firstPoint, secondPoint, lineParameters4);

		cout << "[ DEBUG ] Calculate Point 1 for plane " << i << "\n";
		point1 = calculateIntersectionOfLines( lineParameters1, lineIntersectionOfPlanes);
		cout << "[ DEBUG ] Calculate Point 2 for plane " << i << "\n";
		point2 = calculateIntersectionOfLines( lineParameters2, lineIntersectionOfPlanes);
		cout << "[ DEBUG ] Calculate Point 3 for plane " << i << "\n";
		point3 = calculateIntersectionOfLines( lineParameters3, lineIntersectionOfPlanes);
		cout << "[ DEBUG ] Calculate Point 4 for plane " << i << "\n";
		point4 = calculateIntersectionOfLines( lineParameters4, lineIntersectionOfPlanes);

		continuousBoundingBoxPoints[i][secondPlaneBoundingBoxStart] = point3;
		continuousBoundingBoxPoints[i][secondPlaneBoundingBoxStart+4] = point3;
		continuousBoundingBoxPoints[i][secondPlaneBoundingBoxStart+3] = point4;


		continuousBoundingBoxPoints[i][firstPlaneBoundingBoxStart+1] = point1;
		continuousBoundingBoxPoints[i][firstPlaneBoundingBoxStart+2] = point2;
		cout << "[ DEBUG ] continuousBoundingBoxPoints made for plane " << i << "\n";

	}

	cout << "[ DEBUG ] continuousBoundingBoxPoints Completed\n";
	return ;

}
