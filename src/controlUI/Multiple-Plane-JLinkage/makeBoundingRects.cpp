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
		const vector<Point3d> &projectionOf3DPoints,
		const vector< vector<double> > &planeParameters,
		const map<int, pair<int, int> > &planeIndexBounds,
		vector<Point3d> &sortedProjectionsOf3DPoints,
		vector< vector<double> > &sortedPlaneParameters,
		map<int, pair<int, int> > &sortedPlaneIndexBounds ) {


	int numberOfPlanes = planeParameters.size();
	int i, j;
	int startIndex, endIndex;
	vector<double> xPoints;
	vector<double> sortedXPoints;
	vector<int> indices;
	double xCentroid = 0.0;

	for(i = 0; i < numberOfPlanes; i++) {
		startIndex = planeIndexBounds.at(i).first;
		endIndex = planeIndexBounds.at(i).second;
		for (j = startIndex; j <= endIndex; ++j) {
			xPoints.push_back(projectionOf3DPoints[i].x);
			xCentroid += projectionOf3DPoints[i].x;
		}
		xCentroid /= (endIndex-startIndex+1);
		sortData( xPoints, sortedXPoints, indices);
		startIndex = planeIndexBounds.at(indices[i]).first;
		endIndex = planeIndexBounds.at(indices[i]).second;
		for (j = startIndex; j <= endIndex; ++j) {
			sortedProjectionsOf3DPoints.push_back(projectionOf3DPoints[j]);
		}
		sortedPlaneParameters[i] = planeParameters[indices[i]];
		sortedPlaneIndexBounds[i] = make_pair(startIndex, endIndex);
	}

	return ;

}

void orderPlanePointsByCentroids1(
		const vector< vector<Point3d> > &projectionOf3DPoints,
		const vector< vector<double> > &planeParameters,
		vector< vector<Point3d> > &sortedProjectionsOf3DPoints,
		vector< vector<double> > &sortedPlaneParameters ) {


	int numberOfPlanes = planeParameters.size();
	int i, j;
	int numberOfPointsInThisPlane;
	vector<double> xPoints;
	vector<double> sortedXPoints;
	vector<int> indices;
	double xCentroid = 0.0;
	vector<Point3d> points;
	//sortedProjectionsOf3DPoints(vector< vector<Point3d> >(numberOfPlanes));
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
		const vector<Point3d> &sortedProjectionOf3DPoints,
		const vector< vector<double> > &sortedPlaneParameters,
		const map<int, pair<int, int> > &sortedPlaneIndexBounds,
		vector< vector<Point3d> > &boundingBoxPoints ) {


	int numberOfPlanes = sortedPlaneIndexBounds.size();
	int i, j;
	int numberOfPointsInThePlane, indexOne, indexTwo;
	vector<Point3d> pointsInThePlane;
	vector<Point2d> uvCoord;
	vector<double> uCoord, vCoord;
	vector<Point3d> uvAxes;
	vector<Point2d> planeUVBoundingPoints;
	vector<Point3d> planeXYZBoundingPoints;

	for (i = 0; i < numberOfPlanes; ++i) {
		indexOne = sortedPlaneIndexBounds.at(i).first;
		indexTwo = sortedPlaneIndexBounds.at(i).second;
		numberOfPointsInThePlane = indexTwo - indexOne;
		for (j = indexOne; j < indexTwo; ++j) {
			pointsInThePlane.push_back(sortedProjectionOf3DPoints[j]);
		}

		AllXYZToUVCoordinates( pointsInThePlane, sortedPlaneParameters[i],
								uvCoord, uvAxes);

		for (j = 0; j < numberOfPointsInThePlane; ++j) {
			uCoord.push_back(uvCoord[j].x);
		}
		for (j = 0; j < numberOfPointsInThePlane; ++j) {
			vCoord.push_back(uvCoord[j].y);
		}

		double minU = *min_element(uCoord.begin(), uCoord.end());
		double maxU = *max_element(uCoord.begin(), uCoord.end());
		double minV = *min_element(vCoord.begin(), vCoord.end());
		double maxV = *max_element(vCoord.begin(), vCoord.end());
		Point2d bottomLeft = Point2d(minU, minV);
		Point2d bottomRight = Point2d(maxU, minV);
		Point2d topLeft = Point2d(minU, maxV);
		Point2d topRight = Point2d(maxU, maxV);

		planeUVBoundingPoints.push_back(bottomLeft);
		planeUVBoundingPoints.push_back(bottomRight);
		planeUVBoundingPoints.push_back(topLeft);
		planeUVBoundingPoints.push_back(topRight);

		AllUVToXYZCoordinates( planeUVBoundingPoints, uvAxes, sortedPlaneParameters[i][3],
				planeXYZBoundingPoints);

		vector<double> zCoord;
		vector<double> xCoord;
		vector<double> sortedZCoord;
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

void getBoundingBoxCoordinates1 (
		const vector< vector<Point3d> > &sortedProjectionOf3DPoints,
		const vector< vector<double> > &sortedPlaneParameters,
		vector< vector<Point3d> > &boundingBoxPoints ) {


	int numberOfPlanes = sortedProjectionOf3DPoints.size();
	int i, j;
	int numberOfPointsInThePlane, indexOne, indexTwo;
	vector<Point3d> pointsInThePlane;
	vector<Point2d> uvCoord;
	vector<double> uCoord, vCoord;
	vector<Point3d> uvAxes;
	vector<Point2d> planeUVBoundingPoints;
	vector<Point3d> planeXYZBoundingPoints;

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

		double minU = *min_element(uCoord.begin(), uCoord.end());
		double maxU = *max_element(uCoord.begin(), uCoord.end());
		double minV = *min_element(vCoord.begin(), vCoord.end());
		double maxV = *max_element(vCoord.begin(), vCoord.end());
		Point2d bottomLeft = Point2d(minU, minV);
		Point2d bottomRight = Point2d(maxU, minV);
		Point2d topLeft = Point2d(minU, maxV);
		Point2d topRight = Point2d(maxU, maxV);

		planeUVBoundingPoints.push_back(bottomLeft);
		planeUVBoundingPoints.push_back(bottomRight);
		planeUVBoundingPoints.push_back(topLeft);
		planeUVBoundingPoints.push_back(topRight);

		AllUVToXYZCoordinates( planeUVBoundingPoints, uvAxes, sortedPlaneParameters[i][3],
				planeXYZBoundingPoints);

		vector<double> zCoord;
		vector<double> xCoord;
		vector<double> sortedZCoord;
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
		const vector< vector<Point3d> > &boundingBoxPoints,
		const vector< vector<double> > &sortedPlaneParameters,
		vector< vector<Point3d> > &continuousBoundingBoxPoints) {

	int numberOfPlanes = sortedPlaneParameters.size();
	int i, j;
	vector< vector<double> > lineParameters1;
	vector< vector<double> > lineParameters2;
	vector< vector<double> > lineParameters3;
	vector< vector<double> > lineParameters4;
	vector< vector<double> > lineIntersectionOfPlanes;
	int firstPlaneBoundingBoxStart, secondPlaneBoundingBoxStart;
	Point3d firstPoint, secondPoint;
	Point3d point1, point2, point3, point4;

	for (i = 0; i < numberOfPlanes-1; ++i) {
		calculateIntersectionOfPlanes( sortedPlaneParameters[i],
				sortedPlaneParameters[i+1], lineIntersectionOfPlanes);

		firstPlaneBoundingBoxStart = 5*i;
		secondPlaneBoundingBoxStart = 5*(i+1);

		firstPoint = boundingBoxPoints[i][firstPlaneBoundingBoxStart];
		secondPoint = boundingBoxPoints[i][firstPlaneBoundingBoxStart+1];
		makeLineFromPoints( firstPoint, secondPoint, lineParameters1);
		firstPoint = boundingBoxPoints[i][firstPlaneBoundingBoxStart+2];
		secondPoint = boundingBoxPoints[i][firstPlaneBoundingBoxStart+3];
		makeLineFromPoints( firstPoint, secondPoint, lineParameters2);

		firstPoint = boundingBoxPoints[i][secondPlaneBoundingBoxStart];
		secondPoint = boundingBoxPoints[i][secondPlaneBoundingBoxStart+1];
		makeLineFromPoints( firstPoint, secondPoint, lineParameters3);
		firstPoint = boundingBoxPoints[i][secondPlaneBoundingBoxStart+2];
		secondPoint = boundingBoxPoints[i][secondPlaneBoundingBoxStart+3];
		makeLineFromPoints( firstPoint, secondPoint, lineParameters4);

		point1 = calculateIntersectionOfLines( lineParameters1, lineIntersectionOfPlanes);
		point2 = calculateIntersectionOfLines( lineParameters2, lineIntersectionOfPlanes);
		point3 = calculateIntersectionOfLines( lineParameters3, lineIntersectionOfPlanes);
		point4 = calculateIntersectionOfLines( lineParameters4, lineIntersectionOfPlanes);

		continuousBoundingBoxPoints[i][secondPlaneBoundingBoxStart] = point3;
		continuousBoundingBoxPoints[i][secondPlaneBoundingBoxStart+4] = point3;
		continuousBoundingBoxPoints[i][secondPlaneBoundingBoxStart+3] = point4;


		continuousBoundingBoxPoints[i][firstPlaneBoundingBoxStart+1] = point1;
		continuousBoundingBoxPoints[i][firstPlaneBoundingBoxStart+2] = point2;


	}

	return ;

}
