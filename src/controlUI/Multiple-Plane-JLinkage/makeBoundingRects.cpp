/**
 * @file makeBoundingRects.cpp
 * @ingroup makeBoundingRects
 */

/*
 *   File Name: makeBoundingRects.cpp
 *     Project: Multiple Plane JLinkage
 *  Created on: 29-Mar-2016
 *      Author: Sona Praneeth Akula
 *     Details:
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

	//cout << "[ DEBUG ] orderPlanePointsByCentroids Started.\n";
	// Get the number of planes
	int numberOfPlanes = planeParameters.size();
	int i, j;
	int startIndex, endIndex;
	// Vector for x co-ordinate of centroids for each plane
	vector<float> xCentroidPoints;
	// Vector for sorting x co-ordinate of centroids for each plane
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

	// Finding the x centroid of all the planes
	for(i = 0; i < numberOfPlanes; i++) {

		// Finding the x centroid for a particular plane
		//cout << "[ DEBUG ] Finding XCentroid for Plane " << i << ".\n";
		xCentroid = 0.0;
		startIndex = planeIndexBounds.at(i).first;
		endIndex = planeIndexBounds.at(i).second;
		// TODO: Please check if this is j < endIndex (or) j <= endIndex
		for (j = startIndex; j < endIndex; ++j) {
			xCentroid += projectionOf3DPoints[j].x;
		}

		xCentroid /= (endIndex-startIndex+1);
		xCentroidPoints.push_back(xCentroid);
	}

	//cout << "[ DEBUG ] Sorting X Centroid Data Started.\n";

	// Sort the x centroids
	sortData( xCentroidPoints, sortedXCentroidPoints, indices, true);

	//cout << "[ DEBUG ] Sorting X Centroid Data Completed.\n";

	// Push the points based on sorted order of x centroids
	// Also make the plane parameters accordingly
	int sortedStartIndex, sortedEndIndex;
	for(i = 0; i < numberOfPlanes; i++) {

		//cout << "[ DEBUG ] Sorted Projections for Plane " << i << ".\n";
		startIndex = planeIndexBounds.at(indices[i]).first;
		endIndex = planeIndexBounds.at(indices[i]).second;

		//cout << "[ DEBUG ] Pushing the Sorted Points for Plane " << i << ".\n";
		// TODO: Please check if this is j < endIndex (or) j <= endIndex
		sortedStartIndex = sortedProjectionsOf3DPoints.size();
		for (j = startIndex; j < endIndex; ++j) {
			sortedProjectionsOf3DPoints.push_back(projectionOf3DPoints[j]);
		}

		//cout << "[ DEBUG ] Pushing the Sorted Plane Parameters for Plane " << i << ".\n";
		for( j = 0; j < 4; j++) {
			sortedPlaneParameters[i][j] = planeParameters[indices[i]][j];
		}

		// Make the index bounds stating from which index to which index does points
		// related to plane i belong to
		sortedEndIndex = sortedStartIndex+endIndex-startIndex;
		sortedPlaneIndexBounds.insert(make_pair(i,make_pair(sortedStartIndex, sortedEndIndex)));

	}

	//cout << "[ DEBUG ] Original Plane Index Bounds\n";
	for(map<int, pair<int,int> >::const_iterator it = planeIndexBounds.begin();
		    it != planeIndexBounds.end(); ++it) {
		std::cout << it->first << " " << it->second.first << " " << it->second.second << "\n";
	}
	//cout << "[ DEBUG ] Sorted Plane Index Bounds\n";
	for(map<int, pair<int,int> >::const_iterator it = sortedPlaneIndexBounds.begin();
	    it != sortedPlaneIndexBounds.end(); ++it) {
	    std::cout << it->first << " " << it->second.first << " " << it->second.second << "\n";
	}
	//cout << "[ DEBUG ] Original Plane Parameters\n";
	printVectorOfVectors(planeParameters);
	//cout << "[ DEBUG ] Sorted Plane Parameters\n";
	printVectorOfVectors(sortedPlaneParameters);
	//cout << "[ DEBUG ] orderPlanePointsByCentroids Completed.\n";
	return ;

}

void getPercentageOfEachPlane (
		const vector<Point3f> &sortedProjectionOf3DPoints,
		const map<int, pair<int, int> > &sortedPlaneIndexBounds,
		vector< vector<Point3f> > &sorted_3d_points,
		vector<float> &percentageOfEachPlane)
{
	percentageOfEachPlane.clear();
	int size = sorted_3d_points.size();
	for (int i = 0; i < size; ++i)
	{
		sorted_3d_points[i].clear();
	}
	sorted_3d_points.clear();
	int numberOfPlanes = sortedPlaneIndexBounds.size();
	int i, j;
	int indexOne, indexTwo;
	vector<Point3f> pointsInThePlane;
	int totalNumberOfPoints = 0;
	for (i = 0; i < numberOfPlanes; ++i)
	{
		indexOne = sortedPlaneIndexBounds.at(i).first;
		indexTwo = sortedPlaneIndexBounds.at(i).second;
		pointsInThePlane.clear();
		for (j = indexOne; j < indexTwo; ++j)
		{
			pointsInThePlane.push_back(sortedProjectionOf3DPoints[j]);
		}
		sorted_3d_points.push_back(pointsInThePlane);
		percentageOfEachPlane.push_back((float)pointsInThePlane.size());
		totalNumberOfPoints += pointsInThePlane.size();
	}
	float max_percentage = 0.0;
	for (i = 0; i < numberOfPlanes; ++i)
	{
		float percentage = (float)percentageOfEachPlane[i]/totalNumberOfPoints;
		percentageOfEachPlane[i] = percentage;
		if(percentage >= max_percentage)
		{
			max_percentage = percentage;
		}
	}
	return ;
}

void getBoundingBoxCoordinates (
		const vector<Point3f> &sortedProjectionOf3DPoints,
		const vector< vector<float> > &sortedPlaneParameters,
		const map<int, pair<int, int> > &sortedPlaneIndexBounds,
		vector< vector<Point3f> > &boundingBoxPoints ) {

	//cout << "[ DEBUG ] getBoundingBoxCoordinates Started\n";
	// Get the number of planes
	int numberOfPlanes = sortedPlaneIndexBounds.size();
	int i, j;
	int indexOne, indexTwo;
	vector<Point3f> pointsInThePlane;

	for (i = 0; i < numberOfPlanes; ++i) {

		//cout << "[ DEBUG ] Getting the bounding box for Plane " << i << "\n";
		indexOne = sortedPlaneIndexBounds.at(i).first;
		indexTwo = sortedPlaneIndexBounds.at(i).second;

		//cout << "[ DEBUG ] Points are starting from " << indexOne << " to " << indexTwo-1 << "\n";
		//cout << "[ DEBUG ] There are " << numberOfPointsInThePlane << " in the plane\n";
		pointsInThePlane.clear();

		for (j = indexOne; j < indexTwo; ++j) {
			pointsInThePlane.push_back(sortedProjectionOf3DPoints[j]);
		}

		vector<Point3f> sortedPlaneXYZBoundingPoints;
		getBoundingBoxPointsOfPlane(pointsInThePlane, sortedPlaneParameters[i], sortedPlaneXYZBoundingPoints);
		//cout << "[ DEBUG ] Converting XYZ to UV\n";		

		// Make the plane bounding box points for plane i
		//cout << "[ DEBUG ] Created Bounding Box Points for Plane " << i << "\n";
		boundingBoxPoints.push_back(sortedPlaneXYZBoundingPoints);


	}
	//cout << "[ DEBUG ] getBoundingBoxCoordinates Completed.\n";

	return ;

}

void getBoundingBoxPointsOfPlane(const vector<Point3f> &pointsInThePlane, const vector<float> &sortedPlaneParameters, vector<Point3f> &sortedPlaneXYZBoundingPoints){
	vector<Point2f> uvCoord;
	vector<float> uCoord, vCoord;
	vector<Point3f> uvAxes;
	vector<Point2f> planeUVBoundingPoints;
	vector<Point3f> planeXYZBoundingPoints;

	int numberOfPointsInThePlane = pointsInThePlane.size();
	int j;
	AllXYZToUVCoordinates( pointsInThePlane, sortedPlaneParameters,
								uvCoord, uvAxes);

	// Make the X Co-ordinates of plane points
	for ( j = 0; j < numberOfPointsInThePlane; ++j) {
		uCoord.push_back(uvCoord[j].x);
	}
		// Make the Y Co-ordinates of plane points
	for (j = 0; j < numberOfPointsInThePlane; ++j) {
		vCoord.push_back(uvCoord[j].y);
	}

	// Get the minimum and maximum x and y co-ordinates

	float minU = *min_element(uCoord.begin(), uCoord.end());
	float maxU = *max_element(uCoord.begin(), uCoord.end());
	float minV = *min_element(vCoord.begin(), vCoord.end());
	float maxV = *max_element(vCoord.begin(), vCoord.end());
	/*
		cout << "MinU: " << minU << "\n";
		cout << "MaxU: " << maxU << "\n";
		cout << "MinV: " << minV << "\n";
		cout << "MaxV: " << maxV << "\n";
	*/
		// Make the bottom left, bottom right, top left, top right corners of bounding box
	Point2f bottomLeft = Point2f(minU, minV);
	Point2f bottomRight = Point2f(maxU, minV);
	Point2f topLeft = Point2f(minU, maxV);
	Point2f topRight = Point2f(maxU, maxV);
		//cout << "[ DEBUG ] " << bottomLeft << ":" << bottomRight << ":" << topRight << ":" << topLeft << "\n";

	planeUVBoundingPoints.clear();
	planeXYZBoundingPoints.clear();
	//cout << "[ DEBUG ] Making planeUVBoundingPoints\n";
	planeUVBoundingPoints.push_back(bottomLeft);
	planeUVBoundingPoints.push_back(bottomRight);
	planeUVBoundingPoints.push_back(topRight);
	planeUVBoundingPoints.push_back(topLeft);

	//cout << "[ DEBUG ] Converting UV to XYZ\n";
	AllUVToXYZCoordinates( planeUVBoundingPoints, uvAxes, sortedPlaneParameters[3],
				planeXYZBoundingPoints);

	/*
		cout << "Plane XYZ Bounding Points: ";
		cout << planeXYZBoundingPoints;
		cout << "\n";
	*/

	sortXYZCorners(planeXYZBoundingPoints, sortedPlaneXYZBoundingPoints);
	sortedPlaneXYZBoundingPoints.push_back(sortedPlaneXYZBoundingPoints[0]);

}

void sortUVCorners(const vector<Point2f> &planeUVBoundingPoints, vector<Point2f> &sortedPlaneUVBoundingPoints){
    vector<float> vCoord;
    vector<float> uCoord;
    vector<float> sortedVCoord;
    vector<int> sortedVCoordOriginalIndices;

    vCoord.clear();
    uCoord.clear();
    sortedVCoord.clear();
    sortedVCoordOriginalIndices.clear();

    // Now we have arrange the points in proper order based on X and Z co-ordinates
    int j;
    for (j = 0; j < 4; ++j) {
        vCoord.push_back(planeUVBoundingPoints[j].y);
    }

    // Sorting the Z co-ordinates
    //cout << "[ DEBUG ] Sorting by Z Coordinates\n";
    sortData(vCoord, sortedVCoord, sortedVCoordOriginalIndices, false);

    //cout << "[ DEBUG ] Swapping by X Coordinates\n";
	for(j =0; j<4; j++)
    {
        uCoord.push_back(planeUVBoundingPoints[sortedVCoordOriginalIndices[j]].x);
        sortedPlaneUVBoundingPoints.push_back(planeUVBoundingPoints[sortedVCoordOriginalIndices[j]]);
    }

    if(uCoord[0] >= uCoord[1]) {
        Point2f dummy = sortedPlaneUVBoundingPoints[0];
        sortedPlaneUVBoundingPoints[0] = sortedPlaneUVBoundingPoints[1];
        sortedPlaneUVBoundingPoints[1] = dummy;
    }
    if(uCoord[2] < uCoord[3]) {
        Point2f dummy = sortedPlaneUVBoundingPoints[2];
        sortedPlaneUVBoundingPoints[2] = sortedPlaneUVBoundingPoints[3];
        sortedPlaneUVBoundingPoints[3] = dummy;
    }
}

void sortXYZCorners(const vector<Point3f> &planeXYZBoundingPoints, vector<Point3f> &sortedPlaneXYZBoundingPoints){
	vector<float> zCoord;
	vector<float> xCoord;
	vector<float> sortedZCoord;
	vector<int> sortedZCoordOriginalIndices;

	zCoord.clear();
	xCoord.clear();
	sortedZCoord.clear();
	sortedZCoordOriginalIndices.clear();

	// Now we have arrange the points in proper order based on X and Z co-ordinates
	int j;
	for (j = 0; j < 4; ++j) {
		zCoord.push_back(planeXYZBoundingPoints[j].z);
	}

	// Sorting the Z co-ordinates
	//cout << "[ DEBUG ] Sorting by Z Coordinates\n";
	sortData(zCoord, sortedZCoord, sortedZCoordOriginalIndices, false);

	//cout << "[ DEBUG ] Swapping by X Coordinates\n";

	for(j =0; j<4; j++)
	{
		xCoord.push_back(planeXYZBoundingPoints[sortedZCoordOriginalIndices[j]].x);
		sortedPlaneXYZBoundingPoints.push_back(planeXYZBoundingPoints[sortedZCoordOriginalIndices[j]]);
	}

	if(xCoord[0] >= xCoord[1]) {
		Point3f dummy = sortedPlaneXYZBoundingPoints[0];
		sortedPlaneXYZBoundingPoints[0] = sortedPlaneXYZBoundingPoints[1];
		sortedPlaneXYZBoundingPoints[1] = dummy;
	}
	if(xCoord[2] < xCoord[3]) {
		Point3f dummy = sortedPlaneXYZBoundingPoints[2];
		sortedPlaneXYZBoundingPoints[2] = sortedPlaneXYZBoundingPoints[3];
		sortedPlaneXYZBoundingPoints[3] = dummy;
	}
}


void getContinuousBoundingBox (
		const vector< vector<Point3f> > &boundingBoxPoints,
		const vector< vector<float> > &sortedPlaneParameters,
		vector< vector<Point3f> > &continuousBoundingBoxPoints) {

	//cout << "[ DEBUG ] getContinuousBoundingBox Started\n";
	// Get the number of planes
	int numberOfPlanes = sortedPlaneParameters.size();
	int i, j;

	Point3f firstPoint, secondPoint;
	Point3f point1, point2, point3, point4;
	vector<Point3f> continuousPoints;

	int boundingBoxPointsSize = boundingBoxPoints.size();
	//cout << "[ DEBUG ] Initializing continuousBoundingBoxPoints\n";
	for (i = 0; i < boundingBoxPointsSize; ++i) {
		int size = boundingBoxPoints[i].size();
		continuousBoundingBoxPoints.push_back(continuousPoints);
		for(j = 0; j < size; ++j) {
			continuousBoundingBoxPoints[i].push_back(boundingBoxPoints[i][j]);
		}
	}
	//cout << "[ DEBUG ] continuousBoundingBoxPoints Initialized\n";
	for (i = 0; i < numberOfPlanes-1; ++i) {
		vector< vector<float> > lineParameters1;
		vector< vector<float> > lineParameters2;
		vector< vector<float> > lineParameters3;
		vector< vector<float> > lineParameters4;
		vector< vector<float> > lineIntersectionOfPlanes;

		//cout << "[ DEBUG ] Line Parameters Cleared\n";

		//cout << "[ DEBUG ] Calculating intersections for plane " << i << " and plane " << i+1 << "\n";
		calculateIntersectionOfPlanes( sortedPlaneParameters[i],
				sortedPlaneParameters[i+1], lineIntersectionOfPlanes);

		firstPoint = boundingBoxPoints[i][0];
		secondPoint = boundingBoxPoints[i][1];
		//cout << "[ DEBUG ] Calculate Line 1 for plane " << i << "\n";
		makeLineFromPoints( firstPoint, secondPoint, lineParameters1);
		firstPoint = boundingBoxPoints[i][2];
		secondPoint = boundingBoxPoints[i][3];
		//cout << "[ DEBUG ] Calculate Line 2 for plane " << i << "\n";
		makeLineFromPoints( firstPoint, secondPoint, lineParameters2);

		firstPoint = boundingBoxPoints[i+1][0];
		secondPoint = boundingBoxPoints[i+1][1];
		//cout << "[ DEBUG ] Calculate Line 3 for plane " << i << "\n";
		makeLineFromPoints( firstPoint, secondPoint, lineParameters3);
		firstPoint = boundingBoxPoints[i+1][2];
		secondPoint = boundingBoxPoints[i+1][3];
		//cout << "[ DEBUG ] Calculate Line 4 for plane " << i << "\n";
		makeLineFromPoints( firstPoint, secondPoint, lineParameters4);

		//cout << "[ DEBUG ] Calculate Point 1 for plane " << i << "\n";
		calculateIntersectionOfLines( lineParameters1, lineIntersectionOfPlanes, point1);
		//cout << "[ DEBUG ] Calculate Point 2 for plane " << i << "\n";
		calculateIntersectionOfLines( lineParameters2, lineIntersectionOfPlanes, point2);
		//cout << "[ DEBUG ] Calculate Point 3 for plane " << i << "\n";
		calculateIntersectionOfLines( lineParameters3, lineIntersectionOfPlanes, point3);
		//cout << "[ DEBUG ] Calculate Point 4 for plane " << i << "\n";
		calculateIntersectionOfLines( lineParameters4, lineIntersectionOfPlanes, point4);

		continuousBoundingBoxPoints[i][1] = point1;
		continuousBoundingBoxPoints[i][2] = point2;

		continuousBoundingBoxPoints[i+1][0] = point3;
		continuousBoundingBoxPoints[i+1][4] = point3;
		continuousBoundingBoxPoints[i+1][3] = point4;

		//cout << "[ DEBUG ] continuousBoundingBoxPoints made for plane " << i << "\n";

		//cout << "[ DEBUG ] Line Parameters1 size: " << lineParameters1.size() << "\n";
		//cout << "[ DEBUG ] Line Parameters2 size: " << lineParameters2.size() << "\n";
		//cout << "[ DEBUG ] Line Parameters3 size: " << lineParameters3.size() << "\n";
		//cout << "[ DEBUG ] Line Parameters4 size: " << lineParameters4.size() << "\n";
		//cout << "[ DEBUG ] Line ParametersP size: " << lineIntersectionOfPlanes.size() << "\n";
	}

	//cout << "[ DEBUG ] continuousBoundingBoxPoints Completed\n";
	return ;

}


/*********** EXTRA FUNCTIONS ***************************************************/

void orderPlanePointsByCentroids1(
		const vector< vector<Point3f> > &projectionOf3DPoints,
		const vector< vector<float> > &planeParameters,
		vector< vector<Point3f> > &sortedProjectionsOf3DPoints,
		vector< vector<float> > &sortedPlaneParameters ) {


	//cout << "[ DEBUG ] orderPlanePointsByCentroids1 Started.\n";
	// Get the number of planes
	int numberOfPlanes = planeParameters.size();
	//cout << "[ DEBUG ] There are " << numberOfPlanes << " planes\n";
	int i, j, size;
	// Vector for x co-ordinate of centroids for each plane
	vector<float> xCentroidPoints;
	// Vector for sorting x co-ordinate of centroids for each plane
	vector<float> sortedXCentroidPoints;
	vector<int> indices;
	float xCentroid = 0.0;
	vector<float> abcd;
	vector<Point3f> sortedPoints;
	for( j = 0; j < 4; j++) {
		abcd.push_back(0.0);
	}
	for(i = 0; i < numberOfPlanes; i++) {
		sortedPlaneParameters.push_back(abcd);
		sortedProjectionsOf3DPoints.push_back(sortedPoints);
	}

	// Finding the x centroid of all the planes
	for(i = 0; i < numberOfPlanes; i++) {

		// Finding the x centroid for a particular plane
		//cout << "[ DEBUG ] Finding XCentroid for Plane " << i << ".\n";
		xCentroid = 0.0;
		size = projectionOf3DPoints[i].size();
		for (j = 0; j < size; ++j) {
			xCentroid += projectionOf3DPoints[i][j].x;
		}

		xCentroid /= (size);
		xCentroidPoints.push_back(xCentroid);

	}

	//cout << "[ DEBUG ] Sorting X Centroid Data Started.\n";

	// Sort the x centroids
	sortData( xCentroidPoints, sortedXCentroidPoints, indices, true);

	//cout << "[ DEBUG ] Sorting X Centroid Data Completed.\n";

	// Push the points based on sorted order of x centroids
	// Also make the plane parameters accordingly
	for(i = 0; i < numberOfPlanes; i++) {

		//cout << "[ DEBUG ] Sorted Projections for Plane " << i << ".\n";

		//cout << "[ DEBUG ] Pushing the Sorted Points for Plane " << indices[i] << ".\n";
		size = projectionOf3DPoints[indices[i]].size();
		//cout << "[ DEBUG ] Number of points in the plane: " << size << "\n";
		for (j = 0; j < size; ++j) {
			sortedProjectionsOf3DPoints[i].push_back(projectionOf3DPoints[indices[i]][j]);
		}

		//cout << "[ DEBUG ] Pushing the Sorted Plane Parameters for Plane " << i << ".\n";
		for( j = 0; j < 4; j++) {
			sortedPlaneParameters[i][j] = planeParameters[indices[i]][j];
		}

	}

	//cout << "[ DEBUG ] orderPlanePointsByCentroids1 Completed.\n";
	return ;


}

void getBoundingBoxCoordinates1 (
		const vector< vector<Point3f> > &sortedProjectionOf3DPoints,
		const vector< vector<float> > &sortedPlaneParameters,
		vector< vector<Point3f> > &boundingBoxPoints ) {


	//cout << "[ DEBUG ] getBoundingBoxCoordinates1 Started\n";
	// Get the number of planes
	int numberOfPlanes = sortedProjectionOf3DPoints.size();
	int i, j;
	int numberOfPointsInThePlane;
	vector<Point3f> pointsInThePlane;
	vector<Point2f> uvCoord;
	vector<float> uCoord, vCoord;
	vector<Point3f> uvAxes;
	vector<Point2f> planeUVBoundingPoints;
	vector<Point3f> planeXYZBoundingPoints;

	for (i = 0; i < numberOfPlanes; ++i) {

		//cout << "[ DEBUG ] Getting the bounding box for Plane " << i << "\n";

		// Get the number of points in the particular plane
		numberOfPointsInThePlane = sortedProjectionOf3DPoints[i].size();
		//cout << "[ DEBUG ] There are " << numberOfPointsInThePlane << " in the plane\n";
		pointsInThePlane.clear();
		uvAxes.clear();
		uvCoord.clear();

		for (j = 0; j < numberOfPointsInThePlane; ++j) {
			pointsInThePlane.push_back(sortedProjectionOf3DPoints[i][j]);
		}

		//cout << "[ DEBUG ] Converting XYZ to UV\n";
		AllXYZToUVCoordinates( pointsInThePlane, sortedPlaneParameters[i],
								uvCoord, uvAxes);

		uCoord.clear();
		// Make the X Co-ordinates of plane points
		for (j = 0; j < numberOfPointsInThePlane; ++j) {
			uCoord.push_back(uvCoord[j].x);
		}
		vCoord.clear();
		// Make the Y Co-ordinates of plane points
		for (j = 0; j < numberOfPointsInThePlane; ++j) {
			vCoord.push_back(uvCoord[j].y);
		}

		// Get the minimum and maximum x and y co-ordinates
		float minU = *min_element(uCoord.begin(), uCoord.end());
		float maxU = *max_element(uCoord.begin(), uCoord.end());
		float minV = *min_element(vCoord.begin(), vCoord.end());
		float maxV = *max_element(vCoord.begin(), vCoord.end());

		// Make the bottom left, bootom right, top left, top right corners of bounding box
		Point2f bottomLeft = Point2f(minU, minV);
		Point2f bottomRight = Point2f(maxU, minV);
		Point2f topLeft = Point2f(minU, maxV);
		Point2f topRight = Point2f(maxU, maxV);
		//cout << "[ DEBUG ] " << bottomLeft << ":" << bottomRight << ":" << topRight << ":" << topLeft << "\n";

		planeUVBoundingPoints.clear();
		planeXYZBoundingPoints.clear();
		//cout << "[ DEBUG ] Making planeUVBoundingPoints\n";
		planeUVBoundingPoints.push_back(bottomLeft);
		planeUVBoundingPoints.push_back(bottomRight);
		planeUVBoundingPoints.push_back(topLeft);
		planeUVBoundingPoints.push_back(topRight);

		//cout << "[ DEBUG ] Converting UV to XYZ\n";
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

		// Now we have arrange the points in proper order based on X and Z co-ordinates
		for (j = 0; j < numberOfPointsInThePlane; ++j) {
			zCoord.push_back(planeXYZBoundingPoints[j].z);
			xCoord.push_back(planeXYZBoundingPoints[j].x);
		}

		// Sorting the Z co-ordinates
		//cout << "[ DEBUG ] Sorting by Z Coordinates\n";
		sortData(zCoord, sortedZCoord, sortedZCoordOriginalIndices, false);

		//cout << "[ DEBUG ] Swapping by X Coordinates\n";
		if(xCoord[sortedZCoordOriginalIndices[0]] >= xCoord[sortedZCoordOriginalIndices[1]]) {
			swap(planeXYZBoundingPoints[sortedZCoordOriginalIndices[0]],
					planeXYZBoundingPoints[sortedZCoordOriginalIndices[1]]);
		}
		if(xCoord[sortedZCoordOriginalIndices[2]] < xCoord[sortedZCoordOriginalIndices[3]]) {
			swap(planeXYZBoundingPoints[sortedZCoordOriginalIndices[2]],
					planeXYZBoundingPoints[sortedZCoordOriginalIndices[3]]);
		}

		planeXYZBoundingPoints.push_back(planeXYZBoundingPoints[0]);

		// Make the plane bounding box points for plane i
		//cout << "[ DEBUG ] Created Bounding Box Points for Plane " << i << "\n";
		boundingBoxPoints.push_back(planeXYZBoundingPoints);


	}
	//cout << "[ DEBUG ] getBoundingBoxCoordinates1 Completed.\n";

	return ;

}

