/**
 * @file multiplePlanes.cpp
 * @ingroup multiplePlanes
 */

/*
 *   File Name: multiplePlanes.cpp
 *     Project: Multiple Plane JLinkage
 *  Created on: 29-Mar-2016
 *      Author: Sona Praneeth Akula
 *     Details:
 */

#include "allHeaders.hpp"
#include "multiplePlanes.hpp"
#include "JLinkage/PrimitiveFunctions.h"


void callJLinkage(const vector<Point3f> &locations, vector<int> &labels) {
	vector<vector<float> *> * mDataPoints;
	mDataPoints = new vector<vector<float> *>(locations.size());
	for(int i=0; i<locations.size(); i++)
	{
	   (*mDataPoints)[i] = new vector<float>(3);
	   (*(*mDataPoints)[i])[0] = locations[i].x;
	   (*(*mDataPoints)[i])[1] = locations[i].y;
	   (*(*mDataPoints)[i])[2] = locations[i].z;
	}
	int numModels = 10000;
	float mKdTreeCloseProb = 0.8, mKdTreeFarProb = 0.2;
	int mKdTreeRange =10;

	RandomSampler mRandomSampler(GetFunction_Plane, DistanceFunction_Plane, (int)(*(*mDataPoints)[0]).size()-1, 3, (int)mDataPoints->size(),true);
	mRandomSampler.SetPoints(mDataPoints);
	mRandomSampler.SetNFSamplingTypeNN(mKdTreeRange, (float)mKdTreeCloseProb, (float)mKdTreeFarProb, true);
	vector<vector<float> *> *mModels = mRandomSampler.GetNSample(numModels, 0, NULL, NULL);

	float mInlierThreshold = 0.1;
	JLinkage mJLinkage(DistanceFunction_Plane, mInlierThreshold, mModels->size(), true, ((*mDataPoints)[0])->size(), mKdTreeRange);
	vector<const sPtLnk *> mPointMap(mDataPoints->size());
	list<sClLnk *> mClustersList;

	for(unsigned int nModelCount = 0; nModelCount < mModels->size(); nModelCount++){
		mJLinkage.AddModel(((*mModels)[nModelCount]));
	}

	int counter=0;
	for(std::vector<std::vector<float> *>::iterator iterPts = mDataPoints->begin(); iterPts != mDataPoints->end(); ++iterPts ){
		mPointMap[counter] = mJLinkage.AddPoint(*iterPts);
		counter++;
	}

	mClustersList = mJLinkage.DoJLClusterization(NULL);

	labels.resize(locations.size());
	int counterCl =0;
	for(std::list<sClLnk *>::iterator iterCl = mClustersList.begin(); iterCl != mClustersList.end(); ++iterCl){
		for(std::list<sPtLnk *>::iterator iterPt = (*iterCl)->mBelongingPts.begin(); iterPt != (*iterCl)->mBelongingPts.end(); ++iterPt){
			unsigned int counterPt = 0;
			for(std::vector<const sPtLnk *>::iterator iterPtIdx = mPointMap.begin(); iterPtIdx != mPointMap.end(); ++iterPtIdx){
				if((*iterPt) == (*iterPtIdx)){
					labels[counterPt] = counterCl;
					break;
				}
				++counterPt;
			}
		}
		++counterCl;
	}
}


int findMultiplePlanes(
		const vector<Point3f> &points,
		vector< vector<float> > &sortedPlaneParameters,
		vector< vector<Point3f> > &continuousBoundingBoxPoints ){

	// Step 1: Performing JLinkage to find multiple models
	// This vector describes to which plane does point i belong to
	// This is an output from JLinkage
	vector<int> oldPlaneIndices;
	cout << "[ DEBUG ] Step 1: Calling callJLinkage!!!\n";
	callJLinkage(points, oldPlaneIndices);
	cout << "[ DEBUG ] Step 1: callJLinkage Success!!!\n";

	// Step 2: Remove points which contain less than 'minimumNumberOfPoints'
	// We have taken 'minimumNumberOfPoints' to be 30
	// Minimum number of points required to be qualified to be in data
	int minPointsPerPlane = 30;
	// Number of points in 'points' during initial stage
	int numberOfPointsInData = points.size();
	cout << "[ DEBUG ] We have " << numberOfPointsInData << " in our data.\n";
	// Mapping planeIndex -> number of points in that plane
	map<int, int> numberOfPointsPerPlane;
	// Plane Indices after removing unnecessary planes
	vector<int> newPlaneIndices;
	vector<Point3f> newPoints;
	// Number of planes after removing unnecessary planes
	int numberOfPlanes;
	removeUnnecessaryPlanes( points, oldPlaneIndices, minPointsPerPlane,
			numberOfPointsPerPlane, newPoints, newPlaneIndices, numberOfPlanes);
	cout << "[ DEBUG ] Number of planes detected at the end of Step 2: " << numberOfPlanes << "\n";
	cout << "[ DEBUG ] Step 2: removeUnnecessaryPlanes Success!!!\n";

	// Try to find for multiple planes only if there are more than 1 plane.
	if( numberOfPlanes > 1 ) {

		// Step 3: Perform K-means for the new set of planes obtained
		Mat pointsMatrix = Mat(newPoints.size(), 3, CV_32F);
		unsigned int i;
		// Create a pointsMatrix from vector<Point3f> data
		for(i=0; i<newPoints.size(); i++) {
			float * ptr = (float*)(&pointsMatrix.at<float>(i,0));
			ptr[0] = newPoints[i].x;
			ptr[1] = newPoints[i].y;
			ptr[2] = newPoints[i].z;
		}
		// Cluster centroid being returned from kmeans
		Mat clusterCentroids;
		/* Number of rounds for k-means */
		int numberOfRounds = 10;
		cout << "[ DEBUG ] " << pointsMatrix.rows << " " << pointsMatrix.cols << endl;
		cout << "[ DEBUG ] Old number of points: " << points.size() << endl;
		cout << "[ DEBUG ] New number of points: " << newPoints.size() << endl;

		// Perform k-means
		// Reference: http://docs.opencv.org/2.4/modules/core/doc/clustering.html
		// float kmeans(const cv::_InputArray &,
		// 	int, const cv::_OutputArray &,
		// 	cv::TermCriteria, int, int, const cv::_OutputArray &);
		cout << "[ DEBUG ] We've " << numberOfPlanes << " planes for k-means clustering.\n";
		//cout << "[ DEBUG ] newPoints( " << newPoints.size() << "): " << newPoints << endl;
		cout << "[ DEBUG ] newPlaneIndices( " << newPlaneIndices.size() << "): \n";
		//printVector(newPlaneIndices); cout.flush();
		// Performing kmeans using initial labels we've obtained in step 1
		kmeans(pointsMatrix, numberOfPlanes, newPlaneIndices,
				TermCriteria( CV_TERMCRIT_ITER, 10, 2.0), numberOfRounds,
				KMEANS_USE_INITIAL_LABELS, clusterCentroids);
		cout << "[ DEBUG ] newPlaneIndices after k-means ( " << newPlaneIndices.size() << "): \n";
		//printVector(newPlaneIndices); cout.flush();
		cout << "[ DEBUG ] Cluster centroids for kmeans\n";
		cout << clusterCentroids << "\n"; cout.flush();
		cout << "[ DEBUG ] Step 3: kmeans Success!!!\n"; cout.flush();

		// Plane Parameters (a, b, c, d) for plane i
		vector< vector<float> > planeParameters;
		// Obtain the plane parameters for the set of points obtained after Step 2
		// Arrange plane points belonging to a particular plane i
		vector< vector<Point3f> > planeOrderedPoints;
		getPlaneParameters(newPoints, newPlaneIndices, planeParameters, planeOrderedPoints);
		cout << "[ DEBUG ] Step 3: getPlaneParameters Success!!!\n"; cout.flush();

		// Step 4: Remove points which far from the estimated plane after performing k-means
		// Get 3D Projection of points onto the plane
		vector<float> distanceMatrix;
		vector< vector<int> > planePointsIndexMapping;
		vector<Point3f> newSortedPoints;
		vector< vector<float> > newPlaneParameters;
		map<int, pair<int, int> > planeIndexBounds;
		vector<Point3f> projectionsOf3DPoints;
		// Calculate the distances of the points from their respective planes
		calculateDistanceFromPlane( newPoints, planeParameters, newPlaneIndices,
				distanceMatrix, planePointsIndexMapping);
		cout << "[ DEBUG ] Step 4: calculateDistanceFromPlane Success!!!\n";
		cout.flush();
		removePointsFarFromPlane( newPoints, planeParameters, distanceMatrix, planePointsIndexMapping, newSortedPoints, newPlaneParameters, planeIndexBounds);
		cout << "[ DEBUG ] Step 4: removePointsFarFromPlane Success!!!\n";
		cout.flush();
		get3DPlaneProjectionsOfPoints ( newSortedPoints, newPlaneParameters, planeIndexBounds,
							projectionsOf3DPoints	);
		cout << "[ DEBUG ] Step 4: get3DPlaneProjectionsOfPoints Success!!!\n";
		cout.flush();

		// Step 5
		// Get continuous bounding box containing points in the plane
		vector<Point3f> sortedProjectionsOf3DPoints;
		map<int, pair<int, int> > sortedPlaneIndexBounds;
		vector< vector<Point3f> > boundingBoxPoints;
		orderPlanePointsByCentroids( projectionsOf3DPoints, newPlaneParameters, planeIndexBounds,
				sortedProjectionsOf3DPoints, sortedPlaneParameters, sortedPlaneIndexBounds);
		cout << "[ DEBUG ] Step 5: orderPlanePointsByCentroids Success!!!\n";
		cout.flush();
		getBoundingBoxCoordinates ( sortedProjectionsOf3DPoints, sortedPlaneParameters,
				sortedPlaneIndexBounds, boundingBoxPoints );
		cout << "[ DEBUG ] Step 5: getBoundingBoxCoordinates Success!!!\n";
		cout.flush();
		getContinuousBoundingBox ( boundingBoxPoints, sortedPlaneParameters,
								continuousBoundingBoxPoints);
		cout << "[ DEBUG ] Step 5: getContinuousBoundingBox Success!!!\n";
		cout.flush();

	}
	else {
		cout << "[ DEBUG ] Number of planes is less than or equal to 1.\n";
		vector<float> planeParameters;
		vector<Point3f> projectionsOf3DPoints;
		fitPlane3D(newPoints, planeParameters);
		float a = planeParameters[0];
		float b = planeParameters[1];
		float c = planeParameters[2];
		float d = planeParameters[3];
		for (unsigned int j = 0; j < newPoints.size(); ++j) {
			float x0 = newPoints[j].x;
			float y0 = newPoints[j].y;
			float z0 = newPoints[j].z;
			float t = ((-1)*(a*x0+b*y0+c*z0+d))/(a*a+b*b+c*c);
			float projX0 = x0 + a*t;
			float projY0 = y0 + b*t;
			float projZ0 = z0 + c*t;
			projectionsOf3DPoints.push_back(Point3f(projX0, projY0, projZ0));
		}
		vector<Point3f> boundingBoxPoints;
		getBoundingBoxPointsOfPlane(projectionsOf3DPoints, planeParameters, boundingBoxPoints);
		sortedPlaneParameters.push_back(planeParameters);
		continuousBoundingBoxPoints.push_back(boundingBoxPoints);
	}
	return 0;

}


void findPercBoundEachPlane(
		const vector<Point3f> &points,
		vector< vector<float> > &sortedPlaneParameters,
		vector< vector<Point3f> > &continuousBoundingBoxPoints,
		vector< vector<Point3f> > &sorted_3d_points,
		vector<float> &percentageOfEachPlane)
{
	cout << "[ DEBUG] [findPercBoundEachPlane] Started\n";
	// Step 1: Performing JLinkage to find multiple models
	// This vector describes to which plane does point i belong to
	// This is an output from JLinkage
	vector<int> oldPlaneIndices;
	cout << "[ DEBUG] [findPercBoundEachPlane] Calling JLinkage from findPercBoundEachPlane!!!\n";
	callJLinkage(points, oldPlaneIndices);
	// cout << "[ DEBUG ] Step 1: callJLinkage Success!!!\n";
	// Step 2: Remove points which contain less than 'minimumNumberOfPoints'
	// We have taken 'minimumNumberOfPoints' to be 30
	// Minimum number of points required to be qualified to be in data
	int minPointsPerPlane = 30;
	// Number of points in 'points' during initial stage
	int numberOfPointsInData = points.size();
	// cout << "[ DEBUG ] We have " << numberOfPointsInData << " in our data.\n";
	// Mapping planeIndex -> number of points in that plane
	map<int, int> numberOfPointsPerPlane;
	// Plane Indices after removing unnecessary planes
	vector<int> newPlaneIndices;
	vector<Point3f> newPoints;
	// Number of planes after removing unnecessary planes
	int numberOfPlanes;
	cout << "[ DEBUG] [findPercBoundEachPlane] Calling removeUnnecessaryPlanes!!!\n";
	removeUnnecessaryPlanes( points, oldPlaneIndices, minPointsPerPlane,
			numberOfPointsPerPlane, newPoints, newPlaneIndices, numberOfPlanes);
	// cout << "[ DEBUG ] Number of planes detected at the end of Step 2: " << numberOfPlanes << "\n";
	// cout << "[ DEBUG ] Step 2: removeUnnecessaryPlanes Success!!!\n";

	// Try to find for multiple planes only if there are more than 1 plane.
	if( numberOfPlanes > 1 ) {

		// Step 3: Perform K-means for the new set of planes obtained
		Mat pointsMatrix = Mat(newPoints.size(), 3, CV_32F);
		unsigned int i;
		// Create a pointsMatrix from vector<Point3f> data
		for(i=0; i<newPoints.size(); i++) {
			float * ptr = (float*)(&pointsMatrix.at<float>(i,0));
			ptr[0] = newPoints[i].x;
			ptr[1] = newPoints[i].y;
			ptr[2] = newPoints[i].z;
		}
		// Cluster centroid being returned from kmeans
		Mat clusterCentroids;
		/* Number of rounds for k-means */
		int numberOfRounds = 10;
		// cout << "[ DEBUG ] " << pointsMatrix.rows << " " << pointsMatrix.cols << endl;
		// cout << "[ DEBUG ] Old number of points: " << points.size() << endl;
		// cout << "[ DEBUG ] New number of points: " << newPoints.size() << endl;

		// Perform k-means
		// Reference: http://docs.opencv.org/2.4/modules/core/doc/clustering.html
		// float kmeans(const cv::_InputArray &,
		// 	int, const cv::_OutputArray &,
		// 	cv::TermCriteria, int, int, const cv::_OutputArray &);
		// cout << "[ DEBUG ] We've " << numberOfPlanes << " planes for k-means clustering.\n";
		//cout << "[ DEBUG ] newPoints( " << newPoints.size() << "): " << newPoints << endl;
		// cout << "[ DEBUG ] newPlaneIndices( " << newPlaneIndices.size() << "): \n";
		//printVector(newPlaneIndices); cout.flush();
		// Performing kmeans using initial labels we've obtained in step 1
		cout << "[ DEBUG] [findPercBoundEachPlane] Calling Kmeans\n";
		kmeans(pointsMatrix, numberOfPlanes, newPlaneIndices,
				TermCriteria( CV_TERMCRIT_ITER, 10, 2.0), numberOfRounds,
				KMEANS_USE_INITIAL_LABELS, clusterCentroids);
		// cout << "[ DEBUG ] newPlaneIndices after k-means ( " << newPlaneIndices.size() << "): \n";
		//printVector(newPlaneIndices); cout.flush();
		// cout << "[ DEBUG ] Cluster centroids for kmeans\n";
		// cout << clusterCentroids << "\n"; cout.flush();
		// cout << "[ DEBUG ] Step 3: kmeans Success!!!\n"; cout.flush();

		// Plane Parameters (a, b, c, d) for plane i
		vector< vector<float> > planeParameters;
		// Obtain the plane parameters for the set of points obtained after Step 2
		// Arrange plane points belonging to a particular plane i
		vector< vector<Point3f> > planeOrderedPoints;
		cout << "[ DEBUG] [findPercBoundEachPlane] Calling getPlaneParameters\n";
		getPlaneParameters(newPoints, newPlaneIndices, planeParameters, planeOrderedPoints);
		// cout << "[ DEBUG ] Step 3: getPlaneParameters Success!!!\n"; cout.flush();

		// Step 4: Remove points which far from the estimated plane after performing k-means
		// Get 3D Projection of points onto the plane
		vector<float> distanceMatrix;
		vector< vector<int> > planePointsIndexMapping;
		vector<Point3f> newSortedPoints;
		vector< vector<float> > newPlaneParameters;
		map<int, pair<int, int> > planeIndexBounds;
		vector<Point3f> projectionsOf3DPoints;
		// Calculate the distances of the points from their respective planes
		cout << "[ DEBUG] [findPercBoundEachPlane] Calling calculateDistanceFromPlane\n";
		calculateDistanceFromPlane( newPoints, planeParameters, newPlaneIndices,
				distanceMatrix, planePointsIndexMapping);
		// cout << "[ DEBUG ] Step 4: calculateDistanceFromPlane Success!!!\n";
		// cout.flush();
		cout << "[ DEBUG] [findPercBoundEachPlane] Calling removePointsFarFromPlane\n";
		removePointsFarFromPlane( newPoints, planeParameters, distanceMatrix, planePointsIndexMapping, 
			newSortedPoints, newPlaneParameters, planeIndexBounds);
		// cout << "[ DEBUG ] Step 4: removePointsFarFromPlane Success!!!\n";
		// cout.flush();
		cout << "[ DEBUG] [findPercBoundEachPlane] Calling get3DPlaneProjectionsOfPoints\n";
		get3DPlaneProjectionsOfPoints ( newSortedPoints, newPlaneParameters, planeIndexBounds,
							projectionsOf3DPoints);
		// cout << "[ DEBUG ] Step 4: get3DPlaneProjectionsOfPoints Success!!!\n";
		// cout.flush();

		// Step 5
		// Get continuous bounding box containing points in the plane
		vector<Point3f> sortedProjectionsOf3DPoints;
		map<int, pair<int, int> > sortedPlaneIndexBounds;
		vector< vector<Point3f> > boundingBoxPoints;
		cout << "[ DEBUG] [findPercBoundEachPlane] Calling orderPlanePointsByCentroids\n";
		orderPlanePointsByCentroids( projectionsOf3DPoints, newPlaneParameters, planeIndexBounds,
				sortedProjectionsOf3DPoints, sortedPlaneParameters, sortedPlaneIndexBounds);
		// cout << "[ DEBUG ] Step 5: orderPlanePointsByCentroids Success!!!\n";
		// cout.flush();
		cout << "[ DEBUG] [findPercBoundEachPlane] Calling getBoundingBoxCoordinates\n";
		getBoundingBoxCoordinates ( sortedProjectionsOf3DPoints, sortedPlaneParameters,
				sortedPlaneIndexBounds, boundingBoxPoints );
		// cout << "[ DEBUG ] Step 5: getBoundingBoxCoordinates Success!!!\n";
		// cout.flush();
		cout << "[ DEBUG] [findPercBoundEachPlane] Calling getPercentageOfEachPlane\n";
		getPercentageOfEachPlane ( sortedProjectionsOf3DPoints, sortedPlaneIndexBounds, sorted_3d_points, percentageOfEachPlane );
		cout << "[ DEBUG] [findPercBoundEachPlane] Calling getContinuousBoundingBox\n";
		getContinuousBoundingBox ( boundingBoxPoints, sortedPlaneParameters, continuousBoundingBoxPoints);
		cout << "[ DEBUG] [findPercBoundEachPlane] Completed\n";
		// cout << "[ DEBUG ] Step 5: getContinuousBoundingBox Success!!!\n";
		// cout.flush();

	}
	else {
		// cout << "[ DEBUG ] Number of planes is less than or equal to 1.\n";
		vector<float> planeParameters;
		vector<Point3f> projectionsOf3DPoints;
		fitPlane3D(newPoints, planeParameters);
		float a = planeParameters[0];
		float b = planeParameters[1];
		float c = planeParameters[2];
		float d = planeParameters[3];
		for (unsigned int j = 0; j < newPoints.size(); ++j) {
			float x0 = newPoints[j].x;
			float y0 = newPoints[j].y;
			float z0 = newPoints[j].z;
			float t = ((-1)*(a*x0+b*y0+c*z0+d))/(a*a+b*b+c*c);
			float projX0 = x0 + a*t;
			float projY0 = y0 + b*t;
			float projZ0 = z0 + c*t;
			projectionsOf3DPoints.push_back(Point3f(projX0, projY0, projZ0));
		}
		int size = sorted_3d_points.size();
		for (int i = 0; i < size; ++i)
		{
			sorted_3d_points[i].clear();
		}
		sorted_3d_points.clear();
		vector<Point3f> boundingBoxPoints;
		getBoundingBoxPointsOfPlane(projectionsOf3DPoints, planeParameters, boundingBoxPoints);
		sorted_3d_points.push_back(projectionsOf3DPoints);
		percentageOfEachPlane.clear();
		percentageOfEachPlane.push_back(1.0);
		sortedPlaneParameters.push_back(planeParameters);
		continuousBoundingBoxPoints.push_back(boundingBoxPoints);
		cout << "[ DEBUG] [findPercBoundEachPlane] Completed\n";
	}
	return ;

}


/***************** EXTRA FUNCTIONS ********************************************/

int findMultiplePlanes1(
		const vector<Point3f> &points,
		vector< vector<float> > &sortedPlaneParameters,
		vector< vector<Point3f> > &continuousBoundingBoxPoints ){

	// Step 1: Performing JLinkage to find multiple models
	// This vector describes to which plane does point i belong to
	// This is an output from JLinkage
	vector<int> oldPlaneIndices;
	cout << "[ DEBUG ] Step 1: Calling callJLinkage!!!\n";
	callJLinkage(points, oldPlaneIndices);
	cout << "[ DEBUG ] Step 1: callJLinkage Success!!!\n";

	// Step 2: Remove points which contain less than 'minimumNumberOfPoints'
	// We have taken 'minimumNumberOfPoints' to be 30
	// Minimum number of points required to be qualified to be in data
	int minPointsPerPlane = 30;
	// Number of points in 'points' during initial stage
	int numberOfPointsInData = points.size();
	cout << "[ DEBUG ] We have " << numberOfPointsInData << " in our data.\n";
	// Mapping planeIndex -> number of points in that plane
	map<int, int> numberOfPointsPerPlane;
	// Plane Indices after removing unnecessary planes
	vector<int> newPlaneIndices;
	vector<Point3f> newPoints;
	// Number of planes after removing unnecessary planes
	int numberOfPlanes;
	removeUnnecessaryPlanes( points, oldPlaneIndices, minPointsPerPlane,
			numberOfPointsPerPlane, newPoints, newPlaneIndices, numberOfPlanes);
	cout << "[ DEBUG ] Number of planes detected at the end of Step 2: " << numberOfPlanes << "\n";
	cout << "[ DEBUG ] Step 2: removeUnnecessaryPlanes Success!!!\n";

	// Try to find for multiple planes only if there are more than 1 plane.
	if( numberOfPlanes > 1 ) {

		// Step 3: Perform K-means for the new set of planes obtained
		Mat pointsMatrix = Mat(newPoints.size(), 3, CV_32F);
		unsigned int i;
		// Create a pointsMatrix from vector<Point3f> data
		for(i=0; i<newPoints.size(); i++) {
			float * ptr = (float*)(&pointsMatrix.at<float>(i,0));
			ptr[0] = newPoints[i].x;
			ptr[1] = newPoints[i].y;
			ptr[2] = newPoints[i].z;
		}
		// Cluster centroid being returned from kmeans
		Mat clusterCentroids;
		/* Number of rounds for k-means */
		int numberOfRounds = 10;
		cout << "[ DEBUG ] " << pointsMatrix.rows << " " << pointsMatrix.cols << endl;
		cout << "[ DEBUG ] Old number of points: " << points.size() << endl;
		cout << "[ DEBUG ] New number of points: " << newPoints.size() << endl;

		// Perform k-means
		// Reference: http://docs.opencv.org/2.4/modules/core/doc/clustering.html
		// float kmeans(const cv::_InputArray &,
		// 	int, const cv::_OutputArray &,
		// 	cv::TermCriteria, int, int, const cv::_OutputArray &);
		cout << "[ DEBUG ] We've " << numberOfPlanes << " planes for k-means clustering.\n";
		cout << "[ DEBUG ] newPoints( " << newPoints.size() << "): " << newPoints << endl;
		cout << "[ DEBUG ] newPlaneIndices( " << newPlaneIndices.size() << "): \n";
		printVector(newPlaneIndices); cout.flush();
		// Performing kmeans using initial labels we've obtained in step 1
		kmeans(pointsMatrix, numberOfPlanes, newPlaneIndices,
				TermCriteria( CV_TERMCRIT_ITER, 10, 2.0), numberOfRounds,
				KMEANS_USE_INITIAL_LABELS, clusterCentroids);
		cout << "[ DEBUG ] newPlaneIndices after k-means ( " << newPlaneIndices.size() << "): \n";
		printVector(newPlaneIndices); cout.flush();
		cout << "[ DEBUG ] Cluster centroids for kmeans\n";
		cout << clusterCentroids << "\n"; cout.flush();
		cout << "[ DEBUG ] Step 3: kmeans Success!!!\n"; cout.flush();

		// Plane Parameters (a, b, c, d) for plane i
		vector< vector<float> > planeParameters;
		// Obtain the plane parameters for the set of points obtained after Step 2
		// Arrange plane points belonging to a particular plane i
		vector< vector<Point3f> > planeOrderedPoints;
		getPlaneParameters(newPoints, newPlaneIndices, planeParameters, planeOrderedPoints);
		cout << "[ DEBUG ] Step 3: getPlaneParameters Success!!!\n"; cout.flush();

		// Step 4: Remove points which far from the estimated plane after performing k-means
		// Get 3D Projection of points onto the plane
		vector< vector<float> > distanceMatrix;
		vector< vector<float> > newPlaneParameters;
		vector< vector<Point3f> > newOrderedPointsData;
		vector< vector<Point3f> > projectionsOf3DPoints;
		// Calculate the distances of the points from their respective planes
		calculateDistanceFromPlane1( planeOrderedPoints, planeParameters, distanceMatrix);
		cout << "[ DEBUG ] Step 4: calculateDistanceFromPlane1 Success!!!\n";
		cout.flush();
		removePointsFarFromPlane1( planeOrderedPoints, planeParameters, distanceMatrix,
				newOrderedPointsData, newPlaneParameters);
		cout << "[ DEBUG ] Step 4: removePointsFarFromPlane1 Success!!!\n";
		cout.flush();
		get3DPlaneProjectionsOfPoints1( newOrderedPointsData, newPlaneParameters,
							projectionsOf3DPoints	);
		cout << "[ DEBUG ] Step 4: get3DPlaneProjectionsOfPoints1 Success!!!\n";
		cout.flush();

		// Step 5
		// Get continuous bounding box containing points in the plane
		vector< vector<Point3f> > sortedProjectionsOf3DPoints;
		vector< vector<Point3f> > boundingBoxPoints;
		orderPlanePointsByCentroids1( projectionsOf3DPoints, newPlaneParameters,
				sortedProjectionsOf3DPoints, sortedPlaneParameters);
		cout << "[ DEBUG ] Step 5: orderPlanePointsByCentroids1 Success!!!\n";
		cout.flush();
		getBoundingBoxCoordinates1( sortedProjectionsOf3DPoints, sortedPlaneParameters, boundingBoxPoints );
		cout << "[ DEBUG ] Step 5: getBoundingBoxCoordinates1 Success!!!\n";
		cout.flush();
		getContinuousBoundingBox ( boundingBoxPoints, sortedPlaneParameters,
								continuousBoundingBoxPoints);
		cout << "[ DEBUG ] Step 5: getContinuousBoundingBox Success!!!\n";
		cout.flush();

	}
	else {
		cout << "[ DEBUG ] Number of planes is less than or equal to 1.\n";
	}
	return 0;

}

