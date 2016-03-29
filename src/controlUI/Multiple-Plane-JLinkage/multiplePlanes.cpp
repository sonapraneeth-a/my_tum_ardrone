/*
 *   File Name: main.cpp
 *     Project: Multiple Plane JLinkage
 *  Created on: 20-Mar-2016
 *      Author: Sona Praneeth Akula
 *     Details:
 *   TodoNotes: TODO
 */

#include "allHeaders.hpp"
#include "readingData.hpp"
#include "additionalSteps.hpp"
#include "utilities.hpp"
#include "makeBoundingRects.hpp"
#include "JLinkage.h"
#include "RandomSampler.h"
#include "PrimitiveFunctions.h"
#include <opencv2/core/core.hpp>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <list>


using namespace std;
using namespace cv;


// The main file
void callJLinkage(const vector<Point3f> &locations, vector<int> &labels)
{
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


int findMultiplePlanes(const vector<Point3f> &points, vector< vector<float> > &sortedPlaneParameters,  vector< vector<Point3f> > &continuousBoundingBoxPoints){ 

	// Step 1: Performing JLinkage to find multiple models
	// This vector describes to which plane does point i belong to
	/* This is an output from JLinkage */
	vector<int> oldPlaneIndices;
	/* Plane Parameters (a, b, c, d) for plane i */
	vector< vector<float> > planeParameters;
	callJLinkage(points, oldPlaneIndices);
	cout << "[ DEBUG ] Step 1: callJLinkage Success!!!\n";

	// Step 2: Remove points which contain less than minimumNumberOfPoints
	// We have taken minimumNumberOfPoints to be 50
	/* Minimum number of points required to be qualified to be in data*/
	int minPointsPerPlane = 30;
	/* Number of points in the initial stage */
	int numberOfPointsInData = points.size();
	map<int, int> numberOfPointsPerPlane, freqOfSelectedPlanes;
	vector<int> newPlaneIndices, planeIndices;
	vector<Point3f> newPoints;
	int numberOfPlanes;
	removeUnnecessaryPlanes( points, oldPlaneIndices, minPointsPerPlane,
			numberOfPointsPerPlane, newPoints, newPlaneIndices, numberOfPlanes);
	cout << "[ DEBUG ] Number of planes detected at the end of Step 2: " << numberOfPlanes << "\n";
	cout << "[ DEBUG ] Step 2: removeUnnecessaryPlanes Success!!!\n";

	if( numberOfPlanes > 1 ) {
		// Step 3
		// Perform K-means
		Mat pointsMatrix = Mat(newPoints.size(), 3, CV_32F);
		//,0 newPoints.size(), 3, CV_32F);
		// cout << pointsMatrix << endl;
		for(int i=0; i<newPoints.size(); i++)
		{
			float * ptr = (float*)(&pointsMatrix.at<float>(i,0));
			ptr[0] = newPoints[i].x;
			ptr[1] = newPoints[i].y;
			ptr[2] = newPoints[i].z;
		}
		Mat clusterCentroids;//(numberOfPlanes, 3, CV_32F);
		/* Number of rounds for k-means */
		int numberOfRounds = 10;
		/*cout << "[ DEBUG ] " << pointsMatrix.rows << " " << pointsMatrix.cols << endl;*/
		cout << "[ DEBUG ] " << points.size() << endl;
		cout << "[ DEBUG ] " << newPoints.size() << endl;

		/* Perform k-means */
		// Reference: http://docs.opencv.org/2.4/modules/core/doc/clustering.html
		// float kmeans(const cv::_InputArray &,
		// 	int, const cv::_OutputArray &,
		// 	cv::TermCriteria, int, int, const cv::_OutputArray &);
		cout << "[ DEBUG ] We've " << numberOfPlanes << " planes for k-means clustering.\n";
		cout << "[ DEBUG ] newPoints( " << newPoints.size() << "): " << newPoints << endl;
		cout << "[ DEBUG ] newPlaneIndices( " << newPlaneIndices.size() << "): \n"; 
		printVector(newPlaneIndices); cout.flush();
		kmeans(pointsMatrix, numberOfPlanes, newPlaneIndices,
				TermCriteria( CV_TERMCRIT_ITER, 10, 2.0), numberOfRounds, 
				KMEANS_USE_INITIAL_LABELS, clusterCentroids);
		cout << "[ DEBUG ] newPlaneIndices after k-means ( " << newPlaneIndices.size() << "): \n"; 
		printVector(newPlaneIndices); cout.flush();
		cout << "[ DEBUG ] ";
		cout << clusterCentroids << "\n"; cout.flush();
		cout << "[ DEBUG ] Step 3: kmeans Success!!!\n"; cout.flush();
		// int numberOfPoints = points.size();
		/* Obtained planeIndices after perfrming k-means */
		// planeIndices = newPlaneIndices;
		/* Obtain the plane parameters for the set of points obtained after Step 2*/
		getPlaneParameters(newPoints, newPlaneIndices, planeParameters);
		cout << "[ DEBUG ] Step 3: getPlaneParameters Success!!!\n"; cout.flush();

		// Step 4
		// Remove points which far from the estimated plane after performing k-means
		// Get 3D Projection of points onto the plane
		vector<float> distanceMatrix;
		vector< vector<int> > planePointsIndexMapping;
		vector<Point3f> newSortedPoints;
		vector< vector<float> > newPlaneParameters;
		map<int, pair<int, int> > planeIndexBounds;
		vector<Point3f> projectionsOf3DPoints;
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
		cout << "Number of planes is less than or equal to 1.\n";
	}
	return 0;

}
