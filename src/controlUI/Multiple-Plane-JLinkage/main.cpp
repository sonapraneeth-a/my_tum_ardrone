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

// The main file
int test() {

	// Initializing the parameters for the JLinkage
	// Number of initial hypothesis to be generated. 'M' in the paper
	LLI numberOfTrials = 5000;
	// Minimum number of points to be present in the plane. 'S' in the paper
	LLI clusterThreshold = 75;
	// Inliers fraction
	double inliersThreshold = 0.1;
	// $\sigma$ for gaussian distribution to calculate the distribution (CDF)
	double sigmaExp = 0.5;

	// Read 3D points into vector
	string filename = "points_fckohli_02.csv";
	vector<Point3d> points, newPoints;
	readPointsFromCSV(filename, points);
	// Test to see if the points have been printed correctly
	printPointsVector(points, 3);

	// Step 1: Performing JLinkage to find multiple models
	// This vector describes to which plane does point i belong to
	// This is an output from JLinkage
	vector<LLI> planeIndices;
	// Plane Parameters (a, b, c, d) for plane i
	vector< vector<double> > planeParameters;
	// performJLinkage(points, sigmaExp, inliersThreshold, planeIndices, ...);
	// planeParameters = fitting_fn(points);

	// Step 2
	map<LLI, LLI> freqOfSelectedPlanes;
	LLI minPointsPerPlane = 50;
	LLI numberOfPointsInData = points.size();
	// LLI numberOfUniquePlanes = numberOfUniquePlanes( planeIndices );
	map<LLI, LLI> numberOfPointsPerPlane;
	vector<LLI> newPlaneIndices;
	LLI numberOfPlanes;
	removeUnnecessaryPlanes( points, planeIndices, minPointsPerPlane,
			numberOfPointsPerPlane, newPoints, newPlaneIndices, numberOfPlanes);

	// Step 3
	// Generate parameters from the new set of points
	// planeParameters = fitting_fn(points);

	// Step 4
	// Perform K-means
	Mat pointsMatrix = Mat(points);
	// Reference: http://docs.opencv.org/2.4/modules/core/doc/clustering.html
	// kmeans(pointsMatrix, numberOfPlanes, planeIndices, TermCriteria::epsilon, clustersCenters);
	// planeParameters = fitting_fn(points);

	// Step 5
	// Remove points which far from the estimated plane after performing k-means
	// Get 3D Projection of points onto the plane
	vector<double> distanceMatrix;
	vector< vector<LLI> > planePointsIndexMapping;
	vector<Point3d> newSortedPoints;
	vector< vector<double> > newPlaneParameters;
	map<LLI, pair<LLI, LLI> > planeIndexBounds;
	vector<Point3d> projectionsOf3DPoints;
	calculateDistanceFromPlane( newPoints, planeParameters, planeIndices,
			distanceMatrix, planePointsIndexMapping);
	removePointsFarFromPlane( newPoints, planeParameters, distanceMatrix, planePointsIndexMapping,
			newSortedPoints, newPlaneParameters, planeIndexBounds);
	get3DPlaneProjectionsOfPoints ( newSortedPoints, newPlaneParameters, planeIndexBounds,
						projectionsOf3DPoints	);


	// Step 6
	vector<Point3d> sortedProjectionsOf3DPoints;
	vector< vector<double> > sortedPlaneParameters;
	map<LLI, pair<LLI, LLI> > sortedPlaneIndexBounds;
	vector< vector<Point3d> > boundingBoxPoints;
	vector< vector<Point3d> > continuousBoundingBoxPoints;
	orderPlanePointsByCentroids( projectionsOf3DPoints, planeParameters, planeIndexBounds,
			sortedProjectionsOf3DPoints, sortedPlaneParameters, sortedPlaneIndexBounds);
	getBoundingBoxCoordinates ( sortedProjectionsOf3DPoints, sortedPlaneParameters,
			sortedPlaneIndexBounds, boundingBoxPoints );
	getContinuousBoundingBox ( boundingBoxPoints, sortedPlaneParameters,
							continuousBoundingBoxPoints);
	return 0;
}
