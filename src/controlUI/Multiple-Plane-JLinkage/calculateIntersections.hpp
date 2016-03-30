/**
 * @defgroup calculateIntersection
 * @file calculateIntersection.hpp
 * @ingroup calculateIntersection
 */

/*
 *   File Name: calculateIntersections.hpp
 *     Project: Multiple Plane JLinkage
 *  Created on: 22-Mar-2016
 *      Author: Sona Praneeth Akula
 *     Details:
 *   TodoNotes: TODO
 */

#ifndef CALCULATEINTERSECTIONS_HPP_
#define CALCULATEINTERSECTIONS_HPP_


#include "allHeaders.hpp"
#include "utilities.hpp"

/**
 * @brief - Determine the line parameters of the line joining two points
 *
 * @param [in]  point1 - A point in 3D format
 * @param [in]  point2 - Another point in 3D format on other end of line
 * @param [out] lineParameters - Parameters determining the
 * 				line equation (a, b, c; x0, y0, z0) Direction and a point
 * @return Nothing
 */
void makeLineFromPoints(
		const Point3f &point1,
		const Point3f &point2,
		vector< vector<float> > &lineParameters);

/**
 * @brief - Determine the line parameters of the line obtained as the intersection
 * 			of two planes
 *
 * @param [in]  plane1 - Parameters of plane1 (a1, b1, c1, d1)
 * @param [in]  plane2 - Parameters of plane2 (a2, b2, c2, d2)
 * @param [out] lineParameters - Parameters determining the
 * 				line equation (x0, y0, z0; a, b, c) Direction and a point
 * @return Nothing
 */
void calculateIntersectionOfPlanes(
		const vector<float> &plane1,
		const vector<float> &plane2,
		vector< vector<float> > &lineParameters);

/**
 * @brief - Determine the line parameters of the line joining two points
 *
 * @param [in]  line1 - Parameters of line1 (x01, y01, z01; a1, b1, c1)
 * @param [in]  line2 - Parameters of line2 (x02, y02, z02; a2, b2, c2)
 *
 * @return [out] intersectionOfLines - Point where two lines intersect
 */
void calculateIntersectionOfLines(
		const vector< vector<float> > &line1,
		const vector< vector<float> > &line2,
		Point3f &intersectionOfLines);


#endif /* CALCULATEINTERSECTIONS_HPP_ */
