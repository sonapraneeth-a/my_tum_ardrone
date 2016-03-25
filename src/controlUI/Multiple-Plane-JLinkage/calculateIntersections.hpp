/**
 * @defgroup calculateIntersection
 *
 */

/**
 * @file calculateIntersection.hpp
 * @ingroup calculateIntersection
 *
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

/**
 * @brief - Determine the line parameters of the line joining two points
 *
 * @param [in]  [Point3d] point1 - A point in 3D format
 * @param [in]  [Point3d] point2 - Another point in 3D format on other end of line
 * @param [out] [vector< vector<double> >] lineParameters - Parameters determining the
 * 				line equation (a, b, c; x0, y0, z0) Direction and a point
 * @return Nothing
 */
void makeLineFromPoints(
		const Point3d &point1,
		const Point3d &point2,
		vector< vector<double> > &lineParameters);

/**
 * @brief - Determine the line parameters of the line obtained as the intersection
 * 			of two planes
 *
 * @param [in]  [vector<double>] plane1 - Parameters of plane1 (a1, b1, c1, d1)
 * @param [in]  [vector<double>] plane2 - Parameters of plane2 (a2, b2, c2, d2)
 * @param [out] [vector< vector<double> >] lineParameters - Parameters determining the
 * 				line equation (x0, y0, z0; a, b, c) Direction and a point
 * @return Nothing
 */
void calculateIntersectionOfPlanes(
		const vector<double> &plane1,
		const vector<double> &plane2,
		vector< vector<double> > &lineParameters);

/**
 * @brief - Determine the line parameters of the line joining two points
 *
 * @param [in]  [vector< vector<double> >] line1 - Parameters of line1 (x01, y01, z01; a1, b1, c1)
 * @param [in]  [vector< vector<double> >] line2 - Parameters of line2 (x02, y02, z02; a2, b2, c2)
 *
 * @return [out] [Point3d] point - Point where two lines intersect
 */
Point3d calculateIntersectionOfLines(
		vector< vector<double> > line1,
		vector< vector<double> > line2);


#endif /* CALCULATEINTERSECTIONS_HPP_ */
