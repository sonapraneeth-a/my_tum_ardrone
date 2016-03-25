/**
 * @defgroup conversion
 *
 */

/**
 * @file conversion.hpp
 * @ingroup conversion
 *
 *   File Name: conversion.hpp
 *     Project: Multiple Plane JLinkage
 *  Created on: 22-Mar-2016
 *      Author: Sona Praneeth Akula
 *     Details:
 *   TodoNotes: TODO
 */

/**
  * 	@brief 		Class having functions for converting between various co-ordinates
  */

#ifndef CONVERSION_HPP_
#define CONVERSION_HPP_
#pragma once

#include "allHeaders.hpp"

/**
 * @brief - Converting XYZ 3D co-ordinates to UV co-ordinates
 *
 * @param [in]  [Point3d] xyzCoordinates  XYZ co-ordinates of the point to be
 * 										transformed
 * @param [in]  [vector<double>] planeParameters - Parameters for the plane
 * 												ax+by+cz+d=0: (a, b, c, d)
 * @param [out] [Point2d] uvCoordinates - The required UV coordinates as output
 * @param [out] [vector<Point3d>] uvAxes - The axes (u, v, n) used in transforming
 * 										XYZ co-ordinates to UV co-ordinates
 *
 * 	@return Nothing
 */
void XYZToUVCoordinates(
		const Point3d &xyzCoordinates,
		const vector<double> &planeParameters,
		Point2d &uvCoordinates,
		vector<Point3d> &uvAxes
		);

void AllXYZToUVCoordinates(
		const vector<Point3d> &xyzCoordinates,
		const vector<double> &planeParameters,
		vector<Point2d> &uvCoordinates,
		vector<Point3d> &uvAxes);

/**
 * @brief - Converting UV 2D co-ordinates to XYZ 3D co-ordinates
 *
 * @param [in]  [Point2d] uvCoordinates  UV co-ordinates of the point to be
 * 										transformed
 * @param [in]  [vector<Point3d>] uvAxes - The axis (u, v, n) used for transforming
 * 										XYZ co-ordinates to UV co-ordinates
 * @param [in]  [double] d - The fourth parameter for the plane ax+by+cz+d=0
 * @param [out] [Point3d] xyzCoordinates - The required output XYZ co-ordinates
 *
 * @return Nothing
 */
void UVToXYZCoordinates(
		const Point2d &uvCoordinates,
		const vector<Point3d> &uvAxes,
		const double d,
		Point3d &xyzCoordinates
		);

void AllUVToXYZCoordinates(
		const vector<Point2d> &uvCoordinates,
		const vector<Point3d> &uvAxes,
		const double d,
		vector<Point3d> &xyzCoordinates);


#endif /* CONVERSION_HPP_ */
