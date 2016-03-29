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
 * @param [in]  [Point3f] xyzCoordinates  XYZ co-ordinates of the point to be
 * 										transformed
 * @param [in]  [vector<float>] planeParameters - Parameters for the plane
 * 												ax+by+cz+d=0: (a, b, c, d)
 * @param [out] [Point2d] uvCoordinates - The required UV coordinates as output
 * @param [out] [vector<Point3f>] uvAxes - The axes (u, v, n) used in transforming
 * 										XYZ co-ordinates to UV co-ordinates
 *
 * 	@return Nothing
 */
void XYZToUVCoordinates(
		const Point3f &xyzCoordinates,
		const vector<float> &planeParameters,
		Point2f &uvCoordinates,
		vector<Point3f> &uvAxes
		);

void AllXYZToUVCoordinates(
		const vector<Point3f> &xyzCoordinates,
		const vector<float> &planeParameters,
		vector<Point2f> &uvCoordinates,
		vector<Point3f> &uvAxes);

/**
 * @brief - Converting UV 2D co-ordinates to XYZ 3D co-ordinates
 *
 * @param [in]  [Point2d] uvCoordinates  UV co-ordinates of the point to be
 * 										transformed
 * @param [in]  [vector<Point3f>] uvAxes - The axis (u, v, n) used for transforming
 * 										XYZ co-ordinates to UV co-ordinates
 * @param [in]  [float] d - The fourth parameter for the plane ax+by+cz+d=0
 * @param [out] [Point3f] xyzCoordinates - The required output XYZ co-ordinates
 *
 * @return Nothing
 */
void UVToXYZCoordinates(
		const Point2f &uvCoordinates,
		const vector<Point3f> &uvAxes,
		const float d,
		Point3f &xyzCoordinates
		);

void AllUVToXYZCoordinates(
		const vector<Point2f> &uvCoordinates,
		const vector<Point3f> &uvAxes,
		const float d,
		vector<Point3f> &xyzCoordinates);


#endif /* CONVERSION_HPP_ */
