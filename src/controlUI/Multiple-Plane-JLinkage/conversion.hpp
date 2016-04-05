/**
 * @defgroup conversion
 * @file conversion.hpp
 * @ingroup conversion
 *
 */

/*
 *   File Name: conversion.hpp
 *     Project: Multiple Plane JLinkage
 *  Created on: 29-Mar-2016
 *      Author: Sona Praneeth Akula
 *     Details:
 *   TodoNotes: TODO
 */

#ifndef CONVERSION_HPP_
#define CONVERSION_HPP_
#pragma once

#include "allHeaders.hpp"

/**
 * @brief - Converting XYZ 3D co-ordinates to UV co-ordinates
 *
 * @param [in] xyzCoordinates  XYZ co-ordinates of the point to be transformed
 * @param [in] planeParameters - Parameters for the plane ax+by+cz+d=0: (a, b, c, d)
 * @param [out] uvCoordinates - The required UV coordinates as output
 * @param [out] uvAxes - The axes (u, v, n) used in transforming XYZ co-ordinates to UV co-ordinates
 *
 */
void XYZToUVCoordinates(
		const vector<float> &planeParameters,
		vector<Point3f> &uvAxes
		);

/**
 * @brief - Converting All XYZ 3D co-ordinates for a plane to UV co-ordinates
 *
 * @param [in] xyzCoordinates  XYZ co-ordinates of the point in a plane to be transformed
 * @param [in] planeParameters - Parameters for the plane ax+by+cz+d=0: (a, b, c, d)
 * @param [out] uvCoordinates - The required UV coordinates of each point as output
 * @param [out] uvAxes - The axes (u, v, n) used in transforming XYZ co-ordinates to UV co-ordinates
 *
 */
void AllXYZToUVCoordinates(
		const vector<Point3f> &xyzCoordinates,
		const vector<float> &planeParameters,
		vector<Point2f> &uvCoordinates,
		vector<Point3f> &uvAxes);

/**
 * @brief - Converting UV 2D co-ordinates to XYZ 3D co-ordinates
 *
 * @param [in] uvCoordinates  UV co-ordinates of the point to be transformed
 * @param [in] uvAxes - The axis (u, v, n) used for transforming XYZ co-ordinates to UV co-ordinates
 * @param [in] d - The fourth parameter for the plane ax+by+cz+d=0
 * @param [out] xyzCoordinates - The required output XYZ co-ordinates
 *
 */
void UVToXYZCoordinates(
		const Point2f &uvCoordinates,
		const vector<Point3f> &uvAxes,
		const float d,
		Point3f &xyzCoordinates
		);

/**
 * @brief - Converting All UV 2D co-ordinates of a plane to XYZ 3D co-ordinates
 *
 * @param [in] uvCoordinates - UV co-ordinates of points in a plane to be transformed
 * @param [in] uvAxes - The axis (u, v, n) used for transforming XYZ co-ordinates to UV co-ordinates
 * @param [in] d - The fourth parameter for the plane ax+by+cz+d=0
 * @param [out] xyzCoordinates - The required output XYZ co-ordinates of the transformed UV point
 *
 */
void AllUVToXYZCoordinates(
		const vector<Point2f> &uvCoordinates,
		const vector<Point3f> &uvAxes,
		const float d,
		vector<Point3f> &xyzCoordinates);


#endif /* CONVERSION_HPP_ */
