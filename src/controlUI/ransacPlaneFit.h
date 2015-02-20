#pragma once


#include <mrpt/math/ransac.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/random.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/opengl/CTexturedPlane.h>

#include <vector>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::gui;
using namespace mrpt::math;
using namespace mrpt::random;
using namespace mrpt::poses;
using namespace std;


void  ransac3Dplane_fit(
	const CMatrixDouble  &allData,
	const vector_size_t  &useIndices,
	vector< CMatrixDouble > &fitModels );

void ransac3Dplane_distance(
	const CMatrixDouble &allData,
	const vector< CMatrixDouble > & testModels,
	const double distanceThreshold,
	unsigned int & out_bestModelIndex,
	vector_size_t & out_inlierIndices );

bool ransac3Dplane_degenerate(
	const CMatrixDouble &allData,
	const mrpt::vector_size_t &useIndices );

std::vector<float> ransacPlaneFit(std::vector<std::vector<float> > points, bool verbose);