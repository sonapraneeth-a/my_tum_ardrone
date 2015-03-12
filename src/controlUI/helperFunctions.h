#pragma once
/*
	Author : Anirudh Vemula
	Date : 12th March 2015
*/
#ifndef _HELPER_FUNCTIONS_H
#define _HELPER_FUNCTIONS_H

#include <math.h>


inline static int signD(float x) {
	return (x>0)-(x<0);
}

inline static bool onLeft(std::vector<int> p1, std::vector<int> p2, std::vector<int> p) {
	if(signD( (p2[0]-p1[0])*(p[1]-p1[1]) - (p2[1]-p1[1])*(p[0]-p1[0]) ) == 1)
		return true;
	else 
		return false;
}

template<typename K>
inline static int position(std::vector<int> p1, std::vector<int> p2, std::vector<K> p) {
	return signD( (p2[0]-p1[0])*(p[1]-p1[1]) - (p2[1]-p1[1])*(p[0]-p1[0]) );
}

inline static bool onSameSide(std::vector<int> p1, std::vector<int> p2, std::vector<int> vertex, std::vector<float> point) {
	if(position(p1, p2, vertex) * position(p1, p2, point) > 0) {
		return true;
	}
	else
		return false;
}

inline static bool liesInside(std::vector<std::vector<int> > ccPoints, std::vector<float> p) {
	for(int i=0; i<ccPoints.size()-1; i++) {
		if(i==ccPoints.size()-2) {
			if(onSameSide(ccPoints[i], ccPoints[i+1], ccPoints[i-1], p))
				continue;
			else 
				return false;
		}
		else {
			if(onSameSide(ccPoints[i], ccPoints[i+1], ccPoints[i+2], p))
				continue;
			else
				return false;
		}
	}

	return true;
}

#endif
