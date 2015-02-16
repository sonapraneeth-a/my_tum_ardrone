#pragma once

/**
Author : Anirudh Vemula
Date : 12th Feb 2015
*/

#ifndef __KIAUTOSCALEINIT_H
#define __KIAUTOSCALEINIT_H

#include "KIProcedure.h"

class KIAutoScaleInit : public KIProcedure
{
private:
	enum {NONE, STARTED, WAIT_FOR_FIRST, TOOK_FIRST, WAIT_FOR_SECOND, MOVE_UP, MOVE_DOWN, DONE} stage;
	int stageStarted;
	bool nextUp;
	bool resetMap;
	int moveTimeMS;
	int waitTimeMS;
	int reachHeightMS;
	float controlCommandMultiplier;
	int numTimes;
	int distanceMS;

	int countTimes;

public:
	KIAutoScaleInit(bool resetMap = true, int imoveTimeMS = 500, int iwaitTimeMS = 800, int reachHeightMS = 6000, float controlMult = 1.0, bool takeoff = true, int numTimes = 2, int distanceMS = 500);
	~KIAutoScaleInit(void);
	bool update(const tum_ardrone::filter_stateConstPtr statePtr);
};

#endif /* __KIAUTOSCALEINIT_H */