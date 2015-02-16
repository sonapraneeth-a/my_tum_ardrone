/**
Author : Anirudh Vemula
Date : 12th Feb 2015
*/

#include "KIAutoScaleInit.h"
#include "../DroneController.h"
#include "../ControlNode.h"
#include "../../HelperFunctions.h"


KIAutoScaleInit::KIAutoScaleInit(bool resetMap, int imoveTimeMS, int iwaitTimeMS, int reachHeightMS, float controlMult, bool takeoff, int numTimes, int distanceMS) {
	stage = NONE;
	this->resetMap = resetMap;
	moveTimeMS = imoveTimeMS;
	waitTimeMS = iwaitTimeMS;
	this->reachHeightMS = reachHeightMS;
	this->controlCommandMultiplier = controlMult;

	nextUp = false;
	stageStarted = false;

	if(!takeoff)
		stage = WAIT_FOR_FIRST;

	char buf[200];
	if(resetMap)
		snprintf(buf,200,"autoScaleInit %d %d", imoveTimeMS, iwaitTimeMS);
	else
		snprintf(buf,200,"takeoff");

	command = buf;

	this->numTimes = numTimes;
	this->distanceMS = distanceMS;

	countTimes = 0;
}

KIAutoScaleInit::~KIAutoScaleInit(void) {

}

bool KIAutoScaleInit::update(const tum_ardrone::filter_stateConstPtr statePtr) {
	// no PTAM initialization, just takeoff.
	if(!resetMap)
	{
		switch(stage)
		{
		case NONE:		// start and proceed
			node->sendTakeoff();
			stageStarted = getMS();
			stage = WAIT_FOR_SECOND;
			node->sendControlToDrone(node->hoverCommand);
			return false;

		case WAIT_FOR_SECOND:
			if(getMS() - stageStarted < 5000)
			{
				node->sendControlToDrone(node->hoverCommand);
				return false;
			}

			controller->setTarget(DronePosition(
					TooN::makeVector(statePtr->x,statePtr->y,statePtr->z),statePtr->yaw));
			node->sendControlToDrone(controller->update(statePtr));
			stage = DONE;
			return true;
		case DONE:
			node->sendControlToDrone(controller->update(statePtr));
			return true;
		default:
			return false;
		}
		return true;	// should never happen....
	}
	else // The actual case
	{
		switch(stage)
		{
		case NONE:		// start and proceed
			node->sendTakeoff();
			node->publishCommand("f reset");	// reset whole filter.

			stageStarted = getMS();
			stage = STARTED;
			nextUp = true;
			node->sendControlToDrone(node->hoverCommand);
			return false;

		case STARTED:	// wait 6s to reach hight.
			if(getMS() - stageStarted > reachHeightMS)
			{
				stageStarted = getMS();
				stage = WAIT_FOR_FIRST;
			}
			node->sendControlToDrone(node->hoverCommand);
			return false;

		case WAIT_FOR_FIRST:	// wait 1s and press space
			if(getMS() - stageStarted > 1000)
			{
				node->publishCommand("p space");
				stageStarted = getMS();
				stage = TOOK_FIRST;
			}
			node->sendControlToDrone(node->hoverCommand);
			return false;

		case TOOK_FIRST:	// go [up/down] 1s and press space if was initializing.
			if(getMS() - stageStarted < moveTimeMS)
			{
				if(nextUp) // controlCommandMultiplier somehow manages the speed in which the drone moves
					node->sendControlToDrone(ControlCommand(0,0,0,0.6*controlCommandMultiplier));
				else
					node->sendControlToDrone(ControlCommand(0,0,0,-0.3*controlCommandMultiplier));
			}
			else if(getMS() - stageStarted < moveTimeMS+waitTimeMS)
			{
				node->sendControlToDrone(node->hoverCommand);
			}
			else	// time is up, take second KF
			{
				if(statePtr->ptamState == statePtr->PTAM_INITIALIZING)	// TODO: ptam status enum, this should be PTAM_INITIALIZING
				{
					node->publishCommand("p space");
					stageStarted = getMS();
					stage = WAIT_FOR_SECOND;
				}
				else	// sth was wrong: try again
				{
					nextUp = !nextUp;
					node->publishCommand("p reset");
					stageStarted = getMS();
					stage = WAIT_FOR_FIRST;
				}
				node->sendControlToDrone(node->hoverCommand);
			}
			return false;

		case WAIT_FOR_SECOND:

			// am i done?
			if(statePtr->ptamState == statePtr->PTAM_BEST || statePtr->ptamState == statePtr->PTAM_GOOD || statePtr->ptamState == statePtr->PTAM_TOOKKF) // TODO: PTAM_GOOD or PTAM_BEST or PTAM_TOOKKF
			{
				//controller->setTarget(DronePosition(
				//					TooN::makeVector(statePtr->x,statePtr->y,statePtr->z),statePtr->yaw));
				//node->sendControlToDrone(controller->update(statePtr));
				//stage = DONE;
				//return true;

				if(nextUp && countTimes!=numTimes && statePtr->scaleAccuracy!=1.0) {
					stage = MOVE_UP;
					stageStarted = getMS();
					node->sendControlToDrone(node->hoverCommand);
					return false;
				}
				else if(!nextUp && countTimes!=numTimes && statePtr->scaleAccuracy!=1.0) {
					stage = MOVE_DOWN;
					stageStarted = getMS();
					node->sendControlToDrone(node->hoverCommand);
					return false;
				}
				else {
					controller->setTarget(DronePosition(
									TooN::makeVector(statePtr->x,statePtr->y,statePtr->z),statePtr->yaw));
					node->sendControlToDrone(controller->update(statePtr));
					stageStarted = getMS();
					stage = DONE;
					return true;
				}
			}

			// am i stil waiting?
			// TODO: change this to something that becomes false, as soon as fail is evident.
			if(getMS() - stageStarted < 2500)	// wait 2s
			{
				node->sendControlToDrone(node->hoverCommand);
				return false;
			}

			// i have failed -> try again.
			nextUp = !nextUp;
			node->publishCommand("p reset");
			stageStarted = getMS();
			stage = WAIT_FOR_FIRST;
			node->sendControlToDrone(node->hoverCommand);
			return false;

		case MOVE_UP:
			if(getMS() - stageStarted < distanceMS) { // Yet to move
				node->sendControlToDrone(ControlCommand(0,0,0,0.6*controlCommandMultiplier));
			}
			else if(getMS() - stageStarted < distanceMS + waitTimeMS) {
				node->sendControlToDrone(node->hoverCommand);
			}
			else { // Reached extremum (top)
				countTimes++;
				if(countTimes==numTimes || statePtr->scaleAccuracy==1.0) { // count elapsed or accuracy reached
					controller->setTarget(DronePosition(
									TooN::makeVector(statePtr->x,statePtr->y,statePtr->z),statePtr->yaw));
					node->sendControlToDrone(controller->update(statePtr));
					stageStarted = getMS();
					stage = DONE;
					return true;
				}
				else { // count remaining and accuracy not reached
					stage = MOVE_DOWN;
					nextUp = !nextUp;
					stageStarted = getMS();
					node->sendControlToDrone(node->hoverCommand);
					return false;
				}
			}
			return false;

		case MOVE_DOWN:
			if(getMS() - stageStarted < distanceMS) { // Yet to move
				node->sendControlToDrone(ControlCommand(0,0,0,-0.6*controlCommandMultiplier));
			}
			else if(getMS() - stageStarted < distanceMS + waitTimeMS) {
				node->sendControlToDrone(node->hoverCommand);
			}
			else { // reached extremum (bottom)
				countTimes++;
				if(countTimes==numTimes || statePtr->scaleAccuracy==1.0) { // count elapsed or accuracy reached
					controller->setTarget(DronePosition(
									TooN::makeVector(statePtr->x,statePtr->y,statePtr->z),statePtr->yaw));
					node->sendControlToDrone(controller->update(statePtr));
					stageStarted = getMS();
					stage = DONE;
					return true;
				}
				else { //count remaining and accuracy not reached
					nextUp = !nextUp;
					stage = MOVE_UP;
					stageStarted = getMS();
					node->sendControlToDrone(node->hoverCommand);
					return false;
				}
			}
			return false;

		case DONE:
			node->sendControlToDrone(controller->update(statePtr));
			return true;


		default:
			return false;


		}
		return false;	// again: should never happen....
	}
}