/**
Created on February 19th 2015
Author : Anirudh Vemula

This node creates a GUI which would enable the user to control the drone

*/

#include "ros/ros.h"
#include "ControlUINode.h"

unsigned int ros_header_timestamp_base = 0;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "drone_controlUI");

	ROS_INFO("Started Control UI Node.");

	ControlUINode controlUI;

	controlUI.Loop();

	return 0;
}