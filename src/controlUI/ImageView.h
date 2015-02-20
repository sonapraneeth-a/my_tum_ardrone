#pragma once

/**
Created on 21st February 2015
Author: Anirudh Vemula

This file is to create a class which would enable a GL window displaying the
camera feed of the drone and recording mouse clicks
*/


#ifndef _IMAGEVIEW_H
#define _IMAGEVIEW_H

#include "ros/ros.h"
#include "GLWindow2.h"
#include "sensor_msgs/Image.h"
#include "cvd/thread.h"
#include "cvd/image.h"
#include "cvd/byte.h"
#include "MouseKeyHandler.h"
#include "boost/thread.hpp"
#include "std_msgs/String.h"

class ControlUINode;

class ImageView : private CVD::Thread, private MouseKeyHandler {

private:
	//comm with drone
	ros::Subscriber vid_sub;
	std::string video_channel;

	//comm with tum_ardrone
	ros::Subscriber tum_ardrone_sub;
	ros::Publisher tum_ardrone_pub;
	std::string command_channel;

	// ROS
	ros::NodeHandle nh_;


	GLWindow2* myGLWindow;
	CVD::ImageRef desiredWindowSize;
	CVD::ImageRef defaultWindowSize;
	bool changeSizeNextRender;

	// the associated thread's run function
	void run();

	CVD::Image<CVD::byte> mimFrameBW;
	CVD::Image<CVD::byte> mimFrameBW_workingCopy;
	int mimFrameTime;
	int mimFrameTime_workingCopy;
	unsigned int mimFrameSEQ;
	unsigned int mimFrameSEQ_workingCopy;
	ros::Time mimFrameTimeRos;
	ros::Time mimFrameTimeRos_workingCopy;
	int frameWidth, frameHeight;

	int videoFramePing;

	// keep running
	bool keepRunning;
	bool lockNextFrame;

	boost::condition_variable new_frame_signal;
	boost::mutex new_frame_signal_mutex;

	// resets
	void ResetInternal();

	//ControlUINode
	ControlUINode *node;

public:
	ImageView(ControlUINode* node);
	~ImageView();

	void newImage(sensor_msgs::ImageConstPtr img);
	bool newImageAvailable;

	virtual void on_key_down(int key);
	virtual void on_mouse_down(CVD::ImageRef where, int state, int button);

	void startSystem();
	void stopSystem();

	//callback functions
	void vidCb (const sensor_msgs::ImageConstPtr img);
	void comCb (const std_msgs::StringConstPtr str);


	//the main rendering function
	void renderFrame();

};


#endif