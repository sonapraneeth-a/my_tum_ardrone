#ifndef _ALL_HEADERS_H
#define _ALL_HEADERS_H

#include "ros/ros.h"

// OpenCV related headers
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

// C Headers
#include <cmath>
#include <ctime>
#include <cstdlib>
#include <cstdio>

// C++ Headers
#include <vector>
#include <list>
#include <string>
#include <chrono>
#include <string>
#include <fstream>
#include <sstream>
#include <list>
#include <map>

#include "cvd/thread.h"
#include "cvd/image.h"
#include "cvd/byte.h"
#include "cvd/rgb.h"

#include "tum_ardrone/keypoint_coord.h"
#include "tum_ardrone/filter_state.h"



#include "Line2.hpp"
#include "Multiple-Plane-JLinkage/utilities.hpp"

#include "DebugUtility/DebugUtility.hpp"
#include "LogUtility/LogUtility.hpp"

// Namespaces
using namespace std;
using namespace cv;


#endif