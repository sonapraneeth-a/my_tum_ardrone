#ifndef _ALL_HEADERS_H
#define _ALL_HEADERS_H

#include "ros/ros.h"

#include "tum_ardrone/keypoint_coord.h"
#include "tum_ardrone/filter_state.h"

#include "ransacPlaneFit.h"
#include "allHeaders.hpp"


// OpenCV related stuff
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "ardrone_autonomy/RecordEnable.h"
#include "Multiple-Plane-JLinkage/conversion.hpp"
#include "Multiple-Plane-JLinkage/utilities.hpp"
#include "Multiple-Plane-JLinkage/makeBoundingRects.hpp"
#include "Multiple-Plane-JLinkage/multiplePlanes.hpp"

#include "std_msgs/String.h"
#include "std_msgs/Empty.h"

#include <string>
#include <fstream>
#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <list>
#include <vector>
#include <ctime>
#include <chrono>
#include <list>
#include <map>


#include "HelperFunctions.h"
#include "VisionHelper.h"

#include "Line2.hpp"
#include "AllHeaders.hpp"
#include "TopView.hpp"
#include "DebugUtility.hpp"
#include "LogUtility.hpp"


using namespace std;
using namespace cv;




#endif