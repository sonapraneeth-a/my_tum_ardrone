/*****************************************************************************************
 * AlignDrone.cpp
 *
 *       Created on: 21-Feb-2015
 *    Last Modified: 23-Sep-2016
 *  Original Author: Anirudh Vemula
 *   Current Author: Meghshyam Govind Prasad
 *   Current Author: Sona Praneeth Akula
 *          Project: Quadcopter
 *      Description: 
 *
 * Date             Author                          Modification
 * 12-Sep-2016  Sona Praneeth Akula         * Added comments to the code
 * 19-Sep-2016  Sona Praneeth Akula         * Added code for on_key_down(); key c, g
 * 21-Sep-2016  Sona Praneeth Akula         * Added threading to TopView
 * 22-Sep-2016  Sona Praneeth Akula         * Added code to destroy TopView once its job is done
 * 23-Sep-2016  Sona Praneeth Akula         * Added code for testing path planning
 *****************************************************************************************/

#include "AlignDrone.hpp"
#include <cvd/gl_helpers.h>
#include <cv_bridge/cv_bridge.h>
#include "ControlUINode.h"

AlignDrone::AlignDrone(ControlUINode *cnode)
{
    node = cnode;
}

AlignDrone::~AlignDrone()
{

}

void
AlignDrone::startSystem()
{
    cout << "AlignDrone Started\n";
    start();
}

void
AlignDrone::stopSystem()
{
    join();
}

void
AlignDrone::run()
{
    node->alignQuadcopterToCurrentPlane();
    cout << "Stopping AlignDrone\n";
    stopSystem();
    cout << "Stopped AlignDrone\n";
}

