#pragma once
/*****************************************************************************************
 * ImageView.h
 *
 *       Created on: 21-Feb-2015
 *    Last Modified: 20-Sep-2016
 *  Original Author: Anirudh Vemula
 *   Current Author: Meghshyam Govind Prasad
 *   Current Author: Sona Praneeth Akula
 *          Project: Quadcopter
 *      Description: This file is to create a class which would enable a GL window displaying
 *                   the camera feed of the drone and recording mouse clicks
 *
 * Date             Author                          Modification
 * 12-Sep-2016  Sona Praneeth Akula         * Added Comments to the code
 *****************************************************************************************/

#ifndef _ALIGN_DRONE_HPP
#define _ALIGN_DRONE_HPP

#include "boost/thread.hpp"

#include "Helper.h"
#include "VisionHelper.h"

#include "Headers.h"

#include "ControlUINode.h"

class ControlUINode;
/**
 * @brief @todo 
 * @details @todo
 */
class AlignDrone : private CVD::Thread
{

    private:

        //ControlUINode
        ControlUINode *node;

    public:

        bool newImageAvailable; /*!< */

        /**
         * @brief Constructor
         * @details 
         * @param
         * @return
         */
        AlignDrone(ControlUINode* node);

        /**
         * @brief Destructor 
         * @details 
         * @param
         * @return
         */
        ~AlignDrone();

        /**
         * @brief @todo May be this call initiates the drone_control GUI
         * @details 
         * @param
         * @return
         */
        void startSystem();

        /**
         * @brief @todo May be this call terminates/closes the drone_control GUI
         * @details 
         * @param
         * @return
         */
        void stopSystem();

        /**
         * @brief The associated thread's run function
         * @details Creates the 'DRONE CAMERA FEED' Window
         * @param
         * @return
         */
        void run();


};


#endif
