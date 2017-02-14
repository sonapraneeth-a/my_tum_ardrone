/*****************************************************************************************
 * ImageView.cpp
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

#include "ImageView.h"
#include <cvd/gl_helpers.h>
//#include <gvars3/instances.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "GLWindow2.h"
#include <iostream>
#include <fstream>
#include <string>
#include "std_msgs/String.h"
#include "../HelperFunctions.h"
#include "ControlUINode.h"

/*typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Polyhedron_3<K> Polyhedron_3;
typedef K::Segment_3 Segment_3;

typedef K::Point_3 Point_3;
typedef CGAL::Creator_uniform_3<double,Point_3> PointCreator;*/
using namespace std;

int plane_num_test = 0;

ImageView::ImageView(ControlUINode *cnode)
{
    printf("[ DEBUG] ImageView Constructor\n");
    frameWidth = frameHeight = 0;

    video_channel = nh_.resolveName("ardrone/front/image_raw"); // Change this for undistorted image
    command_channel = nh_.resolveName("tum_ardrone/com");

    vid_sub = nh_.subscribe(video_channel, 10, &ImageView::vidCb, this);
    tum_ardrone_sub = nh_.subscribe(command_channel, 50, &ImageView::comCb, this);

    tum_ardrone_pub = nh_.advertise<std_msgs::String>(command_channel, 50);

    node = cnode;

    numPointsClicked = 0;
    numKeyPointsDetected = 0;

    considerAllLevels = true;
    renderPoly = false;
    renderRect = false;
    renderSignificantPlane = false;
    renderVisitedPlanes = false;

    numFile = 0;
    translateDistance = 0.5;
    key_board_control = true;
}

ImageView::~ImageView()
{

}

void
ImageView::vidCb (const sensor_msgs::ImageConstPtr img)
{
    newImage(img);
}

void
ImageView::comCb (const std_msgs::StringConstPtr str)
{

}

void
ImageView::startSystem()
{
    keepRunning = true;
    changeSizeNextRender = false;
    start();
}

void
ImageView::stopSystem()
{
    keepRunning = false;
    new_frame_signal.notify_all();
    join();
}

void
ImageView::ResetInternal()
{
    mimFrameBW.resize(CVD::ImageRef(frameWidth, frameHeight));
    mimFrameBW_workingCopy.resize(CVD::ImageRef(frameWidth, frameHeight));
}


void
ImageView::newImage(sensor_msgs::ImageConstPtr img)
{
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::RGB8);

    boost::unique_lock<boost::mutex> lock(new_frame_signal_mutex);

    // Copy to internal image, convert to bw, set flag
    if (ros::Time::now() - img->header.stamp > ros::Duration(30.0))
    {
        mimFrameTimeRos = (ros::Time::now()  - ros::Duration(0.001));
    }
    else
    {
        mimFrameTimeRos = img->header.stamp;
    }

    mimFrameTime = getMS(mimFrameTimeRos);

    mimFrameSEQ = img->header.seq;

    // Copy to mimFrame
    if (mimFrameBW.size().x != (int)img->width || mimFrameBW.size().y != (int)img->height)
        mimFrameBW.resize(CVD::ImageRef(img->width, img->height));

    memcpy(mimFrameBW.data(), cv_ptr->image.data, img->width * img->height * 3);
    newImageAvailable = true;

    lock.unlock();
    new_frame_signal.notify_all();
}


void
ImageView::run()
{
    while(!newImageAvailable)
        usleep(100000);
    newImageAvailable = false;
    while(!newImageAvailable)
        usleep(100000);

    // read image height and width
    frameHeight = mimFrameBW.size().y;
    frameWidth = mimFrameBW.size().x;

    ResetInternal();

    // Create window
    myGLWindow = new GLWindow2(CVD::ImageRef(frameWidth, frameHeight), "DRONE CAMERA FEED", this);
    myGLWindow->set_title("DRONE CAMERA FEED");
    printf("[ DEBUG] new window created\n");

    changeSizeNextRender = true;
    if (frameWidth < 640)
    {
        desiredWindowSize = CVD::ImageRef(frameWidth*2, frameHeight*2);
    }
    else
    {
        desiredWindowSize = CVD::ImageRef(frameWidth, frameHeight);
    }

    boost::unique_lock<boost::mutex> lock(new_frame_signal_mutex);

    while(keepRunning)
    {
        if(newImageAvailable)
        {
            newImageAvailable = false;

            // Copy to working copy
            mimFrameBW_workingCopy.copy_from(mimFrameBW);
            mimFrameTime_workingCopy = mimFrameTime;
            mimFrameSEQ_workingCopy = mimFrameSEQ;
            mimFrameTimeRos_workingCopy = mimFrameTimeRos;

            //release lock
            lock.unlock();

            renderFrame();

            if (changeSizeNextRender)
            {
                myGLWindow->set_size(desiredWindowSize);
                changeSizeNextRender = false;
            }

            lock.lock();
        }
        else
            new_frame_signal.wait(lock);
    }

    lock.unlock();
    delete myGLWindow;

}

void
ImageView::renderFrame()
{

    // Setup
    myGLWindow->SetupViewport();
    myGLWindow->SetupVideoOrtho();
    myGLWindow->SetupVideoRasterPosAndZoom();

    // Render image from drone camera
    glDrawPixels(mimFrameBW_workingCopy);

    glPointSize(6);
    glEnable(GL_BLEND);
    glEnable(GL_POINT_SMOOTH);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glLineWidth(5);

    // Rendering clicked points

    /*glColor3f(1.0,0,0);
    glBegin(GL_POINTS);
    for (int i = 0; i < numPointsClicked; ++i)
    {
        glVertex2i(pointsClicked[i][0], pointsClicked[i][1]);
    }
    glEnd();*/
    
    // Render detected keyPoints
    if(!renderPoly)
    {
        glColor3f(0, 0, 1.0);
        glBegin(GL_POINTS);
        for (int i = 0; i < numKeyPointsDetected; ++i)
        {
            vector<int> p;
            bool found = node->get2DPoint(keyPointsNearest[i], p, considerAllLevels);
            if(found)
            {
                glVertex2i(p[0], p[1]);
            }
            else
            {

            }
        }
        glEnd();
    }
    else if(renderPoly)
    {
        // render convex hull polygon

        /*glColor3f(0, 0, 1.0);
        glBegin(GL_POINTS);
        for (int i = 0; i < numKeyPointsDetected; ++i)
        {
            vector<int> p;
            bool found = node->get2DPoint(keyPointsNearest[i], p, considerAllLevels);
            if(found) {
                glVertex2i(p[0], p[1]);
            }
            else {

            }
        }
        glEnd();*/


        glColor3f(0, 1.0, 0.0);
        glBegin(GL_LINE_STRIP);
        for(unsigned int i=0; i<ccPoints.size(); i++)
        {
            vector<int> p;
            bool found = node->get2DPoint(keyPointsNearest[ccPoints[i]], p, considerAllLevels);
            if(found)
            {
                glVertex2i(p[0], p[1]);
            }
            else
            {

            }
        }
        glEnd();
    }
    if(renderRect)
    {
        unsigned int size = continuousBoundingBoxPoints.size();
        for(unsigned int planeIndex = 0; planeIndex<size; planeIndex++)
        {
            vector<Point3f> planeBoundingBoxPoints = continuousBoundingBoxPoints[planeIndex];
            vector< vector<int> > planePts2D;
            //glColor3f(0, 1.0, 0.0);
            glColor3f(1.0-1.0/(planeIndex+1), 0, 1.0/(planeIndex+1)); 
            glBegin(GL_LINE_STRIP);
            /*  
            for(unsigned int pointIndex = 0; pointIndex<planeBoundingBoxPoints.size(); pointIndex++) {
                vector<float> pt(3);
                pt[0] = planeBoundingBoxPoints[pointIndex].x;
                pt[1] = planeBoundingBoxPoints[pointIndex].y;
                pt[2] = planeBoundingBoxPoints[pointIndex].z;
                vector<int> p;
                bool found = node->get2DPoint(pt, p, true);
                if (found){
                    planePts2D.push_back(p);
                    glVertex2i(p[0],p[1]);
                }
            }
            */
            vector<Point2f> imagePts;
            node->project3DPointsOnImage(planeBoundingBoxPoints, imagePts);
            for(unsigned int pointIndex = 0; pointIndex<imagePts.size(); pointIndex++)
            {
                vector<int> p(2);
                p[0] = imagePts[pointIndex].x;
                p[1] = imagePts[pointIndex].y;
                glVertex2i(p[0],p[1]);
            }
            glEnd();
        }
    }
    if(renderSignificantPlane)
    {
        glColor3f(1.0, 1.0, 1.0); 
        glBegin(GL_LINE_STRIP);
        vector<Point2f> imagePts;
        node->project3DPointsOnImage(sigPlaneBoundingBoxPoints, imagePts);
        for(unsigned int pointIndex = 0; pointIndex < imagePts.size(); pointIndex++)
        {
            vector<int> p(2);
            p[0] = imagePts[pointIndex].x;
            p[1] = imagePts[pointIndex].y;
            glVertex2i(p[0], p[1]);
        }
        glEnd();
    }
    if(renderVisitedPlanes)
    {
        unsigned int size = visitedBoundingBoxPoints.size();
        for(unsigned int planeIndex = 0; planeIndex < size; planeIndex++)
        {
            vector<Point3f> planeBoundingBoxPoints = visitedBoundingBoxPoints[planeIndex];
            vector< vector<int> > planePts2D;
            glColor3f(0.0, 0.0, 0.0);
            glBegin(GL_LINE_STRIP);
            vector<Point2f> imagePts;
            node->project3DPointsOnImage(planeBoundingBoxPoints, imagePts);
            for(unsigned int pointIndex = 0; pointIndex<imagePts.size(); pointIndex++)
            {
                vector<int> p(2);
                p[0] = imagePts[pointIndex].x;
                p[1] = imagePts[pointIndex].y;
                glVertex2i(p[0],p[1]);
            }
        
            glEnd();
        }
    }
    glDisable(GL_BLEND);

    myGLWindow->swap_buffers();
    myGLWindow->HandlePendingEvents();
}

/**
 * @brief Available functionality for various key press
 * @details
 *          Key a - [Used stage 01] Just for testing
 *          Key b - Extracts bounding rect
 *          Key c - [Used stage 01] Collect the bounding box points and plane parameters for all the planes
 *          Key d - Clear the vectors 'pointsClicked' and 'keyPointsNearest'
 *          Key e - Extract Multiple planes using Jlinkage and store the bounding box points for the planess
 *          Key f - [Used stage 01] For testing
 *          Key g - Navigating the planes using the planned path and collecting the video footage
 *          Key n - [Used stage 01] Uses the points written from a file (obtained from key c) and does the functionality of Key g
 *          Key q - Quit the keyboard control
 *          Key r - [Used stage 01] Reset all the vectors and rendering
 *          Key s - Saves the keypoint information '_3d_points' to file named by 'numFile'
 *          Key t - Extracts bounding polygon
 *          Key space - [NOT IMPLEMENTED]
 *          Key + - [NOT IMPLEMENTED]
 *          Key - - [NOT IMPLEMENTED]
 *          Key 0-9 - [Used stage 01] Testing
 *          Key X - [Used in stage 01] Overall testing - Start Capturing
 *          Key Q - [Used in stage 01] Overall testing - Align the quadcopter to the current plane
 * @param [int] key - ASCII value of key
 */
void
ImageView::on_key_down(int key)
{
    cout << "[ DEBUG] [on_key_down] Key pressed: " << key << ", " << (char)key << "\n";
    // Key b - Extracts bounding rect
    if(key == 'b')
    {
        // renders the bounding rectangle
        // renderPoly = false;
        // renderRect = !renderRect;
        extractBoundingRect();
    }
    // Key d - Clear the vectors 'pointsClicked' and 'keyPointsNearest'
    else if(key == 'd')
    {
        // node->publishCommand("i delete");
        // Need to delete the last point
        if(numPointsClicked != 0)
        {
            numPointsClicked--;
            pointsClicked.pop_back();
        }
        if(numKeyPointsDetected != 0)
        {
            numKeyPointsDetected--;
            keyPointsNearest.pop_back();
        }
    }
    // Key e - Extract Multiple planes using Jlinkage and store the bounding box points for the planess
    else if(key == 'e')
    {
        /*
         * TODO Currently commented code which fits single plane and moves the drone accordingly
         * Appropriate code needs to be added which will handle multiple planes
         * For time being for multiple planes each plane is shown in different color
         *
            vector<float> plane = node->fitPlane3d (ccPoints, pointsClicked);
            vector<vector<float> > pPoints =  node->projectPoints (ccPoints, keyPointsNearest);
            grid g = node->buildGrid(pPoints);
            vector<vector<double> > tPoints =  node->getTargetPoints(g, plane);
            node->moveDrone(tPoints);
        */
        // Extract multiple planes
        extractBoundingPoly();
        renderRect = true;
        unsigned int size = continuousBoundingBoxPoints.size();
        assert(size == planeParameters.size());
        unsigned int i,j;
        for (i = 0; i < size; ++i)
        {
            continuousBoundingBoxPoints[i].clear();
            planeParameters[i].clear();
        }
        continuousBoundingBoxPoints.clear();
        planeParameters.clear();
        // Calls JLinkage
        node->fitMultiplePlanes3d(ccPoints, pointsClicked, planeParameters, continuousBoundingBoxPoints);
        cout << "[ DEBUG ] continuousBoundingBoxPoints from ImageView\n";
        size = continuousBoundingBoxPoints.size();
        for (i = 0; i < size; ++i)
        {
            for (j = 0; j < 5; ++j)
            {
                cout << continuousBoundingBoxPoints[i][j] << " ";
            }
            cout << "\n";
        }
        //vector<float> translatedPlane = node->translatePlane (translateDistance);
    }
    // Key g - Navigating the planes using the planned path and collecting the video footage
    else if(key == 'g')
    {
        // Cover multiple planes
        if(renderRect)
        {
            renderRect = false;  //While moving the quadcopter we don't want bounding box to appear
            node->moveQuadcopter(planeParameters, continuousBoundingBoxPoints);
        }
    }
    // Key r - Reset all the vectors and rendering
    else if(key == 'r')
    {
        // node->publishCommand("i reset");
        // Need to reset all the entire points to initial state
        numPointsClicked = 0;
        numKeyPointsDetected = 0;
        pointsClicked.clear();
        keyPointsNearest.clear();
        renderPoly = false;
        renderRect= false;
        renderSignificantPlane = false;
    }
    // Key s - Saves the keypoint information '_3d_points' to file named by 'numFile'
    else if(key == 's')
    {
        // save all the keypoint information
        node->saveKeyPointInformation(numFile);
        numFile++;
    }
    //Key  t - Extracts bounding polygon
    else if(key == 't')
    {
        // renders the polygon
        //renderRect = false;
        renderPoly = !renderPoly;
        extractBoundingPoly();
    }
    // Key space
    else if(key == 32)
    {
        // node->publishCommand("i run");
        // Send control commands to drone. TODO need to implement
    }
    /* Below are the keys to be implemented properly fro mtp stage 01 */
    /** PRANEETH's CODE **/
    // Key a - Just for testing
    else if(key == 'a')
    {
        int min_height_of_plane = 2.0;
        int max_height_of_plane = 5.0;
        float min_distance = getDistanceToSeePlane(min_height_of_plane);
        float max_distance = getDistanceToSeePlane(max_height_of_plane);
        vector<double> curr_pos_of_drone;
        curr_pos_of_drone.clear();
        /* node->getCurrentPositionOfDrone(curr_pos_of_drone);
        cout << "Max dist. allowed: " << max_distance << ", Min dist. allowed: " << min_distance << "\n";
        cout << "Current Quadcopter Position: (" << curr_pos_of_drone[0] 
                << ", " << curr_pos_of_drone[1] << ", " << curr_pos_of_drone[1]
                << ", " << curr_pos_of_drone[3]  << ")\n";*/
        extractBoundingPoly();
        renderRect = true;
        unsigned int size;
        size = continuousBoundingBoxPoints.size();
        for (unsigned int i = 0; i < size; ++i)
        {
            continuousBoundingBoxPoints[i].clear();
        }
        size = planeParameters.size();
        for (unsigned int i = 0; i < size; ++i)
        {
            planeParameters[i].clear();
        }
        continuousBoundingBoxPoints.clear();
        planeParameters.clear();
        // Calls JLinkage
        node->fitMultiplePlanes3d(ccPoints, pointsClicked, planeParameters, continuousBoundingBoxPoints);
        string filename = "Plane_Info.txt";
        cout << "[ DEBUG] Writing info of plane " << plane_num_test << " to file " << filename << "\n";
        plane_num_test++;
        vector<Point3f> bounding_box_points;
        vector<float> plane_parameters;
        bounding_box_points.clear();
        plane_parameters.clear();
        for (unsigned int i = 0; i < continuousBoundingBoxPoints[0].size(); ++i)
        {
            bounding_box_points.push_back(continuousBoundingBoxPoints[0][i]);
        }
        for (unsigned int i = 0; i < planeParameters[0].size(); ++i)
        {
            plane_parameters.push_back(planeParameters[0][i]);
        }
        WriteInfoToFile(bounding_box_points, plane_parameters, plane_num_test, filename);
    }
    // Key c - Collect the bounding box points and plane parameters for all the planes
    else if(key == 'c')
    {
        /* Launches GUI: Return approx. angles and orientations */
        /* Angle with which quadcopter has to rotate to orient itself to the new plane */
        std::vector< double > main_angles;
        /* Direction in which quadcopter should rotate to orient its yaw with normal of new plane */
        std::vector< RotateDirection > main_directions;
        /* Initiating the GUI (Runs in a thread) */
        TopView *top = new TopView();
        top->startSystem();
        /* GUI has been closed here */
        cout << "[Key 'c'] Calling the GUI for drawing top view sketch of the surface\n";
        while(!(top->getExitStatus()))
        {}
        /* Get the directions, angles and number of planes */
        top->getDirections(main_directions);
        top->getAngles(main_angles);
        /* Printing the information to the terminal */
        int number_of_planes = top->getNumberOfPlanes();
        int type_of_surface = top->getTypeOfSurface();
        int max_height_of_plane = top->getMaxHeightOfPlane();
        int view_dir = top->getViewingDirection();
        if(number_of_planes == 0)
        {
            cout << "[ ERROR] [Key 'c'] No diagram drawn\n";
            cout << "[ ERROR] [Key 'c'] So, landing the quadcopter\n";
            node->sendLand();
            cout << "[ INFO] [Key 'c'] Quadcopter has landed.\n";
        }
        else
        {
            cout << "[Key 'c'] Number of planes: " << number_of_planes << "\n";
            cout << "[Key 'c'] Max Height of the plane (as estimated by the user): " << max_height_of_plane << "\n";
            if(type_of_surface == 0)
            {
                cout << "[Key 'c'] Surface Drawn: " << "Open Surface" << "\n";
                if(view_dir == 0)
                {
                    cout << "[Key 'c'] Viewing the surface from front\n";
                }
                if(view_dir == 1)
                {
                    cout << "[Key 'c'] Viewing the surface from back\n";
                }
            }
            if(top->getTypeOfSurface() == 1)
            {
                cout << "[Key 'c'] Surface Drawn: " << "Closed Surface" << "\n";
                if(view_dir == 2)
                {
                    cout << "[Key 'c'] Viewing the surface from outside the structure\n";
                }
                if(view_dir == 3)
                {
                    cout << "[Key 'c'] Viewing the surface from inside the structure\n";
                }
            }
            top->destroy();
            cout << "[Key 'c'] Angles: ";
            for (unsigned int i = 0; i < main_angles.size(); ++i)
            {
                cout << main_angles[i] << " ";
            }
            cout << "\n";
            cout << "[Key 'c'] Directions: ";
            for (unsigned int i = 0; i < main_directions.size(); ++i)
            {
                if(main_directions[i] == 0)
                    cout << "CLOCKWISE" << " ";
                else if(main_directions[i] == 1)
                    cout << "ANTI-CLOCKWISE" << " ";
            }
            cout << "\n";
            int min_height_of_plane = 2.0;
            
            float min_distance = getDistanceToSeePlane(min_height_of_plane);
            float max_distance = getDistanceToSeePlane(max_height_of_plane);
            cout << "[Key 'c'] Min. Distance: " << min_distance << ", Max. Distance: " << max_distance << "\n";
            node->setValues(number_of_planes, min_height_of_plane, min_distance, max_height_of_plane, max_distance);
            node->setMainAngles(main_angles);
            node->setMainDirections(main_directions);
            node->alignQuadcopterToCurrentPlane();
            
        }
    }
    // Key 0-9 - For testing
    else if(key >= '0' && key <= '9')
    {
        cout << "Pressed Key 1\n";
        int num = key-'0';
        node->testUtility(num);
    }
    else if(key == 'Z')
    {
        int min_height_of_plane = 2;
        int max_height_of_plane = 4;
        float min_distance = getDistanceToSeePlane(min_height_of_plane);
        float max_distance = getDistanceToSeePlane(max_height_of_plane);
        int number_of_planes = 1;
        /* Angle with which quadcopter has to rotate to orient itself to the new plane */
        std::vector< double > main_angles;
        /* Direction in which quadcopter should rotate to orient its yaw with normal of new plane */
        std::vector< RotateDirection > main_directions;
        main_angles.clear();
        main_directions.clear();
        cout << "[ DEBUG] [on_key_down] Setting the values\n";
        node->setValues(number_of_planes, min_height_of_plane, min_distance, max_height_of_plane, max_distance);
        node->setMainAngles(main_angles);
        node->setMainDirections(main_directions);
        node->alignQuadcopterToCurrentPlane();
    }
    // X
    else if(key == 'X')
    {
        if(numPointsClicked == 4)
        {
            /*int min_height_of_plane = 2;
            int max_height_of_plane = 4;
            float min_distance = getDistanceToSeePlane(min_height_of_plane);
            float max_distance = getDistanceToSeePlane(max_height_of_plane);
            node->setValues(1, min_height_of_plane, min_distance, max_height_of_plane, max_distance);*/
            cout << "[ DEBUG] [on_key_down] pointsClicked are:\n";
            for(unsigned int i = 0; i < pointsClicked.size(); i++)
            {
                cout << "[" << pointsClicked[i][0] << ", " << pointsClicked[i][1] << "]\n";
            }
            cout << "\n";
            cout << "[ DEBUG] [on_key_down] Capturing the current plane\n";
            node->captureTheCurrentPlane();
            // Clear all Vectors
            clearInputVectors();
            numPointsClicked = 0;
        }
        else
        {
            cout << "[ DEBUG] [on_key_down] Please click 4 points on the screen and press press key 'p' again\n";
        }
    }
    else if(key == 'Q')
    {
        int min_height_of_plane = 3;
        int max_height_of_plane = 5;
        float min_distance = getDistanceToSeePlane(min_height_of_plane);
        float max_distance = getDistanceToSeePlane(max_height_of_plane);
        int number_of_planes = 2;
        RotateDirection dir = COUNTERCLOCKWISE;
        double angle = 30.0;
        /* Angle with which quadcopter has to rotate to orient itself to the new plane */
        std::vector< double > main_angles;
        /* Direction in which quadcopter should rotate to orient its yaw with normal of new plane */
        std::vector< RotateDirection > main_directions;
        main_angles.clear();
        main_directions.clear();
        main_angles.push_back(angle);
        main_directions.push_back(dir);
        cout << "[ DEBUG] [on_key_down] Setting the values\n";
        node->setValues(number_of_planes, min_height_of_plane, min_distance, max_height_of_plane, max_distance);
        node->setMainAngles(main_angles);
        node->setMainDirections(main_directions);
        node->alignQuadcopterToCurrentPlane();
    }
    // W
    else if(key == 'W')
    {
        if(numPointsClicked == 4)
        {
            /*int min_height_of_plane = 2;
            int max_height_of_plane = 4;
            float min_distance = getDistanceToSeePlane(min_height_of_plane);
            float max_distance = getDistanceToSeePlane(max_height_of_plane);
            node->setValues(1, min_height_of_plane, min_distance, max_height_of_plane, max_distance);*/
            cout << "[ DEBUG] [on_key_down] pointsClicked are:\n";
            for(unsigned int i = 0; i < pointsClicked.size(); i++)
            {
                cout << "[" << pointsClicked[i][0] << ", " << pointsClicked[i][1] << "]\n";
            }
            cout << "\n";
            cout << "[ DEBUG] [on_key_down] Capturing the current plane\n";
            node->captureTheCurrentPlane();
            // Clear all Vectors
            clearInputVectors();
            numPointsClicked = 0;
        }
        else
        {
            cout << "[ DEBUG] [on_key_down] Please click 4 points on the screen and press press key 'p' again\n";
        }
    }
    else if(key == 'E')
    {
        int min_height_of_plane = 1;
        int max_height_of_plane = 2;
        float min_distance = getDistanceToSeePlane(min_height_of_plane);
        float max_distance = getDistanceToSeePlane(max_height_of_plane);
        int number_of_planes = 2;
        RotateDirection dir = CLOCKWISE;
        double angle = 15.0;
        /* Angle with which quadcopter has to rotate to orient itself to the new plane */
        std::vector< double > main_angles;
        /* Direction in which quadcopter should rotate to orient its yaw with normal of new plane */
        std::vector< RotateDirection > main_directions;
        main_angles.clear();
        main_directions.clear();
        main_angles.push_back(angle);
        main_directions.push_back(dir);
        cout << "[ DEBUG] [on_key_down] Setting the values\n";
        node->setValues(number_of_planes, min_height_of_plane, min_distance, max_height_of_plane, max_distance);
        node->setMainAngles(main_angles);
        node->setMainDirections(main_directions);
        node->alignQuadcopterToCurrentPlane();
    }
    // Key n - Uses the points written from a file (obtained from key c)
    else if(key == 'n')
    {
        string inputDirectory = "/home/sonapraneeth/";
        string filename = inputDirectory + "Plane_Info.txt";
        vector< vector<float> > sortedPlaneParameters;
        vector< vector<Point3f> > boundingBoxPoints;
        readInfo(filename, sortedPlaneParameters, boundingBoxPoints);
        cout << "Plane Parameters\n";
        for (unsigned int i = 0; i < sortedPlaneParameters.size(); ++i)
        {
            unsigned int param = sortedPlaneParameters[i].size();
            for (unsigned int j = 0; j < param; ++j)
            {
                cout << sortedPlaneParameters[i][j] << " ";
            }
            cout << "\n";
        }
        cout << "Bounding Box Points\n";
        for (unsigned int i = 0; i < boundingBoxPoints.size(); ++i)
        {
            unsigned int param = boundingBoxPoints[i].size();
            for (unsigned int j = 0; j < param; ++j)
            {
                cout << boundingBoxPoints[i][j] << "\n";
            }
            cout << "\n";
        }
        // Get the bounding box points
        getContinuousBoundingBox ( boundingBoxPoints, sortedPlaneParameters, continuousBoundingBoxPoints);
        cout << "Continuous Bounding Box Points\n";
        for (unsigned int i = 0; i < continuousBoundingBoxPoints.size(); ++i)
        {
            unsigned int param = continuousBoundingBoxPoints[i].size();
            for (unsigned int j = 0; j < param; ++j)
            {
                cout << continuousBoundingBoxPoints[i][j] << "\n";
            }
            cout << "\n";
        }
        // Path planning: Cover multiple planes
        if(renderRect)
        {
            renderRect = false;  // While moving the quadcopter we don't want bounding box to appear
            renderSignificantPlane = false;
        }
        cout << "[ DEBUG] Calling moveQuadcopter()\n";
        node->moveQuadcopter(sortedPlaneParameters, continuousBoundingBoxPoints);
    }
    /* Movement letter keys */
    else if(key == 'F')
    {
        node->moveForward();
    }
    else if(key == 'B')
    {
        node->moveBackward();
    }
    else if(key == 'L')
    {
        node->moveLeft();
    }
    else if(key == 'R')
    {
        node->moveRight();
    }
    else if(key == 'U')
    {
        node->moveUp();
    }
    else if(key == 'D')
    {
        node->moveDown();
    }
    else if(key == 'C')
    {
        node->rotateClockwise();
    }
    else if(key == 'A')
    {
        node->rotateCounterClockwise();
    }
}

void
ImageView::on_mouse_down(CVD::ImageRef where, int state, int button)
{
    //double x = 4*(where.x/(double)this->myGLWindow->size().x - 0.5);
    //double y = -4*(where.y/(double)this->myGLWindow->size().y - 0.5);

    int x;
    int y;

    if(this->myGLWindow->size().x==640)
    {
        x = where.x;
    }
    else if(this->myGLWindow->size().x!=0)
    {
        x = (int)((640.0*where.x)/(this->myGLWindow->size().x));
    }

    if(this->myGLWindow->size().y==360)
    {
        y = where.y;
    }
    else if(this->myGLWindow->size().y!=0)
    {
        y = (int)((360.0*where.y)/(this->myGLWindow->size().y));
    }

    printf("X and Y of point clicked : (%d, %d)\n", x, y);

    numPointsClicked++;
    vector<int> v;
    v.clear();
    v.push_back(x);
    v.push_back(y);
    pointsClicked.push_back(v);
    search(v);
    // if(numPointsClicked == 4)
    // {
    //  numPointsClicked = 0;
    //  cout << "[ DEBUG] pointsClicked are:\n";
    //  for(unsigned int i = 0; i < pointsClicked.size(); i++)
    //  {
    //      cout << "[" << pointsClicked[i][0] << ", " << pointsClicked[i][1] << "]\n";
    //  }
    //  cout << "\n";
    //  cout << "[ DEBUG] [on_mouse_down] Capturing the current plane\n";
    //  node->captureTheCurrentPlane();
    //  // Clear all Vectors
    //  clearInputVectors();
    // }
}



void
ImageView::search(vector<int> pt)
{
    vector<float> kp = node->searchNearest(pt, considerAllLevels);

    keyPointsNearest.push_back(kp);
    numKeyPointsDetected++;
    printf("X, Y and Z of nearest keypoint : (%f, %f, %f)\n", kp[0], kp[1], kp[2]);
}

void
ImageView::extractBoundingPoly()
{
    // Check if the number of clicked points is exactly 4
    //assert(pointsClicked.size()==4);
    
    //vector<vector<int> > ccPoints;
    ccPoints.clear();

    vector<int> minXPoint;
    float minX = -1;
    int minXIndex = -1;
    for(unsigned int i=0; i<pointsClicked.size(); i++)
    {
        if(minX==-1)
        {
            minX = pointsClicked[i][0];
            minXPoint = pointsClicked[i];
            minXIndex = i;
        }
        else if(pointsClicked[i][0]<minX)
        {
            minX = pointsClicked[i][0];
            minXPoint = pointsClicked[i];
            minXIndex = i;
        }
    }

    //ccPoints.push_back(minXPoint);

    vector<int> endPoint;
    int endIndex;
    int i=0;
    do
    {
        ccPoints.push_back(minXIndex);
        endPoint = pointsClicked[0];
        endIndex = 0;
        for(unsigned int j=1;j<pointsClicked.size();j++)
        {
            if((endPoint[0]==minXPoint[0] && 
                endPoint[1]==minXPoint[1]) 
                || 
                onLeft(pointsClicked[j], minXPoint, endPoint))
            {
                endPoint = pointsClicked[j];
                endIndex = j;
            }
        }
        i = i+1;
        minXPoint = endPoint;
        minXIndex = endIndex;

    } while(endPoint[0]!=pointsClicked[ccPoints[0]][0] 
        || endPoint[1]!=pointsClicked[ccPoints[0]][1]);
    ccPoints.push_back(endIndex);

}


void
ImageView::extractBoundingRect()
{

    bPoints.clear();

    int minX = pointsClicked[0][0], minY = pointsClicked[0][1];
    int maxX = pointsClicked[0][0], maxY = pointsClicked[0][1];
    int minXIndex = 0, minYIndex =0, maxXIndex = 0, maxYIndex = 0;

    for(unsigned int i=0; i<pointsClicked.size(); i++)
    {
        if(pointsClicked[i][0] < minX)
        {
            minX = pointsClicked[i][0];
            minXIndex = i;
        }
        if(pointsClicked[i][0] > maxX)
        {
            maxX = pointsClicked[i][0];
            maxXIndex = i;
        }
        if(pointsClicked[i][1] < minY)
        {
            minY = pointsClicked[i][1];
            minYIndex = i;
        }
        if(pointsClicked[i][1] > maxY)
        {
            maxY = pointsClicked[i][1];
            maxYIndex = i;
        }
    }

    bPoints.push_back(minXIndex);
    bPoints.push_back(minYIndex);
    bPoints.push_back(maxXIndex);
    bPoints.push_back(maxYIndex);
}

/*** PRANEETH's CODE ***/

/**
 * @brief Read information about various file froma   file
 * @details 
 * @param [in] [string] filename - Name of the file (along with directory) from which the information is to read
 * @param [out] [vector< vector<float> >] planeParameters - Plane parameters of all planes
 * @param [out] [vector< vector<Point3f> >] boundingBoxPoints - Corresponding boudning box points
 */
void
ImageView::readInfo(string filename,
                    vector< vector<float> > &planeParameters,
                    vector< vector<Point3f> > &boundingBoxPoints)
{
    unsigned int size;
    size = planeParameters.size();
    for (unsigned int i = 0; i < size; ++i)
    {
        planeParameters[i].clear();
    }
    planeParameters.clear();
    size = boundingBoxPoints.size();
    for (unsigned int i = 0; i < size; ++i)
    {
        boundingBoxPoints[i].clear();
    }
    boundingBoxPoints.clear();
    ifstream plane_info;
    plane_info.open(filename.c_str(), std::ifstream::in);
    string line;
    vector<Point3f> box_points;
    Point3f location;
    vector<float> copyVector;
    string plane_string("Plane-Parameters");
    string box_string("Bounding-Box-Points");
    if (plane_info.is_open())
    {
        while ( getline (plane_info, line) )
        {
            /*cout << "Line: " << line << "\n";
            cout << "1-Compare: " << line.compare(plane_string) << "\n";
            cout << "2-Compare: " << line.compare(box_string) << "\n";*/
            if(!line.empty() && line.compare(plane_string)==0)
            {
                getline(plane_info, line);
                split(line, copyVector);
                planeParameters.push_back(copyVector);
                copyVector.clear();
            }
            if(!line.empty() && line.compare(box_string)==0)
            {
                for (int i = 1; i <= 4; ++i)
                {
                    getline(plane_info, line);
                    split(line, copyVector);
                    Point3f point;
                    point.x = copyVector[0];
                    point.y = copyVector[1];
                    point.z = copyVector[2];
                    box_points.push_back(point);
                    copyVector.clear();
                }
                boundingBoxPoints.push_back(box_points);
                box_points.clear();
            }
        }
    }
    else
    {
        cerr << "[ DEBUG] File " << filename << " doesn't open\n";
        return ;
    }
    plane_info.close();
}

void
ImageView::split(   const string &s,
                    vector<float> &elems)
{
    istringstream ss(s);
    while( ! ss.eof() )
    {
        float tmp_f;
        if ( ss >> tmp_f )
        {
            elems.push_back(tmp_f);
        }
    }
}

/**
 * @brief Write the plane parameters and bounding box points of all planes to file
 * @details See points_00.csv for details of the written file
 * @param [vector<Point3f>] bounding_box_points - Bounding box points of the plane
 * @param [vector<float>] plane_parameters - (a, b, c, d) of the plane
 * @param [int] plane_num - Plane number whose points are written to file
 * @param [string] file_name - Name of the file to which the points are written
 */
void
ImageView::WriteInfoToFile(const vector<Point3f> &bounding_box_points, 
            const vector<float> &plane_parameters, 
            int plane_num, string filename)
{
    unsigned int num_bounding_box_points = bounding_box_points.size();
    const char* outFilename = filename.c_str();
    ofstream outFile;
    // Open the object in writing and appending mode
    if(plane_num == 1)
        outFile.open(outFilename, ios::out);
    else
        outFile.open(outFilename, ios::app);
    // Check if the file is open
    if (!outFile.is_open())
    {
        cerr << "\nFile " << filename << " cannot be opened for writing.\n";
        cerr << "Please check if the file is existing and has required permissions ";
        cerr << " for writing.\n";
    }
    outFile << "Plane " << std::setfill ('0') << std::setw (2) << to_string(plane_num) << "\n";
    outFile << "Bounding-Box-Points\n";
    for (unsigned int i = 0; i < num_bounding_box_points; ++i)
    {
        outFile << bounding_box_points[i].x << " "
            << bounding_box_points[i].y << " "
            << bounding_box_points[i].z << "\n";
    }
    outFile << "Plane-Parameters\n";
    for (unsigned int i = 0; i < plane_parameters.size(); ++i)
    {
        if(i != plane_parameters.size()-1)
            outFile << plane_parameters[i] << " ";
        else
            outFile << plane_parameters[i] << "\n";
    }
    outFile << "\n";
    // Close the file
    outFile.close();
    return ;
}

void
ImageView::split(   const string &s,
                    char delim,
                    vector<float> &elems)
{
    stringstream ss(s);
    string item;
    while (getline(ss, item, delim))
    {
        elems.push_back(stof(item));
    }
}

void
ImageView::getContinuousBoundingBoxPoints(vector< vector<Point3f> > &continuous_bounding_box_points)
{
    unsigned int size = continuous_bounding_box_points.size();
    for (unsigned int i = 0; i < size; ++i)
    {
        continuous_bounding_box_points[i].clear();
    }
    continuous_bounding_box_points.clear();
    vector<Point3f> dummy_points;
    for (unsigned int i = 0; i < continuousBoundingBoxPoints.size(); ++i)
    {
        dummy_points.clear();
        for (unsigned int j = 0; j < continuousBoundingBoxPoints[i].size(); ++j)
        {
            dummy_points.push_back(continuousBoundingBoxPoints[i][j]);
        }
        continuous_bounding_box_points.push_back(dummy_points);
    }
}

void
ImageView::setContinuousBoundingBoxPoints(const vector< vector<Point3f> > &continuous_bounding_box_points)
{
    unsigned int size = continuousBoundingBoxPoints.size();
    for (unsigned int i = 0; i < size; ++i)
    {
        continuousBoundingBoxPoints[i].clear();
    }
    continuousBoundingBoxPoints.clear();
    vector<Point3f> dummy_points;
    for (unsigned int i = 0; i < continuous_bounding_box_points.size(); ++i)
    {
        dummy_points.clear();
        for (unsigned int j = 0; j < continuous_bounding_box_points[i].size(); ++j)
        {
            dummy_points.push_back(continuous_bounding_box_points[i][j]);
        }
        continuousBoundingBoxPoints.push_back(dummy_points);
    }
}

void
ImageView::setVisitedBoundingBoxPoints(const vector< vector<Point3f> > &visit_bound_box_points)
{
    unsigned int size = visitedBoundingBoxPoints.size();
    for (unsigned int i = 0; i < size; ++i)
    {
        visitedBoundingBoxPoints[i].clear();
    }
    visitedBoundingBoxPoints.clear();
    vector<Point3f> dummy_points;
    for (unsigned int i = 0; i < visit_bound_box_points.size(); ++i)
    {
        dummy_points.clear();
        for (unsigned int j = 0; j < visit_bound_box_points[i].size(); ++j)
        {
            dummy_points.push_back(visit_bound_box_points[i][j]);
        }
        continuousBoundingBoxPoints.push_back(dummy_points);
    }
}

void
ImageView::getPlaneParameters(vector< vector<float> > &plane_parameters)
{
    unsigned int size = plane_parameters.size();
    for (unsigned int i = 0; i < size; ++i)
    {
        plane_parameters[i].clear();
    }
    plane_parameters.clear();
    vector<float> dummy_points;
    for (unsigned int i = 0; i < planeParameters.size(); ++i)
    {
        dummy_points.clear();
        for (unsigned int j = 0; j < planeParameters[i].size(); ++j)
        {
            dummy_points.push_back(planeParameters[i][j]);
        }
        plane_parameters.push_back(dummy_points);
    }
}

void
ImageView::getPointsClicked(vector< vector<int> > &points_clicked)
{
    unsigned int size = points_clicked.size();
    for (unsigned int i = 0; i < size; ++i)
    {
        points_clicked[i].clear();
    }
    points_clicked.clear();
    vector<int> dummy_points;
    cout << "[ DEBUG] [getPointsClicked] Adding " << pointsClicked.size() << " points\n";
    for (unsigned int i = 0; i < pointsClicked.size(); ++i)
    {
        dummy_points.clear();
        for (unsigned int j = 0; j < pointsClicked[i].size(); ++j)
        {
            dummy_points.push_back(pointsClicked[i][j]);
        }
        points_clicked.push_back(dummy_points);
    }
    cout << "[ DEBUG] [getPointsClicked] Added " << points_clicked.size() << " points\n";
}

void
ImageView::getKeyPointsNearest(vector< vector<int> > &key_points_nearest)
{
    unsigned int size = key_points_nearest.size();
    for (unsigned int i = 0; i < size; ++i)
    {
        key_points_nearest[i].clear();
    }
    key_points_nearest.clear();
    vector<int> dummy_points;
    for (unsigned int i = 0; i < keyPointsNearest.size(); ++i)
    {
        dummy_points.clear();
        for (unsigned int j = 0; j < keyPointsNearest[i].size(); ++j)
        {
            dummy_points.push_back(keyPointsNearest[i][j]);
        }
        key_points_nearest.push_back(dummy_points);
    }
}

void
ImageView::getCCPoints(vector<int> &cc_points)
{
    cc_points.clear();
    for (unsigned int i = 0; i < ccPoints.size(); ++i)
    {
        cc_points.push_back(ccPoints[i]);
    }
}

void
ImageView::setRender(bool render_poly, bool render_rect, bool render_significant_plane, bool render_visit_plane = false)
{
    renderPoly = render_poly;
    renderRect = render_rect;
    renderSignificantPlane = render_significant_plane;
    renderVisitedPlanes = render_visit_plane;
}

void
ImageView::setNumberOfPoints(int num_points_clicked, int num_key_points_detected)
{
    numKeyPointsDetected = num_key_points_detected;
    numPointsClicked = num_points_clicked;
}

void
ImageView::clearOutputVectors()
{
    unsigned int size = continuousBoundingBoxPoints.size();
    assert(size == planeParameters.size());
    for (unsigned int i = 0; i < size; ++i)
    {
        continuousBoundingBoxPoints[i].clear();
        planeParameters[i].clear();
    }
    continuousBoundingBoxPoints.clear();
    planeParameters.clear();
}

void
ImageView::clearInputVectors()
{
    ccPoints.clear();
    pointsClicked.clear();
    keyPointsNearest.clear();
}

void
ImageView::setSigPlaneBoundingBoxPoints(const vector<Point3f> &sigplane_bounding_box_points)
{
    sigPlaneBoundingBoxPoints.clear();
    for (unsigned int i = 0; i < sigplane_bounding_box_points.size(); ++i)
    {
        sigPlaneBoundingBoxPoints.push_back(sigplane_bounding_box_points[i]);
    }
    return ;
}

bool
ImageView::is_keyBoardActive()
{
    return key_board_control;
}