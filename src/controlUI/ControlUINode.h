#pragma once
/*****************************************************************************************
 * ControlUINode.h
 *
 *       Created on: 19-Feb-2015
 *    Last Modified: 24-Sep-2016
 *  Original Author: Anirudh Vemula
 *   Current Author: Meghshyam Govind Prasad
 *   Current Author: Sona Praneeth Akula
 *          Project: Quadcopter
 *      Description:
 *
 * Date             Author                          Modification
 * 12-Sep-2016  Sona Praneeth Akula Added       * Added comments to the code
 *****************************************************************************************/

#ifndef _CONTROLUINODE_H
#define _CONTROLUINODE_H

#include "std_msgs/String.h"
#include "Helper.h"
#include "VisionHelper.h"

#include "Line2.hpp"
#include "AllHeaders.hpp"
#include "TopView.hpp"
#include "Headers.h"
#include "ardrone_autonomy/RecordEnable.h"
#include "ardrone_autonomy/Navdata.h"


class ImageView;

// Enum Constants for specifying moving directions - 8 possible direction
enum MOVE_DIRECTIONS {LEFT, RIGHT, FORWARD, BACKWARD, UP, DOWN, CLOCK, COUNTERCLOCK};

/**
 * @brief @todo Structure for creating a small square in a whole grid
 * @details @todo
 */
struct gridSquare
{
    public:
        float u, v;                  /*!< left upper point */
        float width, height; /*!< width and height of the gridSquare */

        /**
         * @brief Constructor
         * @details Sets the public parameters
         * @param [vector<float>] lu - Co-ordinates for the left upper point
         * @param [float] width - Width of the gridSquare
         * @param [float] height - Height of the gridSquare
         * @return
         */
        gridSquare(std::vector<float> lu, float width, float height)
        {
            this->width = width;
            this->height = height;
            u = lu[0];
            v = lu[1];
        }

        /**
         * @brief Constructor
         * @details Sets the public parameters
         * @param [float] u - X co-ordinate for the left upper point
         * @param [float] v - Y co-ordinate for the left upper point
         * @param [float] width - Width of the gridSquare
         * @param [float] height - Height of the gridSquare
         * @return
         */
        gridSquare(float u, float v, float width, float height)
        {
            this->width = width;
            this->height = height;
            this->u = u;
            this->v = v;
        }

        /**
         * @brief Function for printing debug information
         * @details
         * @param
         * @return
         */
        void debugPrint()
        {
            ROS_INFO("Square created at (%f, %f) with width %f and height %f", u, v, width, height);
        }

        /**
         * @brief Function for printing all co-ordinates of the gridSquare
         * @details
         * @param
         * @return
         */
        void printCoord()
        {
            std::cout << std::endl;
            std::cout << u << ", " << v << std::endl;
            std::cout << u+width << ", " << v << std::endl;
            std::cout << u+width << ", " << v-height << std::endl; // @doubt - Why v-height and not v?
            std::cout << u << ", " << v-height << std::endl;
            std::cout << std::endl;
        }
};

/**
 * @brief @todo Grid of rectangular gridSquares
 * @details @todo
 */
struct grid
{
    public:
        // bounds
        float minU, minV, maxU, maxV, width, height, overlap;
        int row;
        std::vector<std::vector<gridSquare> > rowSquares;

        /**
         * @brief Constructor
         * @param [float] _min_u_ - Minimum X co-ordinate of the grid
         * @param [float] _min_u_ - Minimum X co-ordinate of the grid
         * @param [float] _min_u_ - Minimum X co-ordinate of the grid
         * @param [float] _min_u_ - Minimum X co-ordinate of the grid
         * @param [float] width - Width of the grid
         * @param [float] height - Height of the grid
         * @param [float] overlap - Amount of overlap between two grids
         */
        grid( float _min_u_, float _min_v_, float _max_u_, float _max_v_,
                    float width, float height, float overlap)
        {
            this->minU = _min_u_;
            this->minV = _min_v_;
            this->maxU = _max_u_;
            this->maxV = _max_v_;
            this->width = width;
            this->height = height;
            this->overlap = overlap;
            std::vector<gridSquare> v;
            rowSquares.push_back(v);
            row = 0;
        }

        /**
         * @brief @todo Add a gridSquare to grid at the end of a specific row
         * @details
         * @param [gridSquare] square - Grid Square to be added to grid at specific row
         * @return
         */
        void add(gridSquare square)
        {
            rowSquares[row].push_back(square);
        }

        /**
         * @brief @todo What is getting translated and how?
         * @details
         * @param [gridSquare] g -
         * @return
         */
        bool translate(gridSquare g)
        {
            // Whenever row empty. Check v bounds too.
            if(rowSquares[row].empty())
            {
                float unew = this->minU;
                float vnew = g.v - (1 - overlap) * height;
                if(vnew - height >= this->minV)
                {
                    // Clean down shift
                    gridSquare gnew(unew, vnew, width, height);
                    add(gnew);
                }
                else if(vnew - height < minV)
                {
                    // grid with less height - ? No. grid square must always have a fixed height
                    gridSquare gnew(unew, vnew, width, height);
                    if(vnew - minV > 0)
                        add(gnew);
                    else
                        return false;
                }
            }
            else
            {
                float unew = g.u + (1-overlap)*width;
                // Only right translation
                if(unew+width <= maxU)
                {
                    // Clean right shift
                    gridSquare gnew(unew, g.v, width, g.height);
                    add(gnew);
                }
                else if(unew + width > maxU)
                {
                    // grid with less width - ? No. grid square must always have a fixed width
                    gridSquare gnew(unew, g.v, width, g.height);
                    if(maxU - unew > 0) // lower bound on the width of the square - ?
                        add(gnew);
                    // Row completed
                    row++;
                    std::vector<gridSquare> v;
                    rowSquares.push_back(v);
                }
            }
            return true;
        }

        /**
         * @brief Get the latest gridSquare in the grid (last gridSquare pushed in the grid)
         * @details
         * @param
         * @return
         */
        gridSquare getLatest()
        {
            if(rowSquares[row].empty())
                return rowSquares[row-1].back();
            else
                return rowSquares[row].back();
        }

        /**
         * @brief
         * @param
         * @param
         * @param
         */
        void print(std::vector<float> plane)
        {
            for(unsigned int i = 0; i < rowSquares.size(); i++)
            {
                for(unsigned int j = 0; j < rowSquares[i].size(); j++)
                {
                    float x = rowSquares[i][j].u;
                    float z = rowSquares[i][j].v;
                    float y = getY(x, z, plane);
                    printf("%f %f %f\n", x, y, z);
                    printf("%f %f %f\n", x + width, getY(x+width, z, plane), z);
                    printf("%f %f %f\n", x + width, getY(x+width, z - height, plane), z - height);
                    printf("%f %f %f\n", x, getY(x, z - height, plane), z - height);
                }
            }
        }
};

/**
 * @brief
 * @details
 */
struct pGridSquare
{
    public:
        float u, v; // left upper point
        float width, height;
        std::vector<float> rd, dd; // right and down direction vector

        /**
         * @brief
         * @param
         * @param
         * @param
         */
        pGridSquare(float u, float v,
                                float width, float height,
                                std::vector<float> rd, std::vector<float> dd)
        {
            this->u = u;
            this->v = v;
            this->width = width;
            this->height = height;
            this->rd = rd;
            this->dd = dd;
        }

        /**
         * @brief
         * @param
         * @param
         * @param
         */
        void debugPrint()
        {
            ROS_INFO("Square created at (%f, %f) with width %f along (%f, %f) and height %f along (%f, %f)",
                    u, v, width,
                    rd[0], rd[1],height,dd[0],dd[1]);
        }
};

/**
 * @brief
 * @details
 */
struct pGrid
{
    public:
        float au, av; //left upper point
        float maxR, maxD; // maximum right and down distances along rd and dd resp.
        std::vector<float> rd; // right direction vector
        std::vector<float> dd; // down direction vector
        float width; // distance along rd
        float height; // height along dd
        float overlap; // overlap across grid squares
        int row;
        std::vector<std::vector<pGridSquare> > rowSquares;

        /**
         * @brief
         * @param
         * @param
         * @param
         */
        pGrid(float au, float av,
                    std::vector<float> rd, std::vector<float> dd,
                    float width, float height, float overlap,
                    float maxR, float maxD)
        {
            this->au = au;
            this->av = av;
            this->rd = rd;
            this->dd = dd;
            this->width = width; this->height = height; this->overlap = overlap;
            this->maxR = maxR; this->maxD = maxD;
            std::vector<pGridSquare> v;
            rowSquares.push_back(v);
            row = 0;
        }

        /**
         * @brief
         * @param
         * @param
         * @param
         */
        void add(pGridSquare gs) {
            rowSquares[row].push_back(gs);
        }

        /**
         * @brief
         * @param
         * @param
         * @param
         */
        bool translate(pGridSquare g)
        {
            // Whenever row empty. Check v bounds too.
            if(rowSquares[row].empty())
            {
                float unew = au + (1-overlap)*height*dd[0];
                float vnew = g.v - (1-overlap)*height*dd[1];
                if(av - (vnew - height*dd[1]) <= maxD*dd[1])
                {
                    // Clean down shift
                    pGridSquare gnew(unew, vnew, width, height, rd, dd);
                    add(gnew);
                }
                else if(av - (vnew - height*dd[1]) > maxD*dd[1])
                {
                    // grid with less height - ? No. grid square must always have a fixed height
                    pGridSquare gnew(unew, vnew, width, height, rd, dd);
                    if(vnew > av - maxD*dd[1])
                        add(gnew);
                    else
                        return false;
                }
            }
            else
            {
                float unew = g.u + (1-overlap)*width*rd[0];
                float vnew = g.v + (1-overlap)*width*rd[1];
                // Only right translation
                if(unew + width*rd[0] - au <= maxR*rd[0])
                {
                    // Clean right shift
                    pGridSquare gnew(unew, vnew, width, height, rd, dd);
                    add(gnew);
                }
                else if(unew + width*rd[0] - au > maxR*rd[0])
                {
                    // grid with less width - ? No. grid square must always have a fixed width
                    pGridSquare gnew(unew, vnew, width, height, rd, dd);
                    if(unew < au + maxR*rd[0]) // lower bound on the width of the square - ?
                        add(gnew);
                    // Row completed
                    row++;
                    std::vector<pGridSquare> v;
                    rowSquares.push_back(v);
                }
            }
            return true;
        }

        /**
         * @brief
         * @param
         * @param
         * @param
         */
        pGridSquare getLatest()
        {
            if(rowSquares[row].empty())
                return rowSquares[row-1].back();
            else
                return rowSquares[row].back();
        }

        /**
         * @brief
         * @param
         * @param
         * @param
         */
        void print(vector<float> plane)
        {
            for(unsigned int i=0; i<rowSquares.size(); i++)
            {
                for(unsigned int j=0; j<rowSquares[i].size(); j++)
                {
                        float x = rowSquares[i][j].u;
                        float z = rowSquares[i][j].v;
                        float y = getY(x, z, plane);
                        printf("%f %f %f\n", x, y, z);
                        printf("%f %f %f\n", x + width, getY(x+width, z, plane), z);
                        printf("%f %f %f\n", x + width, getY(x+width, z - height, plane), z - height);
                        printf("%f %f %f\n", x, getY(x, z - height, plane), z - height);
                }
            }
        }

};


/**
 * @brief
 * @details
 */
class ControlUINode
{
    private:

        ros::Subscriber keypoint_coord_sub;     // Key point co-ordinates subscriber
        ros::Subscriber pose_sub;               // Pose information (x, y, z, roll pitch, yaw) subscriber
        ros::Subscriber navdata_sub;
        ros::Time lastKeyStamp;
        ros::Subscriber tum_ardrone_sub;
        ros::Publisher tum_ardrone_pub;
        ros::Publisher land_pub;                // Publish land command
        std::string land_channel;
        ros::NodeHandle nh_;                    // Node handler for controlUINode

        ros::ServiceClient video;

        ros::Timer timer_checkPos;
        ros::Timer timer_record;

        ros::Time last;

        // Key point information
        std::vector<std::vector<float> > _3d_points;    // 3d world key point information
        std::vector<std::vector<float> > _2d_points;    // 2d image co-ordinates infoprmation
        std::vector<int> _levels;                       // Level (resolution) at which feature points were captured
        bool calibrated;
        cv::Mat cameraMatrix;                           // Camera matrix of the quadcopter
        cv::Mat distCoeffs;
        std::vector<cv::Mat> rvecs, tvecs;              // Rotation and translation vectors

        int numPoints;

        float scale;                        // PTAM X-Y scale
        float scale_z;                      // PTAM Z scale
        float x_offset, y_offset, z_offset; // PTAM offsets

        // Drone state variables
        float x_drone, y_drone, z_drone, roll, pitch, yaw; // drone pose variables

        std::vector<float> _3d_plane; // stored as a 4 length vector with constants a,b,c,d
                                     // corresponding to ax+by+cz+d = 0. Enforcing the constraint that d = 1 for uniformity (except when d = 0)

        std::vector<double> targetPoint;
        std::list<std_msgs::String> commands;
        std::list<std::vector<double> > targetPoints;
        std::vector<int> startTargePtIndex;
        int numberOfPlanes;
        int planeIndex;

        std::string keypoint_channel;   // channel on which keypoint info is received
        std::string command_channel;    // channel on which commands can be posted or received
        std::string pose_channel;       // channel on which pose info is received
        std::string navdata_channel;    // channel on which ardrone navdata is received

        bool ransacVerbose; // Whether we need the ransac verbose output or not
        bool useScaleFactor; // Using scale factors. MUST BE SET TO TRUE
        float threshold; // threshold allowed in finding keypoint in the current frame
        double error_threshold; // threshold allowed in the drone position
        double recordTime; // time to record the video
        bool record; // whether to record or not
        float pollingTime; // Interval at which polling is done to check the drone position
        bool targetSet;
        bool currentCommand;
        bool recordNow;
        bool notRecording;

        /* New pose cb channel */
        ros::Subscriber new_pose_sub;

        /* Both are obtained from ImageView -> TopView */
        list<double> _node_main_angles;
        list<RotateDirection> _node_main_directions;

        /* Get current position of drone in this vector */
        vector<double> _node_current_pos_of_drone;
        /* Get destination positoin of drone wrt current position of drone in this vector */
        vector<double> _node_dest_pos_of_drone;
        /* Get actual destination positoin of drone wrt origin of drone in this vector */
        vector<double> _node_ac_dest_pos_of_drone;
        /* Path to be used for planning internal navigation */
        vector< vector<double> > _interm_path;
        /* */
        vector<double> _last_drone_position;

        bool _stage_of_plane_observation;
        bool _is_big_plane;
        bool _is_plane_covered;
        bool _is_able_to_see_new_plane;
        int _sig_plane_index;
        int _actual_plane_index;

        float _node_max_height_of_plane;
        float _node_min_height_of_plane;
        float _node_min_distance;
        float _node_max_distance;
        int _node_number_of_planes;
        int _node_completed_number_of_planes;
        float _plane_d;
        int _plane_d_num = 0;
        float _step_distance;
        float _fixed_distance;
        bool _fixed_height_set;
        float _fixed_height;
        double _move_heuristic;
        double _angle_heuristic;

        RotateDirection _next_plane_dir;
        double _next_plane_angle;
        bool _is_adjusted;
        float _plane_heuristic;
        std::chrono::high_resolution_clock::time_point _capture_start, _capture_end;
        std::chrono::high_resolution_clock::time_point _align_current_start, _align_current_end;
        std::chrono::high_resolution_clock::time_point _align_next_start, _align_next_end;
        std::chrono::high_resolution_clock::time_point _plane_start, _plane_end;


        // Only for navigating the quadcopter
        bool printDebugInfo;
        bool logInfo;


        // distance between two 2d points
        /**
         * @brief
         * @param
         * @param
         * @param
         */
        float
        distance(std::vector<int> pt_int, std::vector<float> pt_float);

        // distance between two 3d points
        float
        distance3D(std::vector<float> p1, std::vector<float> p2);

        // Thread mutexes
        static pthread_mutex_t tum_ardrone_CS;
        static pthread_mutex_t keyPoint_CS;
        static pthread_mutex_t pose_CS;
        static pthread_mutex_t command_CS;
        static pthread_mutex_t navdata_CS;



    public:

        ImageView *image_gui;

        /* Plane parameters for all planes obtained from jlinkage */
        vector< vector<float> > jlink_all_plane_parameters;
        /* Continuous bounding box points for all planes obtained from jlinkage */
        vector< vector<Point3f> > jlink_all_continuous_bounding_box_points;
        /* Percentage of points from each plane obtained from jlinkage */
        vector< float > jlink_all_percentage_of_each_plane;
        /* Vector for adding three dim points of points obtained form jlinkage */
        vector< vector<Point3f> > jlink_three_d_points;

        /* Plane parameters for a single plane (to be used anywhere) */
        vector<float> this_plane_parameters;
        /* Continuous bounding box points for a single plane (to be used anywhere) */
        vector<Point3f> this_continuous_bounding_box_points;
        /* */
        vector<Point3f> this_sorted_3d_points;

        /* Plane parameters for all planes visited till now */
        vector< vector<float> > visited_plane_parameters;
        /* Continuous bounding box points for all planes visited till now */
        vector< vector<Point3f> > visited_continuous_bounding_box_points;
        //
        vector< vector<double> > visited_motion_points;
        /* Plane parameters for all planes visited till now (temporary) */
        vector< vector<float> > this_visited_plane_parameters;
        /* Continuous bounding box points for all planes visited till now (temporary) */
        vector< vector<Point3f> > this_visited_continuous_bounding_box_points;

        /* Vector for adding three dim points of points to be (if needed) for bestFitPlane */
        vector<Point3f> aug_three_d_points;
        vector<Point3f> aug_plane_bounding_box_points;


        /**
         * @brief Constructor for ControlUINode
         * @details Callbacks called from Constructor -> keyPointDataCb,
         *                  &poseCb, comCb, newPoseCb
         * @param
         * @return
         */
        ControlUINode ();

        /**
         * @brief Destructor for ControlUINode
         * @details Currently Empty
         * @param
         * @return
         */
        ~ControlUINode ();

        // ROS message callbacks
        void
        keyPointDataCb (const tum_ardrone::keypoint_coordConstPtr coordPtr);
        void
        poseCb (const tum_ardrone::filter_stateConstPtr statePtr);
        void
        comCb (const std_msgs::StringConstPtr str);
        void
        navDataCb(const ardrone_autonomy::Navdata navPtr);

        inline bool
        isCalibrated()
        {
            return calibrated;
        }

        /**
         * @brief Main Loop
         * @details
         * @param
         * @return
         */
        void
        Loop ();

        /**
         * @brief writes a string message to "/tum_ardrone/com"
         * @details Is thread safe
         * @param
         * @return
         */
        void
        publishCommand (std::string c);

        // Helper functions
        /**
         * @brief writes a string message to "/tum_ardrone/com"
         * @details Is thread safe
         * @param
         * @return
         */
        void
        load2dPoints (std::vector<float> t_2dPoints_x, std::vector<float> t_2dPoints_y);

        /**
         * @brief writes a string message to "/tum_ardrone/com"
         * @details Is thread safe
         * @param
         * @return
         */
        void
        load3dPoints (std::vector<float> t_3dPoints_x,
                      std::vector<float> t_3dPoints_y, std::vector<float> t_3dPoints_z);

        /**
         * @brief writes a string message to "/tum_ardrone/com"
         * @details Is thread safe
         * @param
         * @return
         */
        void
        loadLevels (std::vector<int> levels);

        // Algorithmic functions
        /**
         * @brief
         * @details
         * @param
         * @return
         */
        std::vector<float>
        fitPlane3d (std::vector<int> ccPoints, std::vector<std::vector<int> > pointsClicked);

        /**
         * @brief Fit multiple planes in 3D
         * @details
         * @param
         * @return
         */
        void
        fitMultiplePlanes3d (vector<int> &ccPoints, vector< vector<int> > &pointsClicked,
                            vector< vector<float> > &planeParameters,
                            vector< vector<Point3f> > &continuousBoundingBoxPoints);

        // Move Quadopter to required position
        /**
         * @brief writes a string message to "/tum_ardrone/com"
         * @details Is thread safe
         * @param
         * @return
         */
        void
        moveQuadcopter(
            const vector< vector<float> >&planeParameters,
            const vector< vector<Point3f> > &continuousBoundingBoxPoints);

        /**
         * @brief writes a string message to "/tum_ardrone/com"
         * @details Is thread safe
         * @param
         * @return
         */
        void
        getInitialPath(const vector<double> &prevPosition, const vector<double> &tPoint,
                       double prevYaw, double desiredYaw, vector<vector<double> > &xyz_yaw);

        //Find target points for plane not parallel to XZ plane
        /**
         * @brief
         * @details
         * @param
         * @return
         */
        void
        getPTargetPoints(const pGrid &g, const vector<float> & plane,
                         const vector<Point3f> &uvAxes, vector<vector<double> > &tPoints );

        //sort target points according to Z
        /**
         * @brief
         * @details
         * @param
         * @return
         */
        void
        sortTargetPoints(int numRows, const vector<int> &numColsPerRow,
                         const vector< vector<double> > &tPoints,
                         vector< vector<double> > &sortedTPoints);

        //Get UV Grid corners
        /**
         * @brief
         * @details
         * @param
         * @return
         */
        void
        getGridSquareUVCorners(const pGridSquare &gs, vector<Point2f> &uvCorners);

        //Print UV grid and according XYZ grid
        /**
         * @brief
         * @details
         * @param
         * @return
         */
        void
        printGrid(const pGrid &, const vector<Point3f> &uvAxes, const vector<float> &plane);

        // Search function : Given a 2d point, find the nearest 2d keypoint and return its 3d position
        /**
         * @brief
         * @details
         * @param
         * @return
         */
        std::vector<float>
        searchNearest(std::vector<int> pt, bool considerAllLevels);

        // Get 2d position (with a threshold) of a key point given its 3d position. Return empty vector if keypoint not found in the current frame (within the threshold)
        /**
         * @brief
         * @details
         * @param
         * @return
         */
        bool
        get2DPoint(std::vector<float> pt, std::vector<int> &p, bool considerAllLevels);

        //Project World Pts on Image plane
        /**
         * @brief
         * @details
         * @param
         * @return
         */
        void
        project3DPointsOnImage(const vector<Point3f> &worldPts, vector<Point2f > & imagePts);

        //calibrate camera
        /**
         * @brief
         * @details
         * @param
         * @return
         */
        void
        calibrate();

        // Get 2d position of the nearest key point given a 3d position
        /**
         * @brief
         * @details
         * @param
         * @return
         */
        bool
        get2DPointNearest(std::vector<float> pt, std::vector<int> &p, bool considerAllLevels);

        /**
         * @brief Equality function for 3d keypoints.
         * @details Does it need to be exact equality? Or some heuristic based distance threshold
         * @param
         * @return
         */
        bool
        equal(std::vector<float> p1, std::vector<float> p2);

        /**
         * @brief Write 3D points obtained to a CSV file
         * @details
         * @param
         * @return
         */
        void
        write3DPointsToCSV(std::vector<std::vector<float> > &_3d_points);

        /**
         * @brief A helper function to get the number of key points in the current frame
         * @details
         * @param
         * @return
         */
        int
        getNumKP(bool considerAllLevels);

        /**
         * @brief Saves the 3d coordinates of the keypoints as a CSV file for external processing
         * @details
         * @param
         * @return
         */
        void
        saveKeyPointInformation(int numFile);

        /**
         * @brief Translates the fitted plane by the given distance along its normal toward origin
         * @details Currently not using
         * @param
         * @return
         */
        std::vector<float>
        translatePlane (float translateDistance);

        /**
         * @brief Projects the 3d points onto the extracted plane
         * @details
         * @param
         * @return
         */
        std::vector< std::vector<float> >
        projectPoints (std::vector<int> ccPoints, std::vector<std::vector<float> > keyPoints);

        /**
         * @brief Builds the grid
         * @details
         * @param
         * @return
         */
        grid
        buildGrid (std::vector<std::vector<float> > pPoints);

        //pGrid buildPGrid (std::vector<std::vector<float> > pPoints);
        /**
         * @brief Builds the PGrid
         * @details
         * @param
         * @return
         */
        pGrid
        buildPGrid(const vector<Point2f> &uvCoordinates);

        /**
         * @brief Gets the target points given the grid and plane
         * @details
         * @param
         * @return
         */
        std::vector< std::vector<double> >
        getTargetPoints (grid g, std::vector<float> plane);

        /**
         * @brief Generate the appropriate goto commands according to the target points
         * @details
         * @param
         * @return
         */
        void
        moveDrone (const std::vector<double> &prevPosition,
                                        std::vector< std::vector<double> > tPoints,
                                        double prevYaw, double yaw);

        /**
         * @brief Checks the position of the drone and whether the error is less than a threshold
         * @details
         * @param
         * @return
         */
        void
        checkPos (const ros::TimerEvent&);

        // Records the video for a fixed amount of time
        // bool recordVideo ();

        // Records the video for a fixed amount of time
        // bool recordVideo ();

        void
        newPoseCb (const tum_ardrone::filter_stateConstPtr statePtr);

        void
        sendLand();

        void
        setMainAngles(const vector<double> &main_angles);

        void
        setMainDirections(const vector<RotateDirection> &main_directions);

        void
        setValues(int number_of_planes, float min_height_of_plane, float min_distance, float max_height_of_plane, float max_distance);

        void
        getCurrentPositionOfDrone();

        void
        getCurrentPositionOfDrone(vector<double> &current_drone_pos);

        void
        getMultiplePlanes3d (const vector<int> &ccPoints, const vector< vector<int> > &pointsClicked,
                                vector< vector<float> > &planeParameters,
                                vector< vector<Point3f> > &continuousBoundingBoxPoints,
                                vector< vector<Point3f> > &sorted_3d_points,
                                vector<float> &percentageOfEachPlane);

        void
        getMultiplePlanes3d (vector< vector<float> > &planeParameters,
                                vector< vector<Point3f> > &continuousBoundingBoxPoints,
                                vector< vector<Point3f> > &sorted_3d_points,
                                vector<float> &percentageOfEachPlane);

        void
        testUtility(int test_no);

        vector<float>
        fitPlane3dForTheCurrentPlane();

        void
        captureTheCurrentPlane();

        void
        moveDroneViaSetOfPoints(const vector< vector<double> > &dest_points);

        void
        designPathToChangeYaw(const vector<double> &curr_point,
                                double dest_yaw);

        void
        alignQuadcopterToCurrentPlane();

        void
        adjustForNextCapture();

        void
        designPathForDrone(const vector< double > &start,
                                const vector< double > &end);

        void
        designPathForDroneRelative(double dest, int direction);

        void
        alignQuadcopterToNextPlane();

        void
        adjustTopBottomEdges();

        void
        adjustLeftEdge();

        int
        checkVisibility(const vector<float> &planeParameters, 
                                const vector<Point3f> &continuous_bounding_box_points,
                                int which_side);

        void
        moveUp(double step_distance = 0.5);

        void
        moveDown(double step_distance = 0.5);

        void
        moveLeft(double step_distance = 0.5);

        void
        moveRight(double step_distance = 0.5);

        void
        moveForward(double step_distance = 0.5);

        void
        moveBackward(double step_distance = 0.5);

        void
        rotateClockwise(double step_angle = 5.0);

        void
        rotateCounterClockwise(double step_angle = 5.0);

        void
        doJLinkage();

        void
        augmentInfo();

        void
        copyNecessaryInfo();

        void
        checkPlaneParametersSign(const vector<double> &position, 
                                const vector<Point3f> &points,
                                vector<float> &plane_parameters);

        void
        getCompleteCurrentPlaneInfo(const vector< vector<float> > &plane_parameters,
                                    const vector< vector<Point3f> > &cbb,
                                    const vector< vector<Point3f> > &points,
                                    const vector<float> &percPlane,
                                    int currPlaneIndex,
                                    vector<float> &out_plane_parameters,
                                    vector<Point3f> &out_cbb,
                                    vector<Point3f> &out_3d_points);

        void
        adjustYawToCurrentPlane();

        void
        move(double distance, int i);

        void
        moveInDirection(const vector<float> &dir, 
                        const vector<double> &position,
                        const vector<Point3f> &points);

};




#endif
