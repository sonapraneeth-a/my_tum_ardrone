/**
 * @ingroup 
 */

/*****************************************************************************************
 * TopView.hpp
 *
 *     Created on: 13-Sep-2016
 *  Last Modified: 21-Sep-2016
 *         Author: Sona Praneeth Akula
 *        Project: Data_Structures_and_Algorithms
 *    Description:
 *
 * Date				Author							Modification
 * 13-Sep-2016	Sona Praneeth Akula
 * 17-Sep-2016	Sona Praneeth Akula		* Fixed the error for callbacks
 * 										* Completed the porting to Object Oriented Design
 * 21-Sep-2016	Sona Praneeth Akula		* Added necessary thread headers
 *****************************************************************************************/
#ifndef TOPVIEW_HPP_
#define TOPVIEW_HPP_

#include "../Line/Line2.hpp"
#include "../AllHeaders.hpp"
#include "../DebugUtility/DebugUtility.hpp"
#include "cvd/thread.h"

enum SHAPES {POLYLINE, POLYGON}; //< Shapes currently available for drawing
enum VIEWING_DIRECTION {FRONT, BACK, OUT, IN}; //< Direction of viewing the surface. BOTTOM indicates front
enum SURFACES {OPEN, CLOSED}; //< Whether the surface is open/closed


class TopView: private CVD::Thread
{
	private:
		std::vector< float > x_coord; //< x co-ordinates of points in points vector
		std::vector< float > y_coord; //< y co-ordinates of points in points vector
		std::vector< float > temp_x_coord; //< Temporary variables to store x coord the end of line
		std::vector< float > temp_y_coord; //< Temporay variables to store the x coord end of line
		std::vector< Point2f > points; //< points of the polygon/polyline
		std::vector< Point2f > temp_points;

		/* For Menu Buttons */
		std::vector< Point2f > menu_box_points;
		std::vector< vector<Point2f> > menu_box_button_points;
		std::vector< Point2f > message_start_points;
		enum MENU_OPTIONS {CLOSED_SURFACE, OPEN_SURFACE, CHANGE_DIRECTION,
				DELETE_LAST_LINE, COMPLETED_DRAWING, CLEAR_SCREEN, QUIT};
		Point2f message_start;
		string menu_text;
		int menu_box_width;
		int menu_box_height;

		int _window_height; //< Height of the GUI
		int _window_width; //< Width of the GUI
		int _draw_screen_height; //< Height of the drawing area
		int _draw_screen_width; // Width of the drawing area
		int _max_plane_height; // Max allowed default plane height. Can go upto 15
		int _window; //< Indicator for the GUI window
		int _number_of_planes; //< Number of planes drawn on the screen
		int _type_of_surface;
		int _viewing_direction;
		bool _exit_app;
		/* Initiating variables for GUI (if needed) */
		int _argc;
		char **_argv;

		void myObjectDrawingTemp();

		/* Callback initiations */
		static TopView * _instance;
		/* Default screen callback */
		static void originalScreenCallback()
		{
			_instance->originalScreen();
		}
		void setupOriginalScreenCallback()
		{
			_instance = this;
			::glutDisplayFunc(TopView::originalScreenCallback);
		}
		/* Mouse motion callback */
		static void motionCallback(int x, int y)
		{
			_instance->myPressedMove(x, y);
		}
		void setupMotionCallback()
		{
			_instance = this;
			::glutMotionFunc(TopView::motionCallback);
		}
		/* Mouse click callback */
		static void mouseClickCallback(int button, int state, int x, int y)
		{
			_instance->myMouse(button, state, x, y);
		}
		void setupMouseClickCallback()
		{
			_instance = this;
			::glutMouseFunc(TopView::mouseClickCallback);
		}

	public:
		vector<double> angles; //< Angles with which quadcopter has to rotate to align itself with new plane
		vector<RotateDirection> direction; //< Direction with which quadcopter has to rotate to align itself with new plane

		int drawing_option; //< Default drawing option set to POLYLINE

		string draw_mode[2]; //< Current available drawing modes
		bool run_status;

		TopView();

		/* These constructors are not implemented */
		TopView(int argc, char **argv);
		TopView(int window_height, int window_width);

		void init();

		/* */
		void startSystem();
		void run();
		void stopSystem();

		/* Callback Functions for glut */
		void originalScreen();
		void myMouse(int button, int state, int x, int y);
		void myPressedMove(int x, int y);

		/* Helper functions for GUI and calculations */
		void setMatrixMode(GLenum mode);
		void drawLines(GLenum mode, vector< Point2f > points);
		void drawMessage(string display_message, float x, float y);
		void makeButton(vector<Point2f> button_box, Point2f msg_start, string button_text);
		void drawPoints(float x, float y);
		void drawPoints(vector< Point2f > points);
		void clearScreen();
		void myDrawing();
		void myObjectDrawing();
		void checkMenu(float x, float y);
		void myDisplay(GLenum mode);
		void destroy();

		/* Drawing Buttons */
		void menuGenerateButtonBoxes();
		void menuDrawOpenSurface();
		void menuDrawClosedSurface();
		void menuQuitApplication();
		void menuCompletedDrawing();
		void menuChangeViewingDirection();
		void menuClearScreen();
		void menuDeleteLastLine();
		void menuChangeHeight();
		void menuShowDrawingMode();
		void menuShowViewingDirection();
		void menuDisplayWarningMessage();
		void menuDrawDrawingScreen();
		void menuDrawDrawingScreen(int bottomLeftX, int bottomLeftY, int width, int height);

		/* Calculating necessary output */
		void calculateAngles();

		/* Derive information about the surface */
		void getDirections(vector<RotateDirection> &dir);
		void getAngles(vector<double> &angle);
		int getNumberOfPlanes();
		int getTypeOfSurface();
		int getMaxHeightOfPlane();
		bool getExitStatus();

		/* Destructor */
		~TopView();
};

/*
TopView* _instance;
extern "C"
void originalScreenCallback()
{
	_instance->originalScreen();
}
void
TopView::setupOriginalScreenCallback()
{
	::_instance = this;
	::glutDisplayFunc(::originalScreenCallback);
}
*/

#endif /* TOPVIEW_HPP_ */
