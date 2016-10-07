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
 * 21-Sep-2016	Sona Praneeth Akula		* Added new function prototypes and removed unnecessary 
 											variables
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
		std::vector< Point2f > _points; //< points of the polygon/polyline
		std::vector< Point2f > _temp_points;

		/* For Menu Buttons */
		std::vector< Point2f > _menu_box_points;
		std::vector< vector<Point2f> > _menu_box_button_points;
		std::vector< Point2f > _message_start_points;
		enum MENU_OPTIONS {CLOSED_SURFACE, OPEN_SURFACE, CHANGE_DIRECTION,
				DELETE_LAST_LINE, COMPLETED_DRAWING, CLEAR_SCREEN, QUIT};
		Point2f _message_start;
		string _menu_text;
		int _menu_box_width = 140;
		int _menu_box_height = 20;

		int _window_height = 600; //< Height of the GUI
		int _window_width = 800; //< Width of the GUI
		int _draw_screen_height = 360; //< Height of the drawing area
		int _draw_screen_width = 640; // Width of the drawing area
		int _max_plane_height = 2; // Max allowed default plane height. Can go upto 15
		int _max_allowed_plane_height = 4;
		int _min_allowed_plane_height = 1;
		int _window; //< Indicator for the GUI window
		int _number_of_planes; //< Number of planes drawn on the screen
		int _type_of_surface = SURFACES::OPEN;
		int _viewing_direction = VIEWING_DIRECTION::FRONT;
		bool _exit_app = false;
		/* Initiating variables for GUI (if needed) */
		char *_argv[2] = { "Top-View-Program", "" };
		int _argc = 1;

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
		/* Mouse motion (hover) callback */
		static void hoverCallback(int x, int y)
		{
			_instance->myMouseHover(x, y);
		}
		void setupHoverCallback()
		{
			_instance = this;
			::glutPassiveMotionFunc(TopView::hoverCallback);
		}


	public:
		vector<double> angles; //< Angles with which quadcopter has to rotate to align itself with new plane
		vector<RotateDirection> direction; //< Direction with which quadcopter has to rotate to align itself with new plane

		int drawing_option = SHAPES::POLYLINE; //< Default drawing option set to POLYLINE

		string draw_mode[2] = {"Open-Surface", "Closed-Surface"}; //< Current available drawing modes
		bool run_status = false;

		TopView();

		/* These constructors are not implemented */
		TopView(int argc, char **argv);
		TopView(int window_height, int window_width);

		void init();

		/* Threading functions */
		void startSystem();
		void run();
		void stopSystem();

		/* Callback Functions for glut */
		void originalScreen();
		void myMouse(int button, int state, int x, int y);
		void myPressedMove(int x, int y);
		void myMouseHover(int x, int y);

		/* Helper functions for GUI and calculations */
		void setMatrixMode(GLenum mode);
		void drawLines(GLenum mode, vector< Point2f > points);
		void drawMessage(string display_message, float x, float y);
		void drawMessage(string display_message, Point3f color, float x, float y);
		void makeButton(vector<Point2f> button_box, Point2f msg_start, string button_text);
		void drawPoints(float x, float y);
		void drawMousePosition(int x, int y);
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
		int getViewingDirection();
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
