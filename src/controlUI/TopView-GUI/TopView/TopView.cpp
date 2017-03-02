/**
 * @ingroup 
 */

/*****************************************************************************************
 * TopView.cpp
 *
 *     Created on: 13-Sep-2016
 *  Last Modified: 21-Sep-2016
 *         Author: Sona Praneeth Akula
 *        Project: Data_Structures_and_Algorithms
 *        Description: 
 *
 * Date				Author							Modification
 * 13-Sep-2016	Sona Praneeth Akula	
 * 17-Sep-2016	Sona Praneeth Akula		* Ported the code to Class
 * 19-Sep-2016	Sona Praneeth Akula		* Moved the code to more organized blocks
 										* Fixed minor GUI issues
 * 21-Sep-2016	Sona Praneeth Akula		* Fixed the code for angle issues
 										* Added mouse pointer position on screen whenever the mouse 
 										 pointer is on the drawing area
 *****************************************************************************************/

#include "TopView.hpp"

/**
 * @brief Initiating a static instance of TopView class to be used for callbacks
 */
TopView* TopView::_instance = NULL;

/**
 * @brief Empty TopView constructor
 */
TopView::TopView()
{
	_number_of_planes = 0; //< Currently number of planes are 0
}

/**
 * @brief Initiating the TopView GUI
 */
void
TopView::init()
{
	// _argc = 0;
	// *_argv = (char*) malloc(sizeof(char*));
	// *_argv = (char*) "";
	/* glutInit will initialize the GLUT library and negotiate a session with the window system */
	glutInit(&(_argc), _argv);
	/* http://stackoverflow.com/questions/28298540/difference-between-single-bufferedglut-single-and-double-buffered-drawingglut */
	/* Currently using single buffer window with RGB Color mode */
	glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB);
	/* Initializing window with default width and height */
	glutInitWindowSize(this->_window_width, this->_window_height);
	/* Position of the top left corner of the window */
	glutInitWindowPosition(100, 150);
	/* glutCreateWindow creates a top-level window */
	this->_window = glutCreateWindow("Draw Top View of the surface");
	/* https://www.opengl.org/sdk/docs/man/html/glClearColor.xhtml */
	/* Specify the red, green, blue, and alpha values used when the color buffers are cleared. The initial values are all 0. */
	glClearColor(1.0f, 1.0f, 1.0f, 0.0);
	/* https://www.opengl.org/sdk/docs/man/html/glClear.xhtml */
	/* Determine which buffer to clear */
	glClear(GL_COLOR_BUFFER_BIT);
	this->setMatrixMode(GL_PROJECTION);
	/* Loads Identity matrix onto transformations stack */
	glLoadIdentity();
	/* https://www.opengl.org/sdk/docs/man2/xhtml/gluOrtho2D.xml */
	/* Define a 2D orthographic projection matrix */
	gluOrtho2D(0.0, this->_window_width, 0.0, this->_window_height);
	this->setMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glPointSize(5.0);
	this->setupOriginalScreenCallback();
	this->setupMouseClickCallback();
	this->setupMotionCallback();
	this->setupHoverCallback();
	glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_CONTINUE_EXECUTION);
	/* https://www.opengl.org/resources/libraries/glut/spec3/node14.html */
	glutMainLoop();
}

/**
 * @brief Draws the default screen
 */
void
TopView::originalScreen()
{
	DEBUG_MSG << "[ ACTION] Default Screen Called\n";
	DEBUG_MSG << "[ DEBUG] Creating Menu\n";

	menuGenerateButtonBoxes();
	menuDrawOpenSurface();
	menuDrawClosedSurface();
	menuChangeViewingDirection();
	menuDeleteLastLine();
	menuCompletedDrawing();
	menuClearScreen();
	menuQuitApplication();
	menuChangeHeight();
	menuDisplayWarningMessage();
	menuDrawDrawingScreen();
	menuShowViewingDirection();
	menuShowDrawingMode();
	/* Draw the menu space to screen */
	glFlush();
}

void
TopView::makeButton(vector<Point2f> button_box, Point2f msg_start, string button_text)
{
	drawLines(GL_LINE_STRIP, button_box);
	drawMessage(button_text, msg_start.x, msg_start.y);
}

void
TopView::setMatrixMode(GLenum mode)
{
	/* https://www.opengl.org/sdk/docs/man2/xhtml/glMatrixMode.xml */
	glMatrixMode(mode);
}

TopView::~TopView()
{

}

void
TopView::myMouse(int button, int state, int x, int y)
{
	int yy;
	yy = glutGet(GLUT_WINDOW_HEIGHT);
	y = yy - y; /* In Glut, Y coordinate increases from top to bottom */
	_temp_points.clear();
	originalScreen();
	if( (x >= 20.0) && (x <= 780.0) && (y >= 100.0) && (y <= 550.0) )
	{
		drawMousePosition(x-20.0, yy-y);
	}
	if(_points.size() == 0)
	{
		if( (button == GLUT_LEFT_BUTTON) && (state == GLUT_DOWN) )
		{
			DEBUG_MSG << "[ MOUSE] Left Mouse Button Down\n";
			if( (x >= 20.0) && (x <= 780.0) && (y >= 50.0) && (y <= 500.0) )
			{
				_points.push_back(Point2f(x, y));
				_temp_points.push_back(Point2f(x, y));
				DEBUG_MSG << "LBD0: x: " << x << ", y: " << y << "\n";
			}
			else
			{
				DEBUG_MSG << "LBD0: Point not in range: (" << x << ", " << y << ")\n";
			}
		}
	}
	if( (button == GLUT_LEFT_BUTTON) && (state == GLUT_UP) )
	{
		DEBUG_MSG << "[ MOUSE] Left Mouse Button Up\n";
		if( (x >= 20.0) && (x <= 780.0) && (y >= 50.0) && (y <= 500.0) )
		{
			_points.push_back(Point2f(x, y));
			_temp_points.push_back(Point2f(x, y));
			DEBUG_MSG << "LBD: x: " << x << ", y: " << y << "\n";
		}
		else
		{
			DEBUG_MSG << "LBU: Point not in range: (" << x << ", " << y << ")\n";
		}
		checkMenu(x, y);
	}
}

void
TopView::myPressedMove(int x, int y)
{
	// Bottom Left corner of the screen is: (0, 0);
	// Top right corner of the screen is: (this->_window_width, this->_window_height)
	// Scaling of window not done
	DEBUG_MSG << "[ KEY] Drag\n";
	int yy = glutGet(GLUT_WINDOW_HEIGHT);
	glClearColor(1.0f, 1.0f, 1.0f, 0.0);
	glClear(GL_COLOR_BUFFER_BIT);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D(0.0, this->_window_width, 0.0, this->_window_height);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glPointSize(5.0);
	originalScreen();
	DEBUG_MSG << "MP: " << x << ", " << yy << ", " << y << ", " << yy-y << "\n";
	if( !( (x >= 20.0) && (x <= 780.0) ) )
	{
		if( x < 20.0) x = 20.0;
		if( x > 780.0) x = 780.0;
	}
	if( !( (y >= 100.0) && (y <= 550.0) ) )
	{
		if( y < 100.0) y = 100.0;
		if( y > 550.0) y = 550.0;
	}
	glBegin(GL_POINTS);
		glColor3f(0.0f, 0.0f, 0.0f);
		glPointSize(20.0);
		glVertex2f( _points.back().x, _points.back().y );
		glVertex2f( x, yy - y );
	glEnd();
	glBegin(GL_LINES);
		glColor3f(0.0f, 0.0f, 0.0f);
		glPointSize(20.0);
		glVertex2f( _points.back().x, _points.back().y );
		glVertex2f( x, yy - y );
	glEnd();
	myObjectDrawingTemp();
	if( (x >= 20.0) && (x <= 780.0) && (y >= 100.0) && (y <= 550.0) )
	{
		drawMousePosition(x-20.0, yy-y);
	}
	_temp_points.push_back(Point2f(x-20.0, yy-y));
	glFlush();
}

void
TopView::checkMenu(float x, float y)
{
	DEBUG_MSG << "CM\n";
	glClearColor(1.0f, 1.0f, 1.0f, 0.0);
	glClear(GL_COLOR_BUFFER_BIT);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D(0.0, this->_window_width, 0.0, this->_window_height);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glPointSize(5.0);
	if( (x >= 170.0) && (x <= 300.0) && (y >= 515.0) && (y <= 535.0) )
	{
		DEBUG_MSG << "[ ACTION] Draw Open Surface\n";
		drawing_option = SHAPES::POLYLINE;
		_type_of_surface = SURFACES::OPEN;
		_viewing_direction = VIEWING_DIRECTION::FRONT;
		originalScreen();
		myDisplay(GL_LINE_STRIP);
		glFlush();
		glutPostRedisplay();
	}
	else if( (x >= 20.0) && (x <= 150.0) && (y >= 515.0) && (y <= 535.0) )
	{
		DEBUG_MSG << "[ ACTION] Draw Closed Surface\n";
		drawing_option = SHAPES::POLYGON;
		_type_of_surface = SURFACES::CLOSED;
		_viewing_direction = VIEWING_DIRECTION::OUT;
		originalScreen();
		myDisplay(GL_LINE_LOOP);
		glFlush();
		glutPostRedisplay();
	}
	else if( (x >= 470.0) && (x <= 600.0) && (y >= 540.0) && (y <= 560.0) )
	{
		DEBUG_MSG << "[ ACTION] Clear Screen\n";
		originalScreen();
		clearScreen();
		glFlush();
		glutPostRedisplay();
	}
	else if( (x >= 170.0) && (x <= 300.0) && (y >= 540.0) && (y <= 560.0) )
	{
		originalScreen();
		DEBUG_MSG << "[ ACTION] Delete Last Line\n";
		if (_points.size() > 2)
		{
			_points.pop_back();
		}
		else if (_points.size() == 2)
		{
			_points.clear();
		}
		myObjectDrawing();
		glFlush();
		glutPostRedisplay();
	}
	else if( (x >= 320.0) && (x <= 450.0) && (y >= 540.0) && (y <= 560.0) )
	{
		DEBUG_MSG << "[ ACTION] Calculating angles\n";
		originalScreen();
		myObjectDrawing();
		calculateAngles();
		glFlush();
		glutPostRedisplay();
	}
	else if( (x >= 20.0) && (x <= 150.0) && (y >= 540.0) && (y <= 560.0) )
	{
		DEBUG_MSG << "[ ACTION] Changing the viewing direction\n";
		DEBUG_MSG << "VD: " << _viewing_direction << "\n";
		if(_type_of_surface == SURFACES::OPEN)
		{
			if(_viewing_direction == VIEWING_DIRECTION::FRONT)
			{
				_viewing_direction = VIEWING_DIRECTION::BACK;
			}
			else
			{
				_viewing_direction = VIEWING_DIRECTION::FRONT;
			}
		}
		else
		{
			if(_viewing_direction == VIEWING_DIRECTION::IN)
			{
				_viewing_direction = VIEWING_DIRECTION::OUT;
			}
			else
			{
				_viewing_direction = VIEWING_DIRECTION::IN;
			}
		}
		originalScreen();
		myObjectDrawing();
		glFlush();
		glutPostRedisplay();
	}
	else if( (x >= 620.0) && (x <= 750.0) && (y >= 540.0) && (y <= 560.0) )
	{
		DEBUG_MSG << "[ CLICK_ACTION] Quitting the application\n";
		calculateAngles();
		originalScreen();
		_points.clear();
		glutDestroyWindow(_window);
		glutLeaveMainLoop();
		stopSystem();
		cout << "[ INFO] Exiting TopView Application...\n";
		_exit_app = true;
		return ;
	}
	else if( (x >= 660.0) && (x <= 680.0) && (y >= 520.0) && (y <= 530.0) )
	{
		_max_plane_height++;
		if(_max_plane_height >= _max_allowed_plane_height)
		{
			_max_plane_height = _max_allowed_plane_height;
		}
		originalScreen();
		myObjectDrawing();
		glFlush();
		glutPostRedisplay();
	}
	else if( (x >= 685.0) && (x <= 705.0) && (y >= 520.0) && (y <= 530.0) )
	{
		_max_plane_height--;
		if(_max_plane_height <= _min_allowed_plane_height)
		{
			_max_plane_height = _min_allowed_plane_height;
		}
		originalScreen();
		myObjectDrawing();
		glFlush();
		glutPostRedisplay();
	}
	else
	{
		DEBUG_MSG << "[ CLICK_ACTION] No action available here\n";
		originalScreen();
		myObjectDrawing();
		glFlush();
		glutPostRedisplay();
	}
}

/**
 * @brief Draw either polyline or polygon
 */
void
TopView::myObjectDrawing()
{
	if(drawing_option == SHAPES::POLYLINE)
		myDisplay(GL_LINE_STRIP);
	else if(drawing_option == SHAPES::POLYGON)
		myDisplay(GL_LINE_LOOP);
	else
		DEBUG_MSG << "Wrong Direction\n";
}

/**
 * @brief Draw either polyline or polygon
 */
void
TopView::myObjectDrawingTemp()
{
	if(drawing_option == SHAPES::POLYLINE)
		myDisplay(GL_LINE_STRIP);
	else if(drawing_option == SHAPES::POLYGON)
		myDisplay(GL_LINE_STRIP);
	else
		DEBUG_MSG << "Wrong Direction\n";
}

/**
 * @brief Calculate the angle between every pair of lines in the drawn figure
 */
void
TopView::calculateAngles()
{
	angles.clear();
	direction.clear();
	Point2f point1, point2, point3;
	// Reference: http://www.math-principles.com/2013/07/derivation-angle-between-two-lines.html
	DEBUG_MSG << _viewing_direction << "\n";
	if( _points.size() >= 3 )
	{
		for (int i = 1; i < _points.size()-1; ++i)
		{
			point1 = _points[i-1];
			point2 = _points[i];
			point3 = _points[i+1];
			point1.x = point1.x - 20.0;
			point1.y = point1.y - 50.0;
			point2.x = point2.x - 20.0;
			point2.y = point2.y - 50.0;
			point3.x = point3.x - 20.0;
			point3.y = point3.y - 50.0;
			Line2f line1(point1, point2);
			Line2f line2(point2, point3);
			Orientation src_to_dest = line1.perp().calculateOrientation(drawing_option, _viewing_direction, line2.perp());
			angles.push_back(src_to_dest.angle);
			direction.push_back(src_to_dest.dir);
		}
		// For calculating the angle between last line and first line for polygon
		if(drawing_option == POLYGON)
		{
			point1 = _points[_points.size()-2];
			point2 = _points[_points.size()-1];
			point3 = _points[0];
			point1.x = point1.x - 20.0;
			point1.y = point1.y - 50.0;
			point2.x = point2.x - 20.0;
			point2.y = point2.y - 50.0;
			point3.x = point3.x - 20.0;
			point3.y = point3.y - 50.0;
			Line2f line1(point1, point2);
			Line2f line2(point2, point3);
			Orientation src_to_dest = line1.perp().calculateOrientation(drawing_option, _viewing_direction, line2.perp());
			angles.push_back(src_to_dest.angle);
			direction.push_back(src_to_dest.dir);
		}
		_number_of_planes = angles.size();
	}
	else
	{
		if(_points.size() == 0)
		{
			cout << "[ WARNING] There is no drawing\n";
			_number_of_planes = 0;
		}
		else if(_points.size() == 2)
		{
			cout << "[ WARNING] There is only a single line\n";
			_number_of_planes = 1;
		}
		else
		{

		}
	}
	/*if( drawing_option == POLYLINE && _points.size() > 2)
	{
		_number_of_planes++;
	}
	else if( drawing_option == POLYGON && _points.size() > 1)
	{
		_number_of_planes++;
	}*/
	if ( _points.size() > 2 )
	{
		_number_of_planes++;
	}
	const char* outFilename = "TopViewInfo.txt";
	ofstream outFile;
	// Open the object in writing mode
	outFile.open(outFilename, ios::out);
	// Check if the file is open
	if (!outFile.is_open())
	{
		cerr << "\nFile " << outFilename << " cannot be opened for writing.\n";
		cerr << "Please check if the file is existing and has required permissions ";
		cerr << " for writing.\n";
	}
	outFile << "Rotate-Directions\n";
	for (int i = 0; i < direction.size(); ++i)
	{
		if(i != direction.size()-1)
			outFile << direction[i] << " ";
		else
			outFile << direction[i] << "\n";
	}
	outFile << "\n";
	outFile << "Angles\n";
	for (int i = 0; i < angles.size(); ++i)
	{
		if(i != angles.size()-1)
			outFile << angles[i] << " ";
		else
			outFile << angles[i] << "\n";
	}
	outFile << "\n";
	outFile << "Points-for-" << draw_mode[drawing_option] << "\n";
	for (int i = 0; i < _points.size(); ++i)
	{
		point1 = _points[i];
		point1.x = point1.x - 20.0;
		point1.y = point1.y - 50.0;
		outFile << "(" << point1.x << ", " << point1.y << ")\n";
	}
	if(drawing_option == POLYGON)
	{
		point1 = _points[0];
		point1.x = point1.x - 20.0;
		point1.y = point1.y - 50.0;
		outFile << "(" << point1.x << ", " << point1.y << ")\n";
	}
	outFile << "\n";
	outFile << "Lines-for-" << draw_mode[drawing_option] << "\n";
	for (int i = 1; i < _points.size(); ++i)
	{
		point1 = _points[i-1];
		point2 = _points[i];
		point1.x = point1.x - 20.0;
		point1.y = point1.y - 50.0;
		point2.x = point2.x - 20.0;
		point2.y = point2.y - 50.0;
		outFile << "(" << point1.x << ", " << point1.y << "); "
				<< "(" << point2.x << ", " << point2.y << ")\n";
	}
	if(drawing_option == POLYGON)
	{
		point2 = _points[_points.size()-1];
		point3 = _points[0];
		point2.x = point2.x - 20.0;
		point2.y = point2.y - 50.0;
		point3.x = point3.x - 20.0;
		point3.y = point3.y - 50.0;
		outFile << "(" << point2.x << ", " << point2.y << "); "
				<< "(" << point3.x << ", " << point3.y << ")\n";
	}
	outFile << "\n";
	// Close the file
	outFile.close();
}

/**
 * @brief Draw a point on the screen at (x, y) co-ordinate
 * @param [float] x - X co-ordinate of point
 * @param [float] y - Y co-ordinate of point
 */
void
TopView::drawPoints(float x, float y)
{
	glBegin(GL_POINTS);
		glColor3f(0.0f, 0.0f, 0.0f);
		glPointSize(20.0);
		glVertex2f( x, y );
	glEnd();
}

/**
 * @brief Draw a vector of points as mentioned in the points vector
 * @param [vector<Point2f>] points
 */
void
TopView::drawPoints(vector< Point2f > points)
{
	glBegin(GL_POINTS);
		glColor3f(0.0f, 0.0f, 0.0f);
		glPointSize(20.0);
		for (int i = 0; i < points.size(); ++i)
		{
			glVertex2f( points[i].x, points[i].y );
		}
	glEnd();
}

/**
 * @brief Draw lines as mentioned by points vector
 * @param [GLenum] mode - GL_LINE_STRIP, GL_LINES
 * @param [vector<Point2f>] points - Vector of points
 */
void
TopView::drawLines(GLenum mode, vector< Point2f > points)
{
	glBegin(mode);
		glColor3f(0.0f, 0.0f, 0.0f);
		glPointSize(20.0);
		for (int i = 0; i < points.size(); ++i)
		{
			glVertex2f( points[i].x, points[i].y );
		}
	glEnd();
}

/**
 * @brief Draw the message on the screen
 * @param [string] display_message - Message to be displayed
 * @param [float] x - X co-ordinate of the point where the text starts
 * @param [float] y - Y co-ordinate of the point where the text starts
 */
void
TopView::drawMessage(string display_message, float x, float y)
{
	glColor3f(0.0, 0.0, 0.0);
	glRasterPos2f(x, y);
	for(int i = 0; i < display_message.size(); i++)
	{
		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, display_message[i]);
	}
}

/**
 * @brief Clear the screen, Draw the default screen, Clear the points buffer
 */
void
TopView::clearScreen()
{
	_points.clear();
	originalScreen();
}


/**
 *
 * @param mode
 */
void
TopView::myDisplay(GLenum mode)
{
	DEBUG_MSG << "[FUNC] myDisplay Function\n";

	if(_points.size() >= 1)
	{
		DEBUG_MSG << "Current Points in the points buffer\n";
		for (size_t i = 0; i < _points.size(); i += 1)
		{
			DEBUG_MSG << "(" << _points[i].x << ", " << _points[i].y << "); ";
		}
		DEBUG_MSG << "\n";
		//cout << points.size() << "\n";
		if(_points.size() >= 1)
			drawPoints(_points);
		if(_points.size() >= 2)
			drawLines(mode, _points);
	}
}

void
TopView::getDirections(vector<RotateDirection> &dir)
{
	dir.clear();
	for (int i = 0; i < direction.size(); ++i)
	{
		dir.push_back(direction[i]);
	}
}

void
TopView::getAngles(vector<double> &angle)
{
	angle.clear();
	for (int i = 0; i < angles.size(); ++i)
	{
		angle.push_back(angles[i]);
	}
}

int
TopView::getNumberOfPlanes()
{
	return _number_of_planes;
}

int
TopView::getTypeOfSurface()
{
	return _type_of_surface;
}

int
TopView::getMaxHeightOfPlane()
{
	return _max_plane_height;
}

void
TopView::menuDrawOpenSurface()
{
	_menu_box_points.clear();
	/* Drawing the button "Draw Polylines" */
	makeButton(_menu_box_button_points[MENU_OPTIONS::OPEN_SURFACE], 
				_message_start_points[MENU_OPTIONS::OPEN_SURFACE], 
				"Draw Open Surface");
	_menu_box_points.clear();
}

void
TopView::menuDrawClosedSurface()
{
	_menu_box_points.clear();
	/* Drawing the button "Draw Polygons" */
	makeButton(_menu_box_button_points[MENU_OPTIONS::CLOSED_SURFACE], 
				_message_start_points[MENU_OPTIONS::CLOSED_SURFACE], 
				"Draw Closed Surface");
	_menu_box_points.clear();
}

void
TopView::menuQuitApplication()
{
	_menu_box_points.clear();
	/* Drawing the button "Quit" */
	makeButton(_menu_box_button_points[MENU_OPTIONS::QUIT], 
				_message_start_points[MENU_OPTIONS::QUIT], 
				"Quit");
	_menu_box_points.clear();
}

void
TopView::menuCompletedDrawing()
{
	_menu_box_points.clear();
	/* Drawing the button "Completed Drawing" */
	makeButton(_menu_box_button_points[MENU_OPTIONS::COMPLETED_DRAWING], 
				_message_start_points[MENU_OPTIONS::COMPLETED_DRAWING], 
				"Completed Drawing");
	_menu_box_points.clear();
}

void
TopView::menuChangeViewingDirection()
{
	_menu_box_points.clear();
	/* Drawing the button "Change Direction" */
	makeButton(_menu_box_button_points[MENU_OPTIONS::CHANGE_DIRECTION], 
				_message_start_points[MENU_OPTIONS::CHANGE_DIRECTION], 
				"Change Direction");
	_menu_box_points.clear();
}

void
TopView::menuClearScreen()
{
	_menu_box_points.clear();
	/* Drawing the button "Clear Screen" */
	makeButton(_menu_box_button_points[MENU_OPTIONS::CLEAR_SCREEN], 
				_message_start_points[MENU_OPTIONS::CLEAR_SCREEN], 
				"Clear Screen");
	_menu_box_points.clear();
}

void
TopView::menuDeleteLastLine()
{
	_menu_box_points.clear();
	/* Drawing the button "Delete Last Line" */
	makeButton(_menu_box_button_points[MENU_OPTIONS::DELETE_LAST_LINE], 
				_message_start_points[MENU_OPTIONS::DELETE_LAST_LINE], 
				"Delete Last Line");
	_menu_box_points.clear();
}

void
TopView::menuChangeHeight()
{
	/* Drawing the indicator for incrasing/decresaing the max plane height */
	std::vector<Point2f> increase_value, decrease_value;
	increase_value.push_back( Point2f(660.0, 520.0) );
	increase_value.push_back( Point2f(680.0, 520.0) );
	increase_value.push_back( Point2f(670.0, 530.0) );
	decrease_value.push_back( Point2f(685.0, 530.0) );
	decrease_value.push_back( Point2f(705.0, 530.0) );
	decrease_value.push_back( Point2f(695.0, 520.0) );
	drawLines(GL_TRIANGLES, increase_value);
	drawLines(GL_TRIANGLES, decrease_value);
	increase_value.clear();
	decrease_value.clear();
}

void
TopView::menuShowDrawingMode()
{
	/* */
	string draw_message = "Drawing ";
	if(drawing_option == SHAPES::POLYLINE)
	{
		draw_message += draw_mode[SHAPES::POLYLINE];
		DEBUG_MSG << draw_message << "\n";
		drawMessage(draw_message, 600.0, 10.0);
	}
	else if(drawing_option == SHAPES::POLYGON)
	{
		draw_message += draw_mode[SHAPES::POLYGON];
		DEBUG_MSG << draw_message << "\n";
		drawMessage(draw_message, 600.0, 10.0);
	}
	else
	{

	}
}

void
TopView::menuShowViewingDirection()
{
	/* Drawing the current viewing direction */
	/*std::vector< Point2f > direction_points;
	if(_viewing_direction)
	{
		direction_points.push_back( Point2f(14.0, 10.0) );
		direction_points.push_back( Point2f(38.0, 10.0) );
		direction_points.push_back( Point2f(26.5, 26.0) );
	}
	else
	{
		direction_points.push_back( Point2f(14.0, 26.0) );
		direction_points.push_back( Point2f(38.0, 26.0) );
		direction_points.push_back( Point2f(26.0, 10.0) );
	}
	drawLines(GL_TRIANGLES, direction_points);
	direction_points.clear();*/
	/* */
	string direction_message = "Viewing Direction: ";
	if(_type_of_surface == SURFACES::OPEN)
	{
		if(_viewing_direction == VIEWING_DIRECTION::FRONT)
		{
			direction_message += "Front";
		}
		else
		{
			direction_message += "Back";
		}
	}
	else
	{
		if(_viewing_direction == VIEWING_DIRECTION::IN)
		{
			direction_message += "Inside";
		}
		else
		{
			direction_message += "Outside";
		}
	}
	drawMessage(direction_message, 40.0, 10.0);
}

void
TopView::menuDisplayWarningMessage()
{
	/* Drawing the warning message */
	string display_message_line = "Please draw the top view of the surface in the box below.";
	display_message_line += " Do not maximize the window.";
	// drawMessage(display_message_line, 20.0, 580.0);
	glColor3f(1.0, 0.0, 0.0);
	glRasterPos2f(20.0, 580.0);
	for(int i = 0; i < display_message_line.size(); i++)
	{
		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, display_message_line[i]);
	}
	stringstream ss; ss.str(string());
	ss << _max_plane_height;
	drawMessage(ss.str(), 710.0, 520.0);
}

void
TopView::menuDrawDrawingScreen()
{
	/* */
	std::vector< Point2f > drawing_screen;
	drawing_screen.push_back( Point2f(20.0, 500.0) );
	drawing_screen.push_back( Point2f(20.0, 50.0) );
	drawing_screen.push_back( Point2f(780.0, 50.0) );
	drawing_screen.push_back( Point2f(780.0, 500.0) );
	drawing_screen.push_back( Point2f(20.0, 500.0) );
	drawLines(GL_LINE_STRIP, drawing_screen);
	drawing_screen.clear();
}

void
TopView::menuDrawDrawingScreen(int bottomLeftX, int bottomLeftY,
								int width, int height)
{
	/* */
	std::vector< Point2f > drawing_screen;
	drawing_screen.push_back( Point2f(20.0, 500.0) );
	drawing_screen.push_back( Point2f(20.0, 30.0) );
	drawing_screen.push_back( Point2f(780.0, 30.0) );
	drawing_screen.push_back( Point2f(780.0, 500.0) );
	drawing_screen.push_back( Point2f(20.0, 500.0) );
	drawLines(GL_LINE_STRIP, drawing_screen);
	drawing_screen.clear();
}

void
TopView::menuGenerateButtonBoxes()
{
	/*** Menu Button Boxes ***/
	/** Bottom Menu **/
	/* Closed Surface */
	_menu_box_points.push_back( Point2f(20.0, 535.0) );
	_menu_box_points.push_back( Point2f(20.0, 515.0) );
	_menu_box_points.push_back( Point2f(150.0, 515.0) );
	_menu_box_points.push_back( Point2f(150.0, 535.0) );
	_menu_box_points.push_back( Point2f(20.0, 535.0) );
	_menu_box_button_points.push_back(_menu_box_points);
	_menu_box_points.clear();
	/* Open Surface */
	_menu_box_points.push_back( Point2f(170.0, 535.0) );
	_menu_box_points.push_back( Point2f(170.0, 515.0) );
	_menu_box_points.push_back( Point2f(300.0, 515.0) );
	_menu_box_points.push_back( Point2f(300.0, 535.0) );
	_menu_box_points.push_back( Point2f(170.0, 535.0) );
	_menu_box_button_points.push_back(_menu_box_points);
	_menu_box_points.clear();
	/** Top Menu **/
	/* Change Viewing Direction */
	_menu_box_points.push_back( Point2f(20.0, 560.0) );
	_menu_box_points.push_back( Point2f(20.0, 540.0) );
	_menu_box_points.push_back( Point2f(150.0, 540.0) );
	_menu_box_points.push_back( Point2f(150.0, 560.0) );
	_menu_box_points.push_back( Point2f(20.0, 560.0) );
	_menu_box_button_points.push_back(_menu_box_points);
	_menu_box_points.clear();
	/* Delete Last Line */
	_menu_box_points.push_back( Point2f(170.0, 560.0) );
	_menu_box_points.push_back( Point2f(170.0, 540.0) );
	_menu_box_points.push_back( Point2f(300.0, 540.0) );
	_menu_box_points.push_back( Point2f(300.0, 560.0) );
	_menu_box_points.push_back( Point2f(170.0, 560.0) );
	_menu_box_button_points.push_back(_menu_box_points);
	_menu_box_points.clear();
	/* Completed Drawing */
	_menu_box_points.push_back( Point2f(320.0, 560.0) );
	_menu_box_points.push_back( Point2f(320.0, 540.0) );
	_menu_box_points.push_back( Point2f(450.0, 540.0) );
	_menu_box_points.push_back( Point2f(450.0, 560.0) );
	_menu_box_points.push_back( Point2f(320.0, 560.0) );
	_menu_box_button_points.push_back(_menu_box_points);
	_menu_box_points.clear();
	/* Clear Screen */
	_menu_box_points.push_back( Point2f(470.0, 560.0) );
	_menu_box_points.push_back( Point2f(470.0, 540.0) );
	_menu_box_points.push_back( Point2f(600.0, 540.0) );
	_menu_box_points.push_back( Point2f(600.0, 560.0) );
	_menu_box_points.push_back( Point2f(470.0, 560.0) );
	_menu_box_button_points.push_back(_menu_box_points);
	_menu_box_points.clear();
	/* Quit */
	_menu_box_points.push_back( Point2f(620.0, 560.0) );
	_menu_box_points.push_back( Point2f(620.0, 540.0) );
	_menu_box_points.push_back( Point2f(750.0, 540.0) );
	_menu_box_points.push_back( Point2f(750.0, 560.0) );
	_menu_box_points.push_back( Point2f(620.0, 560.0) );
	_menu_box_button_points.push_back(_menu_box_points);
	_menu_box_points.clear();
	/*** Menu Button Message Start Points ***/
	/** Bottom Menu **/
	/* Closed Surface */
	_message_start_points.push_back( Point2f(25.0, 520.0) );
	/* Open Surface */
	_message_start_points.push_back( Point2f(175.0, 520.0) );
	/** Top Menu **/
	/* Change Viewing Direction */
	_message_start_points.push_back( Point2f(25.0, 545.0) );
	/* Delete Last Line */
	_message_start_points.push_back( Point2f(175.0, 545.0) );
	/* Completed Drawing */
	_message_start_points.push_back( Point2f(325.0, 545.0) );
	/* Clear Screen */
	_message_start_points.push_back( Point2f(475.0, 545.0) );
	/* Quit */
	_message_start_points.push_back( Point2f(625.0, 545.0) );
}

void
TopView::myMouseHover(int x, int y)
{
	glClearColor(1.0f, 1.0f, 1.0f, 0.0);
	glClear(GL_COLOR_BUFFER_BIT);
	int yy = glutGet(GLUT_WINDOW_HEIGHT);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D(0.0, this->_window_width, 0.0, this->_window_height);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glPointSize(5.0);
	originalScreen();
	if( (x >= 20.0) && (x <= 780.0) && (y >= 100.0) && (y <= 550.0) )
	{
		drawMousePosition(x-20.0, 550.0-y);
	}
	myObjectDrawing();
	glFlush();
}

void
TopView::drawMousePosition(int x, int y)
{
	stringstream ss;
	ss.str(std::string());
	ss << "Mouse Position: (" << x << ", " << y << ")";
	Point3f msg_color(0.0, 0.0, 0.0);
	drawMessage(ss.str(), msg_color, 600.0, 30.0);
}

/**
 * @brief Draw the message on the screen
 * @param [string] display_message - Message to be displayed
 * @param [Point3f] x - Color of the message
 * @param [float] x - X co-ordinate of the point where the text starts
 * @param [float] y - Y co-ordinate of the point where the text starts
 */
void
TopView::drawMessage(string display_message, Point3f color, float x, float y)
{
	glColor3f(color.x, color.y, color.z);
	glRasterPos2f(x, y);
	for(int i = 0; i < display_message.size(); i++)
	{
		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, display_message[i]);
	}
}

int
TopView::getViewingDirection()
{
	return _viewing_direction;
}

void
TopView::startSystem()
{
	run_status = true;
	cout << "[ INFO] Starting TopView...\n";
	start();
}

void
TopView::stopSystem()
{
	run_status = false;
	cout << "[ INFO] Stopping TopView...\n";
	join();
}

void
TopView::run()
{
	cout << "[ INFO] Initiating TopView GUI...\n";
	init();
	/*while(run_status)
	{
		cout << "TopView Run\n";
		init();
	}*/
}

bool
TopView::getExitStatus()
{
	return _exit_app;
}

void
TopView::destroy()
{
	delete this;
}