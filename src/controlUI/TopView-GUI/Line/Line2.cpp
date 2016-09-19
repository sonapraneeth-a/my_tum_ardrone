/**
 * @ingroup Line
 */

/*****************************************************************************************
 * Line2.cpp
 *
 *     Created on: 09-Sep-2016
 *  Last Modified: 19-Sep-2016
 *         Author: Sona Praneeth Akula
 *        Project: Data_Structures_and_Algorithms
 *    Description:
 *
 * Date				Author							Modification
 * 09-Sep-2016	Sona Praneeth Akula	
 * 17-Sep-2016	Sona Praneeth Akula 		* Added some extra functionality like negate, orientation etc,
 * 19-Sep-2016	Sona Praneeth Akula 		* Fixed the issue in calculateOrientation() for different direction
 *****************************************************************************************/


#include "Line2.hpp"

/**
 * @brief 2D Line Default Constructor
 * Example
 * @code
 * 	Line2f line;
 * @endcode
 */
Line2f::Line2f()
{
	start.x = 0.0, end.x = 0.0;
	start.y = 0.0, end.y = 0.0;
}

/**
 * @brief 2D Line Constructor with two points
 *
 * @param point1 - start point of the line segment
 * @param point2 - end point of the line segment
 * Example
 * @code
 * 	Point2f pt1, pt2;
 * 	Line2f line(pt1, pt2);
 * 	Line2f *line = new Line2f(pt1, pt2);
 * @endcode
 */
Line2f::Line2f(Point2f point1, Point2f point2)
{
	start = point1;
	end = point2;
}

/**
 * @brief 2D Line Copy Constructor
 * @param line - Line which has to be copied to the current object
 * Example
 * @code
 *  Line2f line1;
 * 	Line2f line2(line1);
 * @endcode
 */
Line2f::Line2f(const Line2f &line) {
	this->start = line.start;
	this->end = line.end;
}

/**
 * @brief Assigning line contents from line to current line object
 * @param line - Line which has to be assigned to the current object
 * @return [Line2f]
 * Example
 * @code
 * 	Line2f line2 = line1;
 * @endcode
 */
Line2f
Line2f::operator=(Line2f line) {
	Line2f copy_line;
	copy_line.start = line.start;
	copy_line.end = line.end;
	return copy_line;
}

/**
 * @brief Comparing two line objects
 * @param line - Second line with which the current line has to be compared
 * @return [bool] TRUE if two lines are same, else FALSE
 * Example
 * @code
 * 	Line2f line1, line2;
 * 	if(line1 == line2) {}
 * @endcode
 */
bool
Line2f::operator==(const Line2f &line) {
	if(this->start == line.start && this->end == line.end) {
		return true;
	}
	else {
		return false;
	}
}

/**
 * @brief Calculates the slope of the line
 * @return [double] Slope of the line. For vertical lines, it return max value of double
 * Example
 * @code
 * 	Line2f line;
 * 	double slope = line.slope();
 * 	Line2f *line = new Line2f();
 * 	double slope = line->slope();
 * @endcode
 */
double
Line2f::slope() const
{
	if ( end.x - start.x != 0.0 )
	{
		return (double)(end.y - start.y)/(end.x - start.x);
	}
	else
	{
		return numeric_limits<double>::max();
	}
}

/**
 * @brief Angle made by the current line object with X axis counter clockwise
 * @return [double] angle in the range [0, 360]
 * @todo Please check if this is within [0, 360]
 * Example
 * @code
 * 	Line2f line;
 * 	double length = line.angleWithXAxis();
 * 	Line2f *line = new Line2f();
 * 	double length = line->angleWithXAxis();
 * @endcode
 */
double
Line2f::angleWithXAxis() const
{
	double degrees = ( atan2(end.y - start.y, end.x - start.x) * 180.0 ) / PI;
	if(degrees <= 0.0)
	{
		degrees = 180.0 + ( 180.0 - fabs(degrees) );
	}
	if(degrees >= 360.0)
	{
		int divisor = degrees/360;
		degrees = (360.0*divisor) - degrees;
	}
	return degrees;
}

/**
 * @brief Length of the line
 * @return [double] Length of the line
 * Example
 * @code
 * 	Line2f line;
 * 	double length = line.length();
 * 	Line2f *line = new Line2f();
 * 	double length = line->length();
 * @endcode
 */
double
Line2f::length() const
{
	return sqrt(this->length2());
}

/**
 * @brief Squared Length of the line
 * @return [double] Squared length of the line
 * Example
 * @code
 * 	Line2f line;
 * 	double length2 = line.length2();
 * 	Line2f *line = new Line2f();
 * 	double length2 = line->length2();
 * @endcode
 */
double
Line2f::length2() const
{
	return ((end.x - start.x) * (end.x - start.x))
			+ ((end.y - start.y) * (end.y - start.y));
}

/**
 * @brief Finds the angle between two lines
 * @param [int] viewing_direction - Whether you look from front or back (from screen)
 * 				TOP/BOTTOM
 * @param [Line2f] line - Second line
 * @return [double] Angle between current line object and line, Viewing direction
 * 					decides whether to return interior angle or exterior angle
 * Example
 * @code
 * 	Line2f line1, line2;
 * 	double angle = line1.angleBetweenLines(TOP, line2);
 * @endcode
 */
double
Line2f::angleBetweenLines(int viewing_direction, const Line2f &dest_line)
{
	double answer = 0.0;
	DEBUG_MSG << "[ FUNC] angleBetweenLines\n";
	DEBUG_MSG << "Viewing Direction: " << viewing_direction << "\n";
	DEBUG_MSG << "This line: ";
	DEBUG_MSG << "(" << start.x << ", " << start.y << "); ";
	DEBUG_MSG << "(" << end.x << ", " << end.y << ");\n";
	DEBUG_MSG << this->angleWithXAxis() << "\n";
	DEBUG_MSG << "Called line: ";
	DEBUG_MSG << "(" << dest_line.start.x << ", " << dest_line.start.y << "); ";
	DEBUG_MSG << "(" << dest_line.end.x << ", " << dest_line.end.y << ");\n";
	DEBUG_MSG << dest_line.angleWithXAxis() << "\n";
	answer = -this->angleWithXAxis() + dest_line.angleWithXAxis();
	DEBUG_MSG << "Answer: " << answer << "\n";
	if(viewing_direction == 0)
	{
		int divisor = answer / 360;
		answer = answer + (360*divisor);
	}
	else
	{
		int divisor = answer / 360;
		answer = -answer + (360*divisor);
		// answer = -360.0 + answer;
	}
	return answer;
}

/**
 * @brief Finds the angle between two lines
 * @param [Line2f] line - Second line
 * @return [double] Angle between current line object and line
 * Example
 * @code
 * 	Line2f line1, line2;
 * 	double angle = line1.angleBetweenLines(line2);
 * @endcode
 */
double
Line2f::angleBetweenLines(const Line2f &dest_line)
{
	double answer = 0.0;
	DEBUG_MSG << "[ FUNC] angleBetweenLines\n";
	DEBUG_MSG << "This line: ";
	DEBUG_MSG << "(" << start.x << ", " << start.y << "); ";
	DEBUG_MSG << "(" << end.x << ", " << end.y << ");\n";
	DEBUG_MSG << this->angleWithXAxis() << "\n";
	DEBUG_MSG << "Called line: ";
	DEBUG_MSG << "(" << dest_line.start.x << ", " << dest_line.start.y << "); ";
	DEBUG_MSG << "(" << dest_line.end.x << ", " << dest_line.end.y << ");\n";
	DEBUG_MSG << dest_line.angleWithXAxis() << "\n";
	answer = -this->angleWithXAxis() + dest_line.angleWithXAxis();
	DEBUG_MSG << "Answer: " << answer << "\n";
	int divisor = answer / 360;
	answer = answer + (360*divisor);
	return answer;
}

/**
 * @brief Finds the orientation with which current line has to rotate to
 * 			orient itslef with passed line
 * @param [Line2f] line - Second line
 * @return [RotateDirection] Direction of rotation
 * Example
 * @code
 * 	Line2f line1, line2;
 * 	double angle = line1.rotate(line2);
 * @endcode
 */
RotateDirection
Line2f::rotate(const Line2f &line)
{
	RotateDirection d = COUNTERCLOCKWISE;
	Line2f origin = this->perp();
	Line2f dest = line.perp();
	origin = origin.translate(Point2f(0.0, 0.0));
	dest = dest.translate(Point2f(0.0, 0.0));
	int origin_angle = (int)origin.angleWithXAxis();
	int dest_angle = (int)dest.angleWithXAxis();
	DEBUG_MSG << "Origin Line: " << *this;
	DEBUG_MSG << "Destination Line: " << line;
	DEBUG_MSG << "Origin Normal Line: " << origin;
	DEBUG_MSG << "Destination Normal Line: " << dest;
	DEBUG_MSG << "Origin Angle: " << origin_angle << ", Destination Angle: " << dest_angle << "\n";
	if( ( origin_angle >= 180.0 && origin_angle <= 360.0) && (dest_angle >= 0.0 && dest_angle <= 180.0) )
	{
		dest_angle = 360.0 + dest_angle;
	}
	if(origin_angle >= dest_angle)
	{
		d = CLOCKWISE;
	}
	cout << "\n";
	return d;
}

/**
 * @brief Finds the line perpendicular to the current line
 * @return [Line2f] Perpendicualr line to the current line
 * Example
 * @code
 * 	Line2f line1;
 * 	Line2f line2 = line1.perp();
 * @endcode
 */
Line2f
Line2f::perp() const
{
	Line2f perpendicular;
	perpendicular.start = this->end;
	double direction_x = -(this->end.y - this->start.y);
	double direction_y = this->end.x - this->start.x;
	Point2f perp_direction(direction_x, direction_y);
	perpendicular.end.x = perpendicular.start.x + perp_direction.x;
	perpendicular.end.y = perpendicular.start.y + perp_direction.y;
	return perpendicular;
}

/**
 * @brief Find the line in the opposite direction to current line
 * @return [Line2f] Line in the opposite direction
 * Example
 * @code
 * 	Line2f line1, line2;
 * 	line2 = line1.negate();
 * @endcode
 */
Line2f
Line2f::negate() const
{
	// Line with opposite direction
	Line2f opposite_line;
	opposite_line.start = this->start;
	double direction_y = -(this->end.y - this->start.y);
	double direction_x = -(this->end.x - this->start.x);
	Point2f opposite_line_dir(direction_x, direction_y);
	opposite_line.end.x = opposite_line.start.x + opposite_line_dir.x;
	opposite_line.end.y = opposite_line.start.y + opposite_line_dir.y;
	return opposite_line;
}

/**
 * @brief Translate the line such that start of the line is at origin
 * @param [Point2f] origin - Point to which line has to be translated
 * @return [Line2f] Translated line
 * Example
 * @code
 * 	Line2f line1, line2;
 * 	line2 = line1.translate(Point2f(0.0, 0.0));
 * @endcode
 */
Line2f
Line2f::translate(const Point2f &origin)
{
	Line2f translated_line;
	translated_line.start.x = this->start.x - origin.x;
	translated_line.start.y = this->start.x - origin.y;
	translated_line.end.x = this->end.x - origin.x;
	translated_line.end.y = this->end.y - origin.y;
	return translated_line;
}

/**
 * @brief Change the direction of the line
 * Example
 * @code
 * 	Line2f line1;
 * 	line1.swapStartEnd();
 * @endcode
 */
void
Line2f::swapStartEnd()
{
	Point2f temp = this->start;
	this->start = this->end;
	this->end = temp;
}

/**
 * @brief Calculate the direction which quadcopter has to rotate to see the new plane
 * @param [int] drawing_option - Polygon/Polyline
 * @param [int] viewing_option - Top/Bottom
 * @param [Line2f] dest_line - Destination line
 * @return [Orientation] Direction as well as the angle with which the quadcopter has
 * 					to rotate to see the new plane
 * Example
 * @code
 * 	Line2f line1, line2;
 * 	Orientation or = line1.calculateOrientation(POLYGON, TOP, line2);
 * @endcode
 */
Orientation
Line2f::calculateOrientation(int drawing_option, int viewing_direction, const Line2f &dest_line)
{
	Line2f curr_line = *this;
	double angle = 0.0;
	Orientation answer;
	angle = curr_line.angleBetweenLines(dest_line.shiftTo(curr_line.start));
	/*cout << curr_line;
	cout << dest_line;*/
	cout << "\n";
	if(angle < 0.0)
	{
		if(fabs(angle) > 180.0)
		{
			answer.dir = COUNTERCLOCKWISE;
			answer.angle = 360.0+angle;
			/*cout << "Angle: " << angle << "; ";
			cout << "Orientation: ANTI-CLOCKWISE\n";*/
		}
		else
		{
			answer.dir = CLOCKWISE;
			answer.angle = angle;
			/*cout << "Angle: " << angle << "; ";
			cout << "Orientation: CLOCKWISE\n";*/
		}
	}
	else
	{
		answer.dir = COUNTERCLOCKWISE;
		answer.angle = angle;
		/*cout << "Angle: " << angle << "; ";
		cout << "Orientation: ANTI-CLOCKWISE\n";*/
	}
	if(viewing_direction == 1 || viewing_direction == 3)
	{
		angle = (-1.0)*angle;
		answer.angle = angle;
		if(answer.dir == CLOCKWISE)
		{
			answer.dir = COUNTERCLOCKWISE;
		}
		else if(answer.dir == COUNTERCLOCKWISE)
		{
			answer.dir = CLOCKWISE;
		}
	}
	// cout << answer.dir << ", " << answer.angle << "\n";
	return answer;
}


/**
 * @brief Find the line which is shifted to a point keeping the direction same
 * @param [Point2f] point - Point to which the line has to be shifted
 * @return [Line2f] Shifted line
 * Example
 * @code
 * 	Line2f line1, line2;
 * 	line2 = line1.shiftTo(Point2f(x, y));
 * @endcode
 */
Line2f
Line2f::shiftTo(const Point2f &point) const
{
	Line2f shifted_line;
	Line2f curr_line = *this;
	int shift_x = curr_line.start.x - point.x;
	int shift_y = curr_line.start.y - point.y;
	shifted_line.start.x = curr_line.start.x - shift_x;
	shifted_line.start.y = curr_line.start.y - shift_y;
	shifted_line.end.x = curr_line.end.x - shift_x;
	shifted_line.end.y = curr_line.end.y - shift_y;
	return shifted_line;
}
