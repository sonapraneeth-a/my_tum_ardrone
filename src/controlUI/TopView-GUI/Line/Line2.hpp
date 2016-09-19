/**
 * @defgroup Line
 * @ingroup Line
 * @brief Code for handling lines in 2D
 */

/*****************************************************************************************
 * Line2.hpp
 *
 *     Created on: 09-Sep-2016
 *  Last Modified: 17-Sep-2016
 *         Author: Sona Praneeth Akula
 *        Project: Data_Structures_and_Algorithms
 *        Description: 
 *
 * Date				Author							Modification
 * 09-Sep-2016	Sona Praneeth Akula	 	Designed a line class with functions isParallel,
 * 									 	intersect, angleBetweenLines, slope, angleWithXaxis, length
 * 17-Sep-2016	Sona Praneeth Akula	 	* Added Orientation Structure
 * 										* Added new functions in Line2f to support certain calculations
 *****************************************************************************************/
#ifndef LINE2_HPP_
#define LINE2_HPP_

#include "../AllHeaders.hpp"
#include "../DebugUtility/DebugUtility.hpp"

enum LineDirection {HORIZONTAL, VERTICAL, ASCENDING, DESCENDING};
enum RotateDirection {CLOCKWISE, COUNTERCLOCKWISE};

#define PI 3.14159265

/**
 * @brief Structure which captures the Orientation which quadcopter has to take
 * 			to orient itself with the new plane
 * @details
 */
struct Orientation
{
	double angle;
	RotateDirection dir;
};

/**
 * @brief Class for handling Line/Line-Segment related functions
 * @details
 */class Line2f
{
	public:
		Point2f start, end;

		Line2f();
		Line2f(Point2f point1, Point2f point2);
		Line2f(const Line2f &line);
		Line2f operator=(Line2f line);
		bool operator==(const Line2f &line);

		Point2f intersect(const Line2f &line);
		bool isParallel(const Line2f &line);
		double angleBetweenLines(int viewing_direction, const Line2f &dest_line);
		double angleBetweenLines(const Line2f &dest_line);

		friend ostream& operator<<(ostream& stream, const Line2f &line) {
			stream << "[ " << line.start << ", " << line.end << " ]\n";
			return stream;
		}

		/*friend istream& operator>>(istream& stream, const Line2f &line) {
			stream >> line.start.x >> line.start.y;
			stream >> line.end.x >> line.end.y;
			return stream;
		}*/

		double slope() const;
		void swapStartEnd();
		double angleWithXAxis() const;
		double interiorAngle(Line2f line2);
		double length() const;
		double length2() const;
		Line2f perp() const;
		Line2f translate(const Point2f &origin);
		RotateDirection rotate(const Line2f &line);
		Line2f negate() const;
		Orientation calculateOrientation(int drawing_option, int viewing_direction, const Line2f &line);
		Line2f shiftTo(const Point2f &point) const;
};


#endif /* LINE2_HPP_ */
