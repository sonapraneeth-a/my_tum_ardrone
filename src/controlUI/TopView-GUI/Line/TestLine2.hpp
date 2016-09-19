/**
 * @ingroup Line
 * @brief Code for testing Line2 class
 */

/*****************************************************************************************
 * TestLine2.hpp
 *
 *     Created on: 17-Sep-2016
 *  Last Modified: 17-Sep-2016
 *         Author: Sona Praneeth Akula
 *        Project: Data_Structures_and_Algorithms
 *        Description: 
 *
 * Date				Author							Modification
 * 17-Sep-2016	Sona Praneeth Akula	 		* Added some testing code
 *****************************************************************************************/

#ifndef TEST_LINE2_HPP_
#define TEST_LINE2_HPP_

#include "../AllHeaders.hpp"
#include "../DebugUtility/DebugUtility.hpp"
#include "Line2.hpp"

void
TestAngleWithXAxis()
{
	Point2f origin(0.0, 0.0);
	Point2f point1((float)1/2, 0);
	Point2f point2((float)1/2, (float)sqrt(3)/2);
	Point2f point3(0, (float)sqrt(3)/2);
	Point2f point4((float)-1/2, (float)sqrt(3)/2);
	Point2f point5((float)-1/2, 0);
	Point2f point6((float)-1/2, (float)-sqrt(3)/2);
	Point2f point7(0, (float)-sqrt(3)/2);
	Point2f point8((float)1/2, (float)-sqrt(3)/2);

	Line2f line1(origin, point1);
	Line2f line2(origin, point2);
	Line2f line3(origin, point3);
	Line2f line4(origin, point4);
	Line2f line5(origin, point5);
	Line2f line6(origin, point6);
	Line2f line7(origin, point7);
	Line2f line8(origin, point8);

	cout << "Line1: " << line1;
	cout << "Angle of Line 1 with X axis: " << line1.angleWithXAxis() << "\n"; 
	cout << "Line2: " << line2;
	cout << "Angle of Line 2 with X axis: " << line2.angleWithXAxis() << "\n"; 
	cout << "Line3: " << line3;
	cout << "Angle of Line 3 with X axis: " << line3.angleWithXAxis() << "\n"; 
	cout << "Line4: " << line4;
	cout << "Angle of Line 4 with X axis: " << line4.angleWithXAxis() << "\n"; 
	cout << "Line5: " << line5;
	cout << "Angle of Line 5 with X axis: " << line5.angleWithXAxis() << "\n"; 
	cout << "Line6: " << line6;
	cout << "Angle of Line 6 with X axis: " << line6.angleWithXAxis() << "\n"; 
	cout << "Line7: " << line7;
	cout << "Angle of Line 7 with X axis: " << line7.angleWithXAxis() << "\n"; 
	cout << "Line8: " << line8;
	cout << "Angle of Line 8 with X axis: " << line8.angleWithXAxis() << "\n"; 

	return ;
}

void
TestAngleBetweenLines()
{
	Point2f origin(0.0, 0.0);
	Point2f point1((float)1/2, 0);
	Point2f point2((float)1/2, (float)sqrt(3)/2);
	Point2f point3(0, (float)sqrt(3)/2);
	Point2f point4((float)-1/2, (float)sqrt(3)/2);
	Point2f point5((float)-1/2, 0);
	Point2f point6((float)-1/2, (float)-sqrt(3)/2);
	Point2f point7(0, (float)-sqrt(3)/2);
	Point2f point8((float)1/2, (float)-sqrt(3)/2);

	Line2f line1(origin, point1);
	Line2f line2(origin, point2);
	Line2f line3(origin, point3);
	Line2f line4(origin, point4);
	Line2f line5(origin, point5);
	Line2f line6(origin, point6);
	Line2f line7(origin, point7);
	Line2f line8(origin, point8);

	cout << "Line1: " << line1;
	cout << "Line2: " << line2;
	cout << "Line3: " << line3;
	cout << "Line4: " << line4;
	cout << "Line5: " << line5;
	cout << "Line6: " << line6;
	cout << "Line7: " << line7;
	cout << "Line8: " << line8;
	cout << "\n";

	/*cout << "Angle between Line 1 and Line 2: " << line1.angleBetweenLines(0, line2) << "\n";
	cout << "Angle between Line 2 and Line 3: " << line2.angleBetweenLines(0, line3) << "\n";
	cout << "Angle between Line 1 and Line 8: " << line1.angleBetweenLines(0, line8) << "\n";
	cout << "Angle between Line 6 and Line 8: " << line6.angleBetweenLines(0, line8) << "\n";
	cout << "\n";

	cout << "Angle between Line 2 and Line 1: " << line2.angleBetweenLines(0, line1) << "\n";
	cout << "Angle between Line 3 and Line 2: " << line3.angleBetweenLines(0, line2) << "\n";
	cout << "Angle between Line 8 and Line 1: " << line8.angleBetweenLines(0, line1) << "\n";
	cout << "Angle between Line 8 and Line 6: " << line8.angleBetweenLines(0, line6) << "\n";
	cout << "\n";

	cout << "Angle between Line 1 and Line 2: " << line1.angleBetweenLines(1, line2) << "\n";
	cout << "Angle between Line 2 and Line 3: " << line2.angleBetweenLines(1, line3) << "\n";
	cout << "Angle between Line 1 and Line 8: " << line1.angleBetweenLines(1, line8) << "\n";
	cout << "Angle between Line 6 and Line 8: " << line6.angleBetweenLines(1, line8) << "\n";
	cout << "\n";

	cout << "Angle between Line 2 and Line 1: " << line2.angleBetweenLines(1, line1) << "\n";
	cout << "Angle between Line 3 and Line 2: " << line3.angleBetweenLines(1, line2) << "\n";
	cout << "Angle between Line 8 and Line 1: " << line8.angleBetweenLines(1, line1) << "\n";
	cout << "Angle between Line 8 and Line 6: " << line8.angleBetweenLines(1, line6) << "\n";
	cout << "\n";*/

	cout << "Angle between Line 8 and Line 6: " << line8.angleBetweenLines(0, line6) << "\n";
	cout << "Angle between Line 4 and Line 2: " << line4.angleBetweenLines(0, line2) << "\n";
	cout << "Angle between Line 8 and Line 6: " << line8.angleBetweenLines(1, line6) << "\n";
	cout << "Angle between Line 4 and Line 2: " << line4.angleBetweenLines(1, line2) << "\n";

	return ;
}

void
TestPerp()
{
	Point2f origin(0.0, 0.0);
	Point2f point1((float)1/2, 0);
	Point2f point2((float)1/2, (float)sqrt(3)/2);
	Point2f point3(0, (float)sqrt(3)/2);
	Point2f point4((float)-1/2, (float)sqrt(3)/2);
	Point2f point5((float)-1/2, 0);
	Point2f point6((float)-1/2, (float)-sqrt(3)/2);
	Point2f point7(0, (float)-sqrt(3)/2);
	Point2f point8((float)1/2, (float)-sqrt(3)/2);

	Line2f line1(origin, point1);
	Line2f line2(origin, point2);
	Line2f line3(origin, point3);
	Line2f line4(origin, point4);
	Line2f line5(origin, point5);
	Line2f line6(origin, point6);
	Line2f line7(origin, point7);
	Line2f line8(origin, point8);

	cout << "Line 1: " << line1;
	cout << "Line 1 Perp: " << line1.perp();
	cout << "Angle between Line 1 and Perp: " << line1.angleBetweenLines(line1.perp()) << "\n";
	cout << "Line 2: " << line2;
	cout << "Line 2 Perp: " << line2.perp();
	cout << "Angle between Line 2 and Perp: " << line2.angleBetweenLines(line2.perp()) << "\n";
	cout << "Line 3: " << line3;
	cout << "Line 3 Perp: " << line3.perp();
	cout << "Angle between Line 3 and Perp: " << line3.angleBetweenLines(line3.perp()) << "\n";
	return ;
}

void
TestNegate()
{
	Point2f origin(0.0, 0.0);
	Point2f point1((float)1/2, 0);
	Point2f point2((float)1/2, (float)sqrt(3)/2);
	Point2f point3(0, (float)sqrt(3)/2);
	Point2f point4((float)-1/2, (float)sqrt(3)/2);
	Point2f point5((float)-1/2, 0);
	Point2f point6((float)-1/2, (float)-sqrt(3)/2);
	Point2f point7(0, (float)-sqrt(3)/2);
	Point2f point8((float)1/2, (float)-sqrt(3)/2);

	Line2f line1(origin, point1);
	Line2f line2(origin, point2);
	Line2f line3(origin, point3);
	Line2f line4(origin, point4);
	Line2f line5(origin, point5);
	Line2f line6(origin, point6);
	Line2f line7(origin, point7);
	Line2f line8(origin, point8);

	cout << "Line 1: " << line1;
	cout << "Line 1 Negate: " << line1.negate();
	cout << "Angle between Line 1 and Negate: " << line1.angleBetweenLines(line1.negate()) << "\n";
	cout << "Line 2: " << line2;
	cout << "Line 2 Negate: " << line2.negate();
	cout << "Angle between Line 2 and Negate: " << line2.angleBetweenLines(line2.negate()) << "\n";
	cout << "Line 3: " << line3;
	cout << "Line 3 Negate: " << line3.negate();
	cout << "Angle between Line 3 and Negate: " << line3.angleBetweenLines(line3.negate()) << "\n";
	return ;
}

void
TestNegatePerp()
{
	Point2f origin(0.0, 0.0);
	Point2f point1((float)1/2, 0);
	Point2f point2((float)1/2, (float)sqrt(3)/2);
	Point2f point3(0, (float)sqrt(3)/2);
	Point2f point4((float)-1/2, (float)sqrt(3)/2);
	Point2f point5((float)-1/2, 0);
	Point2f point6((float)-1/2, (float)-sqrt(3)/2);
	Point2f point7(0, (float)-sqrt(3)/2);
	Point2f point8((float)1/2, (float)-sqrt(3)/2);

	Line2f line1(origin, point1);
	Line2f line2(origin, point2);
	Line2f line3(origin, point3);
	Line2f line4(origin, point4);
	Line2f line5(origin, point5);
	Line2f line6(origin, point6);
	Line2f line7(origin, point7);
	Line2f line8(origin, point8);

	cout << "Line 1: " << line1;
	cout << "Line 1 Negate: " << line1.negate();
	cout << "Line 1 Perp: " << line1.perp();
	cout << "Line 1 Mix: " << line1.negate().perp();
	cout << "Angle between Line 1 and Negate: " << line1.angleBetweenLines(line1.negate()) << "\n";
	cout << "Angle between Line 1 and Perp: " << line1.angleBetweenLines(line1.perp()) << "\n";
	cout << "Angle between Line 1 and Mix: " << line1.angleBetweenLines(line1.negate().perp()) << "\n";
	cout << "Line 2: " << line2;
	cout << "Line 2 Negate: " << line2.negate();
	cout << "Line 2 Perp: " << line2.perp();
	cout << "Line 2 Mix: " << line2.negate().perp();
	cout << "Angle between Line 2 and Negate: " << line2.angleBetweenLines(line2.negate()) << "\n";
	cout << "Angle between Line 2 and Perp: " << line2.angleBetweenLines(line2.perp()) << "\n";
	cout << "Angle between Line 2 and Mix: " << line2.angleBetweenLines(line2.negate().perp()) << "\n";
	cout << "Line 3: " << line3;
	cout << "Line 3 Negate: " << line3.negate();
	cout << "Line 3 Perp: " << line3.perp();
	cout << "Line 3 Mix: " << line3.negate().perp();
	cout << "Angle between Line 3 and Negate: " << line3.angleBetweenLines(line3.negate()) << "\n";
	cout << "Angle between Line 3 and Perp: " << line3.angleBetweenLines(line3.perp()) << "\n";
	cout << "Angle between Line 3 and Mix: " << line3.angleBetweenLines(line3.negate().perp()) << "\n";

	return ;
}

void
TestCalculateOrientation()
{
	Point2f origin(0.0, 0.0);
	Point2f point1((float)1/2, 0);
	Point2f point2((float)1/2, (float)sqrt(3)/2);
	Point2f point3(0, (float)sqrt(3)/2);
	Point2f point4((float)-1/2, (float)sqrt(3)/2);
	Point2f point5((float)-1/2, 0);
	Point2f point6((float)-1/2, (float)-sqrt(3)/2);
	Point2f point7(0, (float)-sqrt(3)/2);
	Point2f point8((float)1/2, (float)-sqrt(3)/2);

	Line2f line1(origin, point1);
	Line2f line2(origin, point2);
	Line2f line3(origin, point3);
	Line2f line4(origin, point4);
	Line2f line5(origin, point5);
	Line2f line6(origin, point6);
	Line2f line7(origin, point7);
	Line2f line8(origin, point8);

	// POLYLINE - 0, POLYGON - 1
	// TOP - 0, BOTTOM - 1
	cout << "Line 1: " << line1;
	cout << "Line 2: " << line2;
	Orientation or1;
	or1 = line1.calculateOrientation(1, 0, line2);
	cout << "Orientation: (" << or1.angle << ", " << or1.dir << ")\n";
}

void
TestLineOrientation()
{
	vector< Point2f > points;
	points.push_back(Point2f(319, 439));
	points.push_back(Point2f(169, 179));
	points.push_back(Point2f(547, 181));
	points.push_back(Point2f(670, 431));
	points.push_back(Point2f(319, 439));
	// Line2f line1(Point2f(169, 179), Point2f(429, 29));
	// Line2f line2(Point2f(169, 179), Point2f(167, 557));
	// Orientation src_to_dest = line1.calculateOrientation(1, 0, line2);
	if( points.size() >= 3 )
	{
		for (int i = 1; i < points.size()-1; ++i)
		{
			cout << "Set " << i << "\n";
			Line2f line1(points[i-1], points[i]);
			Line2f line2(points[i], points[i+1]);
			Orientation src_to_dest = line1.perp().calculateOrientation(1, 0, line2.perp());
			/*angles.push_back(src_to_dest.angle);
			direction.push_back(src_to_dest.dir);*/
			cout << "\n";
		}
	}
	Line2f line1(points[points.size()-2], points[points.size()-1]);
	Line2f line2(points[points.size()-1], points[1]);
	cout << "Line 1: " << line1;
	cout << "Line 2: " << line2;
	Orientation src_to_dest = line1.perp().calculateOrientation(1, 0, line2.perp());
	/*angles.push_back(src_to_dest.angle);
	direction.push_back(src_to_dest.dir);*/
	/*Line2f line3(points[points.size()-1], points[1]);
	Line2f line4(points[0], points[1]);
	src_to_dest = line3.perp().calculateOrientation(1, 0, line4.perp());*/
	/*angles.push_back(src_to_dest.angle);
	direction.push_back(src_to_dest.dir);*/
}

void
TestShiftTo()
{
	/*Line2f line1(Point2f(319, 439), Point2f(169, 179));
	Line2f line2(Point2f(169, 179), Point2f(547, 181));
	Line2f line1_perp = line1.perp();
	Line2f line2_perp = line2.perp();
	Line2f shift_line2_perp = line2_perp.shiftTo(line1_perp.start);
	cout << "Line 1: " << line1;
	cout << "Line 1 Perp: " << line1_perp;
	cout << "Line 2: " << line2;
	cout << "Line 2 Perp: " << line2_perp;
	cout << "Shifted Line 2 Perp: " << shift_line2_perp;
	cout << "Angle: " << line1_perp.angleBetweenLines(shift_line2_perp) << "\n";*/
	Line2f line3(Point2f(169, 179), Point2f(429, 29));
	Line2f line4(Point2f(169, 179), Point2f(167, 557));
	Orientation or1 = line3.calculateOrientation(1, 0, line4);
	cout << "Angle: " << or1.angle << "\n";
	cout << "Dir: " << or1.dir << "\n";
	cout << "Angle: " << line3.angleBetweenLines(line4) << "\n";
	return ;
}


#endif /* TEST_LINE2_HPP_ */
