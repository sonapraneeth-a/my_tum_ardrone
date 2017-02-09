#pragma once
/*****************************************************************************************
 * VisionHelper.h
 *
 *       Created on: 09-Jan-2017
 *    Last Modified: 09-Jan-2017
 *   Current Author: Sona Praneeth Akula
 *          Project: Quadcopter
 *      Description: 
 *
 * Date       Author              Modification
 * 09-Jan-2017  Sona Praneeth Akula     Added vision related helper functions
                                        from HelperFunctions.h
 *****************************************************************************************/

#ifndef _VISION_HELPER_H
#define _VISION_HELPER_H

#include "Headers.h"
/**
 * @brief @todo Why R-z is being returned which is identity
 * @details
 * @param [float] roll - 
 * @param [float] pitch - 
 * @param [float] yaw - 
 * @return
 */
inline static cv::Mat
getRotationMatrix(float roll, float pitch, float yaw)
{
  roll  = roll*M_PI/180;
  pitch = pitch*M_PI/180;
  yaw   = (-1.0)*yaw*M_PI/180;
  cv::Mat R_x = cv::Mat::eye(3,3, CV_32F);
  cv::Mat R_y = cv::Mat::eye(3,3, CV_32F);
  cv::Mat R_z = cv::Mat::eye(3,3, CV_32F);
  /*
  R_z.at<float>(0,0) = cos(yaw);
  R_z.at<float>(0,2) = -sin(yaw);
  R_z.at<float>(2,0) = sin(yaw);
  R_z.at<float>(2,2) = cos(yaw);
  */
  return R_z; // Roll & Pitch are not reliable, also most of the time we will have yaw only
}

/**
 * @brief Converts a point given from quadcopter's current position as origin to quadcopter's actual origin
 * @details This is required for generating points for quadcopter's autonomous movement
 */
inline static void
convertWRTQuadcopterOrigin(const vector<double> &current_pos_of_drone,
                      const vector<double> &dest_pos_of_drone,
                      vector<double> &ac_dest_pos_of_drone)
{
  assert(current_pos_of_drone.size() == dest_pos_of_drone.size());
  ac_dest_pos_of_drone.clear();
  Mat rotationMatrix = Mat::eye(3, 3, CV_64F);
  double angle = current_pos_of_drone[3];
  angle = (angle*3.14)/180.0;
  rotationMatrix.at<double>(0, 0) = cos(angle);
  rotationMatrix.at<double>(0, 1) = -sin(angle);
  rotationMatrix.at<double>(1, 0) = sin(angle);
  rotationMatrix.at<double>(1, 1) = cos(angle);
  // cout << "[ DEBUG] Rotation Matrix: " << rotationMatrix << "\n";
  Mat translationVector(3, 1, DataType<double>::type);
  translationVector.at<double>(0, 0) = current_pos_of_drone[0];
  translationVector.at<double>(1, 0) = current_pos_of_drone[1];
  translationVector.at<double>(2, 0) = current_pos_of_drone[2];
  // cout << "[ DEBUG] Translation Vector: " << translationVector << "\n";
  Mat dest_point_drone_origin_mat(3, 1, DataType<double>::type);
  dest_point_drone_origin_mat.at<double>(0, 0) = dest_pos_of_drone[0];
  dest_point_drone_origin_mat.at<double>(1, 0) = dest_pos_of_drone[1];
  dest_point_drone_origin_mat.at<double>(2, 0) = dest_pos_of_drone[2];
  Mat sub = rotationMatrix*translationVector;
  Mat b = dest_point_drone_origin_mat + sub;
  // How do I solve Ax = b?
  // Will this always be solvable?
  Mat x = rotationMatrix.inv() * b;
  ac_dest_pos_of_drone.push_back(x.at<double>(0, 0));
  ac_dest_pos_of_drone.push_back(x.at<double>(1, 0));
  ac_dest_pos_of_drone.push_back(x.at<double>(2, 0));
  ac_dest_pos_of_drone.push_back(current_pos_of_drone[3]+dest_pos_of_drone[3]);
}

/**
 * @brief Converts points given from quadcopter's actual origin to quadcopter's current position as origin
 * @details This is required for sorting points based on x centroid;
 */
inline static void
convertWRTCurrentQuadcopterOrigin(const vector<double> &current_pos_of_drone,
                  const vector< vector<Point3f> > &points,
                  vector< vector<Point3f> > &output_points)
{
  assert(current_pos_of_drone.size() == 4);
  vector<Point3f> current_points;
  Mat rotationMatrix = Mat::eye(3, 3, CV_64F);
  double angle = current_pos_of_drone[3];
  angle = (angle*3.14)/180.0;
  rotationMatrix.at<double>(0, 0) = cos(angle);
  rotationMatrix.at<double>(0, 1) = -sin(angle);
  rotationMatrix.at<double>(1, 0) = sin(angle);
  rotationMatrix.at<double>(1, 1) = cos(angle);
  // cout << "[ DEBUG] Rotation Matrix: " << rotationMatrix << "\n";
  Mat translationVector(3, 1, DataType<double>::type);
  translationVector.at<double>(0, 0) = current_pos_of_drone[0];
  translationVector.at<double>(1, 0) = current_pos_of_drone[1];
  translationVector.at<double>(2, 0) = current_pos_of_drone[2];
  // cout << "[ DEBUG] Translation Vector: " << translationVector << "\n";
  Mat x = rotationMatrix*translationVector;
  Mat c(3, 1, DataType<double>::type);
  for (unsigned int i = 0; i < points.size(); ++i)
  {
    current_points.clear();
    for (unsigned int j = 0; j < points[i].size(); ++j)
    {
      c.at<double>(0, 0) = (double)points[i][j].x;
      c.at<double>(0, 1) = (double)points[i][j].y;
      c.at<double>(0, 2) = (double)points[i][j].z;
      Mat output = rotationMatrix*c - x;
      /*cout << rotationMatrix << "\n";
      cout << c << "\n";
      cout << translationVector << "\n";
      cout << x << "\n";
      cout << output << "\n";*/
      Point3f out;
      out.x = (float)output.at<double>(0, 0);
      out.y = (float)output.at<double>(0, 1);
      out.z = (float)output.at<double>(0, 2);
      current_points.push_back(out);
    }
    output_points.push_back(current_points);
  }
  return ;
}

inline static void
fixPlaneOrientation(const vector<double> &position, 
            const vector<Point3f> &points,
            vector<float> &plane_parameters,
            vector<Point3f> &continuous_bounding_box_points)
{
  assert(points.size() >= 3);
  cout << "[ INFO] [fixPlaneOrientation] Started\n";
  /*print1dVector(plane_parameters, "[ DEBUG] [fixPlaneOrientation] Old PP");
  print1dVector(continuous_bounding_box_points, "[ DEBUG] [fixPlaneOrientation] Old CBB");*/
  // Create a matrix out of the vector of points: Dimension: numberOfPoints*3
  float x_c = 0.0, y_c = 0.0, z_c = 0.0;
  for (unsigned int i = 0; i < points.size(); ++i)
  {
    x_c += points[i].x;
    y_c += points[i].y;
    z_c += points[i].z;
  }
  x_c /= points.size();
  y_c /= points.size();
  z_c /= points.size();
  // Calculate the centroid of the points
  float centroidX = x_c;
  float centroidY = y_c;
  float centroidZ = z_c;
  Point3f p(centroidX, centroidY, centroidZ), qp;
  cout << "[ DEBUG] [fixPlaneOrientation] Centroid: " << p << "\n";
  print1dVector(position, "[ DEBUG] [fixPlaneOrientation] Position of drone");
  float a = plane_parameters[0];
  float b = plane_parameters[1];
  float c = plane_parameters[2];
  float mag = ((a*a)+(b*b)+(c*c));
  print1dVector(plane_parameters, "[ DEBUG] [fixPlaneOrientation] Old Plane Parameters");
  Point3f pos((float)position[0], (float)position[1], (float)position[2]);
  qp = pos - p;
  cout << "[ DEBUG] [fixPlaneOrientation] qp: " << qp << "\n";
  float t = (qp.x * (a/mag)) + (qp.y * (b/mag)) + ((qp.z * (c/mag)));
  cout << "[ DEBUG] [fixPlaneOrientation] t: " << t << "\n";
  if(!signbit(t))
  {
    cout << "[ DEBUG] [fixPlaneOrientation] Sign change required\n";
    plane_parameters[0] = -plane_parameters[0];
    plane_parameters[1] = -plane_parameters[1];
    plane_parameters[2] = -plane_parameters[2];
    plane_parameters[3] = -plane_parameters[3];
    Point3f top_left = continuous_bounding_box_points[1];
    Point3f top_right = continuous_bounding_box_points[0];
    Point3f bottom_right = continuous_bounding_box_points[3];
    Point3f bottom_left = continuous_bounding_box_points[2];
    continuous_bounding_box_points.clear();
    continuous_bounding_box_points.push_back(top_left);
    continuous_bounding_box_points.push_back(top_right);
    continuous_bounding_box_points.push_back(bottom_right);
    continuous_bounding_box_points.push_back(bottom_left);
    continuous_bounding_box_points.push_back(top_left);
  }
  /*print1dVector(plane_parameters, "[ DEBUG] [fixPlaneOrientation] New Plane Parameters");
  print1dVector(continuous_bounding_box_points, "[ DEBUG] [fixPlaneOrientation] New CBB");*/
  cout << "[ INFO] [fixPlaneOrientation] Completed\n";
  return ;
}

inline static void
orderPlanesFromQuadcopterPosition(const vector<double> &current_pos_of_drone,
                  const vector< vector<Point3f> > &in_points,
                  const vector< vector<float> > &in_pp,
                  const vector< vector<Point3f> > &in_cbb,
                  const vector<float> &in_p,
                  vector< vector<Point3f> > &out_points,
                  vector< vector<float> > &out_pp,
                  vector< vector<Point3f> > &out_cbb,
                  vector<float> &out_p)
{
  cout << "[ DEBUG] [orderPlanesFromQuadcopterPosition] Started\n";
  clear2dVector(out_points);
  clear2dVector(out_pp);
  clear2dVector(out_cbb);
  out_p.clear();
  float x_c = 0.0;
  vector<Point3f> dummy_points, dummy_cbb;
  vector<float> dummy_pp;
  // Vector for x co-ordinate of centroids for each plane
  vector<float> xCentroidPoints;
  // Vector for sorting x co-ordinate of centroids for each plane
  vector<float> sortedXCentroidPoints;
  vector<int> indices;
  vector< vector<Point3f> > points;
  convertWRTCurrentQuadcopterOrigin(current_pos_of_drone, in_points, points);
  for (unsigned int i = 0; i < points.size(); ++i)
  {
    x_c = 0.0;
    for (unsigned int j = 0; j < points[i].size(); ++j)
    {
      x_c += points[i][j].x;
    }
    x_c /= (float)points[i].size();
    xCentroidPoints.push_back(x_c);
  }
  // Sort the x centroids
  sortData( xCentroidPoints, sortedXCentroidPoints, indices, true);
  print1dVector(indices, "[ DEBUG] [orderPlanesFromQuadcopterPosition] New Plane Indices");
  for (unsigned int i = 0; i < in_points.size(); ++i)
  {
    int index = indices[i];
    dummy_points.clear();
    dummy_pp.clear();
    dummy_cbb.clear();
    for (unsigned int j = 0; j < in_points[index].size(); ++j)
    {
      dummy_points.push_back(in_points[index][j]);
    }
    for (unsigned int j = 0; j < in_pp[index].size(); ++j)
    {
      dummy_pp.push_back(in_pp[index][j]);
    }
    for (unsigned int j = 0; j < in_cbb[index].size(); ++j)
    {
      dummy_cbb.push_back(in_cbb[index][j]);
    }
    out_cbb.push_back(dummy_cbb);
    out_pp.push_back(dummy_pp);
    out_points.push_back(dummy_points);
    out_p.push_back(in_p[index]);
  }
  print2dVector(in_pp, "[ DEBUG] [orderPlanesFromQuadcopterPosition] Input Plane Parameters");
  print2dVector(out_pp, "[ DEBUG] [orderPlanesFromQuadcopterPosition] Output Plane Parameters");
  print2dVector(in_cbb, "[ DEBUG] [orderPlanesFromQuadcopterPosition] Input Plane CBB");
  print2dVector(out_cbb, "[ DEBUG] [orderPlanesFromQuadcopterPosition] Output Plane CBB");
  print1dVector(in_p, "[ DEBUG] [orderPlanesFromQuadcopterPosition] Input Plane Percentage");
  print1dVector(out_p, "[ DEBUG] [orderPlanesFromQuadcopterPosition] Output Plane Percentage");
  cout << "[ DEBUG] [orderPlanesFromQuadcopterPosition] Fixing orientations\n";
  for (unsigned int i = 0; i < out_pp.size(); ++i)
  {
    fixPlaneOrientation(current_pos_of_drone, out_points[i], out_pp[i], out_cbb[i]);
  }
  print2dVector(out_pp, "[ DEBUG] [orderPlanesFromQuadcopterPosition] Fixed Output Plane Parameters");
  print2dVector(out_cbb, "[ DEBUG] [orderPlanesFromQuadcopterPosition] Fixed Output Plane CBB");
  cout << "[ DEBUG] [orderPlanesFromQuadcopterPosition] Fixing Done\n";
  cout << "[ DEBUG] [orderPlanesFromQuadcopterPosition] Completed\n";
  dummy_points.clear();
  dummy_pp.clear();
  dummy_cbb.clear();
  clear2dVector(points);
  return ;
}

/**
 * @brief Get the best fit plane for a set of 3d points
 * @details Uses least squares and svd method
 * @todo-me Decide on which method to use finally after testing
 */
inline static vector<float>
bestFitPlane(const vector<Point3f> &threed_points)
{
  // http://stackoverflow.com/questions/1400213/3d-least-squares-plane
  float x_c = 0.0, y_c = 0.0, z_c = 0.0;
  float a, b, c, d;
  Mat X(3, threed_points.size(), DataType<float>::type);
  Mat Y(4, threed_points.size(), DataType<float>::type);
  for (unsigned int i = 0; i < threed_points.size(); ++i)
  {
    x_c += threed_points[i].x;
    y_c += threed_points[i].y;
    z_c += threed_points[i].z;
    X.at<float>(0, i) = threed_points[i].x;
    X.at<float>(1, i) = threed_points[i].y;
    X.at<float>(2, i) = threed_points[i].z;
    Y.at<float>(0, i) = threed_points[i].x;
    Y.at<float>(1, i) = threed_points[i].y;
    Y.at<float>(2, i) = threed_points[i].z;
    Y.at<float>(3, i) = 1.0;
  }
  x_c /= threed_points.size();
  y_c /= threed_points.size();
  z_c /= threed_points.size();
  // Centering the points
  for (unsigned int i = 0; i < threed_points.size(); ++i)
  {
    X.at<float>(0, i) = X.at<float>(0, i) - x_c;
    X.at<float>(1, i) = X.at<float>(1, i) - y_c;
    X.at<float>(2, i) = X.at<float>(2, i) - z_c;
  }
  // http://www.ilikebigbits.com/blog/2015/3/2/plane-from-points
  // Method 1: Least squares
  /*float val11 = 0.0, val12 = 0.0;
  float val21 = 0.0, val22 = 0.0;
  float val1 = 0.0, val2 = 0.0;
  for (int i = 0; i < threed_points.size(); ++i)
  {
    val11 += (X.at<float>(0,i) * X.at<float>(0,i));
    val12 += (X.at<float>(0,i) * X.at<float>(1,i));
    val21 += (X.at<float>(1,i) * X.at<float>(0,i));
    val22 += (X.at<float>(1,i) * X.at<float>(1,i));
    val1 += (X.at<float>(0,i) * X.at<float>(2,i));
    val2 += (X.at<float>(1,i) * X.at<float>(2,i));
  }
  float D = (val11*val22) - (val12*val21);
  a = ((val2*val12) - (val1*val22))/D;
  b = ((val12*val1) - (val11*val2))/D;
  c = 1.0;
  d = 0.0;
  float mag = sqrt(a*a + b*b + c*c);
  cout << "Method 1: (" << a/mag << ", " << b/mag << ", " << c/mag << ")\n";
  normal.push_back(a/mag);
  normal.push_back(b/mag);
  normal.push_back(c/mag);
  normal.push_back(d);*/
  vector<float> normal;
  // Method 2: SVD Method
  // http://math.stackexchange.com/questions/99299/best-fitting-plane-given-a-set-of-points
  // http://www.ltu.se/cms_fs/1.51590!/svd-fitting.pdf
  /*Mat w(3, threed_points.size(), DataType<float>::type);
  Mat vt(threed_points.size(), threed_points.size(), DataType<float>::type);
  Mat u(3, 3, DataType<float>::type);
  SVD::compute(X, w, u, vt);
  cout << "U:\n" << u << "\n";
  cout << "S:\n" << w << "\n";
  cout << "Method 2: (" << u.at<float>(0, 2) << ", " << u.at<float>(1, 2) << ", " << u.at<float>(2, 2) << ")\n";
  a = u.at<float>(0, 2);
  b = u.at<float>(1, 2);
  c = u.at<float>(2, 2);
  d = (float)0.0;
  normal.push_back(a);
  normal.push_back(b);
  normal.push_back(c);
  normal.push_back(d);*/
  // Method 3: Complete
  Mat w(3, threed_points.size(), DataType<float>::type);
  Mat vt(threed_points.size(), threed_points.size(), DataType<float>::type);
  Mat u(3, 3, DataType<float>::type);
  SVD::compute(Y, w, u, vt);
  cout << "U:\n" << u << "\n";
  cout << "S:\n" << w << "\n";
  cout << "Vt:\n" << vt << "\n";
  cout << "Method 3: (" << u.at<float>(0, 3) << ", " << u.at<float>(1, 3) << ", " 
      << u.at<float>(2, 3) << ", " << u.at<float>(3, 3) << ")\n";
  a = u.at<float>(0, 3);
  b = u.at<float>(1, 3);
  c = u.at<float>(2, 3);
  d = u.at<float>(3, 3);
  normal.push_back(a);
  normal.push_back(b);
  normal.push_back(c);
  normal.push_back(d);
  return normal;
}

/**
 * @brief Generates 2d points on the image on the left, middle and right edge of plane
 * @details
 */
inline static vector<Point2f>
GenerateMy2DPoints()
{
  vector<Point2f> points;
  float x, y;
  /* Point 1 */
  x = 0; y = 0;
  points.push_back(Point2f(x, y));
  /* Point 2 */
  x = 0; y = 180;
  points.push_back(Point2f(x, y));
  /* Point 3 */
  x = 0; y = 360;
  points.push_back(Point2f(x, y));
  /* Point 4 */
  x = 320; y = 0;
  points.push_back(Point2f(x, y));
  /* Point 5 */
  x = 320; y = 180;
  points.push_back(Point2f(x, y));
  /* Point 6 */
  x = 320; y = 360;
  points.push_back(Point2f(x, y));
  /* Point 1 */
  x = 640; y = 0;
  points.push_back(Point2f(x, y));
  /* Point 8 */
  x = 640; y = 180;
  points.push_back(Point2f(x, y));
  /* Point 9 */
  x = 640; y = 360;
  points.push_back(Point2f(x, y));
  return points;
}

/**
 * @brief Generates 3d points on the z = 0 plane for a specific width and height
 * @details
 */
inline static vector<Point3f>
GenerateMy3DPoints(float width, float height)
{
  vector<Point3f> points;
  float x, y, z;
  /*              4
    1 +-----------+------------+ 7
    |           | 5          |
    2 +-----------+------------+ 8
    |           |            |
    3 +-----------+------------+ 9
          6 */
  /* Point 1 */
  x = -width/2; y = height/2; z = 0.0;
  points.push_back(Point3f(x,y,z));
  /* Point 2 */
  x = -width/2; y = 0.0; z = 0.0;
  points.push_back(Point3f(x,y,z));
  /* Point 3 */
  x = -width/2; y = -height/2; z = 0.0;
  points.push_back(Point3f(x,y,z));
  /* Point 4 */
  x = 0.0; y = height/2; z = 0.0;
  points.push_back(Point3f(x,y,z));
  /* Point 5: Origin at the center of the plane */
  x = 0.0; y = 0.0; z = 0.0;
  points.push_back(Point3f(x,y,z));
  /* Point 6 */
  x = 0.0; y = -height/2; z = 0.0;
  points.push_back(Point3f(x,y,z));
  /* Point 7 */
  x = width/2; y = height/2; z = 0.0;
  points.push_back(Point3f(x,y,z));
  /* Point 8 */
  x = width/2; y = 0.0; z = 0.0;
  points.push_back(Point3f(x,y,z));
  /* Point 9 */
  x = width/2; y = -height/2; z = 0.0;
  points.push_back(Point3f(x,y,z));
  return points;
}

/**
 * @brief Calculate the distance to see the plane completely from top to bottom
 * @details From tyhis position the leftmost and rightmost edges of the plane may not be visible
 */
inline static float
getDistanceToSeePlane(int height)
{
  // Read points
  vector<Point2f> imagePoints = GenerateMy2DPoints();
  float width = (16.0/9.0)*height;
  float drone_length = 0.6;
  vector<Point3f> objectPoints = GenerateMy3DPoints(width, height);
  Mat cameraMatrix(3, 3, DataType<double>::type);
  setIdentity(cameraMatrix);
  // From calibration done on our drone
  cameraMatrix.at<double>(0,0) = 565.710890694431;
  cameraMatrix.at<double>(0,1) = 0;
  cameraMatrix.at<double>(0,2) = 329.70046366652;
  cameraMatrix.at<double>(1,0) = 0;
  cameraMatrix.at<double>(1,1) = 565.110297594854;
  cameraMatrix.at<double>(1,2) = 169.873085097623;
  cameraMatrix.at<double>(2,0) = 0;
  cameraMatrix.at<double>(2,1) = 0;
  cameraMatrix.at<double>(2,2) = 1;
  Mat distCoeffs(5, 1, DataType<double>::type);
  // From calibration done on our drone
  distCoeffs.at<double>(0) = -0.516089772391501;
  distCoeffs.at<double>(1) = 0.285181914111246;
  distCoeffs.at<double>(2) = -0.000466469917823537;
  distCoeffs.at<double>(3) = 0.000864792975814983;
  distCoeffs.at<double>(4) = 0;
  Mat rvec(3, 1, DataType<double>::type);
  Mat tvec(3, 1, DataType<double>::type);
  Mat dummy;
  undistortPoints(imagePoints, dummy, cameraMatrix, distCoeffs);
  Mat rot_guess = Mat::eye(3, 3, CV_64F);
  Rodrigues(rot_guess, rvec);
  tvec.at<double>(0)  = 0.0;
  tvec.at<double>(1)  = 0.0;
  tvec.at<double>(2)  = -(height - drone_length);
  solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);
  Mat rot(3, 3, DataType<double>::type);
  Rodrigues(rvec, rot);
  Mat rotinv;
  transpose(rot, rotinv);
  tvec = -rotinv * tvec;
  cout << "Expected Quadcopter Location to see height: " << height << "and width: " << width 
    << ": (" << tvec.at<double>(0) << ", " << tvec.at<double>(2) << ", " << -tvec.at<double>(1) << ")\n";
  double dist = tvec.at<double>(2);
  return dist;
}

#endif


