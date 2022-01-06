/**
 * @file student_planning_utils.hpp
 * @brief Basic data structures used by the ROS component
 * @version 0.1
 * @date 2022-01-06
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef STUDENT_UTILS
#define STUDENT_UTILS

#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>
#include <cstddef>

struct Point 
{
  float x, y;

  Point(float x, float y): x(x), y(y) {}
  Point(): Point(0, 0) {}
};

/**
 * @brief A polygon, expessed as a list of 2D points
 * 
 */
typedef std::vector<Point> Polygon;

/**
 * @brief A configuration of the robot along the path, represented by x, y, orientation and curvature
 * 
 */
struct Pose
{
  float s, x, y, theta, kappa;

  Pose(float s, float x, float y, float theta, float kappa) : s(s), x(x), y(y), theta(theta), kappa(kappa)
  {
  }

  Pose() : Pose(0, 0, 0, 0, 0)
  {
  }

  float distance(float _x, float _y)
  {
    return std::hypot(x - _x, y - _y);
  }
};

/**
 * @brief A sequence of sampled robot configurations composing a (discretization of the) path
 * 
 */
struct Path
{
  std::vector<Pose> points;

  Path(std::vector<Pose> const &points) : points(points)
  {
  }

  Path()
  {
  }

  bool empty() { return points.empty(); }
  size_t size() { return points.size(); }
  void setPoints(const std::vector<Pose> &points) { this->points = points; }
};

#endif