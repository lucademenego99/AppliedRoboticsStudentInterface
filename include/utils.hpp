#ifndef UTILS
#define UTILS

#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>
#include <cstddef>

namespace student
{
  /**
   * @brief A point structure, that can be used as:
   * - a simple 2D point (x,y)
   * - a point representing a position (x,y,theta)
   * 
   */
  struct Point
  {
    double x, y, th;

    Point(double x = -1, double y = -1, double th = -1) : x(x), y(y), th(th) {}
  };

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

  /**
   * @brief A polygon, expessed as a list of 2D points
   * 
   */
  typedef std::vector<Point> Polygon;

  /**
   * @brief Implementation of function sinc(t)
   * 
   * @param t TODO
   * @return double 1 for t==0, and sin(t)/t otherwise
   */
  double sinc(double t);

  /**
   * @brief Normalize an angle (in range [0,2*pi))
   * 
   * @param ang Angle to normalize
   * @return double Normalized angle
   */
  double mod2pi(double angle);

  /**
   * @brief Normalize an angular difference (range (-pi, pi])
   * 
   * @param ang Angular difference to normalize
   * @return double Normalized angular difference
   */
  double rangeSymm(double angle);

  /**
   * @brief Calculates the cross product between two Points
   * 
   * @param a First input point
   * @param b Second input point
   * @return double Cross product between provided points
   */
  double crossProduct(Point a, Point b);

  /**
   * @brief Calculates the dot product between two Points
   * 
   * @param a First input point
   * @param b Second input point
   * @return double Dot product between provided points
   */
  double dot2D(Point a, Point b);

}

#endif