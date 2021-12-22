#include "student_planning_interface.hpp"

#include <stdexcept>
#include <sstream>
#include <iostream>

namespace student
{
  /*!
* Plan a safe and fast path in the arena
* @param[in]  borders        border of the arena [m]
* @param[out] obstacle_list  list of obstacle polygon [m]
* @param[out] gate_list      polygon representing the gate [m]
* @param[out] x              x position of the robot in the arena reference system
* @param[out] y              y position of the robot in the arena reference system
* @param[out] theta          yaw of the robot in the arena reference system
* @param[out] path           list of Pose need to be feed with the path calculated result
* @param[in]  config_folder  A custom string from config file.
*/
  bool planPath(const Polygon& borders, const std::vector<Polygon>& obstacle_list, const std::vector<Polygon>& gate_list, const std::vector<float> x, const std::vector<float> y, const std::vector<float> theta, std::vector<Path>& path, const std::string& config_folder){
    std::vector<visgraph::Point> pol;
    std::vector<std::vector<visgraph::Point>> polygons;
    for(int i = 0; i<obstacle_list.size(); i++) {
      for(int j = 0; j<obstacle_list[i].size(); j++) {
        visgraph::Point trans_point = visgraph::Point(obstacle_list[i][j].x, obstacle_list[i][j].y, i);
        pol.push_back(trans_point);
      }
      polygons.push_back(pol);
    }

    dubins::Dubins dubins = dubins::Dubins(1.5, 0.005);
    dubins::DubinsCurve *result = dubins.findShortestPath(x[0], y[0], theta[0], x[1], y[1], theta[1]);

    if (!result) {
      std::cout << "RESULT NOT VALID\n";
    } else {
      std::cout << "RESULT: \n\n";
      std::cout << "L = " << result->L << "\n\n";
      std::cout << "a1 = \n"
                << "\tL = " << result->a1->L << "\n"
                << "\tk = " << result->a1->k << "\n"
                << "\tx0 = " << result->a1->x0 << "\n"
                << "\ty0 = " << result->a1->y0 << "\n"
                << "\tth0 = " << result->a1->th0 << "\n"
                << "\tdubins_line-x = " << result->a1->dubins_line->x << "\n"
                << "\tdubins_line-y = " << result->a1->dubins_line->y << "\n"
                << "\tdubins_line-th = " << result->a1->dubins_line->th << "\n\n";
      std::cout << "a2 = \n"
                << "\tL = " << result->a2->L << "\n"
                << "\tk = " << result->a2->k << "\n"
                << "\tx0 = " << result->a2->x0 << "\n"
                << "\ty0 = " << result->a2->y0 << "\n"
                << "\tth0 = " << result->a2->th0 << "\n"
                << "\tdubins_line-x = " << result->a2->dubins_line->x << "\n"
                << "\tdubins_line-y = " << result->a2->dubins_line->y << "\n"
                << "\tdubins_line-th = " << result->a2->dubins_line->th << "\n\n";
      std::cout << "a3 = \n"
                << "\tL = " << result->a3->L << "\n"
                << "\tk = " << result->a3->k << "\n"
                << "\tx0 = " << result->a3->x0 << "\n"
                << "\ty0 = " << result->a3->y0 << "\n"
                << "\tth0 = " << result->a3->th0 << "\n"
                << "\tdubins_line-x = " << result->a3->dubins_line->x << "\n"
                << "\tdubins_line-y = " << result->a3->dubins_line->y << "\n"
                << "\tdubins_line-th = " << result->a3->dubins_line->th << "\n\n";
    }

  // *****the controllment for the multi robot*****
  /**  feed the
  e.g path for robot 0
    for (float l=0, s=0; l<3; l++, s+=ds) {
      path[0].points.emplace_back(s, x[0]+ds*l, y[0], 0.0, 0.0);
    }
  ******THE ADDED POINT HAVE THE STRUCTURE POSE, NOT POINT******
  Pose(float s, float x, float y, float theta, float kappa):
    s(s), x(x), y(y), theta(theta), kappa(kappa)
  {}
  **/
  }

}