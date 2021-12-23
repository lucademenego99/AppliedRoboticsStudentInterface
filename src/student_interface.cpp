#include "student_planning_interface.hpp"
#include "student_planning_utils.hpp"
#include "graph.hpp"
#include "dubins.hpp"
#include "visgraph.hpp"
#include "open_edges.hpp"
#include "graphPrint.hpp"
#include "clipper_addons.hpp"

#include <stdexcept>
#include <sstream>
#include <iostream>

namespace student
{
  /**
   * @brief Plan a safe and fast path in the arena
   * 
   * @param borders Border of the arena, expressed in meters
   * @param obstacle_list List of obstacle polygons
   * @param gate_list List of possible gates, expressed as polygons
   * @param x x positions of the robots in the arena
   * @param y y positions of the robots in the arena
   * @param theta Yaw of the robot in the arena reference system
   * @param path List of poses representing the final path of the robot, passed by ref.
   * @param config_folder A custom string from config file
   * @return true We have a valid path available
   * @return false We don't have a valid path available
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

    std::cout << "Start pose x = " << x[0] << ", y = " << y[0] << " , theta = " << theta[0] << std::endl;
    std::cout << "End pose x = " << x[1] << ", y = " << y[1] << " , theta = " << theta[1] << std::endl;

    if (!result) {
      std::cout << "RESULT NOT VALID\n";
    } 

    int npts = 100;

    for (int i = 0; i < npts; i++) {
      double s = result->a1->L/npts * i;
      dubins::DubinsLine *tmp = new dubins::DubinsLine(s, result->a1->x0, result->a1->y0, result->a1->th0, result->a1->k);
      // pts.push_back(cv::Point(tmp->x / size * 500, tmp->y / size * 500));
      path[0].points.emplace_back(s, tmp->x, tmp->y, tmp->th, result->a1->k);
      delete tmp;
    }

    for (int i = 0; i < npts; i++) {
      double s = result->a2->L/npts * i;
      dubins::DubinsLine *tmp = new dubins::DubinsLine(s, result->a2->x0, result->a2->y0, result->a2->th0, result->a2->k);
      // pts.push_back(cv::Point(tmp->x / size * 500, tmp->y / size * 500));
      path[0].points.emplace_back(s, tmp->x, tmp->y, tmp->th, result->a2->k);
      delete tmp;
    }

    for (int i = 0; i < npts; i++) {
      double s = result->a3->L/npts * i;
      dubins::DubinsLine *tmp = new dubins::DubinsLine(s, result->a3->x0, result->a3->y0, result->a3->th0, result->a3->k);
      // pts.push_back(cv::Point(tmp->x / size * 500, tmp->y / size * 500));
      path[0].points.emplace_back(s, tmp->x, tmp->y, tmp->th, result->a3->k);
      if(i == npts-1) {
        std::cout << "x = " << tmp->x << ", y = " << tmp->y << ", theta = " << tmp->th << ", result.theta = " << result->a3->th0 <<std::endl;
      }
      delete tmp;
    }
  }
}