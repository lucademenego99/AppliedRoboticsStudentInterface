#include "student_planning_interface.hpp"

#include <stdexcept>
#include <sstream>
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
    throw std::logic_error( "STUDENT FUNCTION - PLAN PATH - OWN IMPLEMENTED" );
  // modification needed: Polygon: std::vector<Point> -> std::vector<visgraph::Point>
  // may just add the polygon_id one by one

  /** for our task, the evader can be obtained by calling function 

  professor::processRobot(hsv_img, scale, triangle, x, y, theta, ns);
  
  the parameter will lead our function in a mass, also, no data structure up to now deliver this result to student interface!!!
  May ask the professor to add it, or we should develop it by ourself.
  **/

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
