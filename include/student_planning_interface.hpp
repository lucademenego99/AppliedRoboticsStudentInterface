#ifndef STUDENT_PLANNING
#define STUDENT_PLANNING

#include "student_planning_utils.hpp"
#include <string>
#include "graph.hpp"
#include "dubins.hpp"
#include "visgraph.hpp"
#include "open_edges.hpp"
#include "graphPrint.hpp"
#include "utils.hpp"

namespace student
{
   /**
   * @brief Plan a safe and fast path in the arena
   * 
   * @param borders border of the arena [m]
   * @param obstacle_list list of obstacle polygon [m]
   * @param victim_list list of pair victim_id and polygon [m]
   * @param gate polygon representing the gate [m]
   * @param x x position of the robot in the arena reference system
   * @param y y position of the robot in the arena reference system
   * @param theta yaw of the robot in the arena reference system
   * @param path Path returned (passed by ref.)
   * @param config_folder A custom string from config file.
   * @return true The returned path is valid
   * @return false The returned path is not valid
   */
   bool planPath(const Polygon &borders, const std::vector<Polygon> &obstacle_list,
                 const std::vector<std::pair<int, Polygon>> &victim_list,
                 const Polygon &gate, const float x, const float y, const float theta,
                 Path &path,
                 const std::string &config_folder);
   /**
     * @brief NAME STILL TO BE DEFINED, generates an original graph and a visibility graph starting from a vector of polygons
     * 
     * @param polygons Obstables present in the map
     * @param visg Visibility graph we need to obtain
     * @param g Graph we need to generate starting from the obstacles
     */
   void unnamedFunction(std::vector<std::vector<visgraph::Point>> polygons, visgraph::VisGraph visg, visgraph::Graph g);

  /**
   * @brief fulfill the path for the robots
   * 
   * @param curves result of the multi-point shortest path
   * @param path pointer to the structure which will be returned in the function planPath
   * @param pathSize number of segments of the segmented path
   * @param size integrate distance for seperated dubin path
   * @param robotId ID for the robot to be controlled
   */
  void fillPath(dubins::DubinsCurve **curves, std::vector<Path>& path, int pathSize, double size, int robotId);

}

#endif