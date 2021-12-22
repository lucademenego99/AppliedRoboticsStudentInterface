#pragma once

#include "utils.hpp"
#include <string>
#include "dubins.hpp"
#include "graph.hpp"
#include "visgraph.hpp"
#include "open_edges.hpp"
#include "graphPrint.hpp"
#include <iostream>

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

}
