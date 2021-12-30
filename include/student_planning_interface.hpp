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
   * @brief fulfill the path for the robots
   * 
   * @param curves result of the multi-point shortest path
   * @param path pointer to the structure which will be returned in the function planPath
   * @param pathSize number of segments of the segmented path
   * @param size integrate distance for seperated dubin path
   * @param robotId ID for the robot to be controlled
   */
  void fillPath(dubins::DubinsCurve **curves, std::vector<Path>& path, int pathSize, double size, int robotId);

  /**
   * @brief Check if a certain points (x,y) is inside the arena
   * Points are in the order bottom-left, bottom-right, top-right, top-left
   * 
   * @param borderPoints Points representing the arena, in order bottom-left, bottom-right, top-right, top-left
   * @param xPoint X of the point we are considering
   * @param yPoint Y of the point we are considering
   * @return true If the point lies inside the arena
   * @return false If the point does not lie inside the arena
   */
  bool isInsideArena(std::vector<visgraph::Point> borderPoints, double xPoint, double yPoint);

  /**
   * @brief Find out the number of robots based on the values of their position
   * 
   * @param x x positions of the robots
   * @param y y positions of the robots
   * @return int number of robots
   */
  int checkNumberOfRobots(std::vector<float> x, std::vector<float> y);

  /**
   * @brief Given the coordinates of the destination square, find a point that lies within the arena
   * 
   * @param borderPoints Points representing the arena, in order bottom-left, bottom-right, top-right, top-left
   * @param minX Minimum x of the destination square
   * @param maxX Maximum x of the destination square
   * @param minY Minimum y of the destination square
   * @param maxY Maximum y of the destination square
   * @return visgraph::Point A point that lies within the arena and that is on an edge of the destination square
   */
  visgraph::Point findValidDestinationPoint(std::vector<visgraph::Point> borderPoints, double minX, double maxX, double minY, double maxY);

  /**
   * @brief Find the 4 border points
   * 
   * @param borders Polygon representing the border
   * @param borderMaxX Max x point - passed by ref.
   * @param borderMinX Min x point - passed by ref.
   * @param borderMaxY Max y point - passed by ref.
   * @param borderMinY Min y point - passed by ref.
   */
  void findBorderPoints(const Polygon &borders, double &borderMaxX, double &borderMinX, double &borderMaxY, double &borderMinY);

  /**
   * @brief Find the 4 destination points given a destination polygon
   * 
   * @param gate Polygon representing our gate
   * @param maxX Max x point - passed by ref.
   * @param minX Min x point - passed by ref.
   * @param maxY Max y point - passed by ref.
   * @param minY Min y point - passed by ref.
   */
  void findDestinationPoints(const Polygon &gate, double &maxX, double &minX, double &maxY, double &minY);

  /**
   * @brief Convert the obstacles into a structure our algorithms comprehend
   * 
   * @param obstacle_list List of the obstacles as vector of polygons
   * @return std::vector<std::vector<visgraph::Point>> List of obstacles represented as vectors of visgraph::Point
   */
  std::vector<std::vector<visgraph::Point>> getObstacles(const std::vector<Polygon>& obstacle_list);

  /**
   * @brief Add borders increased in size by a certain factor to the polygons vector
   * 
   * @param borderMaxX Max x point
   * @param borderMinX Min x point
   * @param borderMaxY Max y point
   * @param borderMinY Min y point
   * @param offset Offset by which we want to increase the size of the borders
   * @param variant Another value used to determine how much we want to increase the size of the borders
   * @param borderPoints Vector keeping the 4 border points - passed by ref.
   * @param polygons Add the borders to this vector, then they will be used for collision detection - passed by ref.
   */
  void addBorders(double borderMaxX, double borderMinX, double borderMaxY, double borderMinY, double offset, double variant, std::vector<visgraph::Point> &borderPoints, std::vector<std::vector<visgraph::Point>> &polygons);

  /**
   * @brief Given the coordinates of the destination square, find a point that lies within the arena
   * 
   * @param borderPoints Points representing the arena, in order bottom-left, bottom-right, top-right, top-left
   * @param minX Minimum x of the destination square
   * @param maxX Maximum x of the destination square
   * @param minY Minimum y of the destination square
   * @param maxY Maximum y of the destination square
   * @return visgraph::Point A point that lies within the arena and that is on an edge of the destination square
   */
  visgraph::Point findValidDestinationPoint(std::vector<visgraph::Point> borderPoints, double minX, double maxX, double minY, double maxY);

  /**
   * @brief Reach a certain destination points with a certain robot
   * 
   * @param robot Robot ID
   * @param origin Origin point
   * @param destination Destination Point
   * @param theta Robot's starting angle
   * @param polygons Obstacles for collision detection
   * @param polygonsForVisgraph Obstacles for the roadmap generation
   * @param path Path to fill - passed by ref.
   * @param max_k Curvature of the robot
   * @param size Discritizer size for the path generation
   * @return true If a path has been found
   * @return false If a path hasn't been found
   */
  bool reachDestinationForRobot(int robot, visgraph::Point origin, visgraph::Point destination, double theta, std::vector<std::vector<visgraph::Point>> polygons, std::vector<std::vector<visgraph::Point>> polygonsForVisgraph,std::vector<Path> &path, double max_k, double size);

}

#endif