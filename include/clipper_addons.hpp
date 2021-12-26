#ifndef CLIPPER_ADDONS
#define CLIPPER_ADDONS

#include "clipper.hpp"

/**
 * @brief Clipper Polygon Offsetting helper function
 * We multiply the points by 1000 because Clipper works best under the assumption of working on a certain scale
 * 
 * @param points Points of the original polygon
 * @param offset Offset that must be used to enlarge or shrink the polygon
 * @return std::vector<visgraph::Point> Array of points of the resulting polygon
 */
std::vector<visgraph::Point> enlarge(std::vector<visgraph::Point> points, double offset);

/**
 * @brief Clipper Polygon Offsetting helper function, works only for Walls
 * We multiply the points by 1000 because Clipper works best under the assumption of working on a certain scale
 * 
 * @param points Points of the original line that describes the wall
 * @param offset Offset that must be used to enlarge or shrink the wall
 * @return std::vector<visgraph::Point> Array of points of the resulting line
 */
std::vector<visgraph::Point> enlargeWalls(std::vector<visgraph::Point> points, double offset);
/**
 * @brief Print a clipper polygon offsetting solution using OpenCV
 * 
 * @param startingPoints Points of the original polygon
 * @param solution Solution given by clipper
 */
void printSolution(std::vector<ClipperLib::IntPoint> points, ClipperLib::Paths solution);


/**
 * @brief Verifies if two clipper polygons interact
 * 
 * @param subj The first polygon to check for intersections
 * @param clip The second polygon to check for intersections
 * @return true If the two polygons intersect
 * @return false If the two polygons do not intersect
 */
bool intersect (dubins::DubinsPoint *subj, dubins::DubinsPoint *clip);

/**
 * @brief Given some obstacles and an offset, creates a "bigger" version using clipper and a "slightly bigger" one, then returns them
 * 
 * @param polygon Obstacles we are considering
 * @param offset Offset for obstacle offsetting
 * @return std::vector<std::vector<visgraph::Point>> Array containing at position 0 the bigger obstacles and at position 1 the smaller ones
 */
std::vector<std::vector<visgraph::Point>> enlargeObstaclesWithTwoOffsets(std::vector<visgraph::Point> polygon, double offset);

/**
 * @brief Given some walls and an offset, creates a "bigger" version using clipper and a "slightly bigger" one, then returns them
 * 
 * @param polygon Walls we are considering
 * @param offset Offset for Walls offsetting
 * @return std::vector<std::vector<student::Point>> Array containing at position 0 the bigger Walls and at position 1 the smaller ones
 */
std::vector<std::vector<visgraph::Point>> enlargeObstaclesWithTwoOffsetsWalls(std::vector<visgraph::Point> polygon, double offset);

/**
 * @brief Enlarge the obstacles using the function enlargeObstaclesWithTwoOffsets, then join them in case they collide between each other
 * The function also removes possible holes that can be formed during the join phase
 * 
 * @param polygonsList All the obstacles
 * @param offset Offset for enlarging them
 * @return std::vector<std::vector<std::vector<visgraph::Point>>> Array containing at position 0 the bigger obstacles and at position 1 the smaller ones, joined in case of collisions
 */
std::vector<std::vector<std::vector<visgraph::Point>>> enlargeAndJoinObstacles(std::vector<std::vector<visgraph::Point>> polygonsList, double offset);

#endif