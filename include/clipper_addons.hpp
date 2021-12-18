#ifndef CLIPPER_ADDONS
#define CLIPPER_ADDONS

#include "clipper.hpp"

using namespace ClipperLib;

/**
 * @brief Clipper Polygon Offsetting helper function
 * 
 * @param points Points of the original polygon
 * @param offset Offset that must be used to enlarge or shrink the polygon
 * @return Paths Array of resulting polygons
 */
std::vector<student::Point> enlarge(std::vector<student::Point> points, double offset);

/**
 * @brief Print a clipper polygon offsetting solution using OpenCV
 * 
 * @param startingPoints Points of the original polygon
 * @param solution Solution given by clipper
 */
void printSolution(std::vector<IntPoint> points, Paths solution);


/**
 * @brief Verifies if two clipper polygons interact
 * 
 * @param subj The first polygon
 * @param clip The second polygon
 * @return true 
 * @return false 
 */
bool intersect (student::Point *subj, student::Point *clip);
/**
 * @brief Given two polygons verifies if they are colling, if so, joins them and enlarges it as a whole
 * 
 * @param firstPoly Points of the first polygon
 * @param secondPoly Points of the second polygon
 */
std::vector<std::vector<student::Point>> verifyAndJoin (student::Point *firstPoly, student::Point *secondPoly);

/**
 * @brief Verifies if a list of polygons has some intersections, if so joins them and then enlarges them all
 * 
 * @param points Matrix of polygons
 */
std::vector<std::vector<student::Point>> joinAndEnlarge (std::vector<std::vector<IntPoint>> points);

/**
 * @brief Given some obstacles and an offset, creates a "bigger" version using clipper and a "slightly bigger" one, then returns them
 * 
 * @param polygon Obstacles we are considering
 * @param offset Offset for obstacle enhancing
 * @return std::vector<std::vector<student::Point>> 
 */
std::vector<std::vector<student::Point>> applyChanges(std::vector<visgraph::Point> polygon, int offset);
#endif