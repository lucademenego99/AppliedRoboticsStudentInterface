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
Paths enlarge(std::vector<IntPoint> points, int offset);

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
//bool intersect (IntPoint *subj, IntPoint *clip);
/**
 * @brief Given two polygons verifies if they are colling, if so, joins them and enlarges it as a whole
 * 
 * @param firstPoly Points of the first polygon
 * @param secondPoly Points of the second polygon
 */
//void verifyAndJoin (IntPoint *firstPoly, IntPoint *secondPoly);

/**
 * @brief Verifies if a list of polygons has some intersections, if so joins them and then enlarges them all
 * 
 * @param points Matrix of polygons
 */
std::vector<Paths> joinAndEnlarge (std::vector<std::vector<IntPoint>> points);
#endif