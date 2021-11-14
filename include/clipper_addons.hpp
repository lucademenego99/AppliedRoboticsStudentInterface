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
Paths enlarge(IntPoint *points, int offset);

/**
 * @brief Print a clipper polygon offsetting solution using OpenCV
 * 
 * @param startingPoints Points of the original polygon
 * @param solution Solution given by clipper
 */
void printSolution(IntPoint *points, Paths solution);

#endif