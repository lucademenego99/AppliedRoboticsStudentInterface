/**
 * @file graphPrint.hpp
 * @brief Useful functions to print generated graphs using opencv
 * @version 0.1
 * @date 2022-01-06
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef GRAPHPRINT_H
#define GRAPHPRINT_H

#include "graph.hpp"

/**
 * @brief Function to display an opencv window with the results of the visibility graph
 * 
 * @param g graph we want to display
 * @param origin starting point
 * @param destination destination point
 * @param shortestPath points that compose the shortest path
 */
void printGraph(std::map<visgraph::Point, std::vector<visgraph::Edge>> g, visgraph::Point origin, visgraph::Point destination, std::vector<visgraph::Point> shortestPath);

#endif