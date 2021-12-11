#ifndef GRAPHPRINT_H
#define GRAPHPRINT_H

#include "graph.hpp"

int printGraph(std::map<visgraph::Point, std::vector<visgraph::Edge>> g, visgraph::Point origin, visgraph::Point destination, std::vector<visgraph::Point> shortestPath);

#endif