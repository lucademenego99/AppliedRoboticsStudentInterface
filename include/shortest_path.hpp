#ifndef SPATH_H
#define SPATH_H
#include <map>
#include <vector>
#include "graph.hpp"

namespace visgraph
{
    struct MyPoint
    {
        Point p;
        double dist;
        MyPoint(const Point p,double dist);
        bool operator<(const struct MyPoint & right)const;
        bool operator==(const struct MyPoint & right)const;
    };
    double edge_distance(Point p1, Point p2);
    std::map<Point, Point> dijkstra(DictG graph, Point origin, Point destination);
    std::vector<Point> shortestPath(DictG graph, Point origin, Point destination);
}

#endif
