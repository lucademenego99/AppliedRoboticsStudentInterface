#ifndef DUBINSCURVES
#define DUBINSCURVES

#include "utils.hpp"
#include "graph.hpp"
#include <vector>
#include <iostream>

namespace student
{
    /**
 * @brief Class used to create the visibility graph given a certain environment
 * 
 */
    class VisGraph
    {
    private:
        const int INF = 10000;
        const int CCW = 1;
        const int CW = -1;
        const int COLLINEAR = 0;
        const int COLIN_TOLERANCE = 10;
        const int T = pow(10, COLIN_TOLERANCE);
        const double T2 = pow(10.0, COLIN_TOLERANCE);

        std::vector<Point> getVisibleVertices(Point from, Graph graph);
        double getAngle(Point center, Point p);
        bool onSegment(Point p, Point q, Point r);
        int getOrientation(Point p, Point q, Point r);
        bool polygonCrossing(Point p, std::vector<Edge> polygonEdges);
        bool edgeInPolygon(Point p1, Point p2, Graph graph);
        bool pointInPolygon(Point p, Graph graph);
        double edgeDistance(Point p1, Point p2);
        Point getIntersectPoint(Point p1, Point p2, Edge edge);

    public:
        /**
     * @brief Construct a new Vis Graph object
     * 
     */
        VisGraph() = default;

        bool edgeIntersect(Point p1, Point q1, Edge edge);
        double pointEdgeDistance(Point p1, Point p2, Edge edge);
        double getAngle2(Point a, Point b, Point c);
        Graph computeVisibilityGraph(std::vector<Point> points);
    };

}

#endif