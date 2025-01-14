/**
 * @file visgraph.hpp
 * @brief Headers for the visibility graph generation
 * @version 0.1
 * @date 2022-01-06
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef VISGRAPH
#define VISGRAPH

#include "graph.hpp"
#include <vector>
#include <iostream>

using namespace visgraph;

/**
 * @brief Namespace for the visibility graph generation
 * 
 */
namespace visgraph
{
    /**
     * @brief Class used to create the visibility graph given a certain environment
     * 
     */
    class VisGraph
    {
    private:
        /**
         * @brief Value to simulate a line starting from a certain point P and going to infinity (positive x axis)
         * 
         */
        double INF = 100000;

        /**
         * @brief CounterClockwise value
         * 
         */
        const int CCW = 1;

        /**
         * @brief Clockwise value
         * 
         */
        const int CW = -1;

        /**
         * @brief Collinear points value
         * 
         */
        const int COLLINEAR = 0;

        /**
        * @brief Computes the visible points given a single starting point and a single destination
        * 
        * @param point Point we want to consider
        * @param graph Graph of the map
        * @param origin Starting point
        * @param destination Final point
        * @return vector<Point> All the visible points that can be seen from "point"
        */
        std::vector<Point> getVisibleVertices(Point from, Graph graph, Point origin = Point(-1, -1), Point destination = Point(-1, -1));

        /**
         * @brief Computes the visible points given multiple starting points and multiple destinations
         * 
         * @param point Point we want to consider
         * @param graph Graph of the map
         * @param origin Starting points
         * @param destination Final points
         * @return vector<Point> All the visible points that can be seen from "point"
         */
        std::vector<Point> getVisibleVerticesMultipleOD(Point point, Graph graph, std::vector<Point> origins, std::vector<Point> destinations);

        /**
        * @brief Get the angle between a point and the half line we are considering
        * 
        * @param center Point from which we want to find the visible vertices
        * @param point Point we are currently considering
        * @return double Value of the calculated angle
        */
        double getAngle(Point center, Point p);

        /**
        * @brief Is the edge from point p1 to point p2 interior to any polygon?
        * 
        * @param p1 First point we are considering
        * @param p2 Second point we are considering
        * @param graph The graph containing all the polygons
        * @return true The edge [p1, p2] is interior to a polygon
        * @return false The edge [p1, p2] is not interior to a polygon
        */
        bool edgeInPolygon(Point p1, Point p2, Graph graph);

        /**
        * @brief Computes the distance of two points
        * 
        * @param p1 First input point
        * @param p2 Second input point
        * @return double 
        */
        double edgeDistance(Point p1, Point p2);

        /**
        * @brief Verifies if two edges intersect and returns the point of intersection
        * 
        * @param p1 The first point
        * @param p2 The second point
        * @param edge The other edge to intersect
        * @return Point 
        */
        Point getIntersectPoint(Point p1, Point p2, Edge edge);

    public:
        /**
         * @brief Construct a new Vis Graph object
         * 
         */
        VisGraph() = default;

        /**
        * @brief https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
        * 
        * @param p1 
        * @param q1 
        * @param edge 
        * @return true 
        * @return false 
        */
        bool edgeIntersect(Point p1, Point q1, Edge edge);

        /**
        * @brief Computes the distance between a point  and the intersection of two segments
        * 
        * @param p1 First point of the segment and also the point we consider for the distance
        * @param p2 Second point of the segment
        * @param edge Segment we want to compare
        * @return double 
        */
        double pointEdgeDistance(Point p1, Point p2, Edge edge);

        /**
        * @brief Verifies if a point crosses the edges of a polygon
        * 
        * @param p1 The point we consider
        * @param polygonEdges The list of edges of the polygon
        * @return true 
        * @return false 
        */
        bool polygonCrossing(Point p, std::vector<Edge> polygonEdges);

        /**
        * @brief Checks if given three points p, q, r point q lies on line segment "pr"
        * 
        * @param p First input point
        * @param q Second input point
        * @param r Third input point
        * @return true 
        * @return false 
        */
        bool onSegment(Point p, Point q, Point r);

        /**
        * @brief Finds the orientation of ordered triplet
        * 
        * @param p First input point
        * @param q Second input point
        * @param r Third input point
        * @return int 0, if the points are collinear, -1 if clockwise and 1 if counterclockwise
        */
        int getOrientation(Point p, Point q, Point r);

        /**
        * @brief Calculate the angle used to sort the edges in the OpenEdges structure, based on the point we are currently considering and a certain edge composed of the points b and c
        * 
        * @param a Point we are currently considering
        * @param b First point of the edge
        * @param c Second point of the edge
        * @return double Angle value in radians
        */
        double getAngle2(Point a, Point b, Point c);

        /**
        * @brief Verifies if a point is within a polygon
        * 
        * @param p The point we consider
        * @param graph The graph of the map
        * @return vector<Edge> 
        */
        std::vector<Edge> pointInPolygon(Point p, Graph graph);

        /**
        * @brief Computes the visibility graph given a list of points, the origin and the destination
        * 
        * @param points Polygons we encounter in the map
        * @param origin Starting point
        * @param destination Destination point
        * @return Graph Visibility graph
        */
        Graph computeVisibilityGraph(std::vector<std::vector<Point>> points, Point origin, Point destination);

        /**
        * @brief Computes the visibility graph given a list of points, a list of origins and a list of destinations
        * 
        * @param points Polygons we encounter in the map
        * @param origins Starting points
        * @param destinations Destination points
        * @return Graph Visibility graph
        */
        Graph computeVisibilityGraphMultipleOD(std::vector<std::vector<Point>> points, std::vector<Point> origin, std::vector<Point> destination);
    };

}

#endif
