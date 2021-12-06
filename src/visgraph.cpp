#include <iostream>
#include "visgraph.hpp"
#include "open_edges.hpp"
#include "graph.hpp"
#include "shortest_path.hpp"
#include <math.h>
#include <algorithm>

#define _USE_MATH_DEFINES

using namespace std;

namespace visgraph
{
    Graph VisGraph::computeVisibilityGraph(vector<vector<Point>> points, Point origin, Point destination)
    {
        // Initial graph with all our shapes
        Graph initial = Graph(points);

        // Final visibility graph
        Graph result = Graph(points);

        // Get all the points we need to consider
        vector<Point> allPoints = initial.getPoints();

        // Loop through all the points we have
        for (int i = 0; i < allPoints.size(); i++)
        {
            // Get the visible vertices from the point we are considering
            vector<Point> visibleVertices = getVisibleVertices(allPoints[i], initial, origin, destination);

            // Add an edge (a,b) if b is visible from a
            for (int j = 0; j < visibleVertices.size(); j++)
            {
                result.addEdge(Edge(allPoints[i], visibleVertices[j]));
            }
        }

        return result;
    }

    vector<Point> VisGraph::getVisibleVertices(Point point, Graph graph, Point origin, Point destination)
    {
        vector<Edge> edges = graph.getEdges();
        vector<Point> points = graph.getPoints();

        if (!(origin == Point(-1, -1)))
            points.push_back(origin);
        if (!(destination == Point(-1, -1)))
            points.push_back(destination);

        // Sort the points in counter-clockwise order. If the angle is the same, take the closer ones.
        sort(points.begin(), points.end(), [&](const Point &lhs, const Point &rhs) -> bool
             {
                 double angleLhs = getAngle(point, lhs);
                 double angleRhs = getAngle(point, rhs);
                 if (angleLhs == angleRhs)
                 {
                     return edgeDistance(point, lhs) < edgeDistance(point, rhs);
                 }
                 return angleLhs < angleRhs;
             });

        OpenEdges openEdges = OpenEdges();
        Point pointInf = Point(INF, point.y);
        for (Edge edge : edges)
        {
            // If the point is not part of the edge we are considering
            if (!(edge.contains(point)))
            {
                // And if the line that starts from point and goes to the right (positive x axis) intersect the edge
                if (edgeIntersect(point, pointInf, edge))
                {
                    // And if one of the points of the edge is on the line that starts from point and goes to the right
                    if (!onSegment(point, edge.p1, pointInf) && !onSegment(point, edge.p2, pointInf))
                    {
                        // Add the edge inside the binary tree OpenEdges
                        openEdges.insertEdge(point, pointInf, edge);
                    }
                }
            }
        }

        vector<Point> visible;  // Final result: all the visible points from the point [point]
        Point prev = Point(-1, -1); // Keep the previous point to understand what is the line that moves counter-clockwise (the one that at first was [point - pointInf])
        bool prevVisible = false;   // Remember if the previous point was visible or not

        // Loop through all points (expcept the point we are considering)
        for (Point p : points)
        {
            if (!(p == point))
            {
                // if (getAngle(point, p) > M_PI)
                // {
                //     break;
                // }

                // Update open edges - remove clock wise edges because we already considered them (remember we are moving counter-clockwise)
                if (openEdges.openEdges.size() > 0)
                {
                    for (Edge e : graph.graph[p])
                    {
                        // So check the orientation: if it's clockwise delete the edge from openEdges
                        if (getOrientation(point, p, e.getAdjacent(p)) == CW)
                        {
                            openEdges.deleteEdge(point, p, e);
                        }
                    }
                }

                // Initialize the visibility of the point we are considering to false
                bool isVisible = false;

                // If it's the first iteration (prev is default point) or the orientation is not collinear or it's collinear and the previous point is between point and p
                if (prev == Point(-1, -1) || getOrientation(point, prev, p) != COLLINEAR || !onSegment(point, prev, p))
                {
                    // If there is nothing inside openEdges, then the point is visible because there is no edge that hides it
                    if (openEdges.openEdges.size() == 0)
                    {
                        isVisible = true;
                    }
                    // If there is something inside openEdges, but the segment [point - p] does not intersect the closer openEdge, then no segment is hiding the point and the point is visible
                    else if (!edgeIntersect(point, p, openEdges.getSmallest()))
                    {
                        isVisible = true;
                    }
                }
                else if (!prevVisible)
                {
                    // This is the case of collinear points. If the previous point was not visible, for sure even p is not because they are collinear
                    isVisible = false;
                }
                else
                {
                    // This is the case of collinear points, If the previous point was visible, we need to check (as always)
                    // that the edge from prev to p does not intersect any open edge
                    isVisible = true;
                    for (Edge e : openEdges.openEdges)
                    {
                        if (!(e.contains(prev)) && edgeIntersect(prev, p, e))
                        {
                            isVisible = false;
                            // We break because if there is at least one intersecting openEdge, we are sure the point won't be visible
                            break;
                        }
                    }
                    if (isVisible && edgeInPolygon(prev, p, graph))
                    {
                        isVisible = false;
                    }
                }

                // Check if the visible edge is interior to its polygon
                vector<Point> adjPoints = graph.getAdjacentPoints(point);
                if (isVisible && !(count(adjPoints.begin(), adjPoints.end(), p)))
                {
                    isVisible = !edgeInPolygon(point, p, graph);
                }

                if (isVisible)
                {
                    visible.push_back(p);
                }

                // Update open edges - Add counter clock wise edges incident on p
                for (Edge e : graph.graph[p])
                {
                    if (!(e.contains(point)) && getOrientation(point, p, e.getAdjacent(p)) == CCW)
                    {
                        openEdges.insertEdge(point, p, e);
                    }
                }

                prev = p;
                prevVisible = isVisible;
            }
        }

        return visible;
    }

    bool VisGraph::polygonCrossing(Point p1, vector<Edge> polygonEdges)
    {
        Point p2 = Point(INF, p1.y);
        int intersectCount = 0;
        for (Edge edge : polygonEdges)
        {
            if (!(p1.y < edge.p1.y && p1.y < edge.p2.y) && !(p1.y > edge.p1.y && p1.y > edge.p2.y) && !(p1.x > edge.p1.x && p1.x > edge.p2.x))
            {
                // Deal with points collinear to p1
                bool edgeP1Collinear = (getOrientation(p1, edge.p1, p2) == COLLINEAR);
                bool edgeP2Collinear = (getOrientation(p1, edge.p2, p2) == COLLINEAR);
                if (!edgeP1Collinear || !edgeP2Collinear)
                {
                    if (edgeP1Collinear || edgeP2Collinear)
                    {
                        Point collinearPoint = edgeP1Collinear ? edge.p1 : edge.p2;
                        if (edge.getAdjacent(collinearPoint).y > p1.y)
                        {
                            intersectCount += 1;
                        }
                    }
                    else if (edgeIntersect(p1, p2, edge))
                    {
                        intersectCount += 1;
                    }
                }
            }
        }
        return !(intersectCount % 2 == 0);
    }

    /**
     * @brief Is the edge from point p1 to point p2 interior to any polygon?
     * 
     * @param p1 
     * @param p2 
     * @param graph 
     * @return true The edge [p1, p2] is interior to a polygon
     * @return false The edge [p1, p2] is not interior to a polygon
     */
    bool VisGraph::edgeInPolygon(Point p1, Point p2, Graph graph)
    {
        if (p1.polygonId != p2.polygonId)
        {
            return false;
        }
        if (p1.polygonId == -1 || p2.polygonId == -1)
        {
            return false;
        }
        Point midPoint = Point((p1.x + p2.x) / 2, (p1.y + p2.y) / 2);
        return polygonCrossing(midPoint, graph.polygons[p1.polygonId]);
    }

    vector<Edge> VisGraph::pointInPolygon(Point p, Graph graph)
    {
        std::map<int, std::vector<Edge>>::iterator it;
        for (it = graph.polygons.begin(); it != graph.polygons.end(); it++)
        {
            if (polygonCrossing(p, it->second))
            {
                return it->second;
            }
        }
        vector<Edge> empty;
        return empty;
    }

    /**
 * @brief https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
 * 
 * @param p1 
 * @param q1 
 * @param edge 
 * @return true 
 * @return false 
 */
    bool VisGraph::edgeIntersect(Point p1, Point q1, Edge edge)
    {
        Point p2 = edge.p1;
        Point q2 = edge.p2;

        // Find the four orientations needed for general and
        // special cases
        int o1 = getOrientation(p1, q1, p2);
        int o2 = getOrientation(p1, q1, q2);
        int o3 = getOrientation(p2, q2, p1);
        int o4 = getOrientation(p2, q2, q1);

        // General case
        if (o1 != o2 && o3 != o4)
            return true;

        // Special Cases
        // p1, q1 and p2 are collinear and p2 lies on segment p1q1
        if (o1 == 0 && onSegment(p1, p2, q1))
            return true;

        // p1, q1 and q2 are collinear and q2 lies on segment p1q1
        if (o2 == 0 && onSegment(p1, q2, q1))
            return true;

        // p2, q2 and p1 are collinear and p1 lies on segment p2q2
        if (o3 == 0 && onSegment(p2, p1, q2))
            return true;

        // p2, q2 and q1 are collinear and q1 lies on segment p2q2
        if (o4 == 0 && onSegment(p2, q1, q2))
            return true;

        return false; // Doesn't fall in any of the above cases
    }

    // To find orientation of ordered triplet (p, q, r).
    // The function returns following values
    // 0 --> p, q and r are collinear
    // -1 --> Clockwise
    // 1 --> Counterclockwise
    int VisGraph::getOrientation(Point p, Point q, Point r)
    {
        // See https://www.geeksforgeeks.org/orientation-3-ordered-points/
        // for details of below formula.
        int val = (q.y - p.y) * (r.x - q.x) -
                  (q.x - p.x) * (r.y - q.y);

        if (val == 0)
            return 0; // collinear

        return (val < 0) ? 1 : -1; // clock or counterclock wise
    }

    bool VisGraph::onSegment(Point p, Point q, Point r)
    {
        return (q.x <= max(p.x, r.x) && (q.x >= min(p.x, r.x)) && ((q.y <= max(p.y, r.y) && (q.y >= min(p.y, r.y)))));
    }

    double VisGraph::getAngle(Point center, Point point)
    {
        double dx = point.x - center.x;
        double dy = point.y - center.y;
        if (dx == 0)
        {
            if (dy < 0)
                return M_PI * 3 / 2;
            return M_PI / 2;
        }
        if (dy == 0)
        {
            if (dx < 0)
                return M_PI;
            return 0;
        }
        if (dx < 0)
            return M_PI + atan(dy / dx);
        if (dy < 0)
            return (2 * M_PI) + atan(dy / dx);
        return atan(dy / dx);
    }

    double VisGraph::getAngle2(Point a, Point b, Point c)
    {
        double aValue = pow(c.x - b.x, 2) + pow(c.y - b.y, 2);
        double bValue = pow(c.x - a.x, 2) + pow(c.y - a.y, 2);
        double cValue = pow(b.x - a.x, 2) + pow(a.y - a.y, 2);
        double cosValue = (aValue + cValue - bValue) / (2.0 * sqrt(aValue) * sqrt(cValue));
        return acos(int(cosValue * T) / (1.0 * T2));
    }

    Point VisGraph::getIntersectPoint(Point p1, Point p2, Edge edge)
    {
        if (edge.contains(p1))
            return p1;
        if (edge.contains(p2))
            return p2;
        if (edge.p1.x == edge.p2.x)
        {
            if (p1.x == p2.x)
                return Point(-1, -1);
            double pSlope = double(p1.y - p2.y) / (p1.x - p2.x);
            double intersectX = edge.p1.x;
            double intersectY = pSlope * (intersectX - p1.x) + p1.y;
            return Point(intersectX, intersectY);
        }

        double pSlope = double(p1.y - p2.y) / (p1.x - p2.x);
        double eSlope = double(edge.p1.y - edge.p2.y) / (edge.p1.x - edge.p2.x);
        if (eSlope == pSlope)
            return Point(-1, -1);
        double intersectX = ((eSlope * edge.p1.x) - (pSlope * p1.x) + p1.y - edge.p1.y) / (eSlope - pSlope);
        double intersectY = eSlope * (intersectX - edge.p1.x) + edge.p1.y;
        return Point(intersectX, intersectY);
    }

    double VisGraph::edgeDistance(Point p1, Point p2)
    {
        return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));
    }

    double VisGraph::pointEdgeDistance(Point p1, Point p2, Edge edge)
    {
        Point intersectPoint = getIntersectPoint(p1, p2, edge);
        return (!(intersectPoint == Point(-1, -1))) ? edgeDistance(p1, intersectPoint) : 0;
    }

    std::vector<Point> VisGraph::shortest_path(DictG graph, Point origin, Point destination) {
        if(!(graph.find(origin) == graph.end() || graph.find(destination) == graph.end()))
            return shortestPath(graph, origin, destination);
        else {
            std::vector<Point> path;
            std::cout << "ERROR: origin point or destination point not available in graph!" << std::endl;
            return path;
        }
    }
}
