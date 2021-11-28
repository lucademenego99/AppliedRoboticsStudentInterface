#include <iostream>
#include "visgraph.hpp"
#include "open_edges.hpp"
#include "graph.hpp"
#include <math.h>
#include <algorithm>

#define _USE_MATH_DEFINES

using namespace std;

namespace visgraph
{
    Graph VisGraph::computeVisibilityGraph(vector<vector<Point>> points)
    {
        Graph result = Graph(points);
        vector<Point> allPoints = result.getPoints();
        for (int i = 0; i < allPoints.size(); i++)
        {
            vector<Point> visibleVertices = getVisibleVertices(allPoints[i], result);
            for (int j = 0; j < visibleVertices.size(); j++)
            {
                result.addEdge(Edge(allPoints[i], visibleVertices[j]));
            }
        }
        return result;
    }

    vector<Point> VisGraph::getVisibleVertices(Point point, Graph graph)
    {
        vector<Edge> edges = graph.getEdges();
        vector<Point> points = graph.getPoints();

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
            if (!(edge.p1 == point || edge.p2 == point))
            {
                if (edgeIntersect(point, pointInf, edge))
                {
                    if (!onSegment(point, edge.p1, pointInf) && !onSegment(point, edge.p2, pointInf))
                    {
                        openEdges.insertEdge(point, pointInf, edge);
                    }
                }
            }
        }

        vector<Point> visible;
        Point prev = Point(-1, -1);
        bool prevVisible = false;

        for (Point p : points)
        {
            if (!(p == point))
            {
                if (getAngle(point, p) > M_PI)
                {
                    break;
                }

                // Update open edges - remove clock wise edges incident on p
                if (openEdges.openEdges.size() > 0)
                {
                    vector<Edge> edges = graph.getItems(p);
                    for (Edge e : edges)
                    {
                        if (getOrientation(point, p, e.getAdjacent(p)) == CW)
                        {
                            openEdges.deleteEdge(point, p, e);
                        }
                    }
                }

                bool isVisible = false;

                if (prev == Point(-1, -1) || getOrientation(point, prev, p) != COLLINEAR || !onSegment(point, prev, p))
                {
                    if (openEdges.openEdges.size() == 0)
                    {
                        isVisible = true;
                    }
                    else if (!edgeIntersect(point, p, openEdges.getSmallest()))
                    {
                        isVisible = true;
                    }
                }
                else if (!prevVisible)
                {
                    // For collinear points, if previous point was not visible, p is not
                    isVisible = false;
                }
                else
                {
                    // For collinear points, if previous point was visible, we need to check
                    // that the edge from prev to p does not intersect any open edge
                    isVisible = true;
                    vector<Edge> edges = graph.getItems(p);
                    for (Edge e : edges)
                    {
                        if (!(prev == e.p1) && !(prev == e.p2) && edgeIntersect(prev, p, e))
                        {
                            isVisible = false;
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
                vector<Edge> edges = graph.getItems(p);
                for (Edge e : edges)
                {
                    if (!e.contains(point) && getOrientation(point, p, e.getAdjacent(p)) == CCW)
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

        return (val > 0) ? 1 : -1; // clock or counterclock wise
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
}