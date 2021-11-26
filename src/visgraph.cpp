#include <iostream>
#include "visgraph.hpp"
#include <math.h>

#define _USE_MATH_DEFINES

using namespace student;
using namespace std;

Graph *VisGraph::computeVisibilityGraph(vector<vector<Point> > points) {
    Graph result = new Graph(points);
    for (int i = 0; i < points.size(); i++) {
        vector<Point> visibleVertices = getVisibleVertices(points[i], result);
        for (int j = 0; j < visibleVertices.size(); j++) {
            result.add_edge(Edge(points[i], visibleVertices[j]));
        }
    }
    return result;
}

vector<Point> VisGraph::getVisibleVertices(Point point, Graph graph) {
    vector<Edge> edges = graph.getEdges();
    vector<Point> points = graph.getPoints();

    // TODO: sort points according to the clockwise angle that the half line from
    // [point] to each vertex makes with the positive x-axis. In case of ties,
    // vertices closer to [point] should come before vertices farther.

    OpenEdges openEdges = new OpenEdges();
    Point pointInf = Point(INF, point.y);
    for (int i = 0; i < edges.size(); i++) {
        if (!(edges[i].p1 == point || edges[i].p2 == point)) {
            if (edgeIntersect(point, pointInf, edges[i])) {
                if (!onSegment(point, edge.p1, pointInf) && !onSegment(point, edge.p2, pointInf)) {
                    openEdges.insert(point, pointInf, edges[i]);
                }
            }
        }
    }

    vector<Point> visible;
    Point prev;
    bool prevVisible = false;

    for (int i = 0; i < points.size(); i++) {
        if (points[i] != point) {
            if (angle(point, points[i] > M_PI) {
                break;
            }

            // Update open edges - remove clock wise edges incident on p
            if (openEdges.size > 0) {
                for (int j = 0; j < graph.at(points[i]).size(); j++) {
                    if (getOrientation(point, p, graph.at(points[i])[j]) == CW) {
                        openEdges.delete(point, points[i], graph.at(points[i])[j]);
                    }
                }
            }

            bool isVisible = false;

            if (prev == nullptr || getOrientation(point, prev, points[i] != COLLINEAR || !onSegment(point, prev, points[i]))) {
                if (openEdges.size() == 0) {
                    isVisible = true;
                } else if (!edgeIntersect(point, points[i], openEdges.smallest())) {
                    isVisible = true;
                }
            }
            // For collinear points, if previous point was not visible, p is not
            else if (!prevVisible) {
                isVisible = false;
            } else {
                // For collinear points, if previous point was visible, we need to check
                // that the edge from prev to p does not intersect any open edge
                isVisible = true;
                for (int j = 0; j < graph.at(points[i]).size(); j++) {
                    if ((prev != graph.at(points[i]).p1 && prev != graph.at(points[i]).p2) && edgeIntersect(prev, points[i], graph.at(points[i]))) {
                        isVisible = false;
                        break;
                    }
                }
                if (isVisible && edgeInPolygon(prev, points[i], graph)) {
                    isVisible = false;
                }
            }

            // Check if the visible edge is interior to its polygon


        }
    }
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
bool VisGraph::edgeIntersect(Point p1, Point q1, Edge edge) {
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
    if (o1 == 0 && onSegment(p1, p2, q1)) return true;
 
    // p1, q1 and q2 are collinear and q2 lies on segment p1q1
    if (o2 == 0 && onSegment(p1, q2, q1)) return true;
 
    // p2, q2 and p1 are collinear and p1 lies on segment p2q2
    if (o3 == 0 && onSegment(p2, p1, q2)) return true;
 
     // p2, q2 and q1 are collinear and q1 lies on segment p2q2
    if (o4 == 0 && onSegment(p2, q1, q2)) return true;
 
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
 
    if (val == 0) return 0;  // collinear
 
    return (val > 0)? 1: -1; // clock or counterclock wise
}

bool VisGraph::onSegment(Point p, Point q, Point r) {
    return (q.x <= max(p.x, r.x) && (q.x >= min(p.x, r.x)) && ((q.y <= max(p.y, r.y) && (q.y >= min(p.y, r.y)))));
}

double VisGraph::getAngle(Point center, Point point) {
    double dx = point.x - center.x;
    double dy = point.y - center.y;
    if (dx == 0) {
        if (dy < 0)
            return M_PI * 3 / 2;
        return M_PI / 2;
    }
    if (dy == 0) {
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