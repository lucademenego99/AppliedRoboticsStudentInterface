#include <iostream>
#include <map>
#include <algorithm>
#include <vector>
#include "graph.hpp"

namespace visgraph
{
    /* Point struct, with a X and Y coordinate and an ID */

    /**
 * @brief Point constructor
 * @param x coordinate
 * @param Y coordinate
 * @param polygonId the ID of the polygon
*/
    Point::Point(double x_coor, double y_coor, int polygonId) : x(x_coor), y(y_coor), polygonId(polygonId) {}

    /**
 * @brief Prints a more detailed representation
 */
    void Point::print()
    {
        std::cout << "Point(%.2f, %.2f)" << x, y;
    }

    bool Point::operator<(const Point &ob) const
    {
        return x < ob.x || (x == ob.x && y < ob.y);
    }

    bool Point::operator==(const Point &ob) const
    {
        return ob.x == x && ob.y == y && ob.polygonId == polygonId;
    }
    /**
 * @brief Constructor for Edge
 * @param p1 First point
 * @param p2 Second point
 */
    Edge::Edge(const Point point1, const Point point2) : p1(point1), p2(point2) {}

    /**
 * @brief Fetches the adjacent points to a certain point
 * @param point Point we're interested in
 * @return returns the adjacent
 */
    Point Edge::getAdjacent(Point point)
    {
        if (point == p1)
        {
            return p2;
        }
        return p1;
    }

    /**
 * @brief Checks if a point is part of the edge
 * @param point Point we need to find
 * @return return whether or not the point is present
 */
    bool Edge::contains(Point point)
    {
        return p1 == point || p2 == point;
    }

    /**
 * @brief Prints the basic information of the edge
 */
    void Edge::print()
    {
        std::cout << "P1.x: %f" << p1.x << std::endl;
        std::cout << "P1.y: %f" << p1.y << std::endl;
        std::cout << "P2.x: %f" << p2.x << std::endl;
        std::cout << "P2.y: %f" << p2.y << std::endl;
    }

    bool Edge::operator==(const Edge &ob) const
    {
        return ob.p1 == p1 && ob.p2 == p2;
    }

    /**
 * @brief Constructor of the graph
 * @param polygons a map of polygons, keys are IDs and values the edges
 * @returns the complete graph
 */
    Graph::Graph(std::vector<std::vector<Point>> shapes)
    {
        pid = 0;
        for (std::vector<Point> it : shapes)
        {
            if (it[0] == it[it.size() - 1] && it.size() > 1)
            {
                it.pop_back();
            }
            int count = 0;
            for (Point p : it)
            {
                Point siblingPoint = it[(count + 1) % (it.size())];
                Edge edge = Edge(p, siblingPoint);
                if ((it.size()) > 2)
                {
                    p.polygonId = pid;
                    siblingPoint.polygonId = pid;
                    polygons.find(pid)->second.push_back(edge);
                }
                addEdge(edge);
                count++;
            }
            if (it.size() > 2)
            {
                pid += 1;
            }
        }
    }
    /**
 * @brief returns the adjacent edges to a certain point
 * @param point the point we want to consider
 * @param edges an empty list to fill with the edges that have been found
 */
    std::vector<Point> Graph::getAdjacentPoints(Point point)
    {
        std::vector<Point> points;
        for (Edge e : this->graph[point])
        {
            Point p1 = e.getAdjacent(point);
            points.push_back(p1);
        }
        return points;
    }
    /**
 * @brief returns a list of all the points in the map
 * @param points the empty vector of points
 */
    std::vector<Point> Graph::getPoints()
    {
        std::vector<Point> points;
        std::map<Point, std::vector<Edge>>::iterator it;
        for (it = graph.begin(); it != graph.end(); it++)
        {
            Point point = Point(it->first.x, it->first.y);
            points.push_back(it->first);
        }
        return points;
    }
    /**
 * @brief returns all the edges
 * @param edges the empty vector of edges
 */
    std::vector<Edge> Graph::getEdges()
    {
        std::vector<Edge> edges;
        for (Edge e : edges)
        {
            edges.push_back(e);
        }
        return edges;
    }
    /**
 * @brief adds an edge to the list of edges
 * @param edge the one we want to insert
 */
    void Graph::addEdge(Edge edge)
    {
        graph.find(Point(edge.p1.x, edge.p1.y))->second.push_back(edge);
        graph.find(Point(edge.p2.x, edge.p2.y))->second.push_back(edge);
        edges.push_back(edge);
    }
    /**
 * @brief get every edge associated to a point
 * @param point point we want to search
 * @param edges empty vector to fill
 */
    std::vector<Edge> Graph::getItems(Point point)
    {
        std::vector<Edge> edges;
        if (graph.find(point) != graph.end())
        {
            for (Edge e : this->graph[point])
            {
                edges.push_back(e);
            }
        }
        return edges;
    }
    /** 
 * @brief checks if the map contains a point, if it does returns the edges of that point
 * @param point the point we need to check
 * @param edges the empty vector to fill
 */
    std::vector<Edge> Graph::containsP(Point point)
    {
        std::vector<Edge> edges;
        if (graph.count(point) > 1)
        {
            edges = graph.find(point)->second;
        }
        return edges;
    }
    /**
 * @brief checks if the vector of edges contains a specific edge, returns it after retrieving it
 * @param e the edge we are looking for
 * @param edge the element we are look
 */
    Edge Graph::containsE(Edge e)
    {
        std::vector<Edge>::iterator it = std::find(edges.begin(), edges.end(), e);
        if (it != edges.end())
        {
            int index = std::distance(edges.begin(), it);
            return edges.at(index);
        }
        return Edge(Point(-1, -1), Point(-1, -1));
    }
}