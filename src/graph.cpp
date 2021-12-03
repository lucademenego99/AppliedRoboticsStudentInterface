#include <iostream>
#include <map>
#include <algorithm>
#include <vector>
#include "graph.hpp"

using namespace visgraph;

/* Point struct, with a X and Y coordinate and an ID */

/**
 * @brief Point constructor
 * @param x coordinate
 * @param Y coordinate
 * @param polygonId the ID of the polygon
*/
Point::Point (double x_coor, double y_coor, int polygonId) : x(x_coor), y(y_coor), polygonId(polygonId){
}

/**
 * @brief Prints the coordinates of the point
 */
void Point::print (){
    std::cout<<"( "<< this->x << ", " << this->y << ")";
}

/**
 * @brief Prints a more detailed representation
 */
void Point::repr (){
    std::cout<<"Point( "<<this->x << ", " << this->y << ")";
}

bool Point::operator<(const Point &ob) const {
    return x < ob.x || (x == ob.x && y < ob.y);
}

bool Point::operator==(const Point &ob) const{
    return ob.x == x && ob.y == y && ob.polygonId == polygonId;
}
/**
 * @brief Constructor for Edge
 * @param p1 First point
 * @param p2 Second point
 */
Edge::Edge(const Point point1, const Point point2): p1(point1) , p2(point2){
}

/**
 * @brief Fetches the adjacent points to a certain point
 * @param point Point we're interested in
 * @return returns the adjacent
 */
Point Edge::getAdjacent(Point point){
    if(point == this->p1){
        return this->p2;
    }
    return this->p1;
}

/**
 * @brief Checks if a point is part of the edge
 * @param point Point we need to find
 * @return return whether or not the point is present
 */
bool Edge::contains (Point point){
    return this->p1 == point || this->p2 == point;
}

/**
 * @brief Prints the basic information of the edge
 */
void Edge::print (){
    std::cout<<"P1.x: "<< p1.x << "\n";
    std::cout<<"P1.y: "<< p1.y << "\n";
    std::cout<<"P2.x: "<< p2.x << "\n";
    std::cout<<"P2.y: "<< p2.y << "\n";
}

bool Edge::operator==(const Edge &ob) const{
    return ob.p1 == p1 && ob.p2 == p2;
}
/**
 * @brief Constructor of the graph
 * @param polygons a map of polygons, keys are IDs and values the edges
 * @returns the complete graph
 */
Graph::Graph (std::vector<std::vector<Point>> shapes){
    pid = 0;
    for(std::vector<Point> it : shapes){
        if(it[0] == it[it.size()-1] && it.size() > 1){
            it.pop_back();
        }
        int count = 0;
        for(Point p : it){
            Point siblingPoint = it[(count + 1)%(it.size())]; 
            Edge edge = Edge(p, siblingPoint);
            if((it.size()) > 2){
                p.polygonId = pid;
                siblingPoint.polygonId = pid;
                polygons[pid].push_back(edge);
            }
            addEdge(edge); 
            count++;
        }
        if(it.size() > 2){
            pid += 1;
        }
    }
}
/**
 * @brief returns the adjacent edges to a certain point
 * @param point the point we want to consider
 * @param edges an empty list to fill with the edges that have been found
 */
std::vector<Point> Graph::getAdjacentPoints(Point point){
    std::vector<Point> points;
    for(Edge e : graph[point]){
        Point p1 = e.getAdjacent(point);
        points.push_back(p1); 
    }
    return points;
}
/**
 * @brief returns a list of all the points in the map
 * @param points the empty vector of points
 */
std::vector<Point> Graph::getPoints(){
    std::vector<Point> points;
    std::map<Point, std::vector<Edge>>::iterator it;
    for(it = graph.begin(); it != graph.end(); it++){
        for(Edge e : it->second){
            points.push_back(e.p1);
            points.push_back(e.p2);
        } 
    }
    return points;
}
/**
 * @brief returns all the edges
 * @param edges the empty vector of edges
 */
std::vector<Edge> Graph::getEdges(){
    std::vector<Edge> results;
    for(Edge e : edges){
        results.push_back(e);
    }
    return results;
}
/**
 * @brief adds an edge to the list of edges
 * @param edge the one we want to insert
 */
void Graph::addEdge(Edge edge){
    graph[Point(edge.p1.x, edge.p1.y)].push_back(edge);
    graph[Point(edge.p2.x, edge.p2.y)].push_back(edge);
    edges.push_back(edge);
}
/**
 * @brief get every edge associated to a point
 * @param point point we want to search
 * @param edges empty vector to fill
 */
std::vector<Edge> Graph::getItems(Point point){
    std::vector<Edge> edges;
    if(graph.find(point) != graph.end()){
        for(Edge e : graph[point]){
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
std::vector<Edge> Graph::containsP(Point point){
    std::vector<Edge> edges;
    for(Edge e : graph[point]){
        edges.push_back(e);
    }
    return edges;
}
/**
 * @brief checks if the vector of edges contains a specific edge, returns it after retrieving it
 * @param e the edge we are looking for
 */
Edge Graph::containsE (Edge e) {
    if(std::find(edges.begin(), edges.end(), e) != edges.end()){
        std::vector<Edge>::iterator it = std::find(edges.begin(), edges.begin(), e);
        int index = std::distance(edges.begin(), it);
        Edge edge = edges.at(index);
        return edge;
    }
    return Edge(Point(-1, -1), Point(-1, -1));
}