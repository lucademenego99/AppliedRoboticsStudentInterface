#include <iostream>
#include <map>
#include <algorithm>
#include <vector>
#include "graph.hpp"

/* Point struct, with a X and Y coordinate and an ID */

/**
 * @brief Point constructor
 * @param x coordinate
 * @param Y coordinate
 * @param polygon_id the ID of the polygon
*/
Point::Point (double x_coor, double y_coor, int polygon_id = -1) : x(x_coor), y(y_coor), polygon_id(polygon_id){
}

/**
 * @brief Verifies if two points are equivalent
 * @param point Point that wants to be compared
 * @return boolean Either true or false depending on the result
 */
bool Point::eq (Point point){
    return this != NULL && this->x == point.x && this->y == point.y;
}

/**
 * @brief Verifies if two points are not equivalent
 * @param point Point that wants to be compared
 * @return boolean Either true or false depending on the result
 */
bool Point::notEq (Point point){
    return !(eq(point));
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
    return ob.x == x && ob.y == y && ob.polygon_id == polygon_id;
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
Point Edge::get_adjacent(Point point){
    if(point.eq(this->p1)){
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
    return this->p1.eq(point) || this->p2.eq(point);
}
/**
 * @brief Checks if two edges are qual
 * @param edge The edge we want to compare
 * @return true or false depending on the result of the check
 */
bool Edge::eq (Edge edge){
    if((p1.eq(edge.p1)) && (p2.eq(edge.p2))){
        return true;
    }
    if((p1.eq(edge.p2)) && (p2.eq(edge.p1))){
        return true;
    }
    return false;
}
/**
 * @brief Checks if two edges are not equal
 * @param edge The edge we want to compare
 * @result true or false depending on the result of the check
 */
bool Edge::notEq (Edge edge){
    return !(eq(edge));
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
                p.polygon_id = pid;
                siblingPoint.polygon_id = pid;
                polygons[pid].push_back(edge);
            }
            add_edge(edge); 
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
void Graph::get_adjacent_points(Point point, std::vector<Point> &points){
    for(Edge e : graph[point]){
        Point p1 = e.get_adjacent(point);
        points.push_back(p1); 
    }
}
/**
 * @brief returns a list of all the points in the map
 * @param points the empty vector of points
 */
void Graph::get_points(std::vector<Point> &points){
    std::map<Point, std::vector<Edge>>::iterator it;
    for(it = graph.begin(); it != graph.end(); it++){
        for(Edge e : it->second){
            points.push_back(e.p1);
            points.push_back(e.p2);
        } 
    }
}
/**
 * @brief returns all the edges
 * @param edges the empty vector of edges
 */
void Graph::get_edges(std::vector<Edge> &results){
    for(Edge e : edges){
        results.push_back(e);
    }

}
/**
 * @brief adds an edge to the list of edges
 * @param edge the one we want to insert
 */
void Graph::add_edge(Edge edge){
    Point point1 = Point(edge.p1.x, edge.p1.y);
    Point point2 = Point(edge.p2.x, edge.p2.y);
    graph[point1].push_back(edge);
    graph[point2].push_back(edge);
    edges.push_back(edge);
}
/**
 * @brief get every edge associated to a point
 * @param point point we want to search
 * @param edges empty vector to fill
 */
void Graph::getItems(Point point, std::vector<Edge> &edges){
    if(graph.find(point) != graph.end()){
        for(Edge e : graph[point]){
            edges.push_back(e);
        }
    }
}
/** 
 * @brief checks if the map contains a point, if it does returns the edges of that point
 * @param point the point we need to check
 * @param edges the empty vector to fill
 */
void Graph::containsP (Point point, std::vector<Edge> &edges){
    for(Edge e : graph[point]){
        edges.push_back(e);
    }
}
/**
 * @brief checks if the vector of edges contains a specific edge, returns it after retrieving it
 * @param e the edge we are looking for
 */
Edge Graph::containsE (Edge e){
    if(std::find(edges.begin(), edges.end(), e) != edges.end()){
        std::vector<Edge>::iterator it = std::find(edges.begin(), edges.begin(), e);
        int index = std::distance(edges.begin(), it);
        Edge edge = edges.at(index);
        return edge;
    }
}