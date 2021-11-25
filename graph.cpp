#include <iostream>
#include <map>
#include <algorithm>
#include <vector>
#include "graph.hpp"

/* Data structure to represent the graph in which we have
   a Map with as keys structure Point and as values a vector 
   of Edge
*/
typedef std::map<Point, std::vector<Edge>> DictG;
/* We define the iterator to traverse the map */
typedef DictG::const_iterator ItG;

/* Data structure to represent the polygons that are present
   in the arena: a Map which has as keys IDs and as values a vector
   of Edge
*/
typedef std::map<int, std::vector<Edge>> DictP;
/* We define the iterator to traverse the map */
typedef DictP::const_iterator ItP

/* Point struct, with a X and Y coordinate and an ID */

/**
 * @brief Point constructor
 * @param x coordinate
 * @param Y coordinate
 * @param polygon_num the 
*/
Point::Point (x, y, polygon_num=-1){
    this->x = x_coor;
    this->y = y_coor;
    this->polygon_id = polygon_num;
}

/**
 * @brief Verifies if two points are equivalent
 * @param point Point that wants to be compared
 * @return boolean Either true or false depending on the result
 */
boolean Point::eq (Point point){
    return this != null && this->x == point.x && this->y == point.y;
}

/**
 * @brief Verifies if two points are not equivalent
 * @param point Point that wants to be compared
 * @return boolean Either true or false depending on the result
 */
boolean Point::notEq (Point point){
    return !(this.equivalence(point));
}

/**
 * @brief Compares the hash of two points
 * @param point Point that wants to be compare
 * @return boolean Either true or false depending on the result
 */
boolean Point::lt (Point point){
    return std::hash(this) < std::hash(point);
}
/**
 * @brief Prints the coordinates of the point
 */
void Point::print (){
    cout<<"(%.2f, %.2f)"<< this->x, this->y;
}

//Hash method, not entirely sure
/**
 * @brief Hash method
 * @return Xor of the hash of X and Y
 */
boolean Point::hash (){
    return this->x->hash() ^ this->y->hash();
}

/**
 * @brief Prints a more detailed representation
 */
void Point::repr (){
    cout<<"Point(%.2f, %.2f)"<<this->x, this->y;
}



Edge::Edge(Point p1, Point p2){
    this.p1 = p1;
    this.p2 = p2;
}

Point Edge::get_adjacent(Point point){
    if(point == this->p1){
        return this->p2;
    }
    return this->p1;
}

boolean Edge::contains (Point point){
    return this->p1 == point || this->p2 == point;
}

boolean Edge::eq (Edge edge){
    if((this->p1 == edge->p1) && (this->p2 == edge->p2)){
        return true;
    }
    if((this->p1 == edge->p2) && (this->p2 == edge->p1)){
        return true;
    }
    return false;
}

boolean Edge::notEq (Edge edge){
    return !(this.eq(edge));
}

void Edge::print (){
    cout<<"P1.x: %f"<< this.p1.x << endl;
    cout<<"P1.y: %f"<< this.p1.y << endl;
    cout<<"P2.x: %f"<< this.p2.x << endl;
    cout<<"P2.y: %f"<< this.p2.y << endl;
}

//Not entirely sure whether or not this is correct
boolean Edge::hash(){
    return this.p1.hash() ^ this.p2.hash();
}

Graph::Graph (DictP polygons){
    pid = 0;
    for(ItP = polygons.begin(); ItP != polygons.end(); ItP++){
        if(ItP[0] == ItP[-1] && ItP.size()>1){
            ItP.pop_back();
        }
        for(Point p in ItP){
            auto it = find(Itp.begin(), ItP.end(), p);
            Point sibling_point = ItP[(it->first)%ItP.size()];
            Edge edge = new Edge(p, sibling_point);
            if(ItP.size() > 2){
                p.polygon_id = pid;
                sibling_point.polygon_id = pid;
                this->polygons[pid].push_back(edge);
            }
            this.add_edge(edge);
        }
        if(polygon.size() > 2){
            pid += 1;
        }
    }
}

void Graph::get_adjacent_points(Point point, std::vector<Edge> edges){
    for(Edge e in this->graph[point]){
        Edge e1 = e.get_adjacent(point);
        edges.push_back(e1);
    }
}

void Graph::get_points(std::vector<Point> &points){
    for(ItG it(this->graph.begin()); it != this->graph.end(); it++){
        Point point = new Point(it->first.first, it->first.second);
        points.push_back(point);
    }
}

void Graph::get_edges(std::vector<Edge> &edges){
    for(Edge e in this->edges){
        edges.pushback(e);
    }
}

void Graph::add_edge(Edge edge){
    Point point = new Point(edge.p1, edge.p2);
    this->graph[point].push_back(edge);
    this->edges.push_back(edge);
}

void Graph::getItems(Point point, std::vector<Edge> &edges){
    if(this->graph[point] != null){
        for(Edge e in this->graph[point]){
            edges.push_back(e)
        }
    }else{
        edges = null;
    }
}

int Graph::containsP (Point point, std::vector<Edge> &edges){
    if(this->graph.count(point)){
        auto search = this->graph.find(point);
        edges = search->second;
    }
    edges = null;

}

int Graph::containsE (Edge e, Edge edge){
    if(std::find(this->edges.begin(), this->edges.end(), e) != vec.end()){
        auto search = std::find(this->edges.begin(), this->edges.end(), e);
        edge = search->second;
    }
    edge = null;
}