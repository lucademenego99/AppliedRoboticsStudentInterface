#ifndef GRAPH_H
#define GRAPH_H
#include <map>
#include <vector>

class Point{
    public:
        //Attributes
        double x;
        double y;
        int polygon_id;

        //Methods
        Point (double x_coor, double y_coor, int polygon_id);
        bool eq (Point point);
        bool notEq (Point point);
        void print ();
        void repr();
        bool operator<(const Point &ob) const;
        bool operator==(const Point &ob) const;
};

class Edge {
    public:
        //Attributes
        Point p1;
        Point p2;

        //Methods
        Edge(const Point point1, const Point point2);
        Point get_adjacent(Point point);
        bool contains (Point point);
        bool eq (Edge edge);
        bool notEq (Edge edge);
        void print ();
        bool operator==(const Edge &ob) const;
};

/* Data structure to represent the graph in which we have
   a Map with as keys structure Point and as values a vector 
   of Edge
*/
typedef std::map<Point, std::vector<Edge>> DictG;

/* Data structure to represent the polygons that are present
   in the arena: a Map which has as keys IDs and as values a vector
   of Edge
*/
typedef std::map<int, std::vector<Edge>> DictP;

class Graph{
    public:
        //Attributes
        //Struct for graph
        DictG graph;
        //Set of edges
        std::vector<Edge> edges;
        int pid;
        //Struct for polygons
        DictP polygons;
        
        //Methods
        Graph (std::vector<std::vector<Point>> polygons);
        void get_adjacent_points(Point point, std::vector<Point> &points); //Correct
        void get_points(std::vector<Point> &points); //Correct
        void get_edges(std::vector<Edge> &results); //Correct
        void add_edge(Edge edge); //Correct
        void getItems(Point point, std::vector<Edge> &edges); //Correct
        void containsP (Point point, std::vector<Edge> &edges); //
        Edge containsE (Edge e);
};

#endif