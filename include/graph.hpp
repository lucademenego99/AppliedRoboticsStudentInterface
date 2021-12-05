#ifndef GRAPH_H
#define GRAPH_H
#include <map>
#include <vector>

namespace visgraph
{
    class Point
    {
    public:
        //Attributes
        double x;
        double y;
        int polygonId;

        //Methods
        Point(double x_coor, double y_coor, int polygonId = -1);
        void print();
        bool operator<(const Point &ob) const;
        bool operator==(const Point &ob) const;
    };

    class Edge
    {
    public:
        //Attributes
        Point p1;
        Point p2;

        //Methods
        Edge(Point point1, Point point2);
        Point getAdjacent(Point point);
        bool contains(Point point);
        void print();
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

    class Graph
    {
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
        Graph(std::vector<std::vector<Point>> polygons);
        std::vector<Point> getAdjacentPoints(Point point);
        std::vector<Point> getPoints();
        std::vector<Edge> getEdges();
        void addEdge(Edge edge);
        std::vector<Edge> getItems(Point point);
        std::vector<Edge> containsP(Point point);
        Edge containsE(Edge e);
    };

}

#endif
