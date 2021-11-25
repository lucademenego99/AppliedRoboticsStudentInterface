#ifndef GRAPH_H
#define GRAPH_H

class Point{
    public:
        //Attributes
        double x;
        double y;
        int polygon_id;

        //Methods
        Point (x, y, polygon_num=-1);
        boolean eq (Point point);
        boolean notEq (Point point);
        boolean lt (Point point);
        void print ();
        boolean hash();
        void repr();
};

class Edge {
    public:
        //Attributes
        Point p1;
        Point p2;

        //Methods
        Edge(Point p1, Point p2);
        Point get_adjacent(Point point);
        boolean contains (Point point);
        boolean eq (Edge edge);
        boolean notEq (Edge edge);
        void print ();
        boolean hash();
};

class Graph{
    public:
        //Attributes
        //Struct for graph
        DictG graph;
        //Set of edges
        std::vector<Edges> edges;
        int pid;
        //Struct for polygons
        DictP polygons;
        
        //Methods
        Graph (DictP polygons);
        void get_adjacent_points(Point point, std::vector<Edge> edges);
        void get_points(std::vector<Point> &points);
        void get_edges(std::vector<Edge> &edges);
        void add_edge(Edge edge);
        void getItems(Point point, std::vector<Edge> &edges);
        int containsP (Point point, std::vector<Edge> &edges);
        int Graph::containsE (Edge e, Edge edge);
}

#endif