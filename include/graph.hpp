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

        int SCALE_FACTOR = 1000;

        /**
        * @brief Point constructor
        * 
        * @param x coordinate
        * @param Y coordinate
        * @param polygonId the ID of the polygon
        */
        Point(double x_coor, double y_coor, int polygonId = -1);

        /**
         * @brief Prints the coordinates of the point
         * 
         */
        void print();

        /**
        * @brief Scale point by a SCALE_FACTOR.
        * In this way during the calculations for the visibility graph
        * there will be no floating point calculations errors due to the
        * fact that we are dealing with too small numbers.
        * 
        */
        void scale();

        /**
        * @brief Convert the point to the original size.
        * After having converted a point with scale(),
        * we can convert it to the original size using this function
        * 
        */
        void rescaleToOriginal();
        bool operator<(const Point &ob) const;
        bool operator==(const Point &ob) const;
    };

    class Edge
    {
    public:
        //Attributes
        Point p1;
        Point p2;

        /**
        * @brief Constructor for Edge
        * 
        * @param p1 First point
        * @param p2 Second point
        */
        Edge(Point point1, Point point2);

        /**
        * @brief Returns the euclidean distance between two points
        * 
        * @param p1 A simple point (x,y)
        * @param p2 A simple point (x,y)
        * @return double Distance between two points
        */
        double weight();

        /**
        * @brief Fetches the adjacent points to a certain point
        * 
        * @param point Point we're interested in
        * @return returns the adjacent
        */
        Point getAdjacent(Point point);

        /**
        * @brief Checks if a point is part of the edge
        * 
        * @param point Point we need to find
        * @return return whether or not the point is present
        */
        bool contains(Point point);

        /**
        * @brief Prints the basic information of the edge
        * 
        */
        void print();
        bool operator==(const Edge &ob) const;
    };

    /**
     * @brief Data structure to represent the graph in which we have a Map with as keys structure Point and as values a vector of Edge
     * 
     */
    typedef std::map<Point, std::vector<Edge>> DictG;

    /**
     * @brief Data structure to represent the polygons that are present in the arena: a Map which has as keys IDs and as values a vector of Edge
     * 
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
        // Is this graph a visibility graph
        bool isVisibilityGraph;

        /**
        * @brief Constructor of the graph
        * 
        * @param polygons a map of polygons, keys are IDs and values the edges
        * @returns the complete graph
        */
        Graph(std::vector<std::vector<Point>> polygons, bool isVisibilityGraph = false, bool isOriginalGraph = false);

        /**
        * @brief Dijkstra Shortest path.
        * Reference: https://www.geeksforgeeks.org/dijkstras-shortest-path-algorithm-using-set-in-stl/
        * Complexity: O(E logV)
        * 
        * @param graph A map <Point, vector of adjacent edges>
        * @param origin The origin point in which we want to start
        * @param destination Our destination
        * @return std::vector<Point> The complete shortest path from origin to destination
        */
        std::vector<Point> shortestPath(Point origin, Point destination);

        /**
         * @brief Dijkstra Shortest path, multiple destinations.
         * Reference: https://www.geeksforgeeks.org/dijkstras-shortest-path-algorithm-using-set-in-stl/
         * Complexity: O(E logV)
         * 
         * @param graph A map <Point, vector of adjacent edges>
         * @param origin The origin point in which we want to start
         * @param destinations All possible destinations
         * @return std::vector<Point> The complete shortest path from origin to destination
         */
        std::vector<Point> shortestPathMultipleD(Point origin, std::vector<Point> destinations);

        /**
        * @brief returns the adjacent edges to a certain point
        * 
        * @param point the point we want to consider
        * @param edges an empty list to fill with the edges that have been found
        */
        std::vector<Point> getAdjacentPoints(Point point);

        /**
        * @brief returns a list of all the points in the map
        * 
        * @param points the empty vector of points
        */
        std::vector<Point> getPoints();

        /**
        * @brief returns all the edges
        * 
        * @param edges the empty vector of edges
        */
        std::vector<Edge> getEdges();

        /**
        * @brief adds an edge to the list of edges
        * 
        * @param edge the one we want to insert
        */
        void addEdge(Edge edge);

        /**
        * @brief get every edge associated to a point
        * 
        * @param point point we want to search
        */
        std::vector<Edge> getItems(Point point);

        /** 
        * @brief checks if the map contains a point, if it does returns the edges of that point
        * 
        * @param point the point we need to check
        * @param edges the empty vector to fill
        */
        std::vector<Edge> containsP(Point point);

        /**
        * @brief checks if the vector of edges contains a specific edge, returns it after retrieving it
        * 
        * @param e the edge we are looking for
        */
        Edge containsE(Edge e);
    };

}

#endif
