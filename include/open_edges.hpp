/**
 * @file open_edges.hpp
 * @brief Headers of a binary search tree, used for the visibility graph generation
 * @version 0.1
 * @date 2022-01-06
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef OPENEDGES
#define OPENEDGES

#include <vector>
#include "graph.hpp"
#include "visgraph.hpp"

namespace visgraph
{
    /**
     * @brief Binary search tree used for the visibility graph generation
     * 
     * It's at the core of the visibility graph generation algorithm, and it contains the edges sorted based on the angle they form with the point we are currently considering
     * 
     */
    class OpenEdges
    {
    private:

        /**
        * @brief Finds an element in the tree given the points and the edge
        * 
        * @param p1 First input point
        * @param p2 Second input point
        * @param edge Edge that contains the points
        * @return int 
        */
        int getIndex(Point p1, Point p2, Edge edge);

        /**
        * @brief Function that positions an element in the tree
        * 
        * @param p1 First input point
        * @param p2 Second input point
        * @param edge1 First input edge
        * @param edge2 Second input edge
        * @return true 
        * @return false 
        */
        bool lessThan(Point p1, Point p2, Edge edge1, Edge edge2);

    public:
        /**
         * @brief Vector containing the sorted OpenEdges
         * 
         */
        std::vector<Edge> openEdges;

        /**
         * @brief Visibility Graph utilities
         * 
         */
        VisGraph visGraph = VisGraph();

        /**
         * @brief Construct a new Open Edges object
         * 
         */
        OpenEdges();

        /**
        * @brief Inserts a new edge in the tree
        * 
        * @param p1 First edge point
        * @param p2 Second edge point
        * @param edge Edge that contains the two points
        */
        void insertEdge(Point p1, Point p2, Edge edge);

        /**
        * @brief Deletes from the tree a given an edge
        * 
        * @param p1 First input point
        * @param p2 Second input point
        * @param edge Edge that contains the two point
        */
        void deleteEdge(Point p1, Point p2, Edge edge);

        /**
        * @brief Gets the closest edge in a tree
        * 
        * @return Edge 
        */
        Edge getSmallest();

        /**
        * @brief Gets an edge from the tree given an index
        * 
        * @param index position of the element we want to retrieve
        * @return Edge 
        */
        Edge getEdge(int index);
    };
}

#endif