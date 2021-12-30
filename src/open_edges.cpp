#include "open_edges.hpp"
#include "utils.hpp"
#include "dubins.hpp"
#include "graph.hpp"
#include "visgraph.hpp"

namespace visgraph
{
    /**
     * @brief Construct a new Open Edges:: Open Edges object
     * 
     */
    OpenEdges::OpenEdges()
    {
        openEdges.clear();
    }

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
    bool OpenEdges::lessThan(Point p1, Point p2, Edge edge1, Edge edge2)
    {
        std::vector<dubins::DubinsPoint> results; //Just for the sake of passing correct parameters to the function
        std::vector<double> t;
        dubins::Dubins dubin;
        if (edge1 == edge2)
            return false;
        if (!dubin.intersLineLine(dubins::DubinsPoint(p1.x, p1.y), dubins::DubinsPoint(p2.x, p2.y), dubins::DubinsPoint(edge2.p1.x, edge2.p1.y), dubins::DubinsPoint(edge2.p2.x, edge2.p2.y), results, t))
            return true;
        double edge1Distance = visGraph.pointEdgeDistance(p1, p2, edge1);
        double edge2Distance = visGraph.pointEdgeDistance(p1, p2, edge2);
        if (edge1Distance == edge2Distance)
        {
            Point samePoint = edge2.contains(edge1.p1) ? edge1.p1 : edge1.p2;
            double angleEdge1 = visGraph.getAngle2(p1, p2, edge1.getAdjacent(samePoint));
            double angleEdge2 = visGraph.getAngle2(p1, p2, edge2.getAdjacent(samePoint));
            return angleEdge1 < angleEdge2;
        }
        else
        {
            return edge1Distance < edge2Distance;
        }
    }

    /**
     * @brief Gets an edge from the tree given an index
     * 
     * @param index position of the element we want to retrieve
     * @return Edge 
     */
    Edge OpenEdges::getEdge(int index)
    {
        if (openEdges.size() > index)
            return openEdges[index];
        return Edge(Point(-1, -1), Point(-1, -1));
    }

    /**
     * @brief Gets the closest edge in a tree
     * 
     * @return Edge 
     */
    Edge OpenEdges::getSmallest()
    {
        if (openEdges.size() > 0)
            return openEdges[0];
        return Edge(Point(-1, -1), Point(-1, -1));
    }

    /**
     * @brief Finds an element in the tree given the points and the edge
     * 
     * @param p1 First input point
     * @param p2 Second input point
     * @param edge Edge that contains the points
     * @return int 
     */
    int OpenEdges::getIndex(Point p1, Point p2, Edge edge)
    {
        int low = 0;
        int high = openEdges.size();
        int mid = -1;
        while (low < high)
        {
            mid = ((low + high) / 2);
            if (lessThan(p1, p2, edge, getEdge(mid)))
                high = mid;
            else
                low = mid + 1;
        }
        return low;
    }

    /**
     * @brief Deletes from the tree a given an edge
     * 
     * @param p1 First edge point
     * @param p2 Second edge point
     * @param edge Edge that contains the two points
     */
    void OpenEdges::deleteEdge(Point p1, Point p2, Edge edge)
    {
        int index = getIndex(p1, p2, edge) - 1;
        if(p1 == Point(7,5,2) && p2 == Point(2,1,0)){
            Edge e = getEdge(index);
        }
        if (openEdges.size() > index && getEdge(index) == edge)
            openEdges.erase(openEdges.begin() + index);
    }

    /**
     * @brief Inserts a new edge in the tree
     * 
     * @param p1 First edge point
     * @param p2 Second edge point
     * @param edge Edge that contains the two points
     */
    void OpenEdges::insertEdge(Point p1, Point p2, Edge edge)
    {
        openEdges.insert(openEdges.begin() + getIndex(p1, p2, edge), edge);
    }

}