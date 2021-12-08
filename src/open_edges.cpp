#include "open_edges.hpp"

#include "graph.hpp"
#include "visgraph.hpp"

namespace visgraph
{

    OpenEdges::OpenEdges()
    {
        openEdges.clear();
    }

    bool OpenEdges::lessThan(Point p1, Point p2, Edge edge1, Edge edge2)
    {
        if (edge1 == edge2)
            return false;
        if (!visGraph.edgeIntersect(p1, p2, edge2))
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

    Edge OpenEdges::getEdge(int index)
    {
        if (openEdges.size() > index)
            return openEdges[index];
        return Edge(Point(-1, -1), Point(-1, -1));
    }

    Edge OpenEdges::getSmallest()
    {
        if (openEdges.size() > 0)
            return openEdges[0];
        return Edge(Point(-1, -1), Point(-1, -1));
    }

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

    void OpenEdges::deleteEdge(Point p1, Point p2, Edge edge)
    {
        int index = getIndex(p1, p2, edge) - 1;
        if (openEdges.size() > index && getEdge(index) == edge)
            openEdges.erase(openEdges.begin() + index);
    }

    void OpenEdges::insertEdge(Point p1, Point p2, Edge edge)
    {
        openEdges.insert(openEdges.begin() + getIndex(p1, p2, edge), edge);
    }

}