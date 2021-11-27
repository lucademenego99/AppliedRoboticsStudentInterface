#ifndef OPENEDGES
#define OPENEDGES

#include <vector>
#include "graph.hpp"
#include "visgraph.hpp"

namespace student
{
    class OpenEdges {
        private:
            int getIndex(Point p1, Point p2, Edge edge);

            bool lessThan(Point p1, Point p2, Edge edge1, Edge edge2);

        public:
            std::vector<Edge> openEdges;
            VisGraph visGraph;
            
            OpenEdges() {
                openEdges.clear();
                visGraph = VisGraph();
            }

            void insertEdge(Point p1, Point p2, Edge edge);

            void deleteEdge(Point p1, Point p2, Edge edge);

            Edge getSmallest();

            Edge getEdge(int index);
    };
}


#endif