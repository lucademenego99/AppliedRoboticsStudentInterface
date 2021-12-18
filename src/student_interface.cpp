#include "student_planning_interface.hpp"
#include "graph.hpp"
#include "dubins.hpp"
#include "clipper_addons.hpp"
#include "visgraph.hpp"
#include "open_edges.hpp"
#include "graphPrint.hpp"
#include "utils.hpp"
#include "clipper.hpp"

#include <stdexcept>
#include <sstream>

namespace student
{
    bool planPath(const Polygon &borders, const std::vector<Polygon> &obstacle_list, const std::vector<std::pair<int, Polygon>> &victim_list, const Polygon &gate, const float x, const float y, const float theta, Path &path)
    {
        throw std::logic_error("STUDENT FUNCTION - PLAN PATH - NOT IMPLEMENTED");
    }
    /**
     * @brief NAME STILL TO BE DEFINED, generates an original graph and a visibility graph starting from a vector of polygons
     * 
     * @param polygons Obstables present in the map
     * @param visg Visibility graph we need to obtain
     * @param g Graph we need to generate starting from the obstacles
     */
    void unnamedFunction(std::vector<std::vector<visgraph::Point>> polygons, visgraph::VisGraph visg, visgraph::Graph g){

        //We might want to change these or include them in the parameters
        visgraph::Point origin = visgraph::Point(9, 2); 
        visgraph::Point destination = visgraph::Point(2, 5.7);

        visgraph::Graph originalGraph = visgraph::Graph(polygons, false, true);

        //visg = visgraph::VisGraph();
        
        //g = visg.computeVisibilityGraph(polygons, origin, destination);
    }
}
