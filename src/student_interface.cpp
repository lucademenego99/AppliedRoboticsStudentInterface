#include "student_planning_interface.hpp"
#include "student_planning_utils.hpp"
#include "graph.hpp"
#include "dubins.hpp"
#include "visgraph.hpp"
#include "open_edges.hpp"
#include "graphPrint.hpp"
#include "clipper_addons.hpp"

#include <stdexcept>
#include <sstream>
#include <iostream>

namespace student
{
  /**
   * @brief Plan a safe and fast path in the arena
   * 
   * @param borders Border of the arena, expressed in meters
   * @param obstacle_list List of obstacle polygons
   * @param gate_list List of possible gates, expressed as polygons
   * @param x x positions of the robots in the arena
   * @param y y positions of the robots in the arena
   * @param theta Yaw of the robot in the arena reference system
   * @param path List of poses representing the final path of the robot, passed by ref.
   * @param config_folder A custom string from config file
   * @return true We have a valid path available
   * @return false We don't have a valid path available
   */
  bool planPath(const Polygon& borders, const std::vector<Polygon>& obstacle_list, const std::vector<Polygon>& gate_list, const std::vector<float> x, const std::vector<float> y, const std::vector<float> theta, std::vector<Path>& path, const std::string& config_folder){
    std::vector<visgraph::Point> pol;
    std::vector<std::vector<visgraph::Point>> polygons, polygonsForVisgraph;

    // TODO: find the correct bound k and the inter size of the dubins
    double max_k = 10, size = 0.005;
    // TODO: find the correct offset based on the robot's size for polygon offsetting
    double offset = 0.065, variant = 10.0;

    // Find the border's points
    double borderMaxX = -INFINITY, borderMinX = INFINITY;
    double borderMaxY = -INFINITY, borderMinY = INFINITY;
    for(int i = 0; i < borders.size(); i++) {
      if (borders[i].x > borderMaxX) {
        borderMaxX = borders[i].x;
      }
      if (borders[i].x < borderMinX) {
        borderMinX = borders[i].x;
      }
      if (borders[i].y > borderMaxY) {
        borderMaxY = borders[i].y;
      }
      if (borders[i].y < borderMinY) {
        borderMinY = borders[i].y;
      }
    }

    // Add obstacles
    for(int i = 0; i<obstacle_list.size(); i++) {
      pol.clear();
      for(int j = 0; j<obstacle_list[i].size(); j++) {
        visgraph::Point trans_point = visgraph::Point(obstacle_list[i][j].x, obstacle_list[i][j].y, i);
        pol.push_back(trans_point);
      }
      polygons.push_back(pol);
    }

    // Set origin
    visgraph::Point origin = visgraph::Point(x[0], y[0]);

    // Find the destination
    double maxX = -INFINITY, minX = INFINITY;
    double maxY = -INFINITY, minY = INFINITY;
    for(int i = 0; i < gate_list[0].size(); i++) {
      if (gate_list[0][i].x > maxX) {
        maxX = gate_list[0][i].x;
      }
      if (gate_list[0][i].x < minX) {
        minX = gate_list[0][i].x;
      }
      if (gate_list[0][i].y > maxY) {
        maxY = gate_list[0][i].y;
      }
      if (gate_list[0][i].y < minY) {
        minY = gate_list[0][i].y;
      }
    }
    /****************IMPORTANT: remember to remove the 0.1 if it works on your machine!!!*************/
    visgraph::Point destination = visgraph::Point((maxX + minX) / 2.0, (maxY + minY) / 2.0 - 0.1); 

    // DEBUG - create the original graph
    visgraph::Graph originalGraphFirst = visgraph::Graph(polygons, false, true);

    // POLYGON OFFSETTING AND JOIN
    std::vector<std::vector<std::vector<visgraph::Point>>> pols = enlargeAndJoinObstacles(polygons, offset);
    polygonsForVisgraph = pols[0];
    polygons = pols[1];

    // Add borders for collision detection
    pol.clear();
    pol.push_back(visgraph::Point(borderMinX+(offset + (offset / variant)), borderMinY+(offset + (offset / variant))));
    pol.push_back(visgraph::Point(borderMaxX-(offset + (offset / variant)), borderMinY+(offset + (offset / variant))));
    polygons.push_back(pol);
    pol.clear();
    pol.push_back(visgraph::Point(borderMaxX-(offset + (offset / variant)), borderMinY+(offset + (offset / variant))));
    pol.push_back(visgraph::Point(borderMaxX-(offset + (offset / variant)), borderMaxY-(offset + (offset / variant))));
    polygons.push_back(pol);
    pol.clear();
    pol.push_back(visgraph::Point(borderMaxX-(offset + (offset / variant)), borderMaxY-(offset + (offset / variant))));
    pol.push_back(visgraph::Point(borderMinX+(offset + (offset / variant)), borderMaxY-(offset + (offset / variant))));
    polygons.push_back(pol);
    pol.clear();
    pol.push_back(visgraph::Point(borderMinX+(offset + (offset / variant)), borderMaxY-(offset + (offset / variant))));
    pol.push_back(visgraph::Point(borderMinX+(offset + (offset / variant)), borderMinY+(offset + (offset / variant))));
    polygons.push_back(pol);


    // COMPUTE VISIBILITY GRAPH
    visgraph::VisGraph visg;
    visgraph::Graph originalGraph = visgraph::Graph(polygons, false, true);
    visgraph::Graph g = visg.computeVisibilityGraph(polygonsForVisgraph, origin, destination);

    // COMPUTE SHORTEST PATH
    std::vector<visgraph::Point> shortestPath = g.shortestPath(origin, destination);

    printGraph(originalGraphFirst.graph, origin, destination, shortestPath);
    printGraph(originalGraph.graph, origin, destination, shortestPath);
    printGraph(g.graph, origin, destination, shortestPath);

    // COMPUTE MULTIPOINT DUBINS SHORTEST PATH
    dubins::Dubins dubins = dubins::Dubins(max_k, size);
    dubins::DubinsPoint **points = new dubins::DubinsPoint *[shortestPath.size()];
    points[0] = new dubins::DubinsPoint(shortestPath[0].x, shortestPath[0].y, theta[0]);
    for(int i = 1; i < shortestPath.size()-1; i++) {
        points[i] = new dubins::DubinsPoint(shortestPath[i].x, shortestPath[i].y);
    }
    points[shortestPath.size()-1] = new dubins::DubinsPoint(shortestPath[shortestPath.size()-1].x, shortestPath[shortestPath.size()-1].y);

    dubins::DubinsCurve **curves = dubins.multipointShortestPath(points, shortestPath.size(), originalGraph);
    if (curves == nullptr) {
        std::cout << "UNABLE TO COMPUTE A PATH FOR GIVEN INPUT\n";
    } else {
        std::cout << "COMPLETED MULTIPOINT SHORTEST PATH SUCCESSFULLY\n";
        dubins.printCompletePath(curves, shortestPath.size()-1, polygons);
    }

    for(int n = 0; n < shortestPath.size()-1; n++) {
      dubins::DubinsCurve *result = curves[n];
      int npts = result->a1->L/size;

      for (int i = 0; i < npts; i++) {
        double s = result->a1->L/npts * i;
        dubins::DubinsLine *tmp = new dubins::DubinsLine(s, result->a1->x0, result->a1->y0, result->a1->th0, result->a1->k);
        path[0].points.emplace_back(s, tmp->x, tmp->y, tmp->th, result->a1->k);
        delete tmp;
      }

      npts = result->a2->L/size;
      for (int i = 0; i < npts; i++) {
        double s = result->a2->L/npts * i;
        dubins::DubinsLine *tmp = new dubins::DubinsLine(s, result->a2->x0, result->a2->y0, result->a2->th0, result->a2->k);
        path[0].points.emplace_back(s, tmp->x, tmp->y, tmp->th, result->a2->k);
        delete tmp;
      }

      npts = result->a3->L/size;
      for (int i = 0; i < npts; i++) {
        double s = result->a3->L/npts * i;
        dubins::DubinsLine *tmp = new dubins::DubinsLine(s, result->a3->x0, result->a3->y0, result->a3->th0, result->a3->k);
        path[0].points.emplace_back(s, tmp->x, tmp->y, tmp->th, result->a3->k);
        delete tmp;
      }
    }
  }
}