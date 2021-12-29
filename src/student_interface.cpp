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
   * @brief Check if a certain points (x,y) is inside the arena
   * Points are in the order bottom-left, bottom-right, top-right, top-left
   * 
   * @param borderPoints Points representing the arena, in order bottom-left, bottom-right, top-right, top-left
   * @param xPoint X of the point we are considering
   * @param yPoint Y of the point we are considering
   * @return true If the point lies inside the arena
   * @return false If the point does not lie inside the arena
   */
  bool isInsideArena(std::vector<visgraph::Point> borderPoints, double xPoint, double yPoint) {
    return (xPoint >= borderPoints[0].x && xPoint <= borderPoints[1].x && yPoint >= borderPoints[0].y && yPoint <= borderPoints[2].y);
  }

  /**
   * @brief Given the coordinates of the destination square, find a point that lies within the arena
   * 
   * @param borderPoints Points representing the arena, in order bottom-left, bottom-right, top-right, top-left
   * @param minX Minimum x of the destination square
   * @param maxX Maximum x of the destination square
   * @param minY Minimum y of the destination square
   * @param maxY Maximum y of the destination square
   * @return visgraph::Point A point that lies within the arena and that is on an edge of the destination square
   */
  visgraph::Point findValidDestinationPoint(std::vector<visgraph::Point> borderPoints, double minX, double maxX, double minY, double maxY) {
    if (isInsideArena(borderPoints, (minX+maxX)/2.0, minY))
      return visgraph::Point((minX+maxX)/2.0, minY);
    if (isInsideArena(borderPoints, (minX+maxX)/2.0, maxY))
      return visgraph::Point((minX+maxX)/2.0, maxY);
    if (isInsideArena(borderPoints, maxX, (minY+maxY)/2.0))
      return visgraph::Point(maxX, (minY+maxY)/2.0);
    if (isInsideArena(borderPoints, minX, (minY+maxY)/2.0))
      return visgraph::Point(minX, (minY+maxY)/2.0);
    
    return visgraph::Point(-1,-1);
  }

  /**
   * @brief fulfill the path for the robots
   * 
   * @param curves result of the multi-point shortest path
   * @param path pointer to the structure which will be returned in the function planPath
   * @param pathSize number of segments of the segmented path
   * @param size integrate distance for seperated dubin path
   * @param robotId ID for the robot to be controlled
   */
  void fillPath(dubins::DubinsCurve **curves, std::vector<Path>& path, int pathSize, double size, int robotId) {
    for(int n = 0; n < pathSize; n++) {
      dubins::DubinsCurve *result = curves[n];
      int npts = result->a1->L/size;
      double s = 0;
      dubins::DubinsLine *tmp;

      for (int i = 0; i < npts; i++) {
        s = result->a1->L/npts * i;
        tmp = new dubins::DubinsLine(s, result->a1->x0, result->a1->y0, result->a1->th0, result->a1->k);
        path[robotId].points.emplace_back(s, tmp->x, tmp->y, tmp->th, result->a1->k);
      }

      if(result->a1->L - result->a1->L/npts*npts > 0.0) {
        s = result->a1->L/npts * npts;
        tmp = new dubins::DubinsLine(s, result->a1->x0, result->a1->y0, result->a1->th0, result->a1->k);
        path[robotId].points.emplace_back(s, tmp->x, tmp->y, tmp->th, result->a1->k);
      }

      npts = result->a2->L/size;
      for (int i = 0; i < npts; i++) {
        s = result->a2->L/npts * i;
        tmp = new dubins::DubinsLine(s, result->a2->x0, result->a2->y0, result->a2->th0, result->a2->k);
        path[robotId].points.emplace_back(s, tmp->x, tmp->y, tmp->th, result->a2->k);
      }

      if(result->a2->L - result->a2->L/npts*npts > 0.0) {
        s = result->a2->L/npts * npts;
        tmp = new dubins::DubinsLine(s, result->a2->x0, result->a2->y0, result->a2->th0, result->a2->k);
        path[robotId].points.emplace_back(s, tmp->x, tmp->y, tmp->th, result->a2->k);
      }

      npts = result->a3->L/size;
      for (int i = 0; i < npts; i++) {
        s = result->a3->L/npts * i;
        tmp = new dubins::DubinsLine(s, result->a3->x0, result->a3->y0, result->a3->th0, result->a3->k);
        path[robotId].points.emplace_back(s, tmp->x, tmp->y, tmp->th, result->a3->k);
      }

      if(result->a3->L - result->a3->L/npts*npts > 0.0) {
        s = result->a3->L/npts * npts;
        tmp = new dubins::DubinsLine(s, result->a3->x0, result->a3->y0, result->a3->th0, result->a3->k);
        path[robotId].points.emplace_back(s, tmp->x, tmp->y, tmp->th, result->a3->k);
      }
    }
  }

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
    double max_k = 8, size = 0.0001;
    // TODO: find the correct offset based on the robot's size for polygon offsetting
    double offset = 0.076, variant = 20.0;

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

    // DEBUG - create the original graph
    visgraph::Graph originalGraphFirst = visgraph::Graph(polygons, false, true);

    // POLYGON OFFSETTING AND JOIN
    std::vector<std::vector<std::vector<visgraph::Point>>> pols = enlargeAndJoinObstacles(polygons, offset);
    polygonsForVisgraph = pols[0];
    polygons = pols[1];

    // Add borders for collision detection
    std::vector<visgraph::Point> borderPoints;
    pol.clear();
    pol.push_back(visgraph::Point(borderMinX+(offset + (offset / variant)), borderMinY+(offset + (offset / variant))));
    pol.push_back(visgraph::Point(borderMaxX-(offset + (offset / variant)), borderMinY+(offset + (offset / variant))));
    borderPoints.push_back(visgraph::Point(borderMinX+(offset + (offset / variant)), borderMinY+(offset + (offset / variant))));
    polygons.push_back(pol);
    pol.clear();
    pol.push_back(visgraph::Point(borderMaxX-(offset + (offset / variant)), borderMinY+(offset + (offset / variant))));
    pol.push_back(visgraph::Point(borderMaxX-(offset + (offset / variant)), borderMaxY-(offset + (offset / variant))));
    borderPoints.push_back(visgraph::Point(borderMaxX-(offset + (offset / variant)), borderMinY+(offset + (offset / variant))));
    polygons.push_back(pol);
    pol.clear();
    pol.push_back(visgraph::Point(borderMaxX-(offset + (offset / variant)), borderMaxY-(offset + (offset / variant))));
    pol.push_back(visgraph::Point(borderMinX+(offset + (offset / variant)), borderMaxY-(offset + (offset / variant))));
    borderPoints.push_back(visgraph::Point(borderMaxX-(offset + (offset / variant)), borderMaxY-(offset + (offset / variant))));
    polygons.push_back(pol);
    pol.clear();
    pol.push_back(visgraph::Point(borderMinX+(offset + (offset / variant)), borderMaxY-(offset + (offset / variant))));
    pol.push_back(visgraph::Point(borderMinX+(offset + (offset / variant)), borderMinY+(offset + (offset / variant))));
    borderPoints.push_back(visgraph::Point(borderMinX+(offset + (offset / variant)), borderMaxY-(offset + (offset / variant))));
    polygons.push_back(pol);

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


    // ************************ DEBUG - Test a simple shortest path ********************************
    // visgraph::Point destination = visgraph::Point(borderMinX + 0.2, borderMaxY - 0.2);
    // dubins::Dubins dubins = dubins::Dubins(max_k, size);
    // dubins::DubinsCurve *curve = dubins.findShortestPath(origin.x, origin.y, theta[0], destination.x, destination.y, M_PI);
    // int npts = curve->a1->L/size;
    // for (int i = 0; i < npts; i++) {
    //   double s = curve->a1->L/npts * i;
    //   dubins::DubinsLine *tmp = new dubins::DubinsLine(s, curve->a1->x0, curve->a1->y0, curve->a1->th0, curve->a1->k);
    //   path[0].points.emplace_back(s, tmp->x, tmp->y, tmp->th, curve->a1->k);
    //   delete tmp;
    // }
    // npts = curve->a2->L/size;
    // for (int i = 0; i < npts; i++) {
    //   double s = curve->a2->L/npts * i;
    //   dubins::DubinsLine *tmp = new dubins::DubinsLine(s, curve->a2->x0, curve->a2->y0, curve->a2->th0, curve->a2->k);
    //   path[0].points.emplace_back(s, tmp->x, tmp->y, tmp->th, curve->a2->k);
    //   delete tmp;
    // }
    // npts = curve->a3->L/size;
    // for (int i = 0; i < npts; i++) {
    //   double s = curve->a3->L/npts * i;
    //   dubins::DubinsLine *tmp = new dubins::DubinsLine(s, curve->a3->x0, curve->a3->y0, curve->a3->th0, curve->a3->k);
    //   path[0].points.emplace_back(s, tmp->x, tmp->y, tmp->th, curve->a3->k);
    //   delete tmp;
    // }


    // ************************ SOLUTION - acutal multipoint shortest path **************************
    visgraph::Point destination = findValidDestinationPoint(borderPoints, minX, maxX, minY, maxY);
    if (destination == visgraph::Point(-1, -1)) {
      std::cout << "UNABLE TO DETERMINE A VALID DESTINATION POINT\n";
      return false;
    }

    // COMPUTE VISIBILITY GRAPH
    visgraph::VisGraph visg;
    visgraph::Graph originalGraph = visgraph::Graph(polygons, false, true);
    visgraph::Graph g = visg.computeVisibilityGraph(polygonsForVisgraph, origin, destination);

    // COMPUTE SHORTEST PATH
    std::vector<visgraph::Point> shortestPath = g.shortestPath(origin, destination);

    // DEBUG
    // printGraph(originalGraphFirst.graph, origin, destination, shortestPath);
    // printGraph(originalGraph.graph, origin, destination, shortestPath);
    // printGraph(g.graph, origin, destination, shortestPath);

    // COMPUTE MULTIPOINT DUBINS SHORTEST PATH
    dubins::Dubins dubins = dubins::Dubins(max_k, size);
    dubins::DubinsPoint **points = new dubins::DubinsPoint *[shortestPath.size()];
    points[0] = new dubins::DubinsPoint(shortestPath[0].x, shortestPath[0].y, theta[0]);
    for(int i = 1; i < shortestPath.size(); i++) {
        points[i] = new dubins::DubinsPoint(shortestPath[i].x, shortestPath[i].y);
    }
    dubins::DubinsCurve **curves = dubins.multipointShortestPath(points, shortestPath.size(), originalGraph);
    if (curves == nullptr) {
        std::cout << "UNABLE TO COMPUTE A PATH FOR GIVEN INPUT\n";
        // return false;
    } else {
        std::cout << "COMPLETED MULTIPOINT SHORTEST PATH SUCCESSFULLY\n";
        // DEBUG
        // dubins.printCompletePath(curves, shortestPath.size()-1, polygons);
        fillPath(curves, path, shortestPath.size()-1, size, 0);
    }
    
    return true;
  }

}