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
   * @brief Find out the number of robots based on the values of their position
   * 
   * @param x x positions of the robots
   * @param y y positions of the robots
   * @return int number of robots
   */
  int checkNumberOfRobots(std::vector<float> x, std::vector<float> y) {
    int size = 0;
    if (x[0] != 0 && y[0] != 0) {
      size++;
      if (x[1] != 0 && y[1] != 0) {
        size++;
        if (x[2] != 0 && y[2] != 0) {
          size++;
        }
      }
    }
    return size;
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
   * @brief Find the 4 border points
   * 
   * @param borders Polygon representing the border
   * @param borderMaxX Max x point - passed by ref.
   * @param borderMinX Min x point - passed by ref.
   * @param borderMaxY Max y point - passed by ref.
   * @param borderMinY Min y point - passed by ref.
   */
  void findBorderPoints(const Polygon &borders, double &borderMaxX, double &borderMinX, double &borderMaxY, double &borderMinY) {
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
  }

  /**
   * @brief Find the 4 destination points given a destination polygon
   * 
   * @param gate Polygon representing our gate
   * @param maxX Max x point - passed by ref.
   * @param minX Min x point - passed by ref.
   * @param maxY Max y point - passed by ref.
   * @param minY Min y point - passed by ref.
   */
  void findDestinationPoints(const Polygon &gate, double &maxX, double &minX, double &maxY, double &minY) {
    for(int i = 0; i < gate.size(); i++) {
      if (gate[i].x > maxX) {
        maxX = gate[i].x;
      }
      if (gate[i].x < minX) {
        minX = gate[i].x;
      }
      if (gate[i].y > maxY) {
        maxY = gate[i].y;
      }
      if (gate[i].y < minY) {
        minY = gate[i].y;
      }
    }
  }

  /**
   * @brief Convert the obstacles into a structure our algorithms comprehend
   * 
   * @param obstacle_list List of the obstacles as vector of polygons
   * @return std::vector<std::vector<visgraph::Point>> List of obstacles represented as vectors of visgraph::Point
   */
  std::vector<std::vector<visgraph::Point>> getObstacles(const std::vector<Polygon>& obstacle_list) {
    std::vector<visgraph::Point> pol;
    std::vector<std::vector<visgraph::Point>> polygons;
    for(int i = 0; i<obstacle_list.size(); i++) {
      pol.clear();
      for(int j = 0; j<obstacle_list[i].size(); j++) {
        visgraph::Point trans_point = visgraph::Point(obstacle_list[i][j].x, obstacle_list[i][j].y, i);
        pol.push_back(trans_point);
      }
      polygons.push_back(pol);
    }
    return polygons;
  }

  /**
   * @brief Add borders increased in size by a certain factor to the polygons vector
   * 
   * @param borderMaxX Max x point
   * @param borderMinX Min x point
   * @param borderMaxY Max y point
   * @param borderMinY Min y point
   * @param offset Offset by which we want to increase the size of the borders
   * @param variant Another value used to determine how much we want to increase the size of the borders
   * @param borderPoints Vector keeping the 4 border points - passed by ref.
   * @param polygons Add the borders to this vector, then they will be used for collision detection - passed by ref.
   */
  void addBorders(double borderMaxX, double borderMinX, double borderMaxY, double borderMinY, double offset, double variant, std::vector<visgraph::Point> &borderPoints, std::vector<std::vector<visgraph::Point>> &polygons) {
    std::vector<visgraph::Point> pol;

    // Bottom Edge
    pol.push_back(visgraph::Point(borderMinX+(offset + (offset / variant)), borderMinY+(offset + (offset / variant))));
    pol.push_back(visgraph::Point(borderMaxX-(offset + (offset / variant)), borderMinY+(offset + (offset / variant))));
    borderPoints.push_back(visgraph::Point(borderMinX+(offset + (offset / variant)), borderMinY+(offset + (offset / variant))));
    polygons.push_back(pol);

    // Right Edge
    pol.clear();
    pol.push_back(visgraph::Point(borderMaxX-(offset + (offset / variant)), borderMinY+(offset + (offset / variant))));
    pol.push_back(visgraph::Point(borderMaxX-(offset + (offset / variant)), borderMaxY-(offset + (offset / variant))));
    borderPoints.push_back(visgraph::Point(borderMaxX-(offset + (offset / variant)), borderMinY+(offset + (offset / variant))));
    polygons.push_back(pol);

    // Top Edge
    pol.clear();
    pol.push_back(visgraph::Point(borderMaxX-(offset + (offset / variant)), borderMaxY-(offset + (offset / variant))));
    pol.push_back(visgraph::Point(borderMinX+(offset + (offset / variant)), borderMaxY-(offset + (offset / variant))));
    borderPoints.push_back(visgraph::Point(borderMaxX-(offset + (offset / variant)), borderMaxY-(offset + (offset / variant))));
    polygons.push_back(pol);

    // Left Edge
    pol.clear();
    pol.push_back(visgraph::Point(borderMinX+(offset + (offset / variant)), borderMaxY-(offset + (offset / variant))));
    pol.push_back(visgraph::Point(borderMinX+(offset + (offset / variant)), borderMinY+(offset + (offset / variant))));
    borderPoints.push_back(visgraph::Point(borderMinX+(offset + (offset / variant)), borderMaxY-(offset + (offset / variant))));
    polygons.push_back(pol);
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
        delete tmp;
      }

      if(result->a1->L - result->a1->L/npts*npts > 0.0) {
        s = result->a1->L/npts * npts;
        tmp = new dubins::DubinsLine(s, result->a1->x0, result->a1->y0, result->a1->th0, result->a1->k);
        path[robotId].points.emplace_back(s, tmp->x, tmp->y, tmp->th, result->a1->k);
        delete tmp;
      }

      npts = result->a2->L/size;
      for (int i = 0; i < npts; i++) {
        s = result->a2->L/npts * i;
        tmp = new dubins::DubinsLine(s, result->a2->x0, result->a2->y0, result->a2->th0, result->a2->k);
        path[robotId].points.emplace_back(s, tmp->x, tmp->y, tmp->th, result->a2->k);
        delete tmp;
      }

      if(result->a2->L - result->a2->L/npts*npts > 0.0) {
        s = result->a2->L/npts * npts;
        tmp = new dubins::DubinsLine(s, result->a2->x0, result->a2->y0, result->a2->th0, result->a2->k);
        path[robotId].points.emplace_back(s, tmp->x, tmp->y, tmp->th, result->a2->k);
        delete tmp;
      }

      npts = result->a3->L/size;
      for (int i = 0; i < npts; i++) {
        s = result->a3->L/npts * i;
        tmp = new dubins::DubinsLine(s, result->a3->x0, result->a3->y0, result->a3->th0, result->a3->k);
        path[robotId].points.emplace_back(s, tmp->x, tmp->y, tmp->th, result->a3->k);
        delete tmp;
      }

      if(result->a3->L - result->a3->L/npts*npts > 0.0) {
        s = result->a3->L/npts * npts;
        tmp = new dubins::DubinsLine(s, result->a3->x0, result->a3->y0, result->a3->th0, result->a3->k);
        path[robotId].points.emplace_back(s, tmp->x, tmp->y, tmp->th, result->a3->k);
        delete tmp;
      }
    }
  }

  /**
   * @brief Reach a certain destination points with a certain robot
   * 
   * @param robot Robot ID
   * @param origin Origin point
   * @param destination Destination Point
   * @param theta Robot's starting angle
   * @param originalGraph Obstacles for collision detection
   * @param g Obstacles for the shortestPath
   * @param path Path to fill - passed by ref.
   * @param max_k Curvature of the robot
   * @param size Discritizer size for the path generation
   * @return true If a path has been found
   * @return false If a path hasn't been found
   */
  bool reachDestinationForRobot(int robot, visgraph::Point origin, std::vector<visgraph::Point> destinations, double theta, visgraph::Graph originalGraph, visgraph::Graph g, std::vector<Path> &path, double max_k, double size) {
    // ********** COMPUTE SHORTEST PATH FROM ORIGIN TO DESTINATION ********** //
    std::vector<visgraph::Point> shortestPath = g.shortestPathMultipleD(origin, destinations);

    // DEBUG - print graphs using opencv
    // printGraph(originalGraph.graph, origin, destination, shortestPath);
    // printGraph(g.graph, origin, destination, shortestPath);


    // ********** COMPUTE MULTIPOINT DUBINS PATH ********** //
    // Create an input valid for multipointShortestPath - we have a source,
    // a list of intermediate points and a destination.
    // The only point in which we specify an angle is the initial one.
    dubins::Dubins dubins = dubins::Dubins(max_k, size);
    dubins::DubinsPoint **points = new dubins::DubinsPoint *[shortestPath.size()];
    points[0] = new dubins::DubinsPoint(shortestPath[0].x, shortestPath[0].y, theta);
    for(int i = 1; i < shortestPath.size(); i++) {
        points[i] = new dubins::DubinsPoint(shortestPath[i].x, shortestPath[i].y);
    }
    // Find the dubins shortest path given the set of intermediate points
    dubins::DubinsCurve **curves = dubins.multipointShortestPath(points, shortestPath.size(), originalGraph);
    if (curves == nullptr) {
        std::cout << "UNABLE TO COMPUTE A PATH FOR GIVEN INPUT\n";
        return false;
    } else {
        std::cout << "COMPLETED MULTIPOINT SHORTEST PATH SUCCESSFULLY\n";
        // DEBUG - print the final complete path using opencv
        // dubins.printCompletePath(curves, shortestPath.size()-1, polygons);


        // ********** CREATE THE PATH FOR THE ROBOT ********** //
        fillPath(curves, path, shortestPath.size()-1, size, robot);
        return true;
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

    std::cout << "----- STUDENT PLAN PATH -----\n";

    // ********** DEFINE USEFUL VARIABLES ********** //
    // Correct bound k and discritizer size
    double max_k = 30;
    // Correct discritizer size
    double size = 0.0001;
    // Offset based on the robot's size for polygon offsetting
    double offset = 0.076;
    // Variant value to increase the size of the outside borders
    double variant = 20.0;


    // ********** FIND THE BORDER'S POINTS ********** //
    double borderMaxX = -INFINITY, borderMinX = INFINITY;
    double borderMaxY = -INFINITY, borderMinY = INFINITY;
    findBorderPoints(borders, borderMaxX, borderMinX, borderMaxY, borderMinY);


    // ********** CONVERT OBSTACLES INTO OUR STRUCTURE ********** //
    std::vector<std::vector<visgraph::Point>> polygons = getObstacles(obstacle_list);

    // DEBUG - create the original graph
    // visgraph::Graph originalGraphFirst = visgraph::Graph(polygons, false, true);


    // ********** POLYGON OFFSETING AND JOIN ********** //
    // We apply the polygon offsetting algorithm because we are treating our robot as a simple point
    // So we need to increase the size of all the obstacles based on the actual robot's size, or better,
    // in our case based on the radius of the circle circumscribed to the robot
    // We perform the join operation because after the polygon offsetting some obstacles may intersect between each other
    // in those cases, we create one bigger obstacle comprehending all the intersecting ones.
    std::vector<std::vector<std::vector<visgraph::Point>>> pols = enlargeAndJoinObstacles(polygons, offset);
  
    // The polygons for the visibility graph are a little bit bigger, in order to be sure the robot
    // will not bump into them and in order to make the collision detection part works well
    std::vector<std::vector<visgraph::Point>> polygonsForVisgraph = pols[0];

    // These are the polygons that will be used for the collision detection part. Their size is a little
    // bit smaller than the ones used for the roadmap generation, but their size is still increased based on the robot's size
    polygons = pols[1];


    // ********** ADD BORDERS FOR COLLISION DETECTION ********** //
    // Even the border must be adapted since we treat our robot as a point. We make them smaller by the same amount "offset",
    // times a very small value to be sure our robot doesn't bump into them. We add them at the end of our obstacles
    // so that they will be used for collision detection too

    // I also keep a vector of the 4 border points, that will be useful later to find a valid destination point
    std::vector<visgraph::Point> borderPoints;
    addBorders(borderMaxX, borderMinX, borderMaxY, borderMinY, offset, variant, borderPoints, polygons);
    

    // ********** GET ALL ORIGINS AND DESTINATIONS ********** //
    int numberOfRobots = checkNumberOfRobots(x,y);
    int numberOfDestinations = gate_list.size();
    std::vector<visgraph::Point> origins, destinations;
    for(int i = 0; i < numberOfRobots; i++) {
      origins.push_back(visgraph::Point(x[i], y[i]));
    }
    for(int i = 0; i < numberOfDestinations; i++) {
      // ********** FIND THE DESTINATION POINTS ********** //
      double maxX = -INFINITY, minX = INFINITY;
      double maxY = -INFINITY, minY = INFINITY;
      findDestinationPoints(gate_list[i], maxX, minX, maxY, minY);


      // ********** FIND A VALID DESTINATION ********** //
      // Since we performed polygon offsetting, some points of the destination may not be available (inside an obstacle,
      // or outside the border).
      visgraph::Point destination = findValidDestinationPoint(borderPoints, minX, maxX, minY, maxY);
      if (destination == visgraph::Point(-1, -1)) {
        std::cout << "UNABLE TO DETERMINE A VALID DESTINATION POINT FOR DESTINATION NUMBER" << i << "\n";
      } else {
        destinations.push_back(destination);
      }
    }

    // DEBUG - if you want to test using other destinations, just add them here with destinations.push_back(your destination)
    

    // ********** COMPUTE THE ROADMAP - SHORTEST PATH VERSION ********** //
    visgraph::VisGraph visg;
    // The original graph comprehend all the obstacles that will be used for collision detection
    visgraph::Graph originalGraph = visgraph::Graph(polygons, false, true);
    // The visibility graph will be used to find the shortest path between a source and a destination
    visgraph::Graph g = visg.computeVisibilityGraphMultipleOD(polygonsForVisgraph, origins, destinations);

    // DEBUG - show graphs using opencv
    // std::vector<visgraph::Point> empty;
    // printGraph(originalGraph.graph, visgraph::Point(0,0), visgraph::Point(0,0), empty);
    // printGraph(g.graph, visgraph::Point(0,0), visgraph::Point(0,0), empty);

    if (numberOfRobots == 1) {
      // ********** WE HAVE JUST ONE ROBOT ********** //
      // *** Try to reach the destination *** //
      std::cout << "THERE IS ONE ONLY ROBOT\nTry to reach the gate\n";


      // ********** SET THE ORIGIN ********** //
      visgraph::Point origin = visgraph::Point(x[0], y[0]);


      // ********** TRY TO REACH THE DESTINATION ********** //
      reachDestinationForRobot(0, origin, destinations, theta[0], originalGraph, g, path, max_k, size);


    } else if (numberOfRobots == 2) {
      // ********** TWO ROBOTS - PROJECT NUMBER 1 - PURSUER EVADER GAME ********** //
      std::cout << "THERE ARE TWO ROBOTS\nPursuer Evader game\n";
      
    } else if (numberOfRobots == 3) {
      // ********** THREE ROBOTS - PROJECT NUMBER 2 - NOT DONE ********** //
      std::cout << "THERE ARE THREE ROBOTS\nProject number 2 not done\n";

    }


    // ************************ DEBUG - Test a simple shortest path ******************************** //
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
    
    return true;
  }

}