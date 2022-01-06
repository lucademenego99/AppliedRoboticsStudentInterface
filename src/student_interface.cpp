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
#include <random>

/**
 * @brief Basic student namespace
 * 
 * This namespace contains all the functions used to plan the path for the ROS environment
 * 
 */
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
   * @param obstaclesGraph graph containing the enlarged obstacles, to find out which part of the destination gate is available
   * @param minX Minimum x of the destination square
   * @param maxX Maximum x of the destination square
   * @param minY Minimum y of the destination square
   * @param maxY Maximum y of the destination square
   * @return visgraph::Point A point that lies within the arena and that is on an edge of the destination square
   */
  visgraph::Point findValidDestinationPoint(std::vector<visgraph::Point> borderPoints, visgraph::Graph obstaclesGraph, double minX, double maxX, double minY, double maxY) {
    visgraph::VisGraph vg;
    if (isInsideArena(borderPoints, (minX+maxX)/2.0, minY) && vg.pointInPolygon(visgraph::Point((minX+maxX)/2.0, minY), obstaclesGraph).empty())
      return visgraph::Point((minX+maxX)/2.0, minY);
    if (isInsideArena(borderPoints, (minX+maxX)/2.0, maxY) && vg.pointInPolygon(visgraph::Point((minX+maxX)/2.0, maxY), obstaclesGraph).empty())
      return visgraph::Point((minX+maxX)/2.0, maxY);
    if (isInsideArena(borderPoints, maxX, (minY+maxY)/2.0) && vg.pointInPolygon(visgraph::Point(maxX, (minY+maxY)/2.0), obstaclesGraph).empty())
      return visgraph::Point(maxX, (minY+maxY)/2.0);
    if (isInsideArena(borderPoints, minX, (minY+maxY)/2.0) && vg.pointInPolygon(visgraph::Point(minX, (minY+maxY)/2.0), obstaclesGraph).empty())
      return visgraph::Point(minX, (minY+maxY)/2.0);
    if (isInsideArena(borderPoints, minX, minY) && vg.pointInPolygon(visgraph::Point(minX, minY), obstaclesGraph).empty())
      return visgraph::Point(minX, minY);
    if (isInsideArena(borderPoints, minX, maxY) && vg.pointInPolygon(visgraph::Point(minX, maxY), obstaclesGraph).empty())
      return visgraph::Point(minX, maxY);
    if (isInsideArena(borderPoints, maxX, minY) && vg.pointInPolygon(visgraph::Point(maxX, minY), obstaclesGraph).empty())
      return visgraph::Point(maxX, minY);
    if (isInsideArena(borderPoints, maxX, maxY) && vg.pointInPolygon(visgraph::Point(maxX, maxY), obstaclesGraph).empty())
      return visgraph::Point(maxX, maxY);
    
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

      for (int i = 0; i < npts; i++) {
        s = result->a1->L/npts * i;
        dubins::DubinsLine *tmp = new dubins::DubinsLine(s, result->a1->x0, result->a1->y0, result->a1->th0, result->a1->k);
        path[robotId].points.emplace_back(s, tmp->x, tmp->y, tmp->th, result->a1->k);
        delete tmp;
      }

      if(result->a1->L - result->a1->L/npts*npts > 0.0) {
        s = result->a1->L/npts * npts;
        dubins::DubinsLine *tmp = new dubins::DubinsLine(s, result->a1->x0, result->a1->y0, result->a1->th0, result->a1->k);
        path[robotId].points.emplace_back(s, tmp->x, tmp->y, tmp->th, result->a1->k);
        delete tmp;
      }

      npts = result->a2->L/size;
      for (int i = 0; i < npts; i++) {
        s = result->a2->L/npts * i;
        dubins::DubinsLine *tmp = new dubins::DubinsLine(s, result->a2->x0, result->a2->y0, result->a2->th0, result->a2->k);
        path[robotId].points.emplace_back(s, tmp->x, tmp->y, tmp->th, result->a2->k);
        delete tmp;
      }

      if(result->a2->L - result->a2->L/npts*npts > 0.0) {
        s = result->a2->L/npts * npts;
        dubins::DubinsLine *tmp = new dubins::DubinsLine(s, result->a2->x0, result->a2->y0, result->a2->th0, result->a2->k);
        path[robotId].points.emplace_back(s, tmp->x, tmp->y, tmp->th, result->a2->k);
        delete tmp;
      }

      npts = result->a3->L/size;
      for (int i = 0; i < npts; i++) {
        s = result->a3->L/npts * i;
        dubins::DubinsLine *tmp = new dubins::DubinsLine(s, result->a3->x0, result->a3->y0, result->a3->th0, result->a3->k);
        path[robotId].points.emplace_back(s, tmp->x, tmp->y, tmp->th, result->a3->k);
        delete tmp;
      }

      if(result->a3->L - result->a3->L/npts*npts > 0.0) {
        s = result->a3->L/npts * npts;
        dubins::DubinsLine *tmp = new dubins::DubinsLine(s, result->a3->x0, result->a3->y0, result->a3->th0, result->a3->k);
        path[robotId].points.emplace_back(s, tmp->x, tmp->y, tmp->th, result->a3->k);
        delete tmp;
      }
    }
  }

  /**
   * @brief Plan to reach a certain destination points with a certain robot
   * 
   * @param robot Robot ID
   * @param origin Origin point
   * @param destination Destination Point
   * @param theta Robot's starting angle
   * @param borderPoints points of the borders of the arena
   * @param originalGraph Obstacles for collision detection
   * @param g Obstacles for the shortestPath
   * @param path Path to fill - passed by ref.
   * @param shortestPath Calculated shortest path
   * @param pathLengths Lengths of the dubins path from source to all intermediate points
   * @param max_k Curvature of the robot
   * @param size Discritizer size for the path generation
   * @return dubins::DubinsCurve** Curves planned
   */
  dubins::DubinsCurve **planDestinationForRobot(int robot, visgraph::Point origin, std::vector<visgraph::Point> destinations, double theta, std::vector<visgraph::Point> borderPoints, visgraph::Graph originalGraph, visgraph::Graph g, std::vector<visgraph::Point> &shortestPath, std::vector<double> &pathLengths, std::vector<Path> &path, double max_k, double size) {
    // ********** COMPUTE SHORTEST PATH FROM ORIGIN TO DESTINATION ********** //
    shortestPath = g.shortestPathMultipleD(origin, destinations, borderPoints);

    // DEBUG - print graphs using opencv
    // printGraph(originalGraph.graph, origin, destinations[0], shortestPath);
    // printGraph(g.graph, origin, destinations[0], shortestPath);


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
        for(int i = 0; i < shortestPath.size(); i++) {
            delete points[i];
        }
        delete[] points;
        return nullptr;
    } else {
        std::cout << "COMPLETED MULTIPOINT SHORTEST PATH SUCCESSFULLY\n";
        // DEBUG - print the final complete path using opencv
        // dubins.printCompletePath(curves, shortestPath.size()-1, polygons);

        // ********** GET THE PATH COMPLETE LENGTH ********** //
        double pathLength = 0;
        pathLengths.clear();
        for (int i = 0; i < shortestPath.size() - 1; i++) {
          pathLength += curves[i]->L;
          pathLengths.push_back(pathLength);
        }

        for(int i = 0; i < shortestPath.size(); i++) {
            delete points[i];
        }
        delete[] points;

        return curves;
    }
  }

  /**
   * @brief Reach a certain destination points with a certain robot
   * 
   * @param robot Robot ID
   * @param origin Origin point
   * @param destination Destination Point
   * @param theta Robot's starting angle
   * @param borderPoints points of the borders of the arena
   * @param originalGraph Obstacles for collision detection
   * @param g Obstacles for the shortestPath
   * @param path Path to fill - passed by ref.
   * @param shortestPath Calculated shortest path
   * @param pathLengths Lengths of the dubins path from source to all intermediate points
   * @param max_k Curvature of the robot
   * @param size Discritizer size for the path generation
   * @return true If a path has been found
   * @return false If a path hasn't been found
   */
  bool reachDestinationForRobot(int robot, visgraph::Point origin, std::vector<visgraph::Point> destinations, double theta, std::vector<visgraph::Point> borderPoints, visgraph::Graph originalGraph, visgraph::Graph g, std::vector<visgraph::Point> &shortestPath, std::vector<double> &pathLengths, std::vector<Path> &path, double max_k, double size) {
    // ********** COMPUTE SHORTEST PATH FROM ORIGIN TO DESTINATION ********** //
    shortestPath = g.shortestPathMultipleD(origin, destinations, borderPoints);

    // DEBUG - print graphs using opencv
    // printGraph(originalGraph.graph, origin, destinations[0], shortestPath);
    // printGraph(g.graph, origin, destinations[0], shortestPath);


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
        for(int i = 0; i < shortestPath.size(); i++) {
            delete points[i];
        }
        delete[] points;
        return false;
    } else {
        std::cout << "COMPLETED MULTIPOINT SHORTEST PATH SUCCESSFULLY\n";
        // DEBUG - print the final complete path using opencv
        // dubins.printCompletePath(curves, shortestPath.size()-1, polygons);

        // ********** GET THE PATH COMPLETE LENGTH ********** //
        double pathLength = 0;
        pathLengths.clear();
        for (int i = 0; i < shortestPath.size() - 1; i++) {
          pathLength += curves[i]->L;
          pathLengths.push_back(pathLength);
        }

        // ********** CREATE THE PATH FOR THE ROBOT ********** //
        fillPath(curves, path, shortestPath.size()-1, size, robot);

        for(int i = 0; i < shortestPath.size(); i++) {
            delete points[i];
            if (i < shortestPath.size()-1) {
              delete curves[i];
            }
        }
        delete[] points;
        delete[] curves;

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
    double max_k = 20;
    // Correct discritizer size
    double size = 0.005;
    // Offset based on the robot's size for polygon offsetting
    double offset = 0.076;
    // Alternative offset
    double altOffset = 0.065;
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
      visgraph::Graph tmpGraph = visgraph::Graph(polygonsForVisgraph, false, true);
      visgraph::Point destination = findValidDestinationPoint(borderPoints, tmpGraph, minX, maxX, minY, maxY);
      if (destination == visgraph::Point(-1, -1)) {
        std::cout << "UNABLE TO DETERMINE A VALID DESTINATION POINT FOR DESTINATION NUMBER" << i << "\n";
      } else {
        destinations.push_back(destination);
      }
    }

    // DEBUG - if you want to test using other destinations, just add them here with destinations.push_back(your destination)
    // destinations.push_back(visgraph::Point(1.3, 0.15));
    // destinations.push_back(visgraph::Point(0.15, 0.8));

    numberOfDestinations = destinations.size();


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

      //Vectors to be filled when trying to reach the destination
      std::vector<visgraph::Point> shortestPath;
      std::vector<double> pathLengths;

      // ********** SET THE ORIGIN ********** //
      visgraph::Point origin = visgraph::Point(x[0], y[0]);


      // ********** TRY TO REACH THE DESTINATION ********** //
      bool foundPath = reachDestinationForRobot(0, origin, destinations, theta[0], borderPoints, originalGraph, g, shortestPath, pathLengths, path, max_k, size);
      if (!foundPath) {
        std::cout << "NO PATH FOUND!\n";
      } else {
        std::cout << "FOUND PATH WITH LENGTH: " << pathLengths[pathLengths.size() - 1] << "\n";
      }


    } else if (numberOfRobots == 2) {
      if(numberOfDestinations == 1) {
        // ********** TWO ROBOTS - PROJECT NUMBER 1 - PURSUER EVADER GAME ********** //
        std::cout << "THERE ARE TWO ROBOTS AND ONE EXIT GATE\nPursuer Evader game\n";

        std::vector<visgraph::Point> shortestPathEvader, shortestPathPursuer;
        std::vector<double> pathLengthsEvader, pathLengthsPursuer;
        
        // ********** EVADER - Try to reach the closer destination ********** //
        bool foundPathFirst = reachDestinationForRobot(0, visgraph::Point(x[0], y[0]), destinations, theta[0], borderPoints, originalGraph, g, shortestPathEvader, pathLengthsEvader, path, max_k, size);
        if (!foundPathFirst) {
          std::cout << "NO PATH FOUND FOR THE EVADER!\n";
          // We compute again all the graps with a smaller offset, higher risk but shorter paths
          pols = enlargeAndJoinObstacles(polygons, altOffset);
          polygonsForVisgraph = pols[0];
          polygons = pols[1];
          //Change the dimension of the borders, as they also need to be smaller
          addBorders(borderMaxX, borderMinX, borderMaxY, borderMinY, altOffset, variant, borderPoints, polygons);
          originalGraph = visgraph::Graph(polygons, false, true);
          g = visg.computeVisibilityGraphMultipleOD(polygonsForVisgraph, origins, destinations);
          bool foundPathFirst = reachDestinationForRobot(0, visgraph::Point(x[0], y[0]), destinations, theta[0], borderPoints, originalGraph, g, shortestPathEvader, pathLengthsEvader, path, max_k, size);
          //We check if the robot was able to reach the destination with a smaller offset, otherwise we just output an error message
          if (!foundPathFirst) {
            std::cout << "NO PATH FOUND EVEN WITH A SMALLER OFFSET FOR THE EVADER!\n";
          }else{
            std::cout << "PATH FOUND WITH A SMALLER OFFSET FOR THE EVADER WITH LENGTH: " << pathLengthsEvader[pathLengthsEvader.size()-1] << "\n";
          }  
        } else {
          std::cout << "PATH FOUND FOR EVADER WITH LENGTH: " << pathLengthsEvader[pathLengthsEvader.size()-1] << "\n";
        }
        
        // DEBUG - Try to reach the closer destination with the pursuer
        // bool foundPathSecond = reachDestinationForRobot(1, visgraph::Point(x[1], y[1]), destinations, theta[1], borderPoints, originalGraph, g, path, max_k, size);
        // if (!foundPathSecond) {
        //   std::cout << "NO PATH FOUND FOR SECOND ROBOT!\n";
        // }


        // ********** PURSUER - Find the path to intercept the evader ********** //
        //Calculate the shortestPathDict both for the first robot position and the second
        std::map<visgraph::Point, double> distancesEvader = g.shortestPathMultipleDDict(visgraph::Point(x[0], y[0]), destinations, borderPoints);
        std::map<visgraph::Point, double> distancesPursuer = g.shortestPathMultipleDDict(visgraph::Point(x[1], y[1]), destinations, borderPoints);
        // Algorithm for the pursuer evader game
        // We assume the evader is in position 0
        int i;
        //We iterate over the shortestPath of the evader, we check every possible node
        for(i = 1; i < shortestPathEvader.size(); i++){
          //Check if the distance of the pursuer in a certain node is inferior to the distance of the evader on the same node
          if (distancesPursuer[shortestPathEvader[i]] < distancesEvader[shortestPathEvader[i]]) {
            std::vector<visgraph::Point> finalDestinations;
            //We push the node into the destinations
            finalDestinations.push_back(visgraph::Point(shortestPathEvader[i].x, shortestPathEvader[i].y));
            bool foundPathPursuer = reachDestinationForRobot(1, visgraph::Point(x[1], y[1]), finalDestinations, theta[1], borderPoints, originalGraph, g, shortestPathPursuer, pathLengthsPursuer, path, max_k, size);
            if (foundPathPursuer) {
              // Check if the length of the multipoint path is really less than the one of the evader
              double completePathLengthPursuer = pathLengthsPursuer[pathLengthsPursuer.size()-1], completePathLengthEvader = pathLengthsEvader[i-1];
              std::cout << "PATH FOUND FOR PURSUER WITH LENGTH: " << completePathLengthPursuer << " WHERE EVADER'S ONE IS " << completePathLengthEvader << "\n";
              if (completePathLengthPursuer < completePathLengthEvader) {
                std::cout << "WITH THIS PATH THE PURSUER WILL BE ABLE TO REACH THE EVADER\n";
                break;
              } else {
                // Cancel the constructed path and retry
                path[1].points.clear();
                std::cout << "WITH THIS PATH THE PURSUER WON'T BE ABLE TO REACH THE EVADER\n";
              }
            }
          }
        }
        //We were not able to identify a good path
        if (i == shortestPathEvader.size()) {
          std::cout << "THE PURSUER WASN'T ABLE TO FIND A PATH TO REACH THE EVADER\n";
        }
      } else if(numberOfDestinations > 1) {
      
        // ********** TWO DESTINATIONS - PROJECT NUMBER 1 - ********** //
        std::cout << "THERE ARE TWO ROBOTS AND TWO DESTINATIONS\nPursuer Evader Game\n";

        std::vector<visgraph::Point> shortestPathTmp;
        std::vector<double> pathLengths;
        std::vector<visgraph::Point> shortestPath;
        std::vector<visgraph::Point> finalDestination;

        // We use random to decide the finalDestination
        std::random_device rd;
        std::default_random_engine eng(rd());
        std::uniform_int_distribution<int> disti(0, numberOfDestinations-1);
        std::uniform_real_distribution<double> distr(0.0, 1.0);

        int counter = 0;  // Keep a counter to limit the number of iterations of the do while
        // chose a valid destination
        for(counter == 0; counter < 50; counter++){
          // Choose a random destination
          if(counter > 0) {
            finalDestination.pop_back();
          }
          finalDestination.push_back(destinations[disti(eng)]);
          // ********** COMPUTE THE FIRST SHORTEST PATH FROM ORIGIN TO DESTINATION ********** //
          shortestPath = g.shortestPath(visgraph::Point(x[0], y[0]), finalDestination[0], borderPoints);
          dubins::DubinsCurve **curvesValid = planDestinationForRobot(0, visgraph::Point(x[0], y[0]), finalDestination, theta[0], borderPoints, originalGraph, g, shortestPath, pathLengths, path, max_k, size);
          if(curvesValid!=nullptr) break;
        } 
        // if no valid destination, reduce the enlarge offset and try again
        if(counter == 50) {
          std::cout << "NO PATH FOUND FOR THE EVADER!\n";
          std::cout << "WE TRY AGAIN, THIS TIMES WITH SMALLER POLYGONS\n";
          // We compute again all the graphs with a smaller offset, higher risk but shorter paths
          pols = enlargeAndJoinObstacles(polygons, altOffset);
          polygonsForVisgraph = pols[0];
          polygons = pols[1];
          addBorders(borderMaxX, borderMinX, borderMaxY, borderMinY, altOffset, variant, borderPoints, polygons);
          originalGraph = visgraph::Graph(polygons, false, true);
          g = visg.computeVisibilityGraphMultipleOD(polygonsForVisgraph, origins, destinations);
          counter = 0;
          //Again we try to find a good path to a destination
          for(counter == 0; counter < 50; counter++){
            // Choose a random destination
            finalDestination.pop_back();
            finalDestination.push_back(destinations[disti(eng)]);
            // ********** COMPUTE THE FIRST SHORTEST PATH FROM ORIGIN TO DESTINATION ********** //
            shortestPath = g.shortestPath(visgraph::Point(x[0], y[0]), finalDestination[0], borderPoints);
            dubins::DubinsCurve **curvesValid = planDestinationForRobot(0, visgraph::Point(x[0], y[0]), finalDestination, theta[0], borderPoints, originalGraph, g, shortestPath, pathLengths, path, max_k, size);
            if(curvesValid!=nullptr) break;
          }
          if (counter == 50) {
            std::cout << "NO PATH FOUND EVEN WITH A SMALLER OFFSET FOR THE EVADER!\n";
          }else{
            std::cout << "PATH FOUND WITH A SMALLER OFFSET FOR THE EVADER WITH LENGTH: " << pathLengths[pathLengths.size()-1] << "\n";
          }
        }

        // fill initial point with a shortest path calculated by using random chosen destination
        // a list of intermediate points and a destination.
        // The only point in which we specify an angle is the initial one.
        dubins::Dubins dubins = dubins::Dubins(max_k, size);
        int pointCnt = 1;

        double lastTheta = theta[0];  // Keep the last angle of the robot
        counter = 0;  // Keep a counter to limit the number of iterations of the do while

        visgraph::Point origin = visgraph::Point(x[0], y[0]); // Keep track of the origin point
        std::vector<std::vector<visgraph::Point>> shortestPathsEvader;  // All shortest paths of the evader
        std::vector<std::vector<double>> pathLengthsEvader; // All lengths of all the evader's paths
        std::vector<visgraph::Point> destinationPointsEvader; // All the destination points considered by the evader in order
        std::vector<double> evaderThetas; // All the destination theta angles of the evader in order
        evaderThetas.push_back(theta[0]);

        std::vector<double> evaderCurvesLengthList;

        do {
          counter++;

          // Choose the exit point randomly
          for(pointCnt = 1; pointCnt < shortestPath.size(); pointCnt++) {
              if(distr(eng) < 0.2) break;
          }

          //Find the dubins shortest path given the set of intermediate points only if there are points added in the list
          if(pointCnt > 1) {
            std::vector<visgraph::Point> v;
            std::vector<double> d;
            std::vector<visgraph::Point> destTmp {shortestPath[pointCnt-1]};
            shortestPathsEvader.push_back(v);
            pathLengthsEvader.push_back(d);

            // Plan a path with the evader
            dubins::DubinsCurve **curvesEvader = planDestinationForRobot(0, origin, finalDestination, lastTheta, borderPoints, originalGraph, g, shortestPathsEvader[shortestPathsEvader.size()-1], pathLengthsEvader[pathLengthsEvader.size()-1], path, max_k, size);
            // If there is no path available with the decided destination, restart from the beginning of the process
            if (curvesEvader == nullptr) {
              shortestPathsEvader.pop_back();
              pathLengthsEvader.pop_back();
              pointCnt = 1;
            } else {
              for (int i = 0; i < shortestPathsEvader[shortestPathsEvader.size()-1].size() - 1; i++) {
                destinationPointsEvader.push_back(shortestPathsEvader[shortestPathsEvader.size()-1][i]);
                evaderThetas.push_back(curvesEvader[i]->a3->dubins_line->th);
              }
              // Remove the destination, it will be added later
              destinationPointsEvader.pop_back();
              evaderThetas.pop_back();
              // Truncate the path based on the value of pointCnt
              fillPath(curvesEvader, path, pointCnt-1, size, 0);
              for (int i = 0; i < pointCnt-1; i++) {
                evaderCurvesLengthList.push_back(curvesEvader[i]->L);
              }
              // Get the last angle of the robot
              lastTheta = curvesEvader[pointCnt-2]->a3->dubins_line->th;
              // evaderThetas.push_back(lastTheta);
              //Clear up some space
              for(int i = 0; i < shortestPathsEvader[shortestPathsEvader.size()-1].size()-1; i++) {
                delete curvesEvader[i];
              }
              delete[] curvesEvader;
            }
          }
          // Did we arrive at our destination? If so, break the loop and return, we save the last Theta and the final destination
          if(shortestPath[pointCnt-1] == finalDestination[0]) {
            destinationPointsEvader.push_back(finalDestination[0]);
            evaderThetas.push_back(lastTheta);
            break;
          }

          // Randomly choose the new destination and replan the path
          finalDestination.pop_back();
          finalDestination.push_back(destinations[disti(eng)]); //Randomly generated element

          // Calculate the new path from the current point to the new destination
          origin = shortestPath[pointCnt-1];
          shortestPath = g.shortestPath(origin, finalDestination[0], borderPoints);

        } while(counter < 100);

        // Now we define the pursuer's path
        // In order to simulate the fact that the pursuer should not be able to predict the future,
        // i.e. the evader's decisions to change destination, we allow the pursuer to change its path
        // accordingly to the evader's new decisions only after:
        // - it actually happens at runtime (we do this by comparing the length of the dubins paths)
        // - it reaches the next node

        // Keep track of the pursuer's starting position and last angle theta
        visgraph::Point originPursuer = visgraph::Point(x[1], y[1]);
        double lastThetaPursuer = theta[1];
        bool reachedEvader = false;
        bool firstPathNotFound = false;

        // Loop through all the evader's nodes. The pursuer will know only the ones that already happened
        for (int z = 0; z < destinationPointsEvader.size() && !reachedEvader; z++) {

          // Just move closer to a point in which it would be easier to reach the evader.
          // We do this because at the first iteration we don't know where the evader wants to go.
          if (z == 0) {
            std::vector<visgraph::Point> sp;
            std::vector<double> pl;

            // Check the closest destination to the evader
            double d1 = (pow(destinations[0].x, 2) - pow(x[0], 2)) + (pow(destinations[0].y, 2) - pow(y[0], 2));
            double d2 = (pow(destinations[1].x, 2) - pow(x[0], 2)) + (pow(destinations[1].y, 2) - pow(y[0], 2));

            // Use the length of the first evader's curve to be sure the pursuer is not "predicting the future" decisions of the evader
            double lengthOfFirstEvaderPath = evaderCurvesLengthList[0];

            // Move towards the closest destination to the evader, if found
            // If not found, move to the other one.
            // If not found, move closer to the current position of the evader
            if (d1 > d2) {
              std::vector<visgraph::Point> dest {destinations[1]};
              dubins::DubinsCurve **curves = planDestinationForRobot(1, visgraph::Point(x[1], y[1]), dest, theta[1], borderPoints, originalGraph, g, sp, pl, path, max_k, size);
              if (curves != nullptr) {
                int index = pl.size();
                for (int b = 0; b < pl.size(); b++) {
                  if (pl[b] >= lengthOfFirstEvaderPath) {
                    index = b+1;
                    break;
                  }
                }
                fillPath(curves, path, index, size, 1);
                originPursuer = sp[1];
                lastThetaPursuer = path[1].points[path[1].points.size()-1].theta;
                for(int i = 0; i < sp.size()-1; i++) {
                  delete curves[i];
                }
                delete[] curves;
              }
              else {
                std::vector<visgraph::Point> dest {destinations[0]};
                dubins::DubinsCurve **curves = planDestinationForRobot(1, visgraph::Point(x[1], y[1]), dest, theta[1], borderPoints, originalGraph, g, sp, pl, path, max_k, size);
                if (curves != nullptr) {
                  int index = pl.size();
                  for (int b = 0; b < pl.size(); b++) {
                    if (pl[b] >= lengthOfFirstEvaderPath) {
                      index = b+1;
                      break;
                    }
                  }
                  fillPath(curves, path, index, size, 1);
                  originPursuer = sp[1];
                  lastThetaPursuer = path[1].points[path[1].points.size()-1].theta;
                  for(int i = 0; i < sp.size()-1; i++) {
                    delete curves[i];
                  }
                  delete[] curves;
                } else {
                  std::vector<visgraph::Point> dest {visgraph::Point(x[0], y[0])};
                  dubins::DubinsCurve **curves = planDestinationForRobot(1, visgraph::Point(x[1], y[1]), dest, theta[1], borderPoints, originalGraph, g, sp, pl, path, max_k, size);
                  if (curves != nullptr) {
                    int index = pl.size();
                    for (int b = 0; b < pl.size(); b++) {
                      if (pl[b] >= lengthOfFirstEvaderPath) {
                        index = b+1;
                        break;
                      }
                    }
                    fillPath(curves, path, index, size, 1);
                    originPursuer = sp[1];
                    lastThetaPursuer = path[1].points[path[1].points.size()-1].theta;
                    for(int i = 0; i < sp.size()-1; i++) {
                      delete curves[i];
                    }
                    delete[] curves;
                  } else {
                    firstPathNotFound = true;
                    std::cout << "NO PATH FOUND FOR PURSUER\n";
                  }
                }
              }
            } else {
              std::vector<visgraph::Point> dest {destinations[0]};
              dubins::DubinsCurve **curves = planDestinationForRobot(1, visgraph::Point(x[1], y[1]), dest, theta[1], borderPoints, originalGraph, g, sp, pl, path, max_k, size);
              if (curves != nullptr) {
                int index = pl.size();
                for (int b = 0; b < pl.size(); b++) {
                  if (pl[b] >= lengthOfFirstEvaderPath) {
                    index = b+1;
                    break;
                  }
                }
                fillPath(curves, path, index, size, 1);
                originPursuer = sp[1];
                lastThetaPursuer = path[1].points[path[1].points.size()-1].theta;
                for(int i = 0; i < sp.size()-1; i++) {
                  delete curves[i];
                }
                delete[] curves;
              }
              else {
                std::vector<visgraph::Point> dest {destinations[1]};
                dubins::DubinsCurve **curves = planDestinationForRobot(1, visgraph::Point(x[1], y[1]), dest, theta[1], borderPoints, originalGraph, g, sp, pl, path, max_k, size);
                if (curves != nullptr) {
                  int index = pl.size();
                  for (int b = 0; b < pl.size(); b++) {
                    if (pl[b] >= lengthOfFirstEvaderPath) {
                      index = b+1;
                      break;
                    }
                  }
                  fillPath(curves, path, index, size, 1);
                  originPursuer = sp[1];
                  lastThetaPursuer = path[1].points[path[1].points.size()-1].theta;
                  for(int i = 0; i < sp.size()-1; i++) {
                    delete curves[i];
                  }
                  delete[] curves;
                } else {
                  std::vector<visgraph::Point> dest {visgraph::Point(x[0], y[0])};
                  dubins::DubinsCurve **curves = planDestinationForRobot(1, visgraph::Point(x[1], y[1]), dest, theta[1], borderPoints, originalGraph, g, sp, pl, path, max_k, size);
                  if (curves != nullptr) {
                    int index = pl.size();
                    for (int b = 0; b < pl.size(); b++) {
                      if (pl[b] >= lengthOfFirstEvaderPath) {
                        index = b+1;
                        break;
                      }
                    }
                    fillPath(curves, path, index, size, 1);
                    originPursuer = sp[1];
                    lastThetaPursuer = path[1].points[path[1].points.size()-1].theta;
                    for(int i = 0; i < sp.size()-1; i++) {
                      delete curves[i];
                    }
                    delete[] curves;
                  } else {
                    firstPathNotFound = true;
                    std::cout << "NO PATH FOUND FOR PURSUER\n";
                  }
                }
              }
            }

          } else {

            if (firstPathNotFound) {
              break;
            }
            // Origin of the evader
            visgraph::Point originEvader = destinationPointsEvader[z];

            // Destination that the evader wanted to reach during this iteration
            std::vector<visgraph::Point> destTmp1 {destinations[0]};
            std::vector<visgraph::Point> destTmp2 {destinations[1]};

            // Calculate the distances using Dijkstra for the evader and the pursuer
            std::map<visgraph::Point, double> distancesEvader = g.shortestPathMultipleDDict(originEvader, destTmp1, borderPoints);
            std::map<visgraph::Point, double> distancesPursuer = g.shortestPathMultipleDDict(originPursuer, destTmp1, borderPoints);

            // Calculate the shortest path and the path lengths of the evader when going to each of the destinations
            std::vector<visgraph::Point> shortestPathEvaderTmp;
            std::vector<double> pathLengthsEvaderTmp;
            std::vector<visgraph::Point> shortestPathEvaderTmp1 = g.shortestPath(originEvader, destTmp1[0], borderPoints);
            std::vector<double> pathLengthsEvaderTmp1;
            for(int i = 0; i < shortestPathEvaderTmp1.size()-1; i++){
              double tmp = sqrt(pow(shortestPathEvaderTmp1[i+1].x - shortestPathEvaderTmp1[i].x, 2.0) + pow(shortestPathEvaderTmp1[i+1].y - shortestPathEvaderTmp1[i].y, 2.0));
              pathLengthsEvaderTmp1.push_back(tmp);
            }
            

            std::vector<visgraph::Point> shortestPathEvaderTmp2 = g.shortestPath(originEvader, destTmp2[0], borderPoints);
            std::vector<double> pathLengthsEvaderTmp2;
            for(int i = 0; i < shortestPathEvaderTmp2.size()-1; i++){
              double tmp = sqrt(pow(shortestPathEvaderTmp2[i+1].x - shortestPathEvaderTmp2[i].x, 2.0) + pow(shortestPathEvaderTmp2[i+1].y - shortestPathEvaderTmp2[i].y, 2.0));
              pathLengthsEvaderTmp2.push_back(tmp);
            }

            // Try to understand which destination is the actual one of the evader
            int numberOfSamePointsDest1 = 0, numberOfSamePointsDest2 = 0;
            for (int m = 0; m < z; m++) {
              if (destinationPointsEvader[m] == shortestPathEvaderTmp1[m]) {
                numberOfSamePointsDest1++;
              }
              if (destinationPointsEvader[m] == shortestPathEvaderTmp2[m]) {
                numberOfSamePointsDest2++;
              }
            }

            if (numberOfSamePointsDest1 > numberOfSamePointsDest2 && !shortestPathEvaderTmp1.empty()) {
              shortestPathEvaderTmp = shortestPathEvaderTmp1;
              pathLengthsEvaderTmp = pathLengthsEvaderTmp1;
            } else if (numberOfSamePointsDest1 < numberOfSamePointsDest2 && !shortestPathEvaderTmp2.empty()) {
              shortestPathEvaderTmp = shortestPathEvaderTmp2;
              pathLengthsEvaderTmp = pathLengthsEvaderTmp2;
            } else {

              // Move closer to the destination that is closer to the evader
              // Check the closest destination to the evader
              double d1 = (pow(destinations[0].x, 2) - pow(destinationPointsEvader[z].x, 2)) + (pow(destinations[0].y, 2) - pow(destinationPointsEvader[z].y, 2));
              double d2 = (pow(destinations[1].x, 2) - pow(destinationPointsEvader[z].x, 2)) + (pow(destinations[1].y, 2) - pow(destinationPointsEvader[z].y, 2));
              
              if (d1 > d2 && !shortestPathEvaderTmp2.empty()) {
                shortestPathEvaderTmp = shortestPathEvaderTmp2;
                pathLengthsEvaderTmp = pathLengthsEvaderTmp2;
              } else if (!shortestPathEvaderTmp1.empty()){
                shortestPathEvaderTmp = shortestPathEvaderTmp1;
                pathLengthsEvaderTmp = pathLengthsEvaderTmp1;
              } else {
                std::cout << "NO PATH FOUND FOR THE PURSUER\n";
                break;
              }
            }
            
            if (!shortestPathEvaderTmp1.empty()) {
              shortestPathEvaderTmp1.clear();
            }
            if (!shortestPathEvaderTmp2.empty()) {
              shortestPathEvaderTmp2.clear();
            }
            
            // Keep track of the shortest path and the path lengths of the pursuer, see the algorithm below (for loop)
            std::vector<visgraph::Point> shortestPathPursuer;
            std::vector<double> pathLengthsPursuer;

            // Algorithm for the pursuer evader game
            // We assume the evader is in position 0
            int i;
            for(i = 1; i < shortestPathEvaderTmp.size(); i++) {
              if (distancesPursuer[shortestPathEvaderTmp[i]] < distancesEvader[shortestPathEvaderTmp[i]] || i == shortestPathEvaderTmp.size()-1) {
                // If the distance from this node of the pursuer is less than the evader's one, then the pursuer may be able to reach the evader setting this node as destination
                std::vector<visgraph::Point> finalDestinations;
                finalDestinations.push_back(shortestPathEvaderTmp[i]);

                // Plan the path
                dubins::DubinsCurve **curvesPursuer = planDestinationForRobot(1, originPursuer, finalDestinations, lastThetaPursuer, borderPoints, originalGraph, g, shortestPathPursuer, pathLengthsPursuer, path, max_k, size);
                if (curvesPursuer != nullptr) {
                  // Check if the length of the multipoint path is really less than the one of the evader
                  double completePathLengthPursuer = pathLengthsPursuer[pathLengthsPursuer.size()-1]; 
                  double completePathLengthEvader = pathLengthsEvaderTmp[i-1];
                  if (completePathLengthPursuer < completePathLengthEvader || i == shortestPathEvaderTmp.size()-1) {

                    // Even the dubins path is smaller than the one of the evader
                    int index = pathLengthsPursuer.size();
                    
                    // Check in which node the pursuer should stop. We stop when the length of the path
                    // of the pursuer is greater than the one of the evader, in order to keep them
                    // synchronized
                    double lengthOfEvaderPath = 0;
                    for (int m = 0; m < z-1; m++) {
                      lengthOfEvaderPath += evaderCurvesLengthList[m];
                    }
                    for (int j = 0; j < pathLengthsPursuer.size(); j++) {
                      if (pathLengthsPursuer[j] > lengthOfEvaderPath) {
                        index = j+1;
                        break;
                      }
                    }
                    // Make the pursuer reach the destination up to the decided node
                    fillPath(curvesPursuer, path, index, size, 1);


                    // Update last angle theta of the pursuer and origin point
                    lastThetaPursuer = path[1].points[path[1].points.size()-1].theta;
                    originPursuer = shortestPathPursuer[index];

                    for(int k = 0; k < shortestPathPursuer.size()-1; k++) {
                      delete curvesPursuer[k];
                    }
                    delete[] curvesPursuer;

                    break;
                  }
                }
              }
            }
            // if (i == shortestPathEvaderTmp.size()) {
            //   std::cout << "THE PURSUER WASN'T ABLE TO FIND A PATH TO REACH THE EVADER AT ITERATION " << z << "\n";
            // }
          }
        }

      }
    } else if (numberOfRobots == 3) {
      std::cout << "THERE ARE THREE ROBOTS - Project Number 2 Not Done\n";
    }

    return true;
  }

}