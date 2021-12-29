#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "clipper.hpp"
#include <iostream>
#include "utils.hpp"
#include "visgraph.hpp"

#include "clipper_addons.hpp"

/**
 * @brief Clipper Polygon Offsetting helper function
 * We multiply the points by 1000 because Clipper works best under the assumption of working on a certain scale
 * 
 * @param points Points of the original polygon
 * @param offset Offset that must be used to enlarge or shrink the polygon
 * @return std::vector<visgraph::Point> Array of points of the resulting polygon
 */
std::vector<visgraph::Point> enlarge(std::vector<visgraph::Point> points, double offset)
{
    ClipperLib::Path subj;
    ClipperLib::Paths solution;
    for (int i = 0; i < points.size(); i++)
    {
        subj << ClipperLib::IntPoint(points[i].x*1000, points[i].y*1000);
    }
    
    ClipperLib::ClipperOffset co;
    co.AddPath(subj, ClipperLib::jtMiter, ClipperLib::etClosedPolygon);
    co.Execute(solution, offset*1000.0);

    CleanPolygons(solution);

    std::vector<visgraph::Point> result;
    if (solution.size() > 0) {
        for (ClipperLib::IntPoint p : solution[0]) {
            result.push_back(visgraph::Point(p.X / 1000.0, p.Y / 1000.0));
        }
    }

    // printSolution(subj, solution);

    return result;
}

/**
 * @brief Print a clipper polygon offsetting solution using OpenCV
 * 
 * @param startingPoints Points of the original polygon
 * @param solution Solution given by clipper
 */
void printSolution(std::vector<ClipperLib::IntPoint> startingPoints, ClipperLib::Paths solution)
{
    // Create opencv plotting tool
    cv::Mat plot(500, 500, CV_8UC3, cv::Scalar(255, 255, 255));

    // Get the initial path
    ClipperLib::Path subj;
    for (int i = 0; i < startingPoints.size(); i++)
    {
        subj.push_back(startingPoints[i]);
    }
    // Draw the initial path
    cv::line(plot, cv::Point2f(subj.at(subj.size() - 1).X, subj.at(subj.size() - 1).Y), cv::Point2f(subj.at(0).X, subj.at(0).Y), cv::Scalar(255, 0, 0), 1);
    for (unsigned int j = 1; j < subj.size(); j++)
    {
        cv::line(plot, cv::Point2f(subj.at(j - 1).X, subj.at(j - 1).Y), cv::Point2f(subj.at(j).X, subj.at(j).Y), cv::Scalar(255, 0, 0), 1);
    }

    // Draw the solution
    for (unsigned int i = 0; i < solution.size(); i++)
    {
        ClipperLib::Path path = solution.at(i);
        cv::line(plot, cv::Point2f(path.at(path.size() - 1).X, path.at(path.size() - 1).Y), cv::Point2f(path.at(0).X, path.at(0).Y), cv::Scalar(255, 255, 0), 2);
        for (unsigned int j = 1; j < path.size(); j++)
        {
            std::cout << path.at(j - 1).X << " , " << path.at(j - 1).Y << " - " << path.at(j).X << " , " << path.at(j).Y << "\n";
            cv::line(plot, cv::Point2f(path.at(j - 1).X, path.at(j - 1).Y), cv::Point2f(path.at(j).X, path.at(j).Y), cv::Scalar(255, 255, 0), 2);
        }
    }
    cv::flip(plot, plot, 0);
    cv::imshow("Clipper", plot);
    cv::waitKey(0);
}

/**
 * @brief Verifies if two clipper polygons interact
 * 
 * @param subj The first polygon to check for intersections
 * @param clip The second polygon to check for intersections
 * @return true If the two polygons intersect
 * @return false If the two polygons do not intersect
 */
bool intersect (dubins::DubinsPoint *subj, dubins::DubinsPoint *clip){
    ClipperLib::Path firstPoly;
    ClipperLib::Paths firstFinalPoly;
    for (int i = 0; i < sizeof(subj); i++){
        firstPoly << ClipperLib::IntPoint(subj[i].x*1000, subj[i].y*1000);
    }
    firstFinalPoly.push_back(firstPoly);

    ClipperLib::Path secondPoly;
    ClipperLib::Paths secondFinalPoly;
    for (int j = 0; j < sizeof(clip); j++){
        secondPoly << ClipperLib::IntPoint(clip[j].x*1000, clip[j].y*1000);
    }
    secondFinalPoly.push_back(secondPoly);

    ClipperLib::Clipper c;
    //Need to make this function take as input points instead of paths, create paths
    c.AddPaths(firstFinalPoly, ClipperLib::ptSubject, true);
    c.AddPaths(secondFinalPoly, ClipperLib::ptClip, true);

    ClipperLib::Paths solution;
    c.Execute(ClipperLib::ctIntersection, solution, ClipperLib::pftNonZero, ClipperLib::pftNonZero);

    return solution.size()!=0;
}

/**
 * @brief Given some obstacles and an offset, creates a "bigger" version using clipper and a "slightly bigger" one, then returns them
 * 
 * @param polygon Obstacles we are considering
 * @param offset Offset for obstacle offsetting
 * @return std::vector<std::vector<student::Point>> Array containing at position 0 the bigger obstacles and at position 1 the smaller ones
 */
std::vector<std::vector<visgraph::Point>> enlargeObstaclesWithTwoOffsets(std::vector<visgraph::Point> polygon, double offset){
    double variant = 2.5;
    std::vector<visgraph::Point> newPath;

    //Convert the polygon to the data structure for enlarge, we need a vector of visgraph points
    for(unsigned int i = 0; i < polygon.size(); i++){
        newPath.push_back(visgraph::Point(polygon[i].x, polygon[i].y));
    }
    std::vector<visgraph::Point> bigSolution;
    std::vector<visgraph::Point> smallSolution;
    smallSolution = enlarge(newPath, offset);
    bigSolution = enlarge(newPath, offset + (offset/variant));
    //Take both solutions, push them back a vector, return it
    std::vector<std::vector<visgraph::Point>> finalResult;
    finalResult.push_back(bigSolution);
    finalResult.push_back(smallSolution);

    return finalResult;
}

/**
 * @brief Enlarge the obstacles using the function enlargeObstaclesWithTwoOffsets, then join them in case they collide between each other
 * The function also removes possible holes that can be formed during the join phase
 * 
 * @param polygonsList All the obstacles
 * @param offset Offset for enlarging them
 * @return std::vector<std::vector<std::vector<visgraph::Point>>> Array containing at position 0 the bigger obstacles and at position 1 the smaller ones, joined in case of collisions
 */
std::vector<std::vector<std::vector<visgraph::Point>>> enlargeAndJoinObstacles(std::vector<std::vector<visgraph::Point>> polygonsList, double offset){

    std::vector<std::vector<visgraph::Point>> bigPolygons;
    std::vector<std::vector<visgraph::Point>> smallPolygons;

    for (int i = 0; i < polygonsList.size(); i++){
        std::vector<std::vector<visgraph::Point>> results;
        results = enlargeObstaclesWithTwoOffsets(polygonsList[i], offset);
        bigPolygons.push_back(results[0]);
        smallPolygons.push_back(results[1]);
        results.clear();
    }

    ClipperLib::Paths subj(bigPolygons.size()), solution;

    for (unsigned int i = 0; i < bigPolygons.size(); i++){
        for (unsigned int j = 0; j < bigPolygons[i].size(); j++) {
            subj[i].push_back(ClipperLib::IntPoint(bigPolygons[i][j].x*1000, bigPolygons[i][j].y*1000));
        }
    }
    ClipperLib::Clipper c;
    c.AddPaths(subj, ClipperLib::ptSubject, true);
    c.Execute(ClipperLib::ctUnion, solution, ClipperLib::pftNonZero);

    CleanPolygons(solution);

    ClipperLib::Paths subj1(smallPolygons.size()), solution1;

    for (unsigned int i = 0; i < smallPolygons.size(); i++){
        for (unsigned int j = 0; j < smallPolygons[i].size(); j++) {
            subj1[i].push_back(ClipperLib::IntPoint(smallPolygons[i][j].x*1000, smallPolygons[i][j].y*1000));
        }
    }

    ClipperLib::Clipper c1;
    c1.AddPaths(subj1, ClipperLib::ptSubject, true);
    c1.Execute(ClipperLib::ctUnion, solution1, ClipperLib::pftNonZero);

    CleanPolygons(solution1);

    std::vector<std::vector<std::vector<visgraph::Point>>> returnValues;

    std::vector<std::vector<visgraph::Point>> intermediateValues;

    for (unsigned int i = 0; i < solution.size(); i++){
        ClipperLib::Path path = solution.at(i);
        std::vector<visgraph::Point> newPath;
        visgraph::VisGraph visg;
        if (Orientation(path)) {
            for(ClipperLib::IntPoint p : path){
                newPath.push_back(visgraph::Point(p.X/1000.0, p.Y/1000.0));
            }
        }
        if (!newPath.empty())
            intermediateValues.push_back(newPath);
        newPath.clear();
    }

    returnValues.push_back(intermediateValues);

    intermediateValues.clear();

    for (unsigned int i = 0; i < solution1.size(); i++){
        ClipperLib::Path path = solution1.at(i);
        std::vector<visgraph::Point> newPath;
        visgraph::VisGraph visg;
        if (Orientation(path)) {
            for(ClipperLib::IntPoint p : path){
                newPath.push_back(visgraph::Point(p.X/1000.0, p.Y/1000.0));
            }
        }
        if (!newPath.empty())
            intermediateValues.push_back(newPath);
        newPath.clear();
    }

    returnValues.push_back(intermediateValues);

    return returnValues;
}

