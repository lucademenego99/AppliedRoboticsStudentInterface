#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "clipper.hpp"
#include <iostream>
#include "utils.hpp"
#include "visgraph.hpp"

#include "clipper_addons.hpp"

using namespace ClipperLib;

/**
 * @brief Clipper Polygon Offsetting helper function
 * 
 * @param points Points of the original polygon
 * @param offset Offset that must be used to enlarge or shrink the polygon
 * @return std::vector<student::Point> Array of points of the resulting polygon
 */
std::vector<student::Point> enlarge(std::vector<student::Point> points, double offset)
{
    Path subj;
    Paths solution;
    for (int i = 0; i < points.size(); i++)
    {
        subj << IntPoint(points[i].x*1000, points[i].y*1000);
    }
    
    ClipperOffset co;
    co.AddPath(subj, jtMiter, etClosedPolygon);
    co.Execute(solution, offset*1000);

    CleanPolygons(solution);

    std::vector<student::Point> result;
    if (solution.size() > 0) {
        for (IntPoint p : solution[0]) {
            result.push_back(student::Point(p.X / 1000.0, p.Y / 1000.0));
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
void printSolution(std::vector<IntPoint> startingPoints, Paths solution)
{
    // Create opencv plotting tool
    cv::Mat plot(500, 500, CV_8UC3, cv::Scalar(255, 255, 255));

    // Get the initial path
    Path subj;
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
        Path path = solution.at(i);
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
 * @param subj The first polygon
 * @param clip The second polygon
 * @return true 
 * @return false 
 */
 
bool intersect (student::Point *subj, student::Point *clip){
    Path firstPoly;
    Paths firstFinalPoly;
    for (int i = 0; i < sizeof(subj); i++){
        firstPoly << IntPoint(subj[i].x*1000, subj[i].y*1000);
    }
    firstFinalPoly.push_back(firstPoly);

    Path secondPoly;
    Paths secondFinalPoly;
    for (int j = 0; j < sizeof(clip); j++){
        secondPoly << IntPoint(clip[j].x*1000, clip[j].y*1000);
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

std::vector<std::vector<student::Point>> verifyAndJoin (student::Point *firstPoly, student::Point *secondPoly){
    //Intersect need to take as input points instead of Paths
    ClipperLib::Clipper c;
    if(intersect(firstPoly, secondPoly)){
        
        Path subj;
        Paths finalSubj;
        for (int i = 0; i < sizeof(firstPoly); i++){
            subj << IntPoint(firstPoly[i].x*1000, firstPoly[i].y*1000);
        }
        finalSubj.push_back(subj);

        Path clip;
        Paths finalClip;
        for (int j = 0; j < sizeof(secondPoly); j++){
            clip << IntPoint(secondPoly[j].x*1000, secondPoly[j].y*1000);
        }

        ClipperLib::Paths solution;
        c.AddPaths(finalSubj, ClipperLib::ptSubject, true);
        c.AddPaths(finalClip, ClipperLib::ptClip, true);
        c.Execute(ClipperLib::ctUnion, solution, ClipperLib::pftEvenOdd, ClipperLib::pftNonZero);

        std::vector<student::Point> newPath;
        std::vector<std::vector<student::Point>> finalPoints;

        for (unsigned int i = 0; i < solution.size(); i++){
            Path path = solution.at(i);
            for(IntPoint p : path){
                newPath.push_back(student::Point(p.X, p.Y));
            }
            finalPoints.push_back(newPath);
            newPath.clear();
        }
        return finalPoints;
    }
}


/**
 * @brief Verifies if a list of polygons has some intersections, if so joins them and then enlarges them all
 * 
 * @param points Matrix of polygons
 */
std::vector<std::vector<student::Point>> joinAndEnlarge (std::vector<std::vector<student::Point>> points){

    Paths subj(1), clip(points.size()-1), solution;
    cv::Mat plot(500, 500, CV_8UC3, cv::Scalar(255, 255, 255));

    for (unsigned int i = 0; i < points[0].size(); i++){
        subj[0].push_back(IntPoint(points[0][i].x*1000, points[0][i].y*1000));
    }
    for(unsigned int i = 0; i < clip.size(); i++){
        for(unsigned int j = 0; j < points[i+1].size(); j++){
            clip[i].push_back(IntPoint(points[i+1][j].x*1000, points[i+1][j].y*1000));
        }    
    }
    Clipper c;
    c.AddPaths(subj, ptSubject, true);
    c.AddPaths(clip, ptClip, true);
    c.Execute(ctUnion, solution, pftNonZero, pftNonZero);

    std::vector<std::vector<student::Point>> enlargedPolygons;
    int offset = 4; //Modify the offset of the enlargement process

    for (unsigned int i = 0; i < solution.size(); i++){
        Path path = solution.at(i);
        std::vector<student::Point> newPath;
        for(IntPoint p : path){
            newPath.push_back(student::Point(p.X, p.Y));
        }
        enlargedPolygons.push_back(enlarge(newPath, offset));
        newPath.clear();
    }
    return enlargedPolygons;
    
    /*
    for (unsigned int i = 0; i < solution.size(); i++){
        Path path = solution.at(i);
        cv::line(plot, cv::Point2f(path.at(path.size() - 1).X, path.at(path.size() - 1).Y), cv::Point2f(path.at(0).X, path.at(0).Y), cv::Scalar(255, 255, 0), 2);
        for (unsigned int j = 1; j < path.size(); j++){
            cv::line(plot, cv::Point2f(path.at(j - 1).X, path.at(j - 1).Y), cv::Point2f(path.at(j).X, path.at(j).Y), cv::Scalar(255, 255, 0), 2);
        }
    }
    cv::flip(plot, plot, 0);
    cv::imshow("Clipper", plot);
    cv::waitKey(0);
    */
}
/**
 * @brief Given some obstacles and an offset, creates a "bigger" version using clipper and a "slightly bigger" one, then returns them
 * 
 * @param polygon Obstacles we are considering
 * @param offset Offset for obstacle enhancing
 * @return std::vector<std::vector<student::Point>> 
 */
std::vector<std::vector<student::Point>> applyChanges(std::vector<visgraph::Point> polygon, int offset){
    double variant = 4.0;
    std::vector<student::Point> newPath;

    //Convert the polygon to the data structure for enlarge, we need a vector of student points
    for(unsigned int i = 0; i < polygon.size(); i++){
        newPath.push_back(student::Point(polygon[i].x, polygon[i].y));
    }
    std::vector<student::Point> bigSolution;
    std::vector<student::Point> smallSolution;
    smallSolution = enlarge(newPath, offset);
    bigSolution = enlarge(newPath, offset + (offset/variant));
    //Take both solutions, push them back a vector, return it
    std::vector<std::vector<student::Point>> finalResult;
    finalResult.push_back(bigSolution);
    finalResult.push_back(smallSolution);

    return finalResult;
}
//Name still needs to be defined
std::vector<std::vector<std::vector<student::Point>>> joinMultiplePolygons(std::vector<std::vector<visgraph::Point>> polygonsList, int offset){

    std::vector<std::vector<student::Point>> bigPolygons;
    std::vector<std::vector<student::Point>> smallPolygons;

    for (int i = 0; i < polygonsList.size(); i++){
        std::vector<std::vector<student::Point>> results;
        results = applyChanges(polygonsList[i], offset);
        bigPolygons.push_back(results[0]);
        smallPolygons.push_back(results[1]);
        results.clear();
    }

    Paths subj(1), clip(bigPolygons.size()-1), solution;

    for (unsigned int i = 0; i < bigPolygons[0].size(); i++){
        subj[0].push_back(IntPoint(bigPolygons[0][i].x*1000, bigPolygons[0][i].y*1000));
    }
    for(unsigned int i = 0; i < clip.size(); i++){
        for(unsigned int j = 0; j < bigPolygons[i+1].size(); j++){
            clip[i].push_back(IntPoint(bigPolygons[i+1][j].x*1000, bigPolygons[i+1][j].y*1000));
        }    
    }
    Clipper c;
    c.AddPaths(subj, ptSubject, true);
    c.AddPaths(clip, ptClip, true);
    c.Execute(ctUnion, solution, pftNonZero, pftNonZero);


    Paths subj1(1), clip1(smallPolygons.size()-1), solution1;

    for (unsigned int i = 0; i < smallPolygons[0].size(); i++){
        subj1[0].push_back(IntPoint(smallPolygons[0][i].x*1000, smallPolygons[0][i].y*1000));
    }
    for(unsigned int i = 0; i < clip1.size(); i++){
        for(unsigned int j = 0; j < smallPolygons[i+1].size(); j++){
            clip1[i].push_back(IntPoint(smallPolygons[i+1][j].x*1000, smallPolygons[i+1][j].y*1000));
        }    
    }
    Clipper c1;
    c1.AddPaths(subj1, ptSubject, true);
    c1.AddPaths(clip1, ptClip, true);
    c1.Execute(ctUnion, solution1, pftNonZero, pftNonZero);

    std::vector<std::vector<std::vector<student::Point>>> returnValues;

    std::vector<std::vector<student::Point>> intermediateValues;

    for (unsigned int i = 0; i < solution.size(); i++){
        Path path = solution.at(i);
        std::vector<student::Point> newPath;
        for(IntPoint p : path){
            newPath.push_back(student::Point(p.X, p.Y));
        }
        intermediateValues.push_back(newPath);
        newPath.clear();
    }

    returnValues.push_back(intermediateValues);

    intermediateValues.clear();

    for (unsigned int i = 0; i < solution1.size(); i++){
        Path path = solution1.at(i);
        std::vector<student::Point> newPath;
        for(IntPoint p : path){
            newPath.push_back(student::Point(p.X, p.Y));
        }
        intermediateValues.push_back(newPath);
        newPath.clear();
    }

    returnValues.push_back(intermediateValues);

    return returnValues;
}

