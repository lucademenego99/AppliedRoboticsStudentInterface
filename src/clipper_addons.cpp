#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "clipper.hpp"
#include <iostream>

#include "clipper_addons.hpp"

using namespace ClipperLib;

/**
 * @brief Clipper Polygon Offsetting helper function
 * 
 * @param points Points of the original polygon
 * @param offset Offset that must be used to enlarge or shrink the polygon
 * @return Paths Array of resulting polygons
 */
Paths enlarge(std::vector<IntPoint> points, int offset)
{
    Path subj;
    Paths solution;
    for (int i = 0; i < sizeof(points); i++)
    {
        subj.push_back(points[i]);
    }
    ClipperOffset co;
    co.AddPath(subj, jtRound, etClosedPolygon);
    co.Execute(solution, offset);
    return solution;
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
    for (int i = 0; i < sizeof(startingPoints); i++)
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
 /*
bool intersect (IntPoint *subj, IntPoint *clip){
    Paths firstPoly;
    for (int i = 0; i < sizeof(subj); i++){
        firstPoly << IntPoint(subj[i].X, subj[i].Y);
    }

    Paths secondPoly;
    for (int j = 0; j < sizeof(clip); j++){
        secondPoly.push_back(clip[i]);
    }

    ClipperLib::Clipper c;
    //Need to make this function take as input points instead of paths, create paths
    c.AddPaths(subj, ClipperLib::ptSubject, true);
    c.AddPaths(clip, ClipperLib::ptClip, true);

    ClipperLib::Paths solution;
    c.Execute(ClipperLib::ctIntersection, solution, ClipperLib::pftNonZero, ClipperLib::pftNonZero);

    return solution.size()!=0;
}

void verifyAndJoin (IntPoint *firstPoly, IntPoint *secondPoly){
    //Intersect need to take as input points instead of Paths
    if(intersect(firstPoly, secondPoly)){
        Clipper c = new Clipper();
        
        Polygons subj = new Polygons(1);
        subj.Add(new Polygon(firstPoly.size()));
        for (int i = 0; i < sizeof(firstPoly); i++){
            subj[0].Add(firstPoly[i]);
        }

        Polygons clip = new Polygons(1);
        clip.Add(new Polygon(secondPoly.size()));
        for (int j = 0; j < sizeof(secondPoly); j++){
            clip[0].push_back(secondPoly[j]);
        }

        Polygons solution = new Polygons();
        c.addPolygons(subj, PolyType.ptSubject);
        c.addPolygons(clip, PolyType.ptClip);
        c.Execute(ClipType.ctUnion, solution, PolyFillType.pftEvenOdd, PolyFillType.pftNonZero);
    }
}
*/

/**
 * @brief Verifies if a list of polygons has some intersections, if so joins them and then enlarges them all
 * 
 * @param points Matrix of polygons
 */
std::vector<Paths> joinAndEnlarge (std::vector<std::vector<IntPoint>> points){
    Paths subj(1), clip(points.size()-1), solution;

    cv::Mat plot(500, 500, CV_8UC3, cv::Scalar(255, 255, 255));

    for (unsigned int i = 0; i < points[0].size(); i++){
        subj[0].push_back(points[0][i]);
    }
    for(unsigned int i = 0; i < clip.size(); i++){
        for(unsigned int j = 0; j < points[i+1].size(); j++){
            clip[i].push_back(points[i+1][j]);
        }    
    }
    Clipper c;
    c.AddPaths(subj, ptSubject, true);
    c.AddPaths(clip, ptClip, true);
    c.Execute(ctUnion, solution, pftNonZero, pftNonZero);
    
    std::vector<Paths> enlargedPolygons;
    int offset = 4; //Modify the offset of the enlargement process

    for (unsigned int i = 0; i < solution.size(); i++)
    {
        Path path = solution.at(i);
        enlargedPolygons.push_back(enlarge(path, offset));
        
    }
    return enlargedPolygons;
    
    /*
    for (unsigned int i = 0; i < solution.size(); i++)
    {
        Path path = solution.at(i);
        cv::line(plot, cv::Point2f(path.at(path.size() - 1).X, path.at(path.size() - 1).Y), cv::Point2f(path.at(0).X, path.at(0).Y), cv::Scalar(255, 255, 0), 2);
        for (unsigned int j = 1; j < path.size(); j++)
        {
            cv::line(plot, cv::Point2f(path.at(j - 1).X, path.at(j - 1).Y), cv::Point2f(path.at(j).X, path.at(j).Y), cv::Scalar(255, 255, 0), 2);
        }
    }
    cv::flip(plot, plot, 0);
    cv::imshow("Clipper", plot);
    cv::waitKey(0);
    */
    
}
