/**
 * @file graphPrint.cpp
 * @brief Useful functions to print generated graphs using opencv
 * @version 0.1
 * @date 2022-01-06
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "graph.hpp" 
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "graphPrint.hpp"
#include "limits"
#include "cmath"

using namespace cv;
using namespace std;

/**
 * @brief Function to display an opencv window with the results of the visibility graph
 * 
 * @param g graph we want to display
 * @param origin starting point
 * @param destination destination point
 * @param shortestPath points that compose the shortest path
 */
void printGraph(std::map<visgraph::Point, std::vector<visgraph::Edge>> g, visgraph::Point origin, visgraph::Point destination, std::vector<visgraph::Point> shortestPath){
    std::map<visgraph::Point, std::vector<visgraph::Edge>>::iterator it;

    double smallestX = INFINITY, biggestX = -INFINITY, smallestY = INFINITY, biggestY = -INFINITY;
    // Loop through points and find the smallest and biggest values
    for (it = g.begin(); it != g.end(); it++) {
        visgraph::Point p = it->first;
        smallestX = p.x < smallestX ? p.x : smallestX;
        biggestX = p.x > biggestX ? p.x : biggestX;
        smallestY = p.y < smallestY ? p.y : smallestY;
        biggestY = p.y > biggestY ? p.y : biggestY;
    }
    smallestX = origin.x < smallestX ? origin.x : smallestX;
    smallestY = origin.y < smallestY ? origin.y : smallestY;
    biggestX = origin.x > biggestX ? origin.x : biggestX;
    biggestY = origin.y > biggestY ? origin.y : biggestY;
    double size = max(biggestX-smallestX, biggestY-smallestY);
    size *= 1.1;

    //Black color
    Mat image(500, 500, CV_8UC3, Scalar(0, 0, 0));
    Mat flipped;


    if(!image.data){
        cout << "Could not open of find the image";
        return;
    }

    int thickness = 1;
    for(it = g.begin(); it != g.end(); it++){
        circle(image, Point(it->first.x / size * 500, it->first.y / size * 500), 5, Scalar(255, 255, 255), FILLED, LINE_8);
        for(visgraph::Edge e : it->second){
            // Rescale the points so that we can clearly see them in the screen
            Point p1(e.p1.x / size * 500, e.p1.y / size * 500), p2(e.p2.x / size * 500, e.p2.y / size * 500);
            line(image, p1, p2, Scalar(0, 0, 255), thickness, LINE_AA);
        }
    }
    // Show origin in blue and destination in green
    circle(image, Point(origin.x / size * 500, origin.y / size * 500), 5, Scalar(252, 19, 3), FILLED, LINE_8);
    circle(image, Point(destination.x / size * 500, destination.y / size * 500), 5, Scalar(32, 252, 3), FILLED, LINE_8);

    // Show shortest path in green
    for(int i = 1; i < shortestPath.size(); i++) {
        Point p1(shortestPath[i-1].x / size * 500, shortestPath[i-1].y / size * 500), p2(shortestPath[i].x / size * 500, shortestPath[i].y / size * 500);
        line(image, p1, p2, Scalar(32, 252, 3), thickness, LINE_AA);
    }

    flip(image, flipped, 0);
    imshow("Output", flipped);
    cv::waitKey(0);
}