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

int printGraph(std::map<visgraph::Point, std::vector<visgraph::Edge>> g){
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
    double size = max(biggestX-smallestX, biggestY-smallestY);

    //Black color
    Mat image(500, 500, CV_8UC3, Scalar(0, 0, 0));
    Mat flipped;


    if(!image.data){
        cout << "Could not open of find the image";
        return 0;
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
    flip(image, flipped, 0);
    imshow("Output", flipped);
    cv::waitKey(0);

    return 0;
}