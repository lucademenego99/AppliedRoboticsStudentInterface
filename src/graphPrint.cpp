#include "graph.hpp" 
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "graphPrint.hpp"

using namespace cv;
using namespace std;

int printGraph(std::map<visgraph::Point, std::vector<visgraph::Edge>> g){
    //Black color
    Mat image(1000, 1000, CV_8UC3, Scalar(0, 0, 0));


    if(!image.data){
        cout << "Could not open of find the image";
        return 0;
    }

    int thickness = 1;
    std::map<visgraph::Point, std::vector<visgraph::Edge>>::iterator it;
    for(it = g.begin(); it != g.end(); it++){
        for(visgraph::Edge e : it->second){
            Point p1(e.p1.x, e.p1.y), p2(e.p2.x, e.p2.y);
            line(image, p1, p2, Scalar(0, 0, 255), thickness, LINE_AA);

        }
    }
    imshow("Output", image);
    cv::waitKey(0);

    return 0;
}