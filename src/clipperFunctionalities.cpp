#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "clipper.hpp"

using namespace ClipperLib;

Paths enlarge(IntPoint* points, int offset){
    Path subj;
    Paths solution;
    for(int i = 0; i < sizeof(points); i++){
        subj->push_back(points[i]);
    }
    ClipperOffset co;
    co.AddPath(subj, jtRound, etClosedPolygon);
    co.Execute(solution, offset)
    return solution;
}

void printSolution(Paths solution){
    printf("solution size = %d\n", (int)solutions.size());
    for(unsigned int i = 0; i < solution.size(); i++){
        Path path = solution.at(i);
        for(unsigned int j = 1; j < path.size(); j++){
            cv::line(plot, cv::Point2f(path.at(j-1).X, path.at(j-1).Y), cv::Points2f(path.at(j).X, path.at(j).Y), cv::Scalar(255,255,0), 2);
        }
    }
    cv::flip(plot, plot, 0);
    cv::imshow("Clipper", plot);
    cv::waitKey(0);
}