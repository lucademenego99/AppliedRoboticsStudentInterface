#include "../include/dubins.hpp"
#include <iostream>

int main(int argc, char *argv[])
{
    Dubins dubins = Dubins(10, 0.005);

    /**
     * SHORTEST PATH TESTS
     * 
     */
    // double i_x0 = 0.6;
    // double i_y0 = 1;
    // double i_th0 = M_PI / 4;
    // double i_xf = 3;
    // double i_yf = 3;
    // double i_thf = -M_PI / 2;
    // std::cout << "\nINPUT: \n";
    // std::cout << "x0 = " << i_x0 << "\n"
    //           << "y0 = " << i_y0 << "\n"
    //           << "th0 = " << i_th0 << "\n"
    //           << "xf = " << i_xf << "\n"
    //           << "yf = " << i_yf << "\n"
    //           << "thf = " << i_thf << "\n\n";
    // DubinsCurve *result = dubins.findShortestPath(i_x0, i_y0, i_th0, i_xf, i_yf, i_thf);
    // if (!result) {
    //     std::cout << "RESULT NOT VALID\n";
    //     return 1;
    // }
    // std::cout << "RESULT: \n\n";
    // std::cout << "L = " << result->L << "\n\n";
    // std::cout << "a1 = \n"
    //           << "\tL = " << result->a1->L << "\n"
    //           << "\tk = " << result->a1->k << "\n"
    //           << "\tx0 = " << result->a1->x0 << "\n"
    //           << "\ty0 = " << result->a1->y0 << "\n"
    //           << "\tth0 = " << result->a1->th0 << "\n"
    //           << "\tdubins_line-x = " << result->a1->dubins_line->x << "\n"
    //           << "\tdubins_line-y = " << result->a1->dubins_line->y << "\n"
    //           << "\tdubins_line-th = " << result->a1->dubins_line->th << "\n\n";
    // std::cout << "a2 = \n"
    //           << "\tL = " << result->a2->L << "\n"
    //           << "\tk = " << result->a2->k << "\n"
    //           << "\tx0 = " << result->a2->x0 << "\n"
    //           << "\ty0 = " << result->a2->y0 << "\n"
    //           << "\tth0 = " << result->a2->th0 << "\n"
    //           << "\tdubins_line-x = " << result->a2->dubins_line->x << "\n"
    //           << "\tdubins_line-y = " << result->a2->dubins_line->y << "\n"
    //           << "\tdubins_line-th = " << result->a2->dubins_line->th << "\n\n";
    // std::cout << "a3 = \n"
    //           << "\tL = " << result->a3->L << "\n"
    //           << "\tk = " << result->a3->k << "\n"
    //           << "\tx0 = " << result->a3->x0 << "\n"
    //           << "\ty0 = " << result->a3->y0 << "\n"
    //           << "\tth0 = " << result->a3->th0 << "\n"
    //           << "\tdubins_line-x = " << result->a3->dubins_line->x << "\n"
    //           << "\tdubins_line-y = " << result->a3->dubins_line->y << "\n"
    //           << "\tdubins_line-th = " << result->a3->dubins_line->th << "\n\n";

    /**
     * MULTIPOINT DUBINS TESTS
     * 
     */
    // Point **points = new Point *[3];
    // points[0] = new Point(0.2, 0.2, 0);
    // points[1] = new Point(0.9, 0.8);
    // points[2] = new Point(1.4, 0.2, 0);
    // dubins.multipointShortestPath(points, 3);

    // Point **points = new Point *[5];
    // points[0] = new Point(0.2, 0.2, M_PI);
    // points[1] = new Point(0.9, 0.8);
    // points[2] = new Point(0.4, 0.5);
    // points[3] = new Point(0.7, 0.8);
    // points[4] = new Point(1.4, 0.2, 0);
    // dubins.multipointShortestPath(points, 5);

    // Point **points = new Point*[10];
    // points[0] = new Point(0.2, 0.2, M_PI);
    // points[1] = new Point(0.9, 0.8);
    // points[2] = new Point(0.4, 0.5);
    // points[3] = new Point(0.7, 0.8);
    // points[4] = new Point(0.9, 0.8);
    // points[5] = new Point(0.4, 0.5);
    // points[6] = new Point(0.7, 0.8);
    // points[7] = new Point(0.9, 0.8);
    // points[8] = new Point(0.4, 0.5);
    // points[9] = new Point(1.4, 0.2, 0);
    // dubins.multipointShortestPath(points, 10);

    // Point **points = new Point*[3];
    // points[0] = new Point(0.2, 0.2, 0);
    // points[1] = new Point(0.4, 0.2);
    // points[2] = new Point(0.6, 0.2, 0);
    // dubins.multipointShortestPath(points, 3);

    /**
     * INTERSECTIONS TESTS
     * 
     */
    // std::vector<Point> intersections = std::vector<Point>();
    // std::vector<double> ts = std::vector<double>();

    // bool res = dubins.intersLineLine(Point(1, 1), Point(66.57, 0.367), Point(56.01, 18.22), Point(56.01, 0.067), intersections, ts);
    // std::cout << "RES: " << res << "\n";
    // if (res)
    // {
    //     std::cout << "Intersections: \n";
    //     for (int i = 0; i < intersections.size(); i++)
    //     {
    //         std::cout << intersections[i].x << " ; " << intersections[i].y << "\n";
    //     }
    //     std::cout << "\nTs: \n";
    //     for (int i = 0; i < ts.size(); i++)
    //     {
    //         std::cout << ts[i] << "\n";
    //     }
    // }

    // bool res = dubins.intersCircleLine(Point(2, 2), 1, Point(0, 0), Point(4, 4), intersections, ts);
    // std::cout << "RES: " << res << "\n";
    // if (res)
    // {
    //     std::cout << "Intersections: \n";
    //     for (int i = 0; i < intersections.size(); i++)
    //     {
    //         std::cout << intersections[i].x << " ; " << intersections[i].y << "\n";
    //     }
    //     std::cout << "\nTs: \n";
    //     for (int i = 0; i < ts.size(); i++)
    //     {
    //         std::cout << ts[i] << "\n";
    //     }
    // }

    return 0;
}
