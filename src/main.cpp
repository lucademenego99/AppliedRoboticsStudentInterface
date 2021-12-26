#include "dubins.hpp"
#include "clipper_addons.hpp"
#include "graph.hpp"
#include "visgraph.hpp"
#include "open_edges.hpp"
#include "graphPrint.hpp"
#include "utils.hpp"
#include "clipper.hpp"
#include <iostream>

using namespace dubins;

void shortestPathDubinsTest(Dubins dubins);
void multipointDubinsAndVisgraphTest(Dubins dubins);
void intersectionsTest(Dubins dubins);
void openEdgesTest();

int main(int argc, char *argv[])
{
    Dubins dubins = Dubins(1.9, 0.005);

    // shortestPathDubinsTest(dubins);

    multipointDubinsAndVisgraphTest(dubins);

    // intersectionsTest(dubins);

    // openEdgesTest();
    
    return 0;
}

void openEdgesTest() {
    visgraph::OpenEdges openEdges = visgraph::OpenEdges();
    visgraph::Edge edgeNull = openEdges.getEdge(3);
    edgeNull.print();

    openEdges.insertEdge(visgraph::Point(1.0, 1.0), visgraph::Point(2.0, 2.0), visgraph::Edge(visgraph::Point(2.0, 2.0), visgraph::Point(3.0, 3.0)));

    visgraph::Edge edge = openEdges.getEdge(0);
    edge.print();
}

void shortestPathDubinsTest(Dubins dubins)
{
    std::cout << "SHORTEST PATH TEST\n";
    double i_x0 = 0.6;
    double i_y0 = 1;
    double i_th0 = M_PI / 4;
    double i_xf = 3;
    double i_yf = 3;
    double i_thf = -M_PI / 2;
    std::cout << "\nINPUT: \n";
    std::cout << "x0 = " << i_x0 << "\n"
              << "y0 = " << i_y0 << "\n"
              << "th0 = " << i_th0 << "\n"
              << "xf = " << i_xf << "\n"
              << "yf = " << i_yf << "\n"
              << "thf = " << i_thf << "\n\n";
    DubinsCurve *result = dubins.findShortestPath(i_x0, i_y0, i_th0, i_xf, i_yf, i_thf);
    if (!result)
    {
        std::cout << "RESULT NOT VALID\n";
    }
    else
    {
        std::cout << "RESULT: \n\n";
        std::cout << "L = " << result->L << "\n\n";
        std::cout << "a1 = \n"
                  << "\tL = " << result->a1->L << "\n"
                  << "\tk = " << result->a1->k << "\n"
                  << "\tx0 = " << result->a1->x0 << "\n"
                  << "\ty0 = " << result->a1->y0 << "\n"
                  << "\tth0 = " << result->a1->th0 << "\n"
                  << "\tdubins_line-x = " << result->a1->dubins_line->x << "\n"
                  << "\tdubins_line-y = " << result->a1->dubins_line->y << "\n"
                  << "\tdubins_line-th = " << result->a1->dubins_line->th << "\n\n";
        std::cout << "a2 = \n"
                  << "\tL = " << result->a2->L << "\n"
                  << "\tk = " << result->a2->k << "\n"
                  << "\tx0 = " << result->a2->x0 << "\n"
                  << "\ty0 = " << result->a2->y0 << "\n"
                  << "\tth0 = " << result->a2->th0 << "\n"
                  << "\tdubins_line-x = " << result->a2->dubins_line->x << "\n"
                  << "\tdubins_line-y = " << result->a2->dubins_line->y << "\n"
                  << "\tdubins_line-th = " << result->a2->dubins_line->th << "\n\n";
        std::cout << "a3 = \n"
                  << "\tL = " << result->a3->L << "\n"
                  << "\tk = " << result->a3->k << "\n"
                  << "\tx0 = " << result->a3->x0 << "\n"
                  << "\ty0 = " << result->a3->y0 << "\n"
                  << "\tth0 = " << result->a3->th0 << "\n"
                  << "\tdubins_line-x = " << result->a3->dubins_line->x << "\n"
                  << "\tdubins_line-y = " << result->a3->dubins_line->y << "\n"
                  << "\tdubins_line-th = " << result->a3->dubins_line->th << "\n\n";
    }
}

void multipointDubinsAndVisgraphTest(Dubins dubins)
{
    std::vector<double> tTmp;

    std::vector<std::vector<visgraph::Point>> polygons, polygonsForVisgraph;
    //std::vector<std::vector<visgraph::Point>> outerWalls;
    // std::vector<visgraph::Point> pol1 {visgraph::Point(1.5, 1.5), visgraph::Point(3.5, 1.5), visgraph::Point(5.0, 3.0), visgraph::Point(3.5, 4.5), visgraph::Point(1.5, 4.5), visgraph::Point(0.0, 3.0)};
    // std::vector<visgraph::Point> pol2 {visgraph::Point(5.0, 5.0), visgraph::Point(8.0, 5.0), visgraph::Point(8.0, 11.0), visgraph::Point(5.0, 11.0)};
    // std::vector<visgraph::Point> pol3 {visgraph::Point(12.0, 1.0), visgraph::Point(14.0, 4.0), visgraph::Point(12.0, 7.0), visgraph::Point(10.0, 4.0)};
    // std::vector<visgraph::Point> pol4 {visgraph::Point(18.0, 3.0), visgraph::Point(23.0, 7.0), visgraph::Point(12.0, 19.0), visgraph::Point(8.0, 14.0)};
    // visgraph::Point origin = visgraph::Point(1.0, 8.0);
    // visgraph::Point destination = visgraph::Point(24.0, 2.0);

    //The first four polygons are the walls
    std::vector<visgraph::Point> pol1 {visgraph::Point(1.0, 1.0), visgraph::Point(1.0, 22.0)};
    std::vector<visgraph::Point> pol2 {visgraph::Point(2.0, 1.0), visgraph::Point(30.0, 1.0)};
    std::vector<visgraph::Point> pol3 {visgraph::Point(30.0, 2.0), visgraph::Point(30.0, 22.0)};
    std::vector<visgraph::Point> pol4 {visgraph::Point(29.0, 22.0), visgraph::Point(2.0, 22.0)};

    std::vector<visgraph::Point> pol5 {visgraph::Point(7.0, 6.0), visgraph::Point(9.0, 6.0), visgraph::Point(9.0, 8.0), visgraph::Point(7.0, 8.0)};
    std::vector<visgraph::Point> pol6 {visgraph::Point(15.0, 8.0), visgraph::Point(22.0, 8.0), visgraph::Point(22.0, 12.0), visgraph::Point(15.0, 12.0)};
    std::vector<visgraph::Point> pol7 {visgraph::Point(7.0, 14.0), visgraph::Point(10.0, 14.0), visgraph::Point(10.0, 17.0), visgraph::Point(7.0, 17.0)};
    visgraph::Point origin = visgraph::Point(4.0, 4.0);
    visgraph::Point destination = visgraph::Point(29.0, 19.0);

    polygons.push_back(pol1);
    polygons.push_back(pol2);
    polygons.push_back(pol3);
    polygons.push_back(pol4);

    polygons.push_back(pol5);
    polygons.push_back(pol6);
    polygons.push_back(pol7);
    std::vector<std::vector<std::vector<visgraph::Point>>> pols = enlargeAndJoinObstacles(polygons, 0.5);

    //std::cout << "Pols.size()" << pols[0].size();
    for(int i = 1; i < pols[0].size(); i++){
        polygonsForVisgraph.push_back(pols[0][i]);
    }
    polygons = pols[1];

    visgraph::VisGraph visg;

    // COMPUTE VISIBILITY GRAPH
    visgraph::Graph originalGraph = visgraph::Graph(polygons, false, true);
    visgraph::Graph originalGraph2 = visgraph::Graph(polygonsForVisgraph, false, true);
    visgraph::Graph g = visg.computeVisibilityGraph(polygonsForVisgraph, origin, destination);

    // COMPUTE SHORTEST PATH
    std::cout << "SHORTEST PATH:\n";
    std::vector<visgraph::Point> path = g.shortestPath(origin, destination);
    for(int it = 0; it < path.size(); it++)
        path[it].print();


    printGraph(originalGraph.graph, origin, destination, path);
    printGraph(originalGraph2.graph, origin, destination, path);
    printGraph(g.graph, origin, destination, path);


    // COMPUTE MULTIPOINT DUBINS SHORTEST PATH
    std::cout << "MULTIPOINT SHORTEST PATH TEST\n";
    dubins::DubinsPoint **points = new dubins::DubinsPoint *[path.size()];
    points[0] = new dubins::DubinsPoint(path[0].x, path[0].y, 0);
    for(int i = 1; i < path.size()-1; i++) {
        points[i] = new dubins::DubinsPoint(path[i].x, path[i].y);
    }
    points[path.size()-1] = new dubins::DubinsPoint(path[path.size()-1].x, path[path.size()-1].y);

    DubinsCurve **curves = dubins.multipointShortestPath(points, path.size(), originalGraph);
    if (curves == nullptr) {
        std::cout << "UNABLE TO COMPUTE A PATH FOR GIVEN INPUT\n";
    } else {
        std::cout << "COMPLETED MULTIPOINT SHORTEST PATH SUCCESSFULLY\n";
        dubins.printCompletePath(curves, path.size()-1, polygons);
    }
}

void intersectionsTest(Dubins dubins)
{
    std::cout << "INTERSECTIONS TEST\n";
    std::vector<dubins::DubinsPoint> intersections = std::vector<dubins::DubinsPoint>();
    std::vector<double> ts = std::vector<double>();

    bool res = dubins.intersLineLine(dubins::DubinsPoint(1, 1), dubins::DubinsPoint(66.57, 0.367), dubins::DubinsPoint(56.01, 18.22), dubins::DubinsPoint(56.01, 0.067), intersections, ts);
    std::cout << "RES: " << res << "\n";
    if (res)
    {
        std::cout << "Intersections: \n";
        for (int i = 0; i < intersections.size(); i++)
        {
            std::cout << intersections[i].x << " ; " << intersections[i].y << "\n";
        }
        std::cout << "\nTs: \n";
        for (int i = 0; i < ts.size(); i++)
        {
            std::cout << ts[i] << "\n";
        }
    }

    // bool res = dubins.intersCircleLine(dubins::DubinsPoint(2, 2), 1, dubins::DubinsPoint(0, 0), dubins::DubinsPoint(4, 4), intersections, ts);
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

    // DubinsArc *arc = new DubinsArc(0, 0, -1.57, 1, 1.57);
    // std::cout << "ARC:\n"
    //           << arc->x0 << " , " << arc->y0 << " , " << arc->th0 << "\n";
    // std::cout << arc->dubins_line->x << " , " << arc->dubins_line->y << " , " << arc->dubins_line->th << "\n";
    // bool res = dubins.intersArcLine(arc, dubins::DubinsPoint(0, -1), dubins::DubinsPoint(1, 0), intersections, ts);
    // std::cout << "\n\nRES: " << res << "\n";
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
}
