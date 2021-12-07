#include "dubins.hpp"
#include "clipper_addons.hpp"
#include "graph.hpp"
#include "visgraph.hpp"
#include "open_edges.hpp"
#include "graphPrint.hpp"
#include <iostream>

using namespace student;

void shortestPathDubinsTest(Dubins dubins);
void multipointDubinsTest(Dubins dubins);
void intersectionsTest(Dubins dubins);
void clipperTest();
void structuresTest();
void openEdgesTest();
void visgraphTest();
void shortestPathTest();

int main(int argc, char *argv[])
{
    //Dubins dubins = Dubins(10, 0.005);

    // shortestPathDubinsTest(dubins);

    // multipointDubinsTest(dubins);

    // intersectionsTest(dubins);

    // clipperTest();

    // structuresTest();

    // openEdgesTest();

    //visgraphTest();

    shortestPathTest();

    return 0;
}

void shortestPathTest() {
    std::vector<std::vector<visgraph::Point>> polygons;

    std::vector<visgraph::Point> pol1 {visgraph::Point(1.0, 1.0), visgraph::Point(1.0, 3.0), visgraph::Point(3.0,3.0), visgraph::Point(1.0, 1.0)};
    std::vector<visgraph::Point> pol2 {visgraph::Point(5.0, 5.0), visgraph::Point(7.0, 7.0), visgraph::Point(7.0,5.0), visgraph::Point(7.0, 7.0)};
    std::vector<visgraph::Point> pol3 {visgraph::Point(1.0, 7), visgraph::Point(3.0, 9.5), visgraph::Point(3.0, 6), visgraph::Point(1.0, 7)};

    polygons.push_back(pol1);
    polygons.push_back(pol2);
    polygons.push_back(pol3);

    visgraph::VisGraph visg = visgraph::VisGraph();

    visgraph::Graph g = visg.computeVisibilityGraph(polygons, visgraph::Point(0.0, 0.0), visgraph::Point(8.0, 6.0));

    printGraph(g.graph);

    std::vector<visgraph::Point> path = visg.shortest_path(g.graph, visgraph::Point(0.0, 0.0), visgraph::Point(8.0, 6.0));
    for(int it = 0; it < path.size(); it++)
        path[it].print();
}

void visgraphTest() {
    std::vector<std::vector<visgraph::Point>> polygons;

    std::vector<visgraph::Point> pol1 {visgraph::Point(1.0, 1.0), visgraph::Point(3.0, 3.0), visgraph::Point(3.0,1.0), visgraph::Point(1.0, 1.0)};
    std::vector<visgraph::Point> pol2 {visgraph::Point(5.0, 5.0), visgraph::Point(7.0, 7.0), visgraph::Point(7.0,5.0), visgraph::Point(5.0, 5.0)};

    polygons.push_back(pol1);
    polygons.push_back(pol2);

    visgraph::VisGraph visg = visgraph::VisGraph();

    visgraph::Graph g = visg.computeVisibilityGraph(polygons, visgraph::Point(0, 0), visgraph::Point(8, 6));

    for (visgraph::Point p : g.getPoints()) {
        p.print();
    }
    std::cout << "\n";
    for (visgraph::Edge e : g.getEdges()) {
        e.print();
    }
    std::cout << "\n";
}

void openEdgesTest() {
    visgraph::OpenEdges openEdges = visgraph::OpenEdges();
    visgraph::Edge edgeNull = openEdges.getEdge(3);
    edgeNull.print();

    openEdges.insertEdge(visgraph::Point(1.0, 1.0), visgraph::Point(2.0, 2.0), visgraph::Edge(visgraph::Point(2.0, 2.0), visgraph::Point(3.0, 3.0)));

    visgraph::Edge edge = openEdges.getEdge(0);
    edge.print();
}

void structuresTest(){
    std::cout << "STRUCTURES TEST\n";
    visgraph::Point p = visgraph::Point(5, 6, 1);
    std::cout << "Point: \n";
    std::cout << "x = " << p.x << "\n"
              << "y = " << p.y << "\n"
              << "polygon id = " << p.polygonId << "\n";
    visgraph::Point p2 = visgraph::Point(10, 5, 2);
    visgraph::Edge e = visgraph::Edge(p, p2);
    std::cout << "x = " << e.p1.x << "\n"
              << "y = " << e.p1.y << "\n"
              << "polygon id = " << e.p1.polygonId << "\n";
    std::cout << "x = " << e.p2.x << "\n"
              << "y = " << e.p2.y << "\n"
              << "polygon id = " << e.p2.polygonId << "\n";
    visgraph::Point p3 = visgraph::Point(5, 6, 1);
    bool value = p3 == p;
    std::cout << "1: Equals : " << value << "\n";
    value = p3 == p2;
    std::cout << "2: Equals : " << value << "\n";
    value = !(p3 == p2);
    std::cout << "3: Equals : " << value << "\n";

    p.print();

    visgraph::Edge e2 = visgraph::Edge(p3, p2);
    visgraph::Point p5 = e2.getAdjacent(p2);
    p5.print();

    value = e2.contains(p3);
    std::cout << "E2 contains E3? " << value << "\n";

    value = e2 == e;
    std::cout << "E2 eq E? " << value << "\n";
    value = !(e2 == e);
    std::cout << "E2 notEq E? " << value << "\n";

    e2.print();
    std::vector<visgraph::Point> points;
    points.push_back(p);
    points.push_back(p2);
    points.push_back(p3);
    points.push_back(p5);

    std::vector<visgraph::Point> points2;
    points2.push_back(p3);
    points2.push_back(p5);
    points2.push_back(p);
    points2.push_back(p2);
    std::vector<std::vector<visgraph::Point>> shapes;
    shapes.push_back(points);
    shapes.push_back(points2);
    visgraph::Graph graph = visgraph::Graph(shapes);
    std::vector<visgraph::Point> results = graph.getAdjacentPoints(p2);
    /*for(Point p : results){
        std::cout << "Get Adjacent: " << p.x << " + " << p.y << " + " << p.polygon_id << "\n";
    }*/
    results.clear();
    results = graph.getPoints();
    /*for(Point p : results){
        std::cout << "Get points: " << p.x << " + " << p.y << " + " << p.polygon_id << "\n";
    }*/
    std::vector<visgraph::Edge> edges = graph.getEdges();
    /*for(Edge e : edges){
        e.print();
    }*/
    edges.clear();
    edges = graph.getItems(p);
    /*for(Edge e : edges){
        e.print();
    }*/
    edges.clear();
    edges = graph.containsP(p2);
    /*for(Edge e : edges){
        e.print();
    }*/
    visgraph::Edge result = graph.containsE(e);
    result.print();
}
/*
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

void multipointDubinsTest(Dubins dubins)
{
    std::cout << "MULTIPOINT SHORTEST PATH TEST\n";
    Point **points = new Point *[3];
    points[0] = new Point(0.2, 0.2, 0);
    points[1] = new Point(0.9, 0.8);
    points[2] = new Point(1.4, 0.2, 0);
    dubins.multipointShortestPath(points, 3);

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
}

void intersectionsTest(Dubins dubins)
{
    std::cout << "INTERSECTIONS TEST\n";
    std::vector<Point> intersections = std::vector<Point>();
    std::vector<double> ts = std::vector<double>();

    bool res = dubins.intersLineLine(Point(1, 1), Point(66.57, 0.367), Point(56.01, 18.22), Point(56.01, 0.067), intersections, ts);
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

    // DubinsArc *arc = new DubinsArc(0, 0, -1.57, 1, 3.1416);
    // std::cout << "ARC:\n"
    //           << arc->x0 << " , " << arc->y0 << " , " << arc->th0 << "\n";
    // std::cout << arc->dubins_line->x << " , " << arc->dubins_line->y << " , " << arc->dubins_line->th << "\n";
    // bool res = dubins.intersArcLine(arc, Point(0, -0.5), Point(3, 1), intersections, ts);
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

void clipperTest()
{
    std::cout << "CLIPPER TEST\n";
    ClipperLib::IntPoint *points = new ClipperLib::IntPoint[9];
    points[0] = ClipperLib::IntPoint(348, 257);
    points[1] = ClipperLib::IntPoint(364, 148);
    points[2] = ClipperLib::IntPoint(362, 148);
    points[3] = ClipperLib::IntPoint(326, 241);
    points[4] = ClipperLib::IntPoint(295, 219);
    points[5] = ClipperLib::IntPoint(258, 88);
    points[6] = ClipperLib::IntPoint(440, 129);
    points[7] = ClipperLib::IntPoint(370, 196);
    points[8] = ClipperLib::IntPoint(372, 275);

    ClipperLib::Paths paths = enlarge(points, 7.0);
    printSolution(points, paths);
}
*/

