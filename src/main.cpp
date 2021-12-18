#include "dubins.hpp"
#include "clipper_addons.hpp"
#include "graph.hpp"
#include "visgraph.hpp"
#include "open_edges.hpp"
#include "graphPrint.hpp"
#include "utils.hpp"
#include "clipper.hpp"
#include <iostream>

using namespace student;

void shortestPathDubinsTest(Dubins dubins);
void multipointDubinsTest(Dubins dubins);
void multipointDubinsAndVisgraphTest(Dubins dubins);
void intersectionsTest(Dubins dubins);
void clipperTest();
void structuresTest();
void openEdgesTest();
void visgraphTest();
void clipperJoinAndEnlargeTest();

int main(int argc, char *argv[])
{
    Dubins dubins = Dubins(1.3, 0.005);

    // shortestPathDubinsTest(dubins);

    // multipointDubinsTest(dubins);

    multipointDubinsAndVisgraphTest(dubins);

    // intersectionsTest(dubins);

    // clipperTest();

    // structuresTest();

    // openEdgesTest();

    // visgraphTest();

    //Small test for the alternative intersection we might considers, since the other one is problematic
    // std::vector<student::Point> points;
    // std::vector<double> t;
    // Dubins dubin;
    // std::cout << dubin.intersLineLine(student::Point(450, 30), student::Point(330,700), student::Point(230,700), student::Point(330,700), points, t) << "\n";
    // for( student::Point p : points){
    //     std::cout << "X intersection: " << p.x << "\n";
    //     std::cout << "Y intersection: " << p.y << "\n";
    // }
    
    return 0;
}

void visgraphTest() {
    std::vector<std::vector<visgraph::Point>> polygons;

    // std::vector<visgraph::Point> pol1 {visgraph::Point(2.0, 1.0), visgraph::Point(3.0, 1.0), visgraph::Point(4.0, 2.0), visgraph::Point(3.0, 3.0), visgraph::Point(2.0, 3.0), visgraph::Point(1.0, 2.0), visgraph::Point(2.0, 1.0)};
    // std::vector<visgraph::Point> pol2 {visgraph::Point(1.0, 3.5), visgraph::Point(6.0, 6.0), visgraph::Point(1.0, 6.0), visgraph::Point(1.0, 3.5)};
    // std::vector<visgraph::Point> pol3 {visgraph::Point(7.0, 2.0), visgraph::Point(8.0, 3.0), visgraph::Point(7.0, 5.0), visgraph::Point(6.0, 3.0), visgraph::Point(7.0,2.0)};

    // std::vector<visgraph::Point> pol1 {visgraph::Point(1.0, 1.0), visgraph::Point(3.0, 1.0), visgraph::Point(4.0, 5.0), visgraph::Point(1.0, 4.0)};
    // std::vector<visgraph::Point> pol2 {visgraph::Point(1.0, 5.0), visgraph::Point(4.0, 5.5), visgraph::Point(4.0, 6.0), visgraph::Point(1.0, 5.5)};
    // std::vector<visgraph::Point> pol3 {visgraph::Point(6.0, 1.0), visgraph::Point(8.0, 1.0), visgraph::Point(9.0, 3.0), visgraph::Point(6.5, 5.0), visgraph::Point(5.0,3.0)};

    // std::vector<visgraph::Point> pol1 {visgraph::Point(1.0, 1.0), visgraph::Point(10.0, 0.3), visgraph::Point(4.0, 3.0), visgraph::Point(2.0, 2.5), visgraph::Point(1.0, 1.5)};
    // std::vector<visgraph::Point> pol2 {visgraph::Point(0.7, 2.0), visgraph::Point(2.0, 3.0), visgraph::Point(1.5, 4.5)};
    // std::vector<visgraph::Point> pol3 {visgraph::Point(2.0, 6.0), visgraph::Point(2.3, 7.0), visgraph::Point(3.3, 7.0), visgraph::Point(6.6, 8.0)};

    std::vector<visgraph::Point> pol1 {visgraph::Point(1.0, 1.0), visgraph::Point(4.0, 1.0), visgraph::Point(4.0, 3.0), visgraph::Point(1.0, 3.0)};
    std::vector<visgraph::Point> pol2 {visgraph::Point(1.0, 3.5), visgraph::Point(4.0, 3.5), visgraph::Point(4.0, 5.5), visgraph::Point(1.0, 5.5)};
    std::vector<visgraph::Point> pol3 {visgraph::Point(1.0, 6.0), visgraph::Point(4.0, 6.0), visgraph::Point(4.0, 8.0), visgraph::Point(1.0, 8.0)};
    std::vector<visgraph::Point> pol4 {visgraph::Point(5.0, 1.0), visgraph::Point(7.0, 1.0), visgraph::Point(7.0, 7.0), visgraph::Point(5.0, 7.0)};

    polygons.push_back(pol1);
    polygons.push_back(pol2);
    polygons.push_back(pol3);
    polygons.push_back(pol4);

    visgraph::Point origin = visgraph::Point(5,0.3);
    visgraph::Point destination = visgraph::Point(9, 2);

    visgraph::VisGraph visg = visgraph::VisGraph();

    visgraph::Graph g = visg.computeVisibilityGraph(polygons, origin, destination);

    std::cout << "SHORTEST PATH:\n";
    std::vector<visgraph::Point> path = g.shortestPath(origin, destination);
    for(int it = 0; it < path.size(); it++)
        path[it].print();
    
    printGraph(g.graph, origin, destination, path);
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
    // std::cout << "MULTIPOINT SHORTEST PATH TEST\n";
    // student::Point **points = new student::Point *[3];
    // points[0] = new student::Point(0.2, 0.2, 0);
    // points[1] = new student::Point(0.9, 0.8);
    // points[2] = new student::Point(1.4, 0.2, 0);
    // dubins.multipointShortestPath(points, 3);

    // student::Point **points = new student::Point *[5];
    // points[0] = new student::Point(0.2, 0.2, M_PI);
    // points[1] = new student::Point(0.9, 0.8);
    // points[2] = new student::Point(0.4, 0.5);
    // points[3] = new student::Point(0.7, 0.8);
    // points[4] = new student::Point(1.4, 0.2, 0);
    // dubins.multipointShortestPath(points, 5);

    // student::Point **points = new student::Point*[10];
    // points[0] = new student::Point(0.2, 0.2, M_PI);
    // points[1] = new student::Point(0.9, 0.8);
    // points[2] = new student::Point(0.4, 0.5);
    // points[3] = new student::Point(0.7, 0.8);
    // points[4] = new student::Point(0.9, 0.8);
    // points[5] = new student::Point(0.4, 0.5);
    // points[6] = new student::Point(0.7, 0.8);
    // points[7] = new student::Point(0.9, 0.8);
    // points[8] = new student::Point(0.4, 0.5);
    // points[9] = new student::Point(1.4, 0.2, 0);
    // dubins.multipointShortestPath(points, 10);

    // student::Point **points = new student::Point*[3];
    // points[0] = new student::Point(0.2, 0.2, 0);
    // points[1] = new student::Point(0.4, 0.2);
    // points[2] = new student::Point(0.6, 0.2, 0);
    // dubins.multipointShortestPath(points, 3);
}

void multipointDubinsAndVisgraphTest(Dubins dubins)
{
    std::vector<std::vector<visgraph::Point>> polygons;
    std::vector<visgraph::Point> pol1 {visgraph::Point(1.0, 1.0), visgraph::Point(4.0, 1.0), visgraph::Point(4.0, 3.0), visgraph::Point(1.0, 3.0)};
    // std::vector<visgraph::Point> pol2 {visgraph::Point(1.0, 3.5), visgraph::Point(4.0, 3.5), visgraph::Point(4.0, 5.5), visgraph::Point(1.0, 5.5)};
    // std::vector<visgraph::Point> pol3 {visgraph::Point(1.0, 6.0), visgraph::Point(4.0, 6.0), visgraph::Point(4.0, 8.0), visgraph::Point(1.0, 8.0)};
    // std::vector<visgraph::Point> pol4 {visgraph::Point(5.0, 1.0), visgraph::Point(7.0, 1.0), visgraph::Point(7.0, 7.0), visgraph::Point(5.0, 7.0)};
    // std::vector<visgraph::Point> pol1 {visgraph::Point(2.0, 1.0), visgraph::Point(3.0, 1.0), visgraph::Point(4.0, 2.0), visgraph::Point(3.0, 3.0), visgraph::Point(2.0, 3.0), visgraph::Point(1.0, 2.0), visgraph::Point(2.0, 1.0)};
    // std::vector<visgraph::Point> pol2 {visgraph::Point(1.0, 3.5), visgraph::Point(6.0, 6.0), visgraph::Point(1.0, 6.0), visgraph::Point(1.0, 3.5)};
    // std::vector<visgraph::Point> pol3 {visgraph::Point(7.0, 2.0), visgraph::Point(8.0, 3.0), visgraph::Point(7.0, 5.0), visgraph::Point(6.0, 3.0), visgraph::Point(7.0,2.0)};
    polygons.push_back(pol1);
    // polygons.push_back(pol2);
    // polygons.push_back(pol3);
    // polygons.push_back(pol4);

    visgraph::Point origin = visgraph::Point(9, 2);
    visgraph::Point destination = visgraph::Point(2, 5.7);

    std::vector<student::Point> pointsToEnlargePol1;
    std::cout << "BEGINNING POINTS:\n";
    for (visgraph::Point p : pol1) {
        p.print();
        pointsToEnlargePol1.push_back(student::Point(p.x*100, p.y*100));
    }
    std::cout << "END BEGINNING POINTS\n";
    std::vector<student::Point> newPointsEnlarged = enlarge(pointsToEnlargePol1, 1);
    std::vector<visgraph::Point> enlargedPol1;
    std::cout << "END POINTS:\n";
    for (student::Point p : newPointsEnlarged) {
        std::cout << p.x << "," << p.y << "\n";
        enlargedPol1.push_back(visgraph::Point(p.x, p.y));
    }
    std::cout << "END END POINTS\n";

    // visgraph::Point origin = visgraph::Point(0, 2);
    // visgraph::Point destination = visgraph::Point(1, 3);

    visgraph::VisGraph visg = visgraph::VisGraph();

    visgraph::Graph originalGraph = visgraph::Graph(polygons, false, true);

    visgraph::Graph g = visg.computeVisibilityGraph(polygons, origin, destination);

    std::cout << "SHORTEST PATH:\n";
    std::vector<visgraph::Point> path = g.shortestPath(origin, destination);
    for(int it = 0; it < path.size(); it++)
        path[it].print();


    printGraph(g.graph, origin, destination, path);

    std::cout << "MULTIPOINT SHORTEST PATH TEST\n";
    student::Point **points = new student::Point *[path.size()];
    points[0] = new student::Point(path[0].x, path[0].y, -M_PI_2);
    for(int i = 1; i < path.size()-1; i++) {
        points[i] = new student::Point(path[i].x, path[i].y);
    }
    points[path.size()-1] = new student::Point(path[path.size()-1].x, path[path.size()-1].y);

    // DubinsCurve *curve = dubins.findShortestPath(points[0]->x, points[0]->y, points[0]->th, points[path.size()-1]->x,points[path.size()-1]->y,points[path.size()-1]->th);
    // dubins.printDubinsCurve(curve);

    DubinsCurve **curves = dubins.multipointShortestPath(points, path.size(), originalGraph);
    if (curves == nullptr) {
        std::cout << "UNABLE TO COMPUTE A PATH FOR GIVEN INPUT\n";
    } else {
        std::cout << "COMPLETED MULTIPOINT SHORTEST PATH SUCCESSFULLY\n";
        dubins.printCompletePath(curves, path.size()-1);
    }
    // student::DubinsCurve *curve = dubins.findShortestPathCollisionDetection(7.0, 1.0, M_PI, origin.x, origin.y, 0, originalGraph);
    // if (curve == nullptr) {
    //     std::cout << "NOT FOUND!\n";
    // } else {
    //     dubins.printDubinsCurve(curve);
    // }
}

void intersectionsTest(Dubins dubins)
{
    std::cout << "INTERSECTIONS TEST\n";
    std::vector<student::Point> intersections = std::vector<student::Point>();
    std::vector<double> ts = std::vector<double>();

    bool res = dubins.intersLineLine(student::Point(1, 1), student::Point(66.57, 0.367), student::Point(56.01, 18.22), student::Point(56.01, 0.067), intersections, ts);
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

    // bool res = dubins.intersCircleLine(student::Point(2, 2), 1, student::Point(0, 0), student::Point(4, 4), intersections, ts);
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
    // bool res = dubins.intersArcLine(arc, student::Point(0, -1), student::Point(1, 0), intersections, ts);
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
    // std::cout << "CLIPPER TEST\n";
    // std::vector<ClipperLib::IntPoint> points;
    // points.push_back(ClipperLib::IntPoint(348, 257));
    // points.push_back(ClipperLib::IntPoint(364, 148));
    // points.push_back(ClipperLib::IntPoint(362, 148));
    // points.push_back(ClipperLib::IntPoint(326, 241));
    // points.push_back(ClipperLib::IntPoint(295, 219));
    // points.push_back(ClipperLib::IntPoint(258, 88));
    // points.push_back(ClipperLib::IntPoint(440, 129));
    // points.push_back(ClipperLib::IntPoint(370, 196));
    // points.push_back(ClipperLib::IntPoint(372, 275));

    // ClipperLib::Paths paths = enlarge(points, 7.0);
    // printSolution(points, paths);
}

void clipperJoinAndEnlargeTest(){
    // std::vector<IntPoint> points;
    // points.push_back(ClipperLib::IntPoint(150, 110));
    // points.push_back(ClipperLib::IntPoint(150, 200));
    // points.push_back(ClipperLib::IntPoint(360, 200));
    // points.push_back(ClipperLib::IntPoint(400, 110));
    // points.push_back(ClipperLib::IntPoint(150, 110));

    // std::vector<std::vector<IntPoint>> polygons;
    // polygons.push_back(points);

    // std::vector<IntPoint> points1;
    // points1.push_back(ClipperLib::IntPoint(10, 10));
    // points1.push_back(ClipperLib::IntPoint(80, 80));
    // points1.push_back(ClipperLib::IntPoint(150, 80));
    // points1.push_back(ClipperLib::IntPoint(10, 10));

    // polygons.push_back(points1);

    // std::vector<IntPoint> points2;
    // points2.push_back(ClipperLib::IntPoint(410, 410));
    // points2.push_back(ClipperLib::IntPoint(320, 320));
    // points2.push_back(ClipperLib::IntPoint(330, 320));
    // points2.push_back(ClipperLib::IntPoint(410, 410));

    // polygons.push_back(points2);

    // std::vector<Paths> enlargedPolygons;

    // enlargedPolygons = joinAndEnlarge(polygons);
}