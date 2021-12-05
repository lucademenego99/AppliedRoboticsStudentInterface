#include <iostream>
#include <map>
#include <set>
#include <algorithm>
#include <vector>
#include <queue>
#include "shortest_path.hpp"

namespace visgraph
{
    MyPoint::MyPoint(const Point p,double dist): p(p), dist(dist){}

    bool MyPoint::operator<(const struct MyPoint & right)const {  //
        if(this->p == right.p)     //do not add the same point
            return false;
        else {
            if(this->dist != right.dist) {
                return this->dist < right.dist;      //the smallest set
            } else {
                return this->p < right.p;     
            }
        }
    }

    bool MyPoint::operator==(const struct MyPoint & right)const {  
        if(this->p == right.p)   
            return true;
        else
            return false;
    }

    double edge_distance(Point p1, Point p2) {
    // Return the Euclidean distance between two Points.
        return sqrt((p2.x - p1.x)*(p2.x - p1.x) + (p2.y - p1.y)*(p2.y - p1.y));
    }

    //def dijkstra(graph, origin, destination, add_to_visgraph):
    //             DictG, Point, Point, DictG
    std::map<Point, Point> dijkstra(DictG graph, Point origin, Point destination) { //, DictG add_to_visgraph) {
        std::map<Point, double> dist;
        std::map<Point, Point> prev;
        double d;
        double elength;

        std::set<MyPoint> q; /* priotity queue node:(Point,weight) and rank by weight=distance, maybe there be some optimized functions
                                    which can both find and replace the parameter and at the same time remains log time sort*/
        std::set<MyPoint>::iterator it1;
        std::set<MyPoint>::iterator it2;
        q.insert(MyPoint(origin,0.0));

        while(!q.empty()) {
            it1 = q.begin();
            Point v = it1->p;
            double d = it1->dist;
            dist.insert(std::pair<Point, double>(v, d));
            // v.print();
   	    // std::cout << "dist = " << dist.find(v)->second << std::endl;
            if(v == destination) {
                break;
            }

            std::vector<Edge> edges = graph.find(v)->second;   // ***get the DictG for the vertex in all graph, then return the edges***
            // if (add_to_visgraph != NULL && add_to_visgraph.find(v)!= add_to_visgraph.end()) {
            //     edges = add_to_visgraph.find(v)->second | graph.find(v)->second;
            // }


            for(std::size_t i = 0; i < edges.size(); ++i) {
                Point w = edges[i].getAdjacent(v);
                elength = dist.find(v)->second + edge_distance(v, w);
                it2 = q.find(MyPoint(w,elength));
                if (dist.find(w) != dist.end()){
                    if(elength < dist.find(w)->second){
                        // error, throw ValueError;
                    }
                }
                else if (it2 == q.end() || elength < it2->dist) {
                    // w.print();
		    // std::cout << "elength = " << elength << ", dist = "<< it2->dist << std::endl;
                    if(it2 != q.end()) q.erase(it2);
                    q.insert(MyPoint(w,elength));
                    prev.erase(w);
                    prev.insert(std::pair<Point, Point>(w,v));
                }
            }
            q.erase(it1);
        }
        return prev;
    }

    std::vector<Point> shortestPath(DictG graph, Point origin, Point destination) { //, DictG add_to_visgraph) {
        std::map<Point, Point> p = dijkstra(graph, origin, destination); //, add_to_visgraph);
        std::vector<Point> path;
        
        while(true){
            path.push_back(destination);
            if(destination == origin) break;
            destination = p.find(destination)->second;
        }  

        reverse(path.begin(), path.end());

        // for(it=path.rbegin(); it!=path.rend(); ++it)
        //     rpath.push_back(it);

        return path;
    }
} 




    

