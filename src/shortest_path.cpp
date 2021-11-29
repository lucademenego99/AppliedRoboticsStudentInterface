#include <iostream>
#include <map>
#include <algorithm>
#include <vector>
#include <queue>
#include "graph.hpp"

namespace visgraph
{
    bool compare(const std::pair<Point, int>& a, const std::pair<Point, int>& b) {
    if(a.second == b.second) return a.first < b.first;
    else return a.second < b.second;
}

    //def dijkstra(graph, origin, destination, add_to_visgraph):
    //             DictG, Point, Point, DictG
    std::map<Point, Point> dijkstra(DictG graph, Point origin, Point destination) { //, DictG add_to_visgraph) {
        std::map<Point, int> dist;
        std::map<Point, Point> prev;
        std::map<Point, int>::iterator it;
        int d;
        float elength;

        std::map<Point, int> q; /* priotity queue node:(Point,weight) and rank by weight=distance, maybe there be some optimized functions
                                    which can both find and replace the parameter and at the same time remains log time sort*/
        
        q.insert(std::pair<Point, int>(origin,0));
        sort(q.begin(), q.end(), compare);

        while(!q.empty()) {
            Point v = q.begin()->first;
            int d = q.begin()->second;
            dist.insert(std::pair<Point, int>(v, d));
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
                if (dist.find(w) != dist.end()){
                    if(elength < dist.find(w)->second){
                        throw ValueError;
                    }
                }
                else if (dist.find(w) == dist.end() || (q.find(w) != q.end() && elength < q.find(w)->second)) {
                    q.erase(w);
                    q.insert(std::pair<Point, int>(w,elength));
                    prev.erase(w);
                    prev.insert(std::pair<Point, Point>(w,v));
                }

            sort(q.begin(), q.end(), compare);
        }
        return prev;
    }

    std::vector<Point> shortest_path(DictG graph, Point origin, Point destination, DictG add_to_visgraph) {
        p = dijkstra(graph, origin, destination, add_to_visgraph);
        std::vector<Point> path;
        std::vector<Point> rpath;
        
        while(true){
            path.push_back(destination);
            if(destination == origin) break;
            destination = p.find(destination)->second;
        }  

        for (it=path.rbegin(); it!=path.rend(); ++it)
            rpath.push_back(it->first, it->second);

        return rpath
    }
} 




    
