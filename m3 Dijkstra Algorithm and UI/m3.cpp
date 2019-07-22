/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
#include "m1.h"
#include "m2.h"
#include "StreetsDatabaseAPI.h"
#include <cmath>
#include <algorithm>
#include "globalVar.hpp"
#include "m3.h"
#include <iostream>
#include <bits/stdc++.h>
#define INF 0x3F3F3F3F

using namespace std;
typedef pair<double, double> ipair;


std::vector<unsigned> shortestPathFinder (const unsigned source, const unsigned destination, const double right_turn_penalty, const double left_turn_penalty);
double latAvg2;

double lonTox2(double lon) {
    double x = (lon * DEG_TO_RAD) * cos(latAvg2);
    return x;
}

double latToy2(double lat) {
    double y = lat * DEG_TO_RAD;
    return y;
}


// Returns the turn type between two given segments.
// street_segment1 is the incoming segment and street_segment2 is the outgoing
// one.
// If the two street segments do not intersect, turn type is NONE.
// Otherwise if the two segments have the same street ID, turn type is
// STRAIGHT. 
// If the two segments have different street ids, turn type is LEFT if
// going from street_segment1 to street_segment2 involves a LEFT turn
// and RIGHT otherwise.  Note that this means that even a 0-degree turn
// (same direction) is considered a RIGHT turn when the two street segments
// have different street IDs.
TurnType find_turn_type(unsigned street_segment1, unsigned street_segment2) {
   
    //Main idea, get 3 LatLon points, a (intersection), b (other end of segment 1), c (other end of segment 2)
    //We take the OPPOSITE point of the intersecting point for segments since we want to use full length of vector in cross product
    //Form vectors A & B and then compute their cross product
    //If cross_product > 0 -> right turn, < 0 -> left turn
   
   
    InfoStreetSegment info1 = getInfoStreetSegment(street_segment1);
    InfoStreetSegment info2 = getInfoStreetSegment(street_segment2);
   
    LatLon seg1_from; //point b
    LatLon seg_intersection; // point a
    LatLon seg2_to; //point c
   
    // no turn, return STRAIGHT
    if(info1.streetID == info2.streetID) {
        return TurnType::STRAIGHT;
    }
   
    //There are four combinations for where street_segments might intersect, these are give by the .from & .to intersection IDs in each if statement below:
   
    if(info1.to == info2.from) {
        //we check if there are any curve points for seg1, here seg1 is intersecting with .to so we use the curve point that is the furthest away for the opposite point
        if(info1.curvePointCount != 0) {
            seg1_from = getStreetSegmentCurvePoint(info1.curvePointCount - 1, street_segment1);
        }
       
        //if there are no curve points, just take the opposite end from where the segment is intersecting
        else {
            seg1_from = getIntersectionPosition(info1.from);
        }
       
        //we check if there are any curve points for seg2, here seg2 is intersecting with .from so we use the curve point that is the furthest away for the opposite point
        if(info2.curvePointCount !=0) {
            seg2_to = getStreetSegmentCurvePoint(0, street_segment2);
        }
       
        //if there are no curve points, just take the opposite end from where the segment is intersecting
        else {
            seg2_to = getIntersectionPosition(info2.to);
        }
       
        //Set the corresponding intersection position (could also be info2.from here)
        seg_intersection = getIntersectionPosition(info1.to);
    }
   
    else if(info1.from == info2.to) {
       
        if(info1.curvePointCount != 0) {
            seg1_from = getStreetSegmentCurvePoint(0, street_segment1);
        }
       
        else {
            seg1_from = getIntersectionPosition(info1.to);
        }
       
       
        if(info2.curvePointCount !=0) {
            seg2_to = getStreetSegmentCurvePoint(info2.curvePointCount - 1, street_segment2);
        }
       
        //if there are no curve points, just take the opposite end from where the segment is intersecting
        else {
            seg2_to = getIntersectionPosition(info2.from);
        }
       
        seg_intersection = getIntersectionPosition(info1.from);
    }
   
    else if(info1.from == info2.from) {
       
        if(info1.curvePointCount != 0) {
            seg1_from = getStreetSegmentCurvePoint(0, street_segment1);
        }
       
        else {
            seg1_from = getIntersectionPosition(info1.to);
        }
       
       
        if(info2.curvePointCount !=0) {
            seg2_to = getStreetSegmentCurvePoint(0, street_segment2);
        }
       
        //if there are no curve points, just take the opposite end from where the segment is intersecting
        else {
            seg2_to = getIntersectionPosition(info2.to);
        }
       
        seg_intersection = getIntersectionPosition(info1.from);
    }
   
    else if(info1.to == info2.to) {
       
        if(info1.curvePointCount != 0) {
            seg1_from = getStreetSegmentCurvePoint(info1.curvePointCount - 1, street_segment1);
        }
       
        //if there are no curve points, just take the opposite end from where the segment is intersecting
        else {
            seg1_from = getIntersectionPosition(info1.from);
        }
       
       
        if(info2.curvePointCount !=0) {
            seg2_to = getStreetSegmentCurvePoint(info2.curvePointCount - 1, street_segment2);
        }
       
        //if there are no curve points, just take the opposite end from where the segment is intersecting
        else {
            seg2_to = getIntersectionPosition(info2.from);
        }
       
        seg_intersection = getIntersectionPosition(info1.to);
    }
   
    else return TurnType::NONE;
   
    latAvg2 = ((seg1_from.lat() * DEG_TO_RAD) + (seg2_to.lat() * DEG_TO_RAD)) / 2.0; ///dont know if this is correct format, copied from m1.coo/m2.cpp
   
    //Avoid converting to cartesian coordinates to preserve accuracy with LatLons, was failing some cases when converting to X,Y coordinates
   
    //Get points, start is end of seg1, mid is intersection point, end is the end of seg2
    double startX = seg1_from.lon();
    double startY = seg1_from.lat();

    double midX = seg_intersection.lon();
    double midY = seg_intersection.lat();

    double endX = seg2_to.lon();
    double endY = seg2_to.lat();
   
    //form vector points, vectors extend from intersection to seg1 endpoint or seg2 endpoint
    double vecAx = startX - midX;
    double vecAy = startY - midY;

    double vecBx = endX - midX;
    double vecBy = endY - midY;

    // z component is 0, therefore cross product expression simplifies to a 2D determinant
    double cross_product = (vecAx * vecBy) - (vecAy * vecBx);
   
    //Final check to decide turn type using right-hand rule
    if(cross_product >= 0) {
        return TurnType::RIGHT;
    }

    else {
        return TurnType::LEFT;
    }
}

// Returns the time required to travel along the path specified, in seconds.
// The path is given as a vector of street segment ids, and this function can
// assume the vector either forms a legal path or has size == 0.  The travel
// time is the sum of the length/speed-limit of each street segment, plus the
// given right_turn_penalty and left_turn_penalty (in seconds) per turn implied
// by the path.  If the turn type is STRAIGHT, then there is no penalty
double compute_path_travel_time(const std::vector<unsigned>& path,
                                const double right_turn_penalty,
                                const double left_turn_penalty) {
   
    double path_time = 0;
    TurnType turn;
    bool last_segment = false;
   
    if(path.size() != 0) {
        if (path.size() == 1) {
            path_time = find_street_segment_travel_time(path[0]);
            goto finish;
        }
        for(unsigned segment_index = 0; segment_index < path.size(); segment_index++) {

            if(!last_segment)
                turn = find_turn_type(path[segment_index], path[segment_index+1]); //find what type of turn connected segments have
           
            //for last street we don't want to add a penalty time so we include it as a condition here
            if(turn == TurnType::STRAIGHT || segment_index == path.size()-1) {
                path_time += find_street_segment_travel_time(path[segment_index]);
            }

            else if(turn == TurnType::RIGHT) {
                path_time += find_street_segment_travel_time(path[segment_index]) + right_turn_penalty;
            }

            else if(turn == TurnType::LEFT) {
                path_time += find_street_segment_travel_time(path[segment_index]) + left_turn_penalty;
            }

            if(segment_index == path.size() - 2){
                last_segment = true;
            }
        }
    }
    finish:
    return path_time;
}


// Returns a path (route) between the start intersection and the end
// intersection, if one exists. This routine should return the shortest path
// between the given intersections, where the time penalties to turn right and
// left are given by right_turn_penalty and left_turn_penalty, respectively (in
// seconds).  If no path exists, this routine returns an empty (size == 0)
// vector.  If more than one path exists, the path with the shortest travel
// time is returned. The path is returned as a vector of street segment ids;
// traversing these street segments, in the returned order, would take one from
// the start to the end intersection.



std::vector<unsigned> find_path_between_intersections(
                  const unsigned intersect_id_start,
                  const unsigned intersect_id_end,
                  const double right_turn_penalty,
                  const double left_turn_penalty) {
    //graph.getAdjacencyList(adjacencyListForPath);
   
    return shortestPathFinder(intersect_id_start, intersect_id_end, right_turn_penalty, left_turn_penalty);
   
}


vector<unsigned> printPath(vector<int> parent, vector<int> parentSegments, unsigned j, unsigned destination) {
    vector<unsigned> newPath;
    vector<unsigned> newPathSegments;
   
    while (parent[j] > -1 && parentSegments[j] > -1) {
        newPath.push_back(parent[j]);
        newPathSegments.push_back(parentSegments[j]);
       
        j = parent[j];
    }
    reverse(newPath.begin(), newPath.end());
    reverse(newPathSegments.begin(), newPathSegments.end());
   
    return newPathSegments;
}

std::vector<unsigned> shortestPathFinder (const unsigned source, const unsigned destination, const double right_turn_penalty, const double left_turn_penalty) {
    // the case to return an empty vector when the source is the same as the destination
    if (source == destination)
        return vector<unsigned>{};
        
    
    // create a priority queue to store vertices that are being preprocessed.
    priority_queue <ipair, vector<ipair>, greater<ipair>> pqueue;
    // used to store the weight are all vertices and initialy set them to be all infinite (to be refined later)
    vector<double> dist(getNumIntersections(), INF);
    vector<int> parent (getNumIntersections());
    vector<int> parentSegments (getNumIntersections());
    vector<bool> visited(getNumIntersections());
    parent[source] = -1;
    parentSegments[source] = -1;
    unsigned prev_segment = 0;
    int u = -1;
    
    LatLon destPos = getIntersectionPosition(destination);
    double destX = lonTox2(destPos.lon());
    double destY = latToy2(destPos.lat());
    LatLon vertexPos;
    double currX = 0;
    double currY = 0;
    
    double A_heuristic = 0;
   
    // initialize visited array to all false
    for (bool x: visited) x = false;
   
    // insert the source into the priority queue and set its weight to 0
    pqueue.push(make_pair(0, source));
    dist[source] = 0;
   
    int prevu = -1; // store the previous value of u
    // loop until the priority queue becomes empty (otherwise the weight would not be finalized)
    while (!pqueue.empty()) {
        // store the weight of the current vertex and pop it off the queue
        u = pqueue.top().second;
        if (u == destination) break;
       
        pqueue.pop();
        if (visited[u]) continue;
        visited[u] = true;
       
        // this pointer 'travelConnectedIntersection is used to loop through each row of the adjacency list matrix
        connectedIntersectionsDetails *travelConnectedIntersection; // points to an adjacent intersection struct
        travelConnectedIntersection = adjacencyListForPath[u];
       
        while (travelConnectedIntersection != NULL){
            // get vertex label and weight of current vertex adjacent to u
            int vertex = travelConnectedIntersection->intersectionID;
            double timeRequired = travelConnectedIntersection->timeWeight;
            unsigned curr_segment = travelConnectedIntersection->segment_ID;
            
            vertexPos = getIntersectionPosition(vertex);
            currX = lonTox2(vertexPos.lon());
            currY = latToy2(vertexPos.lat());
            
            A_heuristic = sqrt((pow(destX - currX, 2)) + (pow(destY - currY, 2)));
            
            // if there is a shorter path to v through u, update the shortest distance required to reach that vertex from the source and consider the turn type
            TurnType leftRight;
            if (prevu == -1) // if u is still at the source point, always treat the travel to the next intersection as a straight line
                leftRight = TurnType::STRAIGHT;
            else {
                /*Need to know which two intersection IDs to look at, how to access them through vector 7 adjacency list
                 Not clear on which vector holds interested intpathersections*/
                InfoStreetSegment detailStreet = getInfoStreetSegment (curr_segment);
               
                prev_segment = parentSegments[u];
                leftRight = find_turn_type(prev_segment, curr_segment);
            }
            // if no turn
            if (dist[vertex] > dist[u] + timeRequired && (leftRight == TurnType::STRAIGHT )) {
                dist[vertex] = dist[u] + timeRequired + A_heuristic;
                pqueue.push(make_pair(dist[vertex], vertex));
                parent[vertex] = u;
                parentSegments[vertex] = curr_segment;
               
            }
            // if right turn
            else if ((dist[vertex] > dist[u] + timeRequired + right_turn_penalty) && leftRight == TurnType::RIGHT) {
                dist[vertex] = dist[u] + timeRequired + right_turn_penalty + A_heuristic;
                pqueue.push(make_pair(dist[vertex], vertex));
                parent[vertex] = u;
                parentSegments[vertex] = curr_segment;
            }
            // if left turn
            else if ((dist[vertex] > dist[u] + timeRequired + left_turn_penalty) && leftRight == TurnType::LEFT) {
                dist[vertex] = dist[u] + timeRequired + left_turn_penalty + A_heuristic;
                pqueue.push(make_pair(dist[vertex], vertex));
                parent[vertex] = u;
                parentSegments[vertex] = curr_segment;
            }
            // increment to the next adjacent intersection of vertex u
            travelConnectedIntersection = travelConnectedIntersection->next;
        }
        prevu = u;
    }
    visited.clear();
    if (u == destination)
        return printPath(parent, parentSegments, destination, destination);
    else return vector<unsigned>{};

}