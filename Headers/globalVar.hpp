/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   globalVar.hpp
 * Author: honggua3
 *
 * Created on February 22, 2019, 7:39 PM
 */
//the extern variables modify current mapPath, so the do while loop in main uses new path to load new map.
#include <string>
#include <vector>
#ifndef GLOBALVAR_HPP
#define GLOBALVAR_HPP

// this struct is used for making the adjacency list of the intersections, so it stores the time weight of an adjacent intersection of a specific intersection
typedef struct connectedIntersectionsDetails{
    unsigned intersectionID;
    double timeWeight;
    unsigned segment_ID;
    connectedIntersectionsDetails *next;
} connectedIntersectionsDetails;

extern std::vector<connectedIntersectionsDetails*> adjacencyListForPath;
extern bool switchMap;
extern std::string mapPath;

//extern std::vector<connectedIntersectionsDetails*> adjacencyMatrixForPath;
#endif /* GLOBALVAR_HPP */

