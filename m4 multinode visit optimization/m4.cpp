#include "m2.h"
#include "ezgl/application.hpp"
#include "ezgl/graphics.hpp"
#include "ezgl/control.hpp"
#include "m1.h"
#include "m3.h"
#include "m4.h"
#include "StreetsDatabaseAPI.h"
#include <cmath>
#include <iostream>
#include <string>
#include <debug/unordered_map>
#include "globalVar.hpp"
#include "ezgl/color.hpp"
#include <algorithm>
#include <list>
#include <ctime>
#include <iomanip>
#include <bits/stdc++.h>
#include "m4.h"
#include <chrono>
#include <random>
#define INF 0x3F3F3F3F
#define TIME_LIMIT 60

using namespace std;
typedef pair<unsigned, float> upair;
typedef pair<bool, int> legalPair;

std::vector<legalPair> deliveryOrder;

std::vector<CourierSubpath> simulatedPath(double temperature, double coolingConstant, const float right_turn_penalty, const float left_turn_penalty,
float truck_capacity, const vector<DeliveryInfo>& deliveries, const std::vector<unsigned>& depots);

bool pathLegalityChecker(float truck_capacity, const vector<DeliveryInfo>& deliveries);


std::vector<CourierSubpath> traveling_courier(
        const std::vector<DeliveryInfo>& deliveries,
        const std::vector<unsigned>& depots,
        const float right_turn_penalty,
        const float left_turn_penalty,
        const float truck_capacity) {

    /* Simulated annealing algorithm

    acceptanceProbability = exp(-delta/(current temperature))

    1. get a deliveryOrder vector of ANY legal solution (could be shit)
    2. Define an initial 'tolerance' (temperature)
    3. Define a 'cooling constant', usually between 0.9 and 0.99,
    that describes how quickly we lower our 'tolerance' (temperature) for accepting worse solutions
    4. create a new deliveryOrder vector by some swapping of two cities
    5. compute delta = ((new solution's quality) - (old solution quality)) / (old solution quality)
    6. i. if delta <= 0, we accept new solution: new solution took less time
      ii. if delta > 0 (worse), we accept it with acceptanceProbability
      use a rand() % 100; if within range, accept; if not, don't.
    7. Set new temperature value to be cooling constant * current temperature.
    8. Repeat until we run out of time.
    */

    //deliveryOrder.push_back(std::make_pair(true, commonPickups[(fullPath[(fullPath.size() - 1)].end_intersection)][packageIndex].first));
    //populate deliveryOrder with numerically ordered delivery items
    //std::cout << "deliveryOrder size: " << deliveryOrder.size();
 
 
    for(int deliveryNumber = 0; deliveryNumber < deliveries.size(); deliveryNumber++) {
        //we know odds are pickups, evens are dropoffs.. that's how deliveries is populated
        deliveryOrder.push_back(std::make_pair(true, deliveryNumber));
        deliveryOrder.push_back(std::make_pair(false, deliveryNumber));
    }
    //std::cout << "deliveryOrder size: " << deliveryOrder.size() << std::endl;
 
//    for(int i = 0; i < deliveryOrder.size(); i++) {
//        std::cout << deliveryOrder[i].second << std::endl;
//    }
 
    long double temperature = 1000;
    long double coolingConstant = 0.9;
 
    //simulatedPath: takes temperature, coolingConstant
    std::vector<CourierSubpath> outputPath = simulatedPath(temperature, coolingConstant,
        right_turn_penalty, left_turn_penalty, truck_capacity, deliveries, depots);

    /*vector<CourierSubpath> fullPath;
    fullPath.clear();

    vector<CourierSubpath> potentialPath;
    int newSolutionDist = 0;
    int distanceOfBestPath = INF;
    vector<legalPair> bestDeliveryOrder;

    for (int i = 0; i < depots.size(); ++i) {
        deliveryOrder.clear();
        potentialPath.clear();
        newSolutionDist = 0;
        potentialPath = possibleRoutes(deliveries, depots, right_turn_penalty, left_turn_penalty, truck_capacity, i);
        // updating potential path distance
        for (int j = 0; j < potentialPath.size(); ++j) {
            newSolutionDist += find_distance_between_two_points(getIntersectionPosition(potentialPath[j].start_intersection), getIntersectionPosition(potentialPath[j].end_intersection));
            //newSolutionDist += compute_path_travel_time(potentialPath[j].subpath, right_turn_penalty, left_turn_penalty);
        }
        // update best found path to start off with depending on start depot
        if (distanceOfBestPath > newSolutionDist) {
            fullPath.clear();
            distanceOfBestPath = newSolutionDist;
            fullPath = potentialPath;
            bestDeliveryOrder.clear();
            bestDeliveryOrder = deliveryOrder;
        }
    }
    if (!fullPath.empty()) std::cout << "fullpath is not empty" << std::endl;
    //want to optimize found solution with 2-opt
    //pass the current best solution, its distance, etc. to simulatedPath.
    deliveryOrder = bestDeliveryOrder;
    std::cout << "delivery order size: " << deliveryOrder.size() << std::endl;*/
 
 
    deliveryOrder.clear();
    return outputPath;
}

/*1. Which intersections are legal? The dropOffIDs associated with current pickUps
  2. pickUps with an acceptable weight
  Collect these into a vector.
 Then, compare the distances of all these legal positions with current end_intersection… get the closest
 */

//returns the approximate length of one solution

std::vector<CourierSubpath> constructNewPath(const vector<DeliveryInfo>& deliveries, const float right_turn_penalty, const float left_turn_penalty, std::vector<legalPair> bestDeliveryOrder, const std::vector<unsigned>& depots) {
    std::vector<CourierSubpath> newPath;
    unsigned closestDepot;
    double depotDistance = 0;
    double closestDepotDistance =  INF;
    unsigned closestEndDepot;
    unsigned firstIntersection = deliveries[bestDeliveryOrder[0].second].pickUp;
 
    /*We are given the ordered path in the global deliveryOrder vector, we go to each ith element of deliveryOrder
     * and see if its a pickUp or dropOff. if its a pickUp, use its .second with the deliveries vector to get its pickUp intersection ID
     * If its a dropOff, use its dropOff intersection ID
     * Do this for the i+1 item as well to form a connected subpath
     */
    //find closest depot to first pickUp location
    for(int depotIndex = 0; depotIndex < depots.size(); ++depotIndex) {
        depotDistance = find_distance_between_two_points(getIntersectionPosition(firstIntersection), getIntersectionPosition(depots[depotIndex]));
        if(closestDepotDistance > depotDistance ) { //if the calculated value is less than the current smallest depot distance, update the closest distance and closestDepot
            closestDepotDistance = depotDistance;
            closestDepot = depots[depotIndex];
        }
    }
 
    CourierSubpath firstSubpath;
    firstSubpath.start_intersection = closestDepot;
    firstSubpath.end_intersection = firstIntersection;
    firstSubpath.subpath = find_path_between_intersections(firstSubpath.start_intersection, firstSubpath.end_intersection, right_turn_penalty, left_turn_penalty);
 
    newPath.push_back(firstSubpath);
 
    //finding subpaths for pickUps/dropOffs
    for (int i = 0; i < bestDeliveryOrder.size(); i++) {
        CourierSubpath sub;
     
        //if deliveryOrder very small, this would skip entirely...
         if(i == bestDeliveryOrder.size()-1) break; //prevent indexing out of bounds vector
       
       //Determine if element if pickUp or dropOff. If it is pickUp, we know that the subpath starts at the pickUp intersection ID in DeliveryInfo struct
        if (bestDeliveryOrder[i].first == true) {
            sub.start_intersection = deliveries[bestDeliveryOrder[i].second].pickUp;
            sub.pickUp_indices.push_back(bestDeliveryOrder[i].second); //push delivery number into vector
         
            //finish connecting subpath but accessing next element, and using pickUp/dropUp intersectionID based on the value of the bool true - pickUp, false - dropOff
            if (bestDeliveryOrder[i + 1].first == true) {
                sub.end_intersection = deliveries[bestDeliveryOrder[i + 1].second].pickUp;
            } else {
                sub.end_intersection = deliveries[bestDeliveryOrder[i + 1].second].dropOff;
            }
        }
            //when the ith element is not a pickUp, subpath starts at dropOff intersection ID
        else {
            sub.start_intersection = deliveries[bestDeliveryOrder[i].second].dropOff;

            //finish connecting subpath but accessing next element, and using pickUp/dropUp intersectionID based on the value of the bool true - pickUp, false - dropOff
            if (bestDeliveryOrder[i + 1].first == true) {
                sub.end_intersection = deliveries[bestDeliveryOrder[i + 1].second].pickUp;
            } else {
                sub.end_intersection = deliveries[bestDeliveryOrder[i + 1].second].dropOff;
            }
        }
        //instead of having another function to calculate total distance, do it here since we have access to subpath start/end intersection
        //@distance is getting accumulated from its caller address... its getting very large
     
        sub.subpath = find_path_between_intersections(sub.start_intersection, sub.end_intersection, right_turn_penalty, left_turn_penalty);
        newPath.push_back(sub); //push subpath back into new solution vector
    }
 
    closestDepotDistance = INF;
    depotDistance = 0;
 
    CourierSubpath lastSubpath;
    lastSubpath.start_intersection = deliveries[bestDeliveryOrder[bestDeliveryOrder.size()-1].second].dropOff;
 
    //find closest end depot to drop off truck
    for(int depotIndex = 0; depotIndex < depots.size(); ++depotIndex) {
        depotDistance = find_distance_between_two_points(getIntersectionPosition(lastSubpath.start_intersection), getIntersectionPosition(depots[depotIndex]));
     
        if(closestDepotDistance > depotDistance) { //if the calculated value lis less than the current smallest depot distance, update the closest distance and closestDepot
            closestDepotDistance = depotDistance;
            closestEndDepot = depots[depotIndex];
        }
    }
 
    lastSubpath.end_intersection = closestEndDepot;
    lastSubpath.subpath = find_path_between_intersections(lastSubpath.start_intersection, lastSubpath.end_intersection, right_turn_penalty, left_turn_penalty);
    newPath.push_back(lastSubpath);
    return newPath;
 
}
double pathLineApprox(const vector<DeliveryInfo>& deliveries) {
    double distance = 0;
    unsigned intersection_ID1;
    unsigned intersection_ID2;
 
    for(int i=0; i < deliveryOrder.size()-1; i++) {
        if(i == deliveryOrder.size() - 1) break; //don't index out of bounds
     
        if (deliveryOrder[i].first == true) {
            intersection_ID1 = deliveries[deliveryOrder[i].second].pickUp;

            //finish connecting subpath but accessing next element, and using pickUp/dropUp intersectionID based on the value of the bool true - pickUp, false - dropOff
            if (deliveryOrder[i + 1].first == true) {
                intersection_ID2 = deliveries[deliveryOrder[i + 1].second].pickUp;
            } else {
                intersection_ID2 = deliveries[deliveryOrder[i + 1].second].dropOff;
            }
        }
            //when the ith element is not a pickUp, subpath starts at dropOff intersection ID
        else {
            intersection_ID1 = deliveries[deliveryOrder[i].second].dropOff;

            //finish connecting subpath but accessing next element, and using pickUp/dropUp intersectionID based on the value of the bool true - pickUp, false - dropOff
            if (deliveryOrder[i + 1].first == true) {
                intersection_ID2 = deliveries[deliveryOrder[i + 1].second].pickUp;
            } else {
                intersection_ID2 = deliveries[deliveryOrder[i + 1].second].dropOff;
            }
        }
     
        distance += find_distance_between_two_points(getIntersectionPosition(intersection_ID1), getIntersectionPosition(intersection_ID2));
     
    }
 
    return distance;
}

std::vector<legalPair> TwoOptSwap(int i, int k) {
    //take route[0] to route [i-1] and add them in order (Existing order) to new route
    std::vector<legalPair> newOrder;
 
    if(i!=0) {
        for(int j=0; j < i; j++) {
            newOrder.push_back(deliveryOrder[j]);
        }
    }
    //take route[i] to route[k] and add them in reverse order to new route
    for(int m = k; m >= i; m--) {
        newOrder.push_back(deliveryOrder[m]);
    }
 
    //take route[k+1] to end and add them in order to new route
    for(int n = k+1; n < deliveryOrder.size(); n++) {
        newOrder.push_back(deliveryOrder[n]);
    }
 
    return newOrder;
}

int factorial(int n) {
    if (n == 0) return 1;
    else return n*factorial(n-1);
}

std::vector<CourierSubpath> simulatedPath(double temperature, double coolingConstant, const float right_turn_penalty, const float left_turn_penalty,
 float truck_capacity, const vector<DeliveryInfo>& deliveries, const std::vector<unsigned>& depots) {

     /* Simulated annealing algorithm

    acceptanceProbability = exp(-delta/(current temperature))

    1. get a deliveryOrder vector of ANY legal solution (could be shit)
    2. Define an initial 'tolerance' (temperature)
    3. Define a 'cooling constant', usually between 0.9 and 0.99,
    that describes how quickly we lower our 'tolerance' (temperature) for accepting worse solutions
    4. create a new deliveryOrder vector by some swapping of two cities
    5. compute delta = ((new solution's quality) - (old solution quality)) / (old solution quality)
    6. i. if delta <= 0, we accept new solution: new solution took less time
      ii. if delta > 0 (worse), we accept it with acceptanceProbability
      use a rand() % 100; if within range, accept; if not, don't.
    7. Set new temperature value to be cooling constant * current temperature.
    8. Repeat until we run out of time.
    */
   //find_distance_between_two_points(getIntersectionPosition(potentialPath[j].start_intersection), getIntersectionPosition(potentialPath[j].end_intersection))
   long double acceptanceProbability;
   int num = factorial(deliveryOrder.size());
 
   //we've found the distance of the original deliveryOrder vector... it's bad, so call it worstDistance

    bool legal = false;
    /* We made a vector of pairs. Each pair has <bool, unsigned> true for pickUp, false for dropOff. the unsigned number is the delivery number index to get the struct
     * info from the deliveries vector
     */
    double currentSolutionDist = pathLineApprox(deliveries);
    //std::vector<CourierSubpath> outputPath = oldPath; //final path returned
    std::vector<CourierSubpath> newPath; //store each legal path modified by 2-opt

    unsigned size = deliveryOrder.size();
    //std::cout << "delivery order size: " << deliveryOrder.size() << std::endl;
    legalPair temp;
    double newSolutionDist = 0;
    std::vector<legalPair> bestDeliveryOrder = deliveryOrder;
    // std::vector<legalPair> originalDeliveryOrder = deliveryOrder;
    std::vector<legalPair> swap;
    //if deliveryOrder is small, this does not make any sense;
    //this never touches the final element in a deliveryOrder vector
    long double delta, currentTemp;
    currentTemp = temperature;
    unsigned seed;
    long double randomNumber;
    bool timeOut = false;
    seed = std::chrono::system_clock::now().time_since_epoch().count();
                //creates generator object with seed
                std::default_random_engine generator (seed);
                //define a distribution object
                std::uniform_real_distribution<long double> distribution(0.1, 1000);
    auto startTime = std::chrono::high_resolution_clock::now();
    do {
        num--;
                /* 1. create a new deliveryOrder vector by some swapping of two cities
                2. compute delta = ((new solution's quality) - (old solution quality)) / (old solution quality)
                3. i. if delta <= 0, we accept new solution: new solution took less time
                    ii. if delta > 0 (worse), we accept it with acceptanceProbability
                    use a rand() % 100; if within range, accept; if not, don't.
                    4. Set new temperature value to be cooling constant * current temperature.
                */
//                swap.clear();
//                swap = TwoOptSwap(rand()%(size-2),rand()%(size-1));
//                deliveryOrder.clear();
//                deliveryOrder = swap;
//                deliveryOrder.clear();
//                deliveryOrder = bestDeliveryOrder;
//                temp = deliveryOrder[i];
//                deliveryOrder[i] = deliveryOrder[k];
//                deliveryOrder[k] = temp;

                // construct a trivial random generator engine from a time-based seed:
                //gets current time
             
                //this line returns the actual random number
                randomNumber = (distribution(generator));
                //std::cout << "random number now is: " << randomNumber << std::endl;
                //check if path is legal and then distance
                legal = pathLegalityChecker(truck_capacity, deliveries);
                if (legal) {
                    //we get the distance of this new deliveryOrder...
                    newSolutionDist = pathLineApprox(deliveries);
                    delta = (newSolutionDist - currentSolutionDist)/currentSolutionDist;
                    if (delta < 0) {
                        std::cout << "yep, a better path was found... " << std::endl;
                        //update what's now defined as best distance
                        currentSolutionDist = newSolutionDist;
                        bestDeliveryOrder.clear();
                        bestDeliveryOrder = deliveryOrder;
                    }
                    else {
                        //we accept this worse solution with a certain probability...
                    acceptanceProbability = (exp(-delta/currentTemp))*10;
                   
                    
                        //accept worse solution
                        if (randomNumber < acceptanceProbability) {
                            //std::cout << "random number: " <<randomNumber << std::endl;
                           // std::cout << "acceptance probablity: " << acceptanceProbability << std::endl;
                            //update what's now defined as best distance
                          
                            currentSolutionDist = newSolutionDist;
                            bestDeliveryOrder.clear();
                            bestDeliveryOrder = deliveryOrder;
                         
                            currentTemp = coolingConstant*currentTemp;
                        }
                    }

                    //checks if new path distance using straight line approximation is less, if it is, store as updated best time and equate output vector to newPath
                // if (legal && (newSolutionDist < distanceOfBestPath)) {
                 
                //     distanceOfBestPath = newSolutionDist;
                //     bestDeliveryOrder.clear();
                //     originalDeliveryOrder = deliveryOrder;
                //     i = 0;
                //     k = 0;
                //     bestDeliveryOrder = deliveryOrder; //store current best deliveryorder for future path construction
                //     std::cout << "yep, a better path was found... " << std::endl;
                 
                //     //we run 2-opt again immediately after this updated path to see if we can get more optimal solutions
                // }
                }

                //looping through all subpath combinations, swapping checking distance and then legality
                //generic swapping technique, check path legality after each & every swap
                //************THIS IS ALTERNATE SWAP VERSION************
                // deliveryOrder.clear();
                // deliveryOrder = originalDeliveryOrder;
                // temp = deliveryOrder[i];
                // deliveryOrder[i] = deliveryOrder[k];
                // deliveryOrder[k] = temp;
             

                //constructNewPath finds the both the new distance (newSolutionDist passed in by reference - may be wrong) and vector of subpaths

        auto currentTime = std::chrono::high_resolution_clock::now();
        auto wallClock = std::chrono::duration_cast<chrono::duration<double>>(currentTime - startTime);
        if (wallClock.count() > TIME_LIMIT) timeOut = true;
    } while (std::next_permutation(deliveryOrder.begin(), deliveryOrder.end()) && num!=0 && !timeOut);
    std::cout << "temperature: " << currentTemp << std::endl;
    std::vector<CourierSubpath> outputPath;
    outputPath.clear();
    outputPath = constructNewPath(deliveries, right_turn_penalty, left_turn_penalty, bestDeliveryOrder, depots);

    return outputPath;
}

//need commonPickUp, commonDropOffs, and visited

bool pathLegalityChecker(float truck_capacity, const vector<DeliveryInfo>& deliveries) {

    double totalCarriedWeight = 0;
    bool beenPickedUp = false;
 
    if(deliveryOrder[0].first != true) return false; //if the first element (after a depot) is NOT a pickup, path is illegal
    if(deliveryOrder[deliveryOrder.size()-1].first != false) return false; //if lthe last element is NOT a dropOff, path is illegal

    for (int i = 0; i < deliveryOrder.size(); i++) {
        /*What is a legal deliveryOrder vector?
         2. picked up items do not exceed truck weight
         3. only dropOff items that's been picked up
         */
        //if this is a pickup,
        beenPickedUp = false;
        if (deliveryOrder[i].first == true) {
            //add the item weight of this item to carried weight.
            totalCarriedWeight += deliveries[deliveryOrder[i].second].itemWeight;
            if (totalCarriedWeight > truck_capacity) return false; //first case of illegal path, exceeding truck capacity
        }
     
        //if we find a dropoff, check that its pickUp counterpart with the same delivery struct ID (.second) comes before it in deliveryOrder vector
        //if found, break out of loop
        if (deliveryOrder[i].first == false) {
            for (int j = 0; j < i; j++) {
                if (deliveryOrder[j].second == deliveryOrder[i].second) {
                    if (deliveryOrder[j].first == true) {
                        totalCarriedWeight -= deliveries[deliveryOrder[i].second].itemWeight;
                        beenPickedUp = true;
                        break;
                    }
                }
            }
            if (beenPickedUp == false) return false; //we found a dropOff before the item has even been picked up

        }
    }
    if (totalCarriedWeight != 0) return false;

    return true;
}