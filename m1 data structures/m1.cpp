/*
 * Copyright 2019 University of Toronto
 *
 * Permission is hereby granted, to use this software and associated
 * documentation files (the "Software") in course work at the University
 * of Toronto, or for personal use. Other uses are prohibited, in
 * particular the distribution of the Software either publicly or to third
 * parties.
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include "m1.h"
#include "StreetsDatabaseAPI.h"
#include <cmath>
#include <algorithm>
#include <map>
#include "m3.h"
#include "globalVar.hpp"


using namespace std;



std::vector<std::vector<unsigned>> street_street_segments;
std::vector<std::vector<unsigned>> intersection_street_segments;
std::vector<double> segment_travel_times;
std::multimap<std::string, unsigned> partial_names; //find_street_ids_from_partial_street_name uses this map
std::vector<unsigned> streetIntersections;
// adjacency list used for path finding in M3

bool load_map(std::string map_path) {
    bool load_successful = loadStreetsDatabaseBIN(map_path); //Indicates whether the map has loaded
   
 

    if (!load_successful) return false;//Make sure this is updated to reflect whether
                            //loading the map succeeded or failed

    //
    //Load your map related data structures here
    //
    //populates a vector with segment travel times, of every segment in database.
    for (int streetSegmentIndex = 0; streetSegmentIndex < getNumStreetSegments(); streetSegmentIndex++) {
        double segment_length = find_street_segment_length(streetSegmentIndex);
        double speed = getInfoStreetSegment(streetSegmentIndex).speedLimit;
        //find_turn_type(streetSegmentIndex, streetSegmentIndex+1);
        segment_travel_times.push_back((segment_length/(speed))*3.6);
    }
   
    //Loop goes through all possible street names, transforms them to lowercase, creates a pair and then inserts it into the map
    //find_street_ids_from_partial_street_name uses this data-structure
    for(unsigned j=0; j < unsigned (getNumStreets()); j++) {
        std::string name = getStreetName(j);
        std::transform(name.begin(), name.end(), name.begin(), ::tolower);
        std::pair<std::string, unsigned>node(name, j);
      
        partial_names.insert(node);
    }
   
   
   
    /* Very thorough: create a vector that correlates perfectly to the number of intersections there are.
     * traverse the entire list of intersections in database; every index of intersection_street_segments
     * is a vector that holds the street segment IDs that are connected to it.
     */
    //find_intersection_street_segments
    intersection_street_segments.resize(getNumIntersections());
    for (int intersectionIndex = 0; intersectionIndex < getNumIntersections(); intersectionIndex++) {
        for (int i = 0; i < getIntersectionStreetSegmentCount(intersectionIndex); i++) {
            intersection_street_segments[intersectionIndex].push_back(getIntersectionStreetSegment(i, intersectionIndex));
        }
    }
  
    //find_street_street_segments
    /*nested vector approach:
     1. instantiate an outer vector with "maxStreetID" number of elements, i.e. getNumStreets()
     * every element of the outer vector is itself a street vector, that holds a vector of streetSegments (inner vector)
     2. traverse through the list of street segments; push every street segment into its corresponding
        streetID vector.
     3. this nested for loop does not cause extreme runtime problems, because the inner for loop checks
     * very few elements: only the segments in a particular street. This func passes perf test */
    street_street_segments.resize(getNumStreets());
    for (unsigned streetSegmentIndex = 0; streetSegmentIndex < unsigned (getNumStreetSegments()); streetSegmentIndex++) {
            InfoStreetSegment currentSegment = getInfoStreetSegment(streetSegmentIndex);
            //go through the list of segments for a street;
             street_street_segments[currentSegment.streetID].push_back(streetSegmentIndex);
           // street_street_segments[currentSegment.streetID].push_back(streetSegmentIndex);
    }
   
   
   
    // create a weighted adjacency matrix using street intersection points as the vertex of the graph for M3 find optimal path algorithm
    // create a vector to store the adjacency list where each element represents a vertex and points to the intersections it is connected to with its time weight
    adjacencyListForPath.resize(getNumIntersections());
    for (int i = 0; i < getNumIntersections(); ++i) {
        std::vector<unsigned> adjacentIntersections = find_adjacent_intersections(i);   // stores the adjacent intersections of an intersection if ID i
        if (adjacentIntersections.empty()) {adjacencyListForPath[i] = NULL; continue;}
        int numberOfAdjacentIntersections = adjacentIntersections.size();
       
        vector<unsigned> allSurroundingSegments = find_intersection_street_segments(i);    // used to store every street segment surrounding an intersection
        vector<int> legalSurroundingSegments(allSurroundingSegments.size(), -1); // used to store the street segments that can be travelled through when at a specific node (useful for one way streets)
        for (int j = 0; j < allSurroundingSegments.size(); ++j) {
            InfoStreetSegment currentSegmentDetails = getInfoStreetSegment(allSurroundingSegments[j]);
            for (int k = 0; k < allSurroundingSegments.size(); ++k) {
                if (currentSegmentDetails.oneWay && i == currentSegmentDetails.from && adjacentIntersections[k] == currentSegmentDetails.to) {
                    
                    if (legalSurroundingSegments[k] == -1)
                        legalSurroundingSegments[k] = allSurroundingSegments[j];
                    else if (find_street_segment_travel_time(allSurroundingSegments[j]) < find_street_segment_travel_time(legalSurroundingSegments[k]) && legalSurroundingSegments[k] != -1)
                        legalSurroundingSegments[k] = allSurroundingSegments[j];
                    
                    break;
                }
                else if (!currentSegmentDetails.oneWay && (adjacentIntersections[k] == currentSegmentDetails.from || adjacentIntersections[k] == currentSegmentDetails.to)) {
                    if (legalSurroundingSegments[k] == -1)
                        legalSurroundingSegments[k] = allSurroundingSegments[j];
                    else if (find_street_segment_travel_time(allSurroundingSegments[j]) < find_street_segment_travel_time(legalSurroundingSegments[k]) && legalSurroundingSegments[k] != -1)
                        legalSurroundingSegments[k] = allSurroundingSegments[j];
                    
                    break;
                }
            }
        }

        // storing the segment times of each legal street segment into a vector
        vector<double> segmentTravelTime;
        for (int j = 0; j < numberOfAdjacentIntersections; ++j) {
            segmentTravelTime.push_back(find_street_segment_travel_time(legalSurroundingSegments[j]));
        }
       
        // convert these vectors into a linked list as a struct
       
        adjacencyListForPath[i] = new connectedIntersectionsDetails;
        adjacencyListForPath[i]->intersectionID = adjacentIntersections[0];
        adjacencyListForPath[i]->timeWeight = segmentTravelTime[0];
        adjacencyListForPath[i]->segment_ID = legalSurroundingSegments[0];
        adjacencyListForPath[i]->next = NULL;
        connectedIntersectionsDetails *index = adjacencyListForPath[i];
        for (int j = 1; j < numberOfAdjacentIntersections; ++j) {
            index->next = new connectedIntersectionsDetails;
            index = index->next;
            index->intersectionID = adjacentIntersections[j];
            index->timeWeight = segmentTravelTime[j];
            index->segment_ID = legalSurroundingSegments[j];
            index->next = NULL;
        }
    }
    return true;
}

//Returns a vector of street segments for some given intersection
std::vector<unsigned> find_intersection_street_segments(unsigned intersection_id) {
        /* iterate through a loop, inserting street segments into some intersection vector;
     */
    return intersection_street_segments[intersection_id];
}

//Returns a vector of street names at the given intersection (includes duplicate street
//names in returned vector)

std::vector<std::string> find_intersection_street_names(unsigned intersection_id) {
    //want a vector of street names connected to an intersection
    //need a for loop; how many street segments connected to this intersection?
    //push_back the name of every street segment into the string vector.
    std::vector<std::string> intersection_streetNames;
    for (int i = 0; i < getIntersectionStreetSegmentCount(intersection_id); i++) {
        /*to find name of street segment:
         * get streetSegmentIndex using getIntersectionStreetSegment
         * instantiate a InfoStreetSegment struct using the streetSegmentIndex
         * Use the dot operator on this struct to get its streetID
         * now call the function getStreetName(StreetIndex streetIdx) to get name of segment
         */
        int streetSegmentIndex = getIntersectionStreetSegment(i, intersection_id);
        intersection_streetNames.push_back(getStreetName(getInfoStreetSegment(streetSegmentIndex).streetID));
    }
    return intersection_streetNames;
}

bool are_directly_connected(unsigned intersection_id1, unsigned intersection_id2) {
    // if the intersections are the same
    if (intersection_id1 == intersection_id2) return true;
   
    // find number of street segments connected to each intersection
    int streetSegmentCount_id1 = getIntersectionStreetSegmentCount(intersection_id1);

    // check if the intersections share a street segment
    InfoStreetSegment detailSharedStreetSegment;
    StreetSegmentIndex sharedStreetSegment;
    for (int index = 0; index < streetSegmentCount_id1; ++index) {
        sharedStreetSegment = getIntersectionStreetSegment(index, intersection_id1);
        detailSharedStreetSegment = getInfoStreetSegment(sharedStreetSegment);
        if ((unsigned (detailSharedStreetSegment.to) == intersection_id2 || unsigned (detailSharedStreetSegment.from) == intersection_id2) && detailSharedStreetSegment.oneWay == false) {
            return true;
        }
        else if ((unsigned (detailSharedStreetSegment.to) == intersection_id2) && detailSharedStreetSegment.oneWay == true) {
            return true;
        }
    }
    return false;
}

//Returns all intersections reachable by traveling down one street segment
//from given intersection (hint: you can't travel the wrong way on a 1-way street)
//the returned vector should NOT contain duplicate intersections
std::vector<unsigned> find_adjacent_intersections(unsigned intersection_id) {
   
    std::vector<unsigned> adj_intersections;
    unsigned segment;
    InfoStreetSegment info;
    bool in;
    for(int i = 0; i <= getIntersectionStreetSegmentCount(intersection_id)-1; i++) {
        segment = getIntersectionStreetSegment(i, intersection_id);
        info = getInfoStreetSegment(segment); //current street segment information
        if (info.from == info.to) continue;
        //checking if duplicate intersection is in vector, 'finds' if info.from/info.to id is in vector
        in = std::find(adj_intersections.begin(), adj_intersections.end(), info.from) != adj_intersections.end() || std::find(adj_intersections.begin(), adj_intersections.end(), info.to) != adj_intersections.end();
        /*Handling one way streets, if info.to is pointing at intersection_id,
         we cannot include that point since we would be traveling the wrong way in a 1 way street */
       
        //so long as the id we have is not ALREADY in the adj_intersections vector...
        if(!in) {
            if(info.oneWay) {
                if(unsigned (info.from) == intersection_id)
                    adj_intersections.push_back(info.to); //one way streets only point in .to direction, so return .to intersection
            }

            else {
                if(unsigned (info.from) == intersection_id)
                    adj_intersections.push_back(info.to);

                else
                    adj_intersections.push_back(info.from); //from ptr not at main intersection, return new adj intersection
            }
        }
    }
   
    return adj_intersections;
}

//Returns all street segments for the given street
//PLEASE RETURN THE SEGMENTS IN ORDER
std::vector<unsigned> find_street_street_segments(unsigned street_id) {
    /* i.e. street segments with the same street_id should be collected
     * 1. Loop through the streetSegments in map.
     * 2. after encountering the first streetSegment with a matching streetID,
     * 3. get the 'from, to' intersections.
     * 4. get the streetSegmentIndices connected to this intersection; check if any of their streetIDs match.
     * 5. if so, get the streetSegment with matching streetID, find its from, to intersections; repeat steps 3-5.
     * 6. Once there is no longer any streetSegments at an intersection that has matching streetId,
     * 7. Terminate search, go back to starting point, now look at the other intersection (e.g. if from first, now to.)
     * 8.
     * matching street_IDs; if not, check in the other direction: as long as streetIDs match,
     * keep traversing in that direction and pushing into the int vector array until no more matches
     */

    //but first, check if street_id is even in bounds.
    if (street_id > unsigned (getNumStreets())) return {};
  
    return street_street_segments[street_id];
}

//Returns all intersections along the a given street
std::vector<unsigned> find_all_street_intersections(unsigned street_id) {
    // the variable 'entireStreet' uses the previous function to store all segments of the street
    // the variable 'allIntersections stores all intersections of the entire street and it is the return variable
    // the paramater is numberOfSegments+1 because there is always one more intersection than segment
    std::vector<unsigned> entireStreet = find_street_street_segments(street_id);
    int numberOfSegments = entireStreet.size();
    std::vector<unsigned> allIntersections;
   
    // storing all intersections into 'allIntersections'
    allIntersections.push_back(getInfoStreetSegment(entireStreet[0]).from);
    if ((getInfoStreetSegment(entireStreet[0]).from) != (getInfoStreetSegment(entireStreet[0]).to))
        allIntersections.push_back(getInfoStreetSegment(entireStreet[0]).to);
    for (int index = 0; index < numberOfSegments; ++index) {
        bool repeatedValueTo = false, repeatedValueFrom = false;
        unsigned streetTo = getInfoStreetSegment(entireStreet[index]).to;
        unsigned streetFrom = getInfoStreetSegment(entireStreet[index]).from;
        int currentNumberofSegments = allIntersections.size();
       
        for (int index2 = 0; index2 < currentNumberofSegments; ++index2) {
            if (allIntersections[index2] == streetTo && !repeatedValueTo) {
                repeatedValueTo = true;
            }
            if (allIntersections[index2] == streetFrom && !repeatedValueFrom) {
                repeatedValueFrom = true;
            }
            if (repeatedValueFrom && repeatedValueTo) break;
        }
        if (!repeatedValueTo)
            allIntersections.push_back(streetTo);
        if (!repeatedValueFrom)
            allIntersections.push_back(streetFrom);
    }
   
    return allIntersections;
}

//Return all intersection ids for two intersecting streets
//This function will typically return one intersection id.
std::vector<unsigned> find_intersection_ids_from_street_ids(unsigned street_id1, unsigned street_id2) {
   //Want to determine intersection ID(s) given two street IDs
    /*1. Use previous func. to get all intersections of 1 street
     *2. Call prev function to store all street names at 1 intersec
     *3. Compare street_id2's name to vector of street names
     *4. If match is found, push ID into vector
    */
   
    //Idea, map with street id pair key and vector value, send DS pair of street IDs and get vector of intersections...
    std::vector<unsigned> streetIntersections1;
    std::vector<unsigned> streetIntersections2;
    std::vector<unsigned> InterIDs; //final vector returned
   
    streetIntersections1 = find_all_street_intersections(street_id1);
    streetIntersections2 = find_all_street_intersections(street_id2);
   
    for(unsigned i = 0; i < streetIntersections1.size(); i++) {
        for(unsigned j=0; j < streetIntersections2.size(); j++) {
            if(streetIntersections1[i] == streetIntersections2[j])
                InterIDs.push_back(streetIntersections1[i]);
        }
    }
     return InterIDs;
}
/* Given two LatLon objects, the distance between them is found
 The formula used is detailed in milestone 1.PDF from ECE297 website*/
double find_distance_between_two_points(LatLon point1, LatLon point2) {
    double latAvg = ((point1.lat() * DEG_TO_RAD) + (point2.lat() * DEG_TO_RAD)) / 2.0;
  
    double xOne = (point1.lon() * DEG_TO_RAD) * cos(latAvg);
    double xTwo = (point2.lon() * DEG_TO_RAD) * cos(latAvg);
    double yOne = point1.lat() * DEG_TO_RAD;
    double yTwo = point2.lat() * DEG_TO_RAD;
    double xSquared = pow((xTwo - xOne), 2.0);
    double ySquared = pow((yTwo - yOne), 2.0);
    double distance = EARTH_RADIUS_IN_METERS * sqrt(ySquared + xSquared);
  
    return distance;
}

// Returns the length of the given street segment in meters
double find_street_segment_length(unsigned street_segment_id) {
    // if there are no curve points, call find_distance_between_two_points
    //to get exact distance.
    InfoStreetSegment initialSegment = getInfoStreetSegment(street_segment_id);
    int numberOfCurvePoints = (initialSegment.curvePointCount);
    if (numberOfCurvePoints == 0) {
        return find_distance_between_two_points(getIntersectionPosition(initialSegment.from), getIntersectionPosition(initialSegment.to));
    }
    // if there are curve points
    // calculate the street segment length by adding up the distance between the curve points and the initial and final points
    double streetSegmentLength = find_distance_between_two_points(getIntersectionPosition(initialSegment.from), getStreetSegmentCurvePoint(0, street_segment_id));
    int curvePointsMaxIndex = numberOfCurvePoints - 1; // largest index of curve points
    for (int index = 0; index < curvePointsMaxIndex; ++index) {
        streetSegmentLength += find_distance_between_two_points(getStreetSegmentCurvePoint(index, street_segment_id), getStreetSegmentCurvePoint(index+1, street_segment_id));
    }
    streetSegmentLength += find_distance_between_two_points(getIntersectionPosition(initialSegment.to), getStreetSegmentCurvePoint(curvePointsMaxIndex, street_segment_id));
    return streetSegmentLength;
}
//Returns the length of the specified street in meters
double find_street_length(unsigned street_id) {
    std::vector<unsigned> all_segments = find_street_street_segments(street_id);
    double streetLength = 0;
    int totalNumberOfSegments = all_segments.size();
    for (int i = 0; i < totalNumberOfSegments; ++i) {
        streetLength += find_street_segment_length(all_segments[i]);
    }
   
    return streetLength;
}
//Returns the travel time to drive a street segment in seconds
//(time = distance/speed_limit)

double find_street_segment_travel_time(unsigned street_segment_id) {
    return segment_travel_times[street_segment_id];
}

/* distance between my_position and point of interest is found using the distance between two points function.
 * minimumDistance is initialized to 'infinity',
 * compared against every interval of distance available
 *  the index of minimum distance is updated alongside;
 * by the end of the for loop we'd have the minimum distance index.
 */
unsigned find_closest_point_of_interest(LatLon my_position) {
  
    double currentDistance, minimumDistance;
    unsigned minDistIndex = 0;
    minimumDistance = std::numeric_limits<double>::max();
    for (int POIindex = 0; POIindex < getNumPointsOfInterest()-1; POIindex++) {
        currentDistance = find_distance_between_two_points(my_position, getPointOfInterestPosition(POIindex));
        if (currentDistance < minimumDistance) {
            minimumDistance = currentDistance;
            minDistIndex = POIindex;
        }
    }
    return minDistIndex;
}

/*the procedure is near equivalent to find_closest_point_of_interest
 returns intersection LatLon closest to current position*/
unsigned find_closest_intersection(LatLon my_position) {
  
    double currentDistance, minimumDistance;
    unsigned minDistIndex = 0;
    minimumDistance = std::numeric_limits<double>::max();
    for (int intersectionIndex = 0; intersectionIndex < getNumIntersections()-1; intersectionIndex++) {
        currentDistance = find_distance_between_two_points(my_position, getIntersectionPosition(intersectionIndex));
        if (currentDistance < minimumDistance) {
            minimumDistance = currentDistance;
            minDistIndex = intersectionIndex;
        }
    }
    return minDistIndex;
  
}

//Mohamed

//Returns all street ids corresponding to street names that start with the given prefix
//The function should be case-insensitive to the street prefix. For example,
//both "bloo" and "BloO" are prefixes to "Bloor Street East".
//If no street names match the given prefix, this routine returns an empty (length 0)
//vector.
//You can choose what to return if the street prefix passed in is an empty (length 0)
//string, but your program must not crash if street_prefix is a length 0 string.
std::vector<unsigned> find_street_ids_from_partial_street_name(std::string street_prefix) {
    /*1. Check if street_prefix is empty, if it is, return an empty vector
     *2. In a loop, iterate through street names using getSteetName from street ids
     *3. Use .find member function for string class to if upper/lowercase prefix are in the street name
     * 4. if true, push street id into vector and continue searching until getNumStreets()-1
     * 5. **if no street names contained street_prefix, return empty vector */
   
    //Pseudo-code
    /* if(!street_prefix.empty()) {
        //How do i get all street_ids/street names?
        for(int i=0; i <= getNumStreets()-1; i++) { //literally looping through all possible streets, ids in the millions probably
            street = getStreetName(i);
            found = street.substr(0, street_prefix.size());
            transform(found.begin(), found.end(), found.begin(), ::tolower);
           
            //idea is if the case insensitive comparison is a match, then the corresponding street id gets pushed into the vector
            if(found == street_prefix) {
                street_ids.push_back(i);
            }
        }
    } */
   
    std::vector<unsigned> street_ids; //vector to store streets that start with street_prefix and return
    std::transform(street_prefix.begin(), street_prefix.end(), street_prefix.begin(), ::tolower);   //converting to lowercase for case-insensitive comparison

    //Specify bounds to search and add from
    auto low = partial_names.lower_bound(street_prefix); //iterator to first key in map that contains comparison to street_prefix
    street_prefix[street_prefix.size()-1]++;            //preventing lower bound == upperbound
    auto high = partial_names.upper_bound(street_prefix); //iterator to key in map that comes AFTER the last key matching street_prefix

    std::multimap<std::string, unsigned> :: iterator itr;

    for(itr = low; itr != high; itr++)
        street_ids.push_back(itr->second);
 
    return street_ids;
}

void close_map() {
    //Clean-up your map related data structures here
    partial_names.clear(); //clearing multi-map data structure
    streetIntersections.clear();
    segment_travel_times.clear();

    for (int i = 0; i < getNumIntersections(); ++i) {
        connectedIntersectionsDetails* current = adjacencyListForPath[i];
        connectedIntersectionsDetails* next = adjacencyListForPath[i];
        adjacencyListForPath[i] = NULL;
        while (next != NULL) {
            next = current->next;
            delete current;
            current = next;
        }
    }
   
    adjacencyListForPath.clear();
   
   
    //clearing nested vector data-structure used by find_street_street_segments
    for (int streetID = 0; streetID < getNumStreets(); streetID++) {
        street_street_segments[streetID].clear();
    }
    for (int intersectionIndex = 0; intersectionIndex < getNumIntersections(); intersectionIndex++) {
        intersection_street_segments[intersectionIndex].clear();
    }
   
    street_street_segments.clear();
    //closeOSMDatabase();
    closeStreetDatabase();
}