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
#pragma once //protects against multiple inclusions of this header file

#include <string>
#include <vector>
class LatLon; //Forward declaration

//use these values if you need earth radius or conversion from degrees to radians
constexpr double EARTH_RADIUS_IN_METERS = 6372797.560856;
constexpr double DEG_TO_RAD = 0.017453292519943295769236907684886;

//Loads a map streets.bin file. Returns true if successful, false if some error
//occurs and the map can't be loaded.
bool load_map(std::string map_name);

//Close the map (if loaded)
void close_map();

//Returns the street segments for the given intersection 

//William will implement this from tutorial code, as reference
std::vector<unsigned> find_intersection_street_segments(unsigned intersection_id);

//Returns the street names at the given intersection (includes duplicate street 
//names in returned vector)

//William
std::vector<std::string> find_intersection_street_names(unsigned intersection_id);

//Returns true if you can get from intersection1 to intersection2 using a single 
//street segment (hint: check for 1-way streets too)
//corner case: an intersection is considered to be connected to itself

//Chris
bool are_directly_connected(unsigned intersection_id1, unsigned intersection_id2);

//Returns all intersections reachable by traveling down one street segment 
//from given intersection (hint: you can't travel the wrong way on a 1-way street)
//the returned vector should NOT contain duplicate intersections

//Mohamed 
std::vector<unsigned> find_adjacent_intersections(unsigned intersection_id);

//Returns all street segments for the given street

//William
std::vector<unsigned> find_street_street_segments(unsigned street_id);

//Returns all intersections along the a given street

//Chris
std::vector<unsigned> find_all_street_intersections(unsigned street_id);


//Return all intersection ids for two intersecting streets
//This function will typically return one intersection id.

//Mohamed
std::vector<unsigned> find_intersection_ids_from_street_ids(unsigned street_id1, 
                                                              unsigned street_id2);

//Returns the distance between two coordinates in meters

//Mohamed
double find_distance_between_two_points(LatLon point1, LatLon point2);

//Returns the length of the given street segment in meters

//William, Chris
double find_street_segment_length(unsigned street_segment_id);

//Returns the length of the specified street in meters

//William, Chris
double find_street_length(unsigned street_id);

//Returns the travel time to drive a street segment in seconds 
//(time = distance/speed_limit)

//Mohamed
double find_street_segment_travel_time(unsigned street_segment_id);

//Returns the nearest point of interest to the given position

//More difficult: 3 of us brainstorm, combine ideas/pick best
unsigned find_closest_point_of_interest(LatLon my_position);

//Returns the nearest intersection to the given position

//Probably follows previous function's implementation
unsigned find_closest_intersection(LatLon my_position);

//Returns all street ids corresponding to street names that start with the given prefix
//The function should be case-insensitive to the street prefix. For example, 
//both "bloo" and "BloO" are prefixes to "Bloor Street East".
//If no street names match the given prefix, this routine returns an empty (length 0) 
//vector.
//You can choose what to return if the street prefix passed in is an empty (length 0) 
//string, but your program must not crash if street_prefix is a length 0 string.
std::vector<unsigned> find_street_ids_from_partial_street_name(std::string street_prefix);
