#include "m2.h"
#include "ezgl/application.hpp"
#include "ezgl/graphics.hpp"
#include "ezgl/control.hpp"
#include "m1.h"
#include "m3.h"
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
#include "m4.h"
//load data structures here

struct intersection_data {
    LatLon position; //position of this intersection
    std::string name; //name of this intersection
    bool highlight = false; //whether or not to highlight this intersection
};

struct POI_Data {
    std::string name; //name of point of interest
    std::string type; //type of point of interest
    LatLon position; //position (in LatLon) of point of interest
    bool highlight = false; //whether or not to highlight this POI
    unsigned index;
};

struct features_data {
    std::vector <LatLon> points; // stores the coordinate points of each feature
    std::string name; //name of feature
    FeatureType details; //type of feature
    int pointCount; // stores the number of points of a feature
};
//void buildHighwayClassification(); logic implemented not used

//vectors that hold the POIs, intersections, and features of entire map.
std::vector<POI_Data> pointsOfInterest;
std::vector<intersection_data> intersections;
std::vector<features_data> features;
std::list<int> beginAndEnd;
std::vector<int> pathIntersections;


//std::unordered_map<OSMID, std::string>OSMHighways; //logic implemented but not used


double latAvg;
LatLon pos;
double xToLon(double x);
double yToLat(double y);
double lonTox(double lon);
double latToy(double lat);
bool isClosedFeature(features_data toCheck);
void drawFeatures(ezgl::renderer &g);
void populatePoiData();
void drawPOIs(ezgl::renderer &g);
void printTextFeatures(ezgl::renderer &g);
void draw_main_canvas(ezgl::renderer &g);
void loadFeatures();
void act_on_mouse_click(ezgl::application* app, GdkEventButton* event, double x, double y);
void populateIntersections(double &minLon, double &minLat, double &maxLon, double &maxLat);
std::string findIntersectionFromStreetNames(std::string firstStreet, std::string secondStreet, ezgl::application *application, double &x, double &y);
unsigned find_nearest_point_of_interest(std::vector<POI_Data> identicalPOI, LatLon my_position);
void find_button(GtkWidget *widget, ezgl::application *application);
void loadMap(GtkWidget *widget, ezgl::application *application);
void act_on_mouse_move(ezgl::application *application, GdkEventButton *event, double x, double y);
void act_on_key_press(ezgl::application *application, GdkEventKey *event, char *key_name);
void initial_setup(ezgl::application *application);
void printTextInterests(ezgl::renderer &g);
void drawStreets(ezgl::renderer &g);
void drawStreetNames(ezgl::renderer &g);
void traversePath(ezgl::application *application, std::vector<int>, int);
void on_dialog_response(GtkWidget *dialog, gint response_id, gpointer user_data);
void travelDirections(GtkWidget *dialog, gint response_id, gpointer user_data);
bool drawPath;
int num = 0;
GObject *window; // the parent window over which to add the dialog
GtkWidget *content_area; // the content area of the dialog
GtkWidget *label; // the label we will create to display a message in the content area
GtkWidget *dialog; // the dialog box we will create
int totalID; //To keep track of which message to display
std::vector<unsigned> path;
bool findP;

double xToLon(double x) {
    double lon = x / (DEG_TO_RAD * cos(latAvg));
    return lon;
}

double yToLat(double y) {
    double lat = y / DEG_TO_RAD;
    return lat;
}

//transform (lon,lat) to (x, y)

double lonTox(double lon) {
    double x = (lon * DEG_TO_RAD) * cos(latAvg);
    return x;
}

double latToy(double lat) {
    double y = lat * DEG_TO_RAD;
    return y;
}

// check if the feature is closed by seeing if the last and first point of the feature is the same

bool isClosedFeature(features_data toCheck) {
    int totalCurvePoints = toCheck.pointCount;
    if (toCheck.points[0].lon() == toCheck.points[totalCurvePoints - 1].lon() && toCheck.points[0].lat() == toCheck.points[totalCurvePoints - 1].lat())
        return true;
    else return false;
}

void drawFeatures(ezgl::renderer &g) {
    // draws out the features
    for (unsigned j = 0; j < features.size(); j++) {
        bool outsideVisible = true;
        bool featureClosed = isClosedFeature(features[j]);
        std::vector<ezgl::point2d> storePoints; // vector to store coordinates of the curve points

        for (int k = 0; k < features[j].pointCount; k++) {

            // only draw buildings if zoomed in more
            if (((g.get_visible_world().bottom() - g.get_visible_world().top()) < -0.00025) && (features[j].details == 6 || features[j].details == 4))
                break;

            // draw out the curve points of each feature

            double x = lonTox(features[j].points[k].lon());
            double y = latToy(features[j].points[k].lat());

            double width = 0.00001;
            double height = width;

            // set the color of the feature's curve point to draw
            if (features[j].details == 0) {
                g.set_color(ezgl::BLACK);
            } else if (features[j].details == 1) {
                g.set_color(ezgl::color(192, 236, 174, 255));
            } else if (features[j].details == 2) {
                g.set_color(ezgl::color(250, 242, 199, 255));
            } else if (features[j].details == 3) {
                g.set_color(ezgl::color(170, 218, 255, 255));
            } else if (features[j].details == 4) {
                g.set_color(ezgl::color(170, 218, 255, 255));
            } else if (features[j].details == 5) {
                g.set_color(ezgl::BISQUE);
            } else if (features[j].details == 6) {
                g.set_color(ezgl::color(210, 210, 210, 255));
            } else if (features[j].details == 7) {
                g.set_color(ezgl::color(212, 241, 201, 255));
            } else if (features[j].details == 8) {
                g.set_color(ezgl::color(212, 241, 201, 255));
            } else if (features[j].details == 9) {
                g.set_color(ezgl::color(170, 218, 255, 255));
            }
            storePoints.push_back(ezgl::point2d(x, y));

            // check if the feature is outside of the visible window. if not, set outsideVisible flag to false.
            if (g.get_visible_world().contains(x, y) && outsideVisible) {
                outsideVisible = false;
            }
            if (outsideVisible) continue; // if feature is not visible, skip drawing it

            // draw features with only one curve point
            if (!(features[j].pointCount >= 2))
                g.fill_rectangle({x, y},
            {
                x + width, y + height
            });
            // draw features with multiple curve points that are not closed
            if (!featureClosed && features[j].pointCount >= 2 && k > 0) {
                g.draw_line(storePoints[k - 1], storePoints[k]);
            }
        }
        if (outsideVisible) continue;
        // draw features that are closed and have multiple curve points
        if (featureClosed && features[j].pointCount >= 2) {
            g.fill_poly(storePoints);
        }
    }
}

//fill the pointsOfInterest vector with relevant info, like name, type, position, and index.

void populatePoiData() {

    int numPOI = getNumPointsOfInterest();
    pointsOfInterest.resize(numPOI);

    for (int POIindex = 0; POIindex < numPOI; POIindex++) {
        pointsOfInterest[POIindex].name = getPointOfInterestName(POIindex);
        pointsOfInterest[POIindex].type = getPointOfInterestType(POIindex);
        pointsOfInterest[POIindex].position = getPointOfInterestPosition(POIindex);
        pointsOfInterest[POIindex].index = POIindex;
    }
}

//labels the POIs with corresponding icons.

void drawPOIs(ezgl::renderer &g) {
    //this for loop goes through all POIs and draws icon at location specified by x, y
    for (unsigned POIindex = 0; POIindex < pointsOfInterest.size(); POIindex++) {

        //let type hold the string corresponding to current POI type, will be used for icons.
        std::string type = pointsOfInterest[POIindex].type;

        // don't draw any POI's if zoomed out too much
        if (((g.get_visible_world().bottom() - g.get_visible_world().top()) < -0.00025)) break;
        //hospitals get priority to be drawn in 'shallower' zooms than everything else
        if (((g.get_visible_world().bottom() - g.get_visible_world().top()) < -0.00006) && type != "hospital") continue;

        //get location to POI in x, y coords
        double x = lonTox(pointsOfInterest[POIindex].position.lon());
        double y = latToy(pointsOfInterest[POIindex].position.lat());

        //do not run rest of this for loop if x, y coord isn't in current screen
        bool outsideVisible = true;
        if (g.get_visible_world().contains(x, y) && outsideVisible) {
            outsideVisible = false;
        }
        if (outsideVisible) continue;

        //imagePath will hold location to png in mapper folder
        std::string imagePath;

        //series of if statements to check which icon to use.
        if (type == "cafe") {
            imagePath = "cafe.png";
        } else if (type == "bar" || type == "nightclub") {
            imagePath = "bar.png";
        } else if (type == "pharmacy" || type == "doctors" || type == "dentist") {
            imagePath = "pharmacy.png";
        } else if (type == "college" || type == "kindergarten" || type == "library" || type == "school" || type == "tutoring") {
            imagePath = "school.png";
        } else if (type == "hospital" || type == "clinic") {
            imagePath = "hospital.png";
        } else if (type == "theatre" || type == "cinema" || type == "arts_centre") {
            imagePath = "theatre.png";
        } else if (type == "atm" || type == "bank") {
            imagePath = "bank.png";
        } else if (type == "fuel") {
            imagePath = "fuel.png";
        } else if (type == "stripclub") {
            imagePath = "strip_club.png";
        } else if (type == "restaurant" || type == "pub" || type == "fast_food" || type == "food_court") {
            imagePath = "restaurant.png";
        }//this line specifies file path
        else {
            imagePath = "newImage.png";
        }
        //initialize a surface pointer to the image wanted
        ezgl::surface* surface = g.load_png(imagePath.c_str());
        //this line draws the icon at the (x,y) location of the POI
        //draw the image
        g.draw_surface(surface, ezgl::point2d(x, y));
        //free the pointer right after so no memory leaks.
        g.free_surface(surface);
    }
}

void printTextInterests(ezgl::renderer &g) {
    for (unsigned j = 0; j < pointsOfInterest.size(); j++) {
        // print the feature's name if sufficiently zoomed in and if the feature actually has a name
        if (((g.get_visible_world().bottom() - g.get_visible_world().top()) < -0.00003)) break;

        bool outsideVisible = true;
        // determine location of POI
        double x = lonTox(pointsOfInterest[j].position.lon());
        double y = latToy(pointsOfInterest[j].position.lat());

        // check if the feature is outside of the visible window. if not, set outsideVisible flag to false.
        if (g.get_visible_world().contains(x, y) && outsideVisible) {
            outsideVisible = false;
        }
        if (outsideVisible) continue; // skip if not within the visible area
        // print the name of the POI
        g.set_color(ezgl::BLACK);
        g.draw_text(ezgl::point2d(x, y), pointsOfInterest[j].name);
    }
}

void printTextFeatures(ezgl::renderer &g) {
    for (unsigned j = 0; j < features.size(); j++) {
        // don't draw names of features if zoomed out too much
        if ((g.get_visible_world().bottom() - g.get_visible_world().top()) < -0.00003) break;

        bool outsideVisible = true;
        bool featureClosed = isClosedFeature(features[j]);
        bool drawn = false;
        std::vector<ezgl::point2d> storePoints; // vector to store coordinates of the curve points

        for (int k = 0; k < features[j].pointCount; k++) {
            // draw out the curve points of each feature

            double x = lonTox(features[j].points[k].lon());
            double y = latToy(features[j].points[k].lat());

            storePoints.push_back(ezgl::point2d(x, y));

            // check if the feature is outside of the visible window. if not, set outsideVisible flag to false.
            if (g.get_visible_world().contains(x, y) && outsideVisible) {
                outsideVisible = false;
            }
            if (outsideVisible) continue;

            // draw features with multiple curve points that are not closed
            if (!featureClosed && features[j].pointCount >= 2 && k > 0) {
                // print the feature's name if sufficiently zoomed in and if the feature actually has a name
                if (features[j].name != "<noname>" && !drawn) {
                    g.set_color(ezgl::RED);
                    g.draw_text(storePoints[0], features[j].name);
                    drawn = true;
                }
            }
        }
        if (outsideVisible) continue;
        // calculate the point that is somewhere within the feature
        double middleOfFeatureX = ((storePoints[0].x - storePoints[(features[j].pointCount) / 2].x) / 2 + storePoints[(features[j].pointCount) / 2].x);
        double middleOfFeatureY = ((storePoints[0].y - storePoints[(features[j].pointCount) / 2].y) / 2 + storePoints[(features[j].pointCount) / 2].y);
        // draw features that are closed and have multiple curve points
        if (featureClosed && features[j].pointCount >= 2) {
            // print the feature's name if sufficiently zoomed in and if the feature actually has a name
            if (features[j].name != "<noname>" && ((g.get_visible_world().bottom() - g.get_visible_world().top()) > -0.00003)) {
                g.set_color(ezgl::RED);
                g.draw_text(ezgl::point2d(middleOfFeatureX, middleOfFeatureY), features[j].name);
            }
        }
    }
}

void draw_main_canvas(ezgl::renderer &g) {
    //findIntersectionFromStreetNames();
    // background colour
    g.set_color(ezgl::color(236, 236, 236, 255));
    g.fill_rectangle({-100, -100},
    {
        100, 100
    });

    populatePoiData();


    drawFeatures(g);
    drawPOIs(g);
    
    //main problem with this before was probably that it was inside the above for loop while having 2-3 loops of its own, complexity was off the wall
    drawStreets(g);
    drawStreetNames(g);

    // draws out the intersections
    for (unsigned i = 0; i < intersections.size(); i++) {
        if ((g.get_visible_world().bottom() - g.get_visible_world().top()) < -0.0003) break;

        double x = lonTox(intersections[i].position.lon());
        double y = latToy(intersections[i].position.lat());

        if (!g.get_visible_world().contains(x, y)) continue;

        double width = 0.0000005;
        double height = width;

        if (intersections[i].highlight) {
            g.set_color(ezgl::RED);
        } else {
            g.set_color(ezgl::WHITE);
        }

        g.fill_rectangle({x, y},
        {
            x + width, y + height
        });


    }
    // print names of features
    g.set_text_rotation(0);
    printTextFeatures(g);
    // print names of POI's
    printTextInterests(g);
}

//load the features vector with its relevant information: name, pointCount, details, points vector.

void loadFeatures() {
    int numFeatures = getNumFeatures();

    features.resize(numFeatures);

    for (int featuresIndex = 0; featuresIndex < numFeatures; featuresIndex++) {
        features[featuresIndex].name = getFeatureName(featuresIndex);
        features[featuresIndex].pointCount = getFeaturePointCount(featuresIndex);
        features[featuresIndex].details = getFeatureType(featuresIndex);

        for (int pointsIndex = 0; pointsIndex < features[featuresIndex].pointCount; ++pointsIndex) {
            features[featuresIndex].points.push_back(getFeaturePoint(pointsIndex, featuresIndex));
        }
    }
}

void drawStreets(ezgl::renderer &g) {
    // go to every intersection and draw segments connected to it
    // get lat lon of every curve point and draw connected parts of segments
    int totalStreets = getNumStreets();

    // properties of the line used to draw the roads
    double new_height = g.get_visible_world().height();

    g.set_line_cap(ezgl::line_cap::round);

    // looping through every possible street for the map
    for (int i = 0; i < totalStreets; ++i) {
        double width = 0.0005 / new_height;
        g.set_line_width(width);

        //g.set_color(ezgl::WHITE);

        std::vector<unsigned> streetStreetSegments = find_street_street_segments(i);
        int totalStreetSegments = streetStreetSegments.size();


        bool isHighway = false;
        bool isMinorAndZoomedOut = false;
        std::string streetName = getStreetName(i);

        for (int j = 0; j < totalStreetSegments; ++j) {
            // determine if highway to set color for highways
            if ((getInfoStreetSegment(streetStreetSegments[j]).speedLimit >= 65) && ((streetName.find("Highway", 0) != -1) || (streetName.find("Freeway", 0) != -1) || (streetName.find("Motorway", 0) != -1) || (streetName.find("Autobahn", 0) != -1) || (streetName.find("Interstate", 0) != -1) || (streetName.find("Expressway", 0) != -1) || (streetName.find("Parkway", 0) != -1))) {
                isHighway = true;
                break;
            }
            // setup a condition (break statement) to skip drawing minor streets if zoomed sufficiently out. Minor street is a street with less than 50 km/h speed limit.
            if (((g.get_visible_world().bottom() - g.get_visible_world().top()) < -0.0008) && getInfoStreetSegment(streetStreetSegments[j]).speedLimit < 50) {
                isMinorAndZoomedOut = true;
                break;
            }
        }
        // if it is a minor road and map is zoomed out, don't draw it
        if (isMinorAndZoomedOut && !(drawPath && ((beginAndEnd.size() == 2) || findP))) continue;

        // properties to set depending on what type of road it is
        if (isHighway) {
            g.set_color(ezgl::ORANGE);
            width = 0.002 / new_height;
            g.set_line_width(width);
        } else g.set_color(ezgl::WHITE);

        // draw the road if above requirements are met
        for (int j = 0; j < totalStreetSegments; ++j) {
            if (isHighway) {
            g.set_color(ezgl::ORANGE);
            width = 0.002 / new_height;
            g.set_line_width(width);
            }
            else g.set_color(ezgl::WHITE);
           
            bool outsideVisible = true;
            InfoStreetSegment info = getInfoStreetSegment(streetStreetSegments[j]);

            //getting Earth coordinates of the intersections position this segment is bounded by
            LatLon begin = getIntersectionPosition(info.from);
            LatLon end = getIntersectionPosition(info.to);

            //Translate beginning and end positions of segments into x,y coordinate system
            double startX = lonTox(begin.lon());
            double startY = latToy(begin.lat());
            double endX = lonTox(end.lon());
            double endY = latToy(end.lat());

            // check if the feature is outside of the visible window. if not, set outsideVisible flag to false.
            if ((g.get_visible_world().contains(startX, startY) || g.get_visible_world().contains(endX, endY)) && outsideVisible) {
                outsideVisible = false;
            }
            if (outsideVisible) continue;

            //if there are no curvePoints, draw a line between .from and .to
            if (info.curvePointCount == 0) {
                if (!path.empty()) {
                    for (int l = 0; l < path.size(); l++) {
                        if ((streetStreetSegments[j] == path[l]) && drawPath && ((beginAndEnd.size() == 2) || findP))
                            g.set_color(ezgl::BLUE);
                    }
                }
                g.draw_line(ezgl::point2d(startX, startY), ezgl::point2d(endX, endY));
            }
                //otherwise, loop through curvePoints connecting .from to first curve point, consecutivec curvePoints to one another, and .to to last curvePoint
                //there are enough curvePoints defined on curvy roads so as to avoid drawing arcs
            else if (info.curvePointCount > 0) {

                LatLon curve = getStreetSegmentCurvePoint(0, streetStreetSegments[j]);
                double u = lonTox(curve.lon());
                double v = latToy(curve.lat());
                //line between segment intersection and first curve point
                if (!path.empty()) {
                    for (int l = 0; l < path.size(); l++) {
                        if ((streetStreetSegments[j] == path[l]) && drawPath && ((beginAndEnd.size() == 2) || findP))
                            g.set_color(ezgl::BLUE);
                    }
                }
                g.draw_line(ezgl::point2d(startX, startY), ezgl::point2d(u, v));

                //2nd set of position variables to draw lines BETWEEN curve points
                double u2;
                double v2;
                LatLon curve2;

                for (int k = 0; k < info.curvePointCount - 1; ++k) {
                    curve = getStreetSegmentCurvePoint(k, streetStreetSegments[j]);
                    curve2 = getStreetSegmentCurvePoint(k + 1, streetStreetSegments[j]);
                    u = lonTox(curve.lon());
                    v = latToy(curve.lat());
                    u2 = lonTox(curve2.lon());
                    v2 = latToy(curve2.lat());
                    g.draw_line(ezgl::point2d(u, v), ezgl::point2d(u2, v2));
                }
                curve2 = getStreetSegmentCurvePoint(info.curvePointCount - 1, streetStreetSegments[j]);
                u2 = lonTox(curve2.lon());
                v2 = latToy(curve2.lat());
                //connecting final curvePoint and other end of segment
                g.draw_line(ezgl::point2d(endX, endY), ezgl::point2d(u2, v2));
            }
        }
    }
}

void drawStreetNames(ezgl::renderer &g) {

    // go to every intersection and draw segments connected to it
    // get lat lon of every curve point and draw connected parts of segments
    int totalStreets = getNumStreets();

    // looping through every possible street for the map
    for (int i = 0; i < totalStreets; ++i) {
        // if zoomed out enough, don't draw any street names
        if ((g.get_visible_world().bottom() - g.get_visible_world().top()) < -0.00025) break;

        std::vector<unsigned> streetStreetSegments = find_street_street_segments(i); // store street segments of a street into a vector
        int totalStreetSegments = streetStreetSegments.size(); // get total number of streets segments in that street

        std::string streetName = getStreetName(i); // get the  street's name

        // record the location of the previously printed name
        double previousStartX = -100000000;
        double previousStartY = -100000000;
        double distanceFromPrevious = 100000000;

        if (!((g.get_visible_world().bottom() - g.get_visible_world().top()) > -0.00005)) {
            // draw the road if above requirements are met where it is at a sufficient zoom level. This one does not draw all the street names at this zoom level.
            for (int j = 2; j < totalStreetSegments && getInfoStreetSegment(streetStreetSegments[j]).speedLimit >= 40; j = j + 2) {
                // print the street names for roads with a speed limit greater than 40 and skip 2 street segments before reprinting the street name
                InfoStreetSegment info = getInfoStreetSegment(streetStreetSegments[j]);
                bool outsideVisible = true;
                LatLon begin = getIntersectionPosition(info.from);
                LatLon end = getIntersectionPosition(info.to);

                //Translate beginning and end positions of segments into x,y coordinate system
                double startX = lonTox(begin.lon());
                double startY = latToy(begin.lat());
                double endX = lonTox(end.lon());
                double endY = latToy(end.lat());
                double distanceStartEnd = std::sqrt(pow(startX - endX, 2) + pow(startY - endY, 2));
                // update the previous distance variable
                if (previousStartX != -100000000 && previousStartY != -100000000)
                    distanceFromPrevious = std::sqrt(pow(startX - previousStartX, 2) + pow(startY - previousStartY, 2));

                // check if the feature is outside of the visible window. if not, set outsideVisible flag to false.
                if ((g.get_visible_world().contains(startX, startY) || g.get_visible_world().contains(endX, endY)) && outsideVisible) {
                    outsideVisible = false;
                } // don't print the street name if the following occur
                if (outsideVisible || streetName == "<unknown>" || info.curvePointCount > 1 || distanceFromPrevious < 5 || distanceStartEnd < 0.00001) continue;

                // rotates the printing of the street name so that it is easier for the audience to read
                double rotationDegree = std::atan2((endY - startY), (endX - startX))*180 / 3.14;
                if (rotationDegree > 90) rotationDegree = rotationDegree - 180;
                else if (rotationDegree < -90) rotationDegree = rotationDegree + 180;
                // set the print text parameters
                g.set_color(ezgl::BLACK);
                g.set_text_rotation(rotationDegree);

                // if the street is one way, print out additional data, specifically the arrow directions
                if (info.oneWay == true) {
                    double rotationDegree2 = std::atan2((endY - startY), (endX - startX))*180 / 3.14; // find angle of the arrows and its directionality
                    g.set_color(ezgl::FIRE_BRICK); // color of the arrows
                    std::string spaces(streetName.length() + 8, ' '); // determine how long the street name is and draw arrows based on that

                    g.set_text_rotation(rotationDegree2); // set angle of the arrows
                    //Was initially using '-->' for arrows. Used UNICODE ASCII arrow characters for to draw arrows spaced out around segment name instead
                    g.draw_text(ezgl::point2d((startX - endX) / 2 + endX, (startY - endY) / 2 + endY), "⟶    " + spaces + "   ⟶"); // print the arrows

                    // print the actual street name
                    g.set_color(ezgl::BLACK);
                    g.set_text_rotation(rotationDegree);
                    g.draw_text(ezgl::point2d((startX - endX) / 2 + endX, (startY - endY) / 2 + endY), streetName); // print at the middle of line segments
                } else {
                    g.draw_text(ezgl::point2d((startX - endX) / 2 + endX, (startY - endY) / 2 + endY), streetName);
                }

                // update previous coordinates of where the previous street name was printed
                previousStartX = startX;
                previousStartY = startY;

            }
        } else if ((g.get_visible_world().bottom() - g.get_visible_world().top()) > -0.00005) {
            // draw the road if above requirements are met. This is for a different level of zoom (closer in)
            for (int j = 0; j < totalStreetSegments; ++j) {
                InfoStreetSegment info = getInfoStreetSegment(streetStreetSegments[j]); // get the info on the street segment
                bool outsideVisible = true;
                LatLon begin = getIntersectionPosition(info.from);
                LatLon end = getIntersectionPosition(info.to);

                //Translate beginning and end positions of segments into x,y coordinate system
                double startX = lonTox(begin.lon());
                double startY = latToy(begin.lat());
                double endX = lonTox(end.lon());
                double endY = latToy(end.lat());

                // get the distance from the previous print
                if (previousStartX != -100000000 && previousStartY != -100000000)
                    distanceFromPrevious = std::sqrt(pow(startX - previousStartX, 2) + pow(startY - previousStartY, 2));

                // check if the feature is outside of the visible window. if not, set outsideVisible flag to false.
                if ((g.get_visible_world().contains(startX, startY) || g.get_visible_world().contains(endX, endY)) && outsideVisible) {
                    outsideVisible = false;
                } // if these requirements are met, skip printing this name
                if (outsideVisible || streetName == "<unknown>" || distanceFromPrevious < 0.5) continue;

                // if street name is angled in a way the reader finds it difficult, rotate it properly
                double rotationDegree = std::atan2((endY - startY), (endX - startX))*180 / 3.14;
                if (rotationDegree > 90) rotationDegree = rotationDegree - 180;
                else if (rotationDegree < -90) rotationDegree = rotationDegree + 180;

                //for one way arrows, must be in direction of .from -> .to slope
                double rotationDegree2 = std::atan2((endY - startY), (endX - startX))*180 / 3.14;

                // set print parameters for street name
                g.set_color(ezgl::BLACK);
                g.set_text_rotation(rotationDegree);

                if (info.curvePointCount < 2) {
                    if (info.oneWay == true) {
                        // if one way, print details of directionality (same as above)
                        std::string spaces(streetName.length() + 8, ' ');
                        g.set_text_rotation(rotationDegree2);
                        g.draw_text(ezgl::point2d((startX - endX) / 2 + endX, (startY - endY) / 2 + endY), "⟶    " + spaces + "   ⟶");

                        // set parameters for printing the street name so that it prints between the curve points of a street segment (like above)
                        g.set_color(ezgl::BLACK);
                        g.set_text_rotation(rotationDegree);
                        g.draw_text(ezgl::point2d((startX - endX) / 2 + endX, (startY - endY) / 2 + endY), streetName);
                    } else {
                        g.draw_text(ezgl::point2d((startX - endX) / 2 + endX, (startY - endY) / 2 + endY), streetName);
                    }
                } else { // if there are curve points more than 2 curve points
                    LatLon curvePointBegin;
                    // choose the curve points on the street segment to print the name on depending on how many curve points there are
                    if (info.curvePointCount / 2 - 1 >= 0)
                        curvePointBegin = getStreetSegmentCurvePoint(info.curvePointCount / 2 - 1, streetStreetSegments[j]);
                    else
                        curvePointBegin = begin;
                    LatLon curvePointEnd = getStreetSegmentCurvePoint(info.curvePointCount / 2, streetStreetSegments[j]);

                    // convert to x y coordinates
                    double startX = lonTox(curvePointBegin.lon());
                    double startY = latToy(curvePointBegin.lat());
                    double endX = lonTox(curvePointEnd.lon());
                    double endY = latToy(curvePointEnd.lat());

                    // find the angle to rotate the street names suitable for the user to read
                    double rotationDegree = std::atan2((endY - startY), (endX - startX))*180 / 3.14;

                    //Conditions so we don't get upside-down text. Keep names within quadrants 1 & 4 in cartesian plane
                    if (rotationDegree > 90) rotationDegree = rotationDegree - 180;
                    else if (rotationDegree < -90) rotationDegree = rotationDegree + 180;

                    g.set_text_rotation(rotationDegree);

                    if (info.oneWay == true) {
                        // if one way, print directionality (same as above), played around with arrow spaces and saw the '8' was spaced out enough from segment text
                        g.set_color(ezgl::FIRE_BRICK);
                        std::string spaces(streetName.length() + 8, ' ');

                        // arrows can rotate in any cartesian quadrant, unlike text which needs to be in direction of .from -> .to slope
                        g.set_text_rotation(rotationDegree2);
                        g.draw_text(ezgl::point2d((startX - endX) / 2 + endX, (startY - endY) / 2 + endY), "⟶    " + spaces + "   ⟶");

                        g.set_color(ezgl::BLACK);
                        g.set_text_rotation(rotationDegree);
                        g.draw_text(ezgl::point2d((startX - endX) / 2 + endX, (startY - endY) / 2 + endY), streetName);
                    } else {
                        g.set_color(ezgl::BLACK);
                        g.draw_text(ezgl::point2d((startX - endX) / 2 + endX, (startY - endY) / 2 + endY), streetName);
                    }
                }

                // update previous coordinate points
                previousStartX = startX;
                previousStartY = startY;
            }
        }
    }
}

void act_on_mouse_click(ezgl::application* app, GdkEventButton* event, double x, double y) {
    std::cout << getIntersectionStreetSegment(0, 9748) << std::endl;

    // code to find the closest intersection and POI to the point where mouse was clicked
    double lon = xToLon(x);
    double lat = yToLat(y);
    pos = LatLon(lat, lon);
    int idIntersection = find_closest_intersection(pos);
    int idPOI = find_closest_point_of_interest(pos);
    if (idIntersection == beginAndEnd.front() || idIntersection == beginAndEnd.back()) { drawPath = false; findP = false; }

    /* Have a list of maximum 2 elements. When an intersection is clicked, push_back it into this list.
     * If size of list is 2... pop front, then push back the most recent click.
     * Whenever user clicks find, pass this list onto find_path_between_intersections.
     */
    /* So what are all the possible scenarios?
     1. If an intersection is highlighted, we do not push_back it: we remove it from the list.
     2. If an intersection is not highlighted, we push_back it.
     3. If path is already drawn, and user clicks on one of the begin/end intersections, remove the drawing, pop that intersection from list
     4. If path is drawn, and user clicks on a new intersection, push_back it.
     5. */

    if (intersections[idIntersection].highlight == true) {
        beginAndEnd.remove(idIntersection);
    } else {
        if ((beginAndEnd.size() == 2)) beginAndEnd.pop_front();
        beginAndEnd.push_back(idIntersection);
    }

    for (std::list<int>::iterator it = beginAndEnd.begin(); it != beginAndEnd.end(); it++) {
        std::cout << "intersections still in the list: " << *it << std::endl;
    }



    //update status bar with intersection name, and print closest POI + its type to console.
    app->update_message("Closest Intersection: " + intersections[idIntersection].name);
    if (intersections[idIntersection].highlight == true) intersections[idIntersection].highlight = false;
    else intersections[idIntersection].highlight = true;


    app->refresh_drawing();

}

/* Fill the intersections vector with positions, and names of every intersection.
 * Function also finds the maximum and minimum LatLon coordinates on the map.
 */
void populateIntersections(double &minLon, double &minLat, double &maxLon, double &maxLat) {

    intersections.resize(getNumIntersections());
    maxLat = getIntersectionPosition(0).lat();
    minLat = maxLat;
    maxLon = getIntersectionPosition(0).lon();
    minLon = maxLon;
    //fill every intersection indexed with its position and name, update max and min LatLons with current extremums.
    for (int intersectionIndex = 0; intersectionIndex < getNumIntersections(); intersectionIndex++) {
        intersections[intersectionIndex].position = getIntersectionPosition(intersectionIndex);
        intersections[intersectionIndex].name = getIntersectionName(intersectionIndex);

        maxLat = std::max(maxLat, intersections[intersectionIndex].position.lat());
        minLat = std::min(minLat, intersections[intersectionIndex].position.lat());
        maxLon = std::max(maxLon, intersections[intersectionIndex].position.lon());
        minLon = std::min(minLon, intersections[intersectionIndex].position.lon());

    }
    //line taken from m1.cpp find_distance_between_two_points that define latAvg.
    latAvg = (maxLat * DEG_TO_RAD + minLat * DEG_TO_RAD) / 2;
}

void draw_map() {


    double minLon, minLat, maxLon, maxLat;
    populateIntersections(minLon, minLat, maxLon, maxLat);
    std::string random;
    loadFeatures();
    //findIntersectionFromStreetNames();
    ezgl::application::settings settings;
    settings.main_ui_resource = "libstreetmap/resources/main.ui";
    settings.window_identifier = "MainWindow";
    settings.canvas_identifier = "MainCanvas";

    ezgl::application application(settings);

    ezgl::rectangle initial_world({lonTox(minLon), latToy(minLat)},
    {
        lonTox(maxLon), latToy(maxLat)
    });
    application.add_canvas("MainCanvas", draw_main_canvas, initial_world);


    application.run(initial_setup, act_on_mouse_click, 0, act_on_key_press);

}
std::vector<unsigned> nonZeroResult;

/* This function returns the name of the intersection between firstStreet and secondStreet
 * and also modifies the x, y values that'll correspond to the intersection's location
 */
std::string findIntersectionFromStreetNames(std::string firstStreet, std::string secondStreet, ezgl::application *application, double &x, double &y) {
    //get vector of streets with first name, and vector of streets with second name
    std::vector<unsigned> vecFirstStreet;
    std::vector<unsigned> vecSecondStreet;
    vecFirstStreet = find_street_ids_from_partial_street_name(firstStreet);
    vecSecondStreet = find_street_ids_from_partial_street_name(secondStreet);
    //std::cout << vecFirstStreet[0];
    //check if there are intersections between the two streets. Once a vector with non-empty results is found, jump to DONE.
    for (unsigned firstStreetIndex = 0; firstStreetIndex < vecFirstStreet.size(); firstStreetIndex++) {
        for (unsigned secondStreetIndex = 0; secondStreetIndex < vecSecondStreet.size(); secondStreetIndex++) {
            nonZeroResult = find_intersection_ids_from_street_ids(vecFirstStreet[firstStreetIndex], vecSecondStreet[secondStreetIndex]);
            if (nonZeroResult.size() > 0) goto DONE;
        }
    }
DONE:
    //the two streets do intersect, print the entire list of intersections between them, and highlight.
    if (nonZeroResult.size() > 0) {
        std::cout << "now printing intersections to console: " << std::endl;
        for (unsigned i = 0; i < nonZeroResult.size(); i++) {
            std::cout << getIntersectionName(nonZeroResult[i]) << std::endl;
            intersections[nonZeroResult[i]].highlight = true;
        }
        //modify x,y to be the location of the intersection
        LatLon intersectionLocation = getIntersectionPosition(nonZeroResult[0]);
        x = lonTox(intersectionLocation.lon());
        y = latToy(intersectionLocation.lat());
        //return the name of the intersection
        return getIntersectionName(nonZeroResult[0]);
    } else {
        //vector was empty, the two streets do not have an intersection.
        application->update_message("Streets do not intersect, or incorrect suffix, e.g. 'drive' instead of 'street'");
        //return the null string.
        return "\0";
    }
}

//this helper function, taken from graphics.cpp, is used for zooming into intersections, POIs, or features.

static ezgl::rectangle zoom_in_world(ezgl::point2d zoom_point, ezgl::rectangle world, double zoom_factor) {
    double const left = zoom_point.x - (zoom_point.x - world.left()) / zoom_factor;
    double const bottom = zoom_point.y + (world.bottom() - zoom_point.y) / zoom_factor;

    double const right = zoom_point.x + (world.right() - zoom_point.x) / zoom_factor;
    double const top = zoom_point.y - (zoom_point.y - world.top()) / zoom_factor;

    return
    {
        {
            left, bottom
        },
        {
            right, top
        }
    };
}

/* The vector parameter holds the POIs with the same name.
 * Returns the POIindex that corresponds to POI of minimum distance.
 */
unsigned find_nearest_point_of_interest(std::vector<POI_Data> identicalPOI, LatLon my_position) {
    /* distance between my_position and point of interest is found using the distance between two points function.
     * minimumDistance is initialized to 'infinity',
     * compared against every interval of distance available
     *  the index of minimum distance is updated alongside;
     * by the end of the for loop we'd have the minimum distance index.
     */
    double currentDistance, minimumDistance;
    unsigned minDistIndex = 0;
    minimumDistance = std::numeric_limits<double>::max();
    for (unsigned POIindex = 0; POIindex < identicalPOI.size(); POIindex++) {
        currentDistance = find_distance_between_two_points(my_position, identicalPOI[POIindex].position);
        if (currentDistance < minimumDistance) {
            minimumDistance = currentDistance;
            minDistIndex = POIindex;
        }
    }
    //this index is specific to identicalPOI
    return minDistIndex;
}

//depending on text entered, find_button finds intersection, POI, or feature and zooms in on it




//This function modifies the extern mapPath, clears m2 DTs, and transfers control to main.cpp

void loadMap(GtkWidget *widget, ezgl::application *application) {
    //text_entry points to the text entry bar
    GtkEntry* text_entry = (GtkEntry *) application->get_object("TextInput");
    // gtk_entry_get_text dereferences the object pointed to by text_entry, extracts information from there.
    const char* text = gtk_entry_get_text(text_entry);
    //user will enter a string such as tehran_iran, so this removes the prefix suffix for them (more usable)
    std::string cityName(text);
    std::transform(cityName.begin(), cityName.end(), cityName.begin(), ::tolower);
    //check if the name entered is valid...
    if (cityName == "beijing" || cityName == "beijing china") cityName = "beijing_china";
    else if (cityName == "cairo" || cityName == "cairo egypt") cityName = "cairo_egypt";
    else if (cityName == "cape-town" || cityName == "cape town" || cityName == "cape-town South Africa") cityName = "cape-town_south-africa";
    else if (cityName == "golden-horseshoe" || cityName == "golden horseshoe" || cityName == "golden horseshoe Canada") cityName = "golden-horseshoe_canada";
    else if (cityName == "hamilton canada" || cityName == "hamilton") cityName = "hamilton_canada";
    else if (cityName == "hongkong" || cityName == "hong kong" || cityName == "hong kong china") cityName = "hong-kong_china";
    else if (cityName == "iceland") cityName = "iceland";
    else if (cityName == "interlaken" || cityName == "interlaken switzerland") cityName = "interlaken_switzerland";
    else if (cityName == "london" || cityName == "london england") cityName = "london_england";
    else if (cityName == "moscow" || cityName == "moscow russia") cityName = "moscow_russia";
    else if (cityName == "new-delhi" || cityName == "new delhi" || cityName == "new-delhi india" || cityName == "new delhi india") cityName = "new-delhi_india";
    else if (cityName == "new-york" || cityName == "new york" || cityName == "new-york usa" || cityName == "new york usa") cityName = "new-york_usa";
    else if (cityName == "rio-de-janeiro" || cityName == "rio de janeiro" || cityName == "rio de janeiro brazil" || cityName == "rio-de-janeiro brazil") cityName = "rio-de-janeiro_brazil";
    else if (cityName == "saint-helena" || cityName == "saint helena") cityName = "saint-helena";
    else if (cityName == "singapore") cityName = "singapore";
    else if (cityName == "sydney" || cityName == "sydney australia") cityName = "sydney_australia";
    else if (cityName == "tehran" || cityName == "tehran iran") cityName = "tehran_iran";
    else if (cityName == "tokyo" || cityName == "tokyo japan") cityName = "tokyo_japan";
    else if (cityName == "toronto" || cityName == "toronto canada") cityName = "toronto_canada";
    else {
        application->update_message("map does not exist :(");
        return;
    }

    std::string directory = "/cad2/ece297s/public/maps/";
    std::string suffix = ".streets.bin";
    //mapPath, the extern, is now modified so load_map uses the correct string.
    mapPath = directory + cityName + suffix;

    //extern switchMap is set to true so the while loop in main() runs again.
    switchMap = true;
    //clear m2 DTs
    pointsOfInterest.clear();
    intersections.clear();
    features.clear();
    //quit, transfer program control to main()
    application->quit();

}
std::list<std::string> instructions;

void on_dialog_response(GtkWidget *dialog, gint response_id, gpointer user_data) {
    // For demonstration purposes, this will show the enum name and int value of the button that was pressed

    totalID = totalID + response_id;

    switch (totalID) {
        case -3:
            label = gtk_label_new("\nTo look up an intersection, enter their names! For example: ");
            gtk_container_add(GTK_CONTAINER(content_area), label);
            gtk_widget_show_all(dialog);
            break;
        case -6:
            label = gtk_label_new("Yonge Street and College Street, or just Yonge & College; then press Find :)");
            gtk_container_add(GTK_CONTAINER(content_area), label);
            gtk_widget_show_all(dialog);
            break;
        case -9:
            label = gtk_label_new("\nTo look up a restaurant, bank, hospital, park, and so on...");
            gtk_container_add(GTK_CONTAINER(content_area), label);
            gtk_widget_show_all(dialog);
            break;
        case -12:
            label = gtk_label_new("Click an intersection, then enter the name of the place, like 'Starbucks Coffee';");
            gtk_container_add(GTK_CONTAINER(content_area), label);
            gtk_widget_show_all(dialog);
            break;
        case -15:
            label = gtk_label_new("And press Find! This'll find you the closest one, relative to your clicked location.");
            gtk_container_add(GTK_CONTAINER(content_area), label);
            gtk_widget_show_all(dialog);
            break;
        case -18:
            label = gtk_label_new("\nTo look up a path between intersections, enter their names, like: ");
            gtk_container_add(GTK_CONTAINER(content_area), label);
            gtk_widget_show_all(dialog);
            break;
        case -21:
            label = gtk_label_new("College & Spadina to College & Yonge; ");
            gtk_container_add(GTK_CONTAINER(content_area), label);
            gtk_widget_show_all(dialog);
            break;
        case -24:
            label = gtk_label_new("Or, if you're extra fancy, click on two intersections, we'll find the path in between!");
            gtk_container_add(GTK_CONTAINER(content_area), label);
            gtk_widget_show_all(dialog);
            break;
        case -27:
            label = gtk_label_new("\nTo move through the route, click the header bar (EZGL Example Application) after looking up a path;\n"
                    " then use the -> key to move towards destination, and <- to move towards source!");
            gtk_container_add(GTK_CONTAINER(content_area), label);
            gtk_widget_show_all(dialog);
            break;
        case -30:
            label = gtk_label_new("\nEnjoy :)");
            gtk_container_add(GTK_CONTAINER(content_area), label);
            gtk_widget_show_all(dialog);
            break;
          
        case -33:
            totalID = 0;
            gtk_widget_destroy(GTK_WIDGET(dialog));
            break;

        case GTK_RESPONSE_DELETE_EVENT:
            std::cout << "GTK_RESPONSE_DELETE_EVENT (i.e. ’X’ button) ";
            break;
        case GTK_RESPONSE_REJECT:
            gtk_widget_destroy(GTK_WIDGET(dialog));
            break;
        default:
            std::cout << "UNKNOWN ";
            break;
    }
}


void find_button(GtkWidget *widget, ezgl::application *application) {
    //path = find_path_between_intersections(9747, 9748, 0, 0);
    // std::cout << "size of path: " << path.size() << std::endl;
    //return;
    //GtkEntry* points to any object main.ui; I named it TextInput in glade's 'general ID' field
   /* std::vector<int> numbers = {0, 1, 2};
    int temp = numbers[0];
    numbers[0] = numbers[1];
    numbers[1] = temp;
    std::cout << numbers[0] << numbers[1] << numbers[2] << std::endl;
    return;*/
    
    
     std::vector<DeliveryInfo> deliveries;
       std::vector<unsigned> depots;
       float right_turn_penalty;
       float left_turn_penalty;
       float truck_capacity;
       std::vector<CourierSubpath> result_path;
       bool is_legal;
       double pathDist;

        deliveries = {DeliveryInfo(15105, 22791, 188.57120), DeliveryInfo(65047, 72971, 120.73338), DeliveryInfo(41755, 44970, 120.28966), DeliveryInfo(43274, 72971, 120.91861), DeliveryInfo(75625, 104239, 99.21876), DeliveryInfo(89518, 72971, 62.35834), DeliveryInfo(107548, 30762, 163.40495), DeliveryInfo(56298, 30762, 67.73763)};
        depots = {69462, 42782, 94619};
        right_turn_penalty = 15.000000000;
        left_turn_penalty = 15.000000000;
        truck_capacity = 7671.847167969;
        result_path = traveling_courier(deliveries, depots, right_turn_penalty, left_turn_penalty, truck_capacity);
        for (int i = 0; i < result_path.size(); i++) {
            pathDist += find_distance_between_two_points(getIntersectionPosition(result_path[i].start_intersection), getIntersectionPosition(result_path[i].end_intersection));
            //distanceOfPotentialPath += compute_path_travel_time(potentialPath[j].subpath, right_turn_penalty, left_turn_penalty);
        }
         //std::cout << "distance of path: " << pathDist;
        if (result_path.empty()) {
            std::cout << "oops, it's empty!" << std::endl;
           return;
        }
        for (int i = 0; i < result_path.size(); i++) {
           // std::cout << "this runs even after reaching finish depot " << std::endl;
           //std::cout << "it enters this while loop" << std::endl;
           std::cout << result_path[i].start_intersection << std::endl;
           std::cout << result_path[i].end_intersection << std::endl;

           std::cout << "then the next courier subpath follows: " << std::endl;
        }
        
       
       return;
    
      
    pathIntersections.clear();
    instructions.clear();
    path.clear();
    GtkEntry* text_entry = (GtkEntry *) application->get_object("TextInput");
    // gtk_entry_get_text dereferences the object pointed to by text_entry, extracts information from there.
    const char* text = gtk_entry_get_text(text_entry);
    //convert the char* to a string
    std::string entireString(text);

    //Mouse clicking find path
    if ((beginAndEnd.size() == 2) && (entireString.empty())) {
        //this collects the list of segments that make up the path
        path.clear();
        path = find_path_between_intersections(beginAndEnd.front(), beginAndEnd.back(), 0, 0);
        for (int m = 0; m < path.size(); m++) {
            pathIntersections.push_back(getInfoStreetSegment(path[m]).from);
        }
        /* So how would travel directions work?
 * I have a list of street segments.
 * I have two pointers that increment, and run TurnType find_turn_type(unsigned street_segment1, unsigned street_segment2)
 * on adjacent segments, to see if it is straight, left, or right.
 *      1. If it is right, the new message is "turn right onto" + streetNameOfSegment
 *      2. If it is left, the new message is "turn left onto" + streetNameOfSegment
 *      3. If it is straight, do not write new directions. Only write a new message if encountering a turn type change.
 *         When turn type changes, write "travel xxx meters on" + streetSegmentsOfStraightStreetID + "then turn right/left onto"...
 */

/* How do I dynamically generate messages into the popup dialog?
 * Have pre-made 'moulds' to fill messages into. Have a list of strings:
 * If it is right, push_back the message rightTurn = "Turn right onto" + streetNameOfSegment
 * If it is left, push_back the message leftTurn = "Turn left onto" + streetNameOfSegment
 * As I keep a pointer to current and next segment, always be summing the distance.
 *      1. If current and next segments are straight, sum them.
 *      2. If current and next compare to 'left', add 'current' to the sum of distances of 'straight' so far,
 *         print it, then clear the sum variable. After it's cleared, start summing again as next becomes current.
 *      3. Same for the 'right' case.
 * Once this list is populated, multiply its size by 3.
 * In travelDirections, while response_id = -3, .front() the message at beginning of list, then pop it;
 * continue doing this until the list is empty.
 */
        TurnType turn;
        int currentSeg = 0;
        int nextSeg = 1;
        double straightSegLength = 0;
        std::string instruction;
        instructions.clear();
        // if there's only one segment, just find its distance and print name of street
        if (path.size() == 1) {
            instruction = "Travel " + std::to_string(find_street_segment_length(path[currentSeg])) + " meters on " +
                    getStreetName(getInfoStreetSegment(path[currentSeg]).streetID);
            instructions.push_back(instruction);
            goto execute;
        }
        //while we have not reached end of vector...
        while (nextSeg != (path.size() - 1)) {
            //find turn type of these two segments
            turn = find_turn_type(path[currentSeg], path[nextSeg]);
            //if it's straight, sum the two segments
            if (turn == TurnType::STRAIGHT) {
                straightSegLength += find_street_segment_length(path[currentSeg]) + find_street_segment_length(path[nextSeg]);
            }
            if (turn == TurnType::RIGHT) {
                //the message is " travel " straightSegLength "meters on" getStreetName(getInfoStreetSegment(path[currentSeg]).streetID)
                //"then turn right onto" getStreetName(getInfoStreetSegment(path[nextSeg]).streetID
                //and then clear the straightSeglength
                straightSegLength += straightSegLength + find_street_segment_length(path[currentSeg]);
                instruction = "Travel " + std::to_string(straightSegLength) + " meters on " + getStreetName(getInfoStreetSegment(path[currentSeg]).streetID)
                        + " then turn right onto " + getStreetName(getInfoStreetSegment(path[nextSeg]).streetID);
                straightSegLength = 0;
            }
            if (turn == TurnType::LEFT) {
                straightSegLength += straightSegLength + find_street_segment_length(path[currentSeg]);
                instruction = "Travel " + std::to_string(straightSegLength) + " meters on " + getStreetName(getInfoStreetSegment(path[currentSeg]).streetID)
                        + " then turn left onto " + getStreetName(getInfoStreetSegment(path[nextSeg]).streetID);
                straightSegLength = 0;
            }
            for (std::list<std::string>::iterator it = instructions.begin(); it != instructions.end(); it++) {
                if (*it == instruction) goto noPush;
            }
            instructions.push_back(instruction);
noPush:
            currentSeg = nextSeg;
            nextSeg++;
        }
        //if we've reached the end...
        if (nextSeg == (path.size() - 1)) {
            //and straightSeg is not 0...
            if (straightSegLength > 0) {
                //std::to_string(find_street_segment_length(path[nextSeg]))
                instruction = "Travel " + std::to_string(straightSegLength) + " meters on " +
                        getStreetName(getInfoStreetSegment(path[currentSeg]).streetID);
                instructions.push_back(instruction);
                goto execute;
            }
        }

execute:
        drawPath = true;
        std::string startingIntersection = getIntersectionName(beginAndEnd.front());
        std::string endingIntersection = getIntersectionName(beginAndEnd.back());
        application->update_message("Path from " + startingIntersection + " to " + endingIntersection);
        // Create the dialog window.
        // Modal windows prevent interaction with other windows in the same application
        dialog = gtk_dialog_new_with_buttons("Directions", (GtkWindow*) window, GTK_DIALOG_USE_HEADER_BAR, ("Continue"), GTK_RESPONSE_ACCEPT, NULL);
        // Create a label and attach it to the content area of the dialog

        content_area = gtk_dialog_get_content_area(GTK_DIALOG(dialog));
        label = gtk_label_new("The travel route is as follows: ");
        gtk_container_add(GTK_CONTAINER(content_area), label);

        // The main purpose of this is to show dialog’s child widget, label
        gtk_widget_show_all(dialog);

        // Connecting the "response" signal from the user to the associated callback function
        g_signal_connect(GTK_DIALOG(dialog), "response", G_CALLBACK(travelDirections), NULL);
        totalID = 0;
        traversePath(application, pathIntersections, 0);
    }

    //if the search for & or and is not the npos, that means there is & or and:
    //hence we'll run the intersection algorithm
    if ((entireString.find("&") != std::string::npos) || (entireString.find("and") != std::string::npos)) {
        //if the word 'to' exists... we're running the path-finding algorithm.
        std::string firstStreet;
        std::string secondStreet;
        std::string thirdStreet;
        std::string fourthStreet;

        // added new possible keywords to separate the street names
        size_t seperationIndexAmpersand = entireString.find("&");
        size_t seperationIndexAmpersandSpace = entireString.find(" & ");
        size_t seperationIndexand = entireString.find(" and ");
        size_t separationIndexTo = entireString.find(" to ");

        /* This series of if else statements puts the correct sequence of characters
         * into firstStreet and secondStreet.
         */

        int newWordIndex = 0;
        if (seperationIndexAmpersandSpace != std::string::npos) {
            newWordIndex = seperationIndexAmpersandSpace;
            secondStreet = entireString.substr(newWordIndex + 3, separationIndexTo - newWordIndex - 3);
        } else if (seperationIndexAmpersand != std::string::npos) {
            newWordIndex = seperationIndexAmpersand;
            secondStreet = entireString.substr(newWordIndex + 1, separationIndexTo - newWordIndex - 1);
        } else if (seperationIndexand != std::string::npos) {
            newWordIndex = seperationIndexand;
            secondStreet = entireString.substr(newWordIndex + 5, separationIndexTo - newWordIndex - 5);
        }
        firstStreet = entireString.substr(0, newWordIndex);

        int toIndex = 0;
        int newWord = 0;
        if (separationIndexTo != std::string::npos) {

            toIndex = separationIndexTo + 4;

            size_t ampersand = entireString.find("&", toIndex);
            size_t ampersandSpace = entireString.find(" & ", toIndex);
            size_t indexAnd = entireString.find(" and ", toIndex);


            if (ampersandSpace != std::string::npos) {
                newWord = ampersandSpace;
                fourthStreet = entireString.substr(newWord + 3);
            } else if (ampersand != std::string::npos) {
                newWord = ampersand;
                fourthStreet = entireString.substr(newWord + 1);
            } else if (indexAnd != std::string::npos) {
                newWord = indexAnd;
                fourthStreet = entireString.substr(newWord + 5);
            }
            thirdStreet = entireString.substr(toIndex, newWord - toIndex);
        }
        //x and y corresponds to the transformed coordinates of LatLon positions
        double x, y;
        //this code runs if we're finding path...
        if (!thirdStreet.empty() && !fourthStreet.empty()) {
            //this vector will hold the two intersection indices
            std::vector<int> pathIntersects;
            std::string firstHalf = findIntersectionFromStreetNames(firstStreet, secondStreet, application, x, y);
            pathIntersects.push_back(nonZeroResult[0]);
            nonZeroResult.clear();
            std::string secondHalf = findIntersectionFromStreetNames(thirdStreet, fourthStreet, application, x, y);
            pathIntersects.push_back(nonZeroResult[0]);
            nonZeroResult.clear();
            path.clear();

            path = find_path_between_intersections(pathIntersects[0], pathIntersects[1], 0, 0);
            for (int m = 0; m < path.size(); m++) {
                pathIntersections.push_back(getInfoStreetSegment(path[m]).from);
            }
            //std::cout << path[0] << std::endl;
            TurnType turn;
            int currentSeg = 0;
            int nextSeg = 1;
            double straightSegLength = 0;
            std::string instruction;
            instructions.clear();
            // if there's only one segment, just find its distance and print name of street
            if (path.size() == 1) {
                instruction = "Travel " + std::to_string(find_street_segment_length(path[currentSeg])) + " meters on " +
                        getStreetName(getInfoStreetSegment(path[currentSeg]).streetID);
                instructions.push_back(instruction);
                goto finish;
            }
            //while we have not reached end of vector...
            while (nextSeg != (path.size() - 1)) {
                //find turn type of these two segments
                turn = find_turn_type(path[currentSeg], path[nextSeg]);
                //if it's straight, sum the two segments
                if (turn == TurnType::STRAIGHT) {
                    straightSegLength += find_street_segment_length(path[currentSeg]) + find_street_segment_length(path[nextSeg]);
                }
                if (turn == TurnType::RIGHT) {
                    //the message is " travel " straightSegLength "meters on" getStreetName(getInfoStreetSegment(path[currentSeg]).streetID)
                    //"then turn right onto" getStreetName(getInfoStreetSegment(path[nextSeg]).streetID
                    //and then clear the straightSeglength
                    straightSegLength += straightSegLength + find_street_segment_length(path[currentSeg]);
                    instruction = "Travel " + std::to_string(straightSegLength) + " meters on " + getStreetName(getInfoStreetSegment(path[currentSeg]).streetID)
                            + " then turn right onto " + getStreetName(getInfoStreetSegment(path[nextSeg]).streetID);
                    straightSegLength = 0;
                }
                if (turn == TurnType::LEFT) {
                    straightSegLength += straightSegLength + find_street_segment_length(path[currentSeg]);
                    instruction = "Travel " + std::to_string(straightSegLength) + " meters on " + getStreetName(getInfoStreetSegment(path[currentSeg]).streetID)
                            + " then turn left onto " + getStreetName(getInfoStreetSegment(path[nextSeg]).streetID);
                    straightSegLength = 0;
                }
                for (std::list<std::string>::iterator it = instructions.begin(); it != instructions.end(); it++) {
                    if (*it == instruction) goto noPus;
                }
                instructions.push_back(instruction);
noPus:
                currentSeg = nextSeg;
                nextSeg++;
            }
            //if we've reached the end...
            if (nextSeg == (path.size() - 1)) {
                //and straightSeg is not 0...
                if (straightSegLength > 0) {
                    //std::to_string(find_street_segment_length(path[nextSeg]))
                    instruction = "Travel " + std::to_string(straightSegLength) + " meters on " +
                            getStreetName(getInfoStreetSegment(path[currentSeg]).streetID);
                    instructions.push_back(instruction);
                    goto finish;
                }
            }

finish:
            drawPath = true;
            findP = true;
            std::string startingIntersection = getIntersectionName(beginAndEnd.front());
            std::string endingIntersection = getIntersectionName(beginAndEnd.back());
            application->update_message("Path from " + startingIntersection + " to " + endingIntersection);
            // Create the dialog window.
            // Modal windows prevent interaction with other windows in the same application
            dialog = gtk_dialog_new_with_buttons("Directions", (GtkWindow*) window, GTK_DIALOG_USE_HEADER_BAR, ("Continue"), GTK_RESPONSE_ACCEPT, NULL);
            // Create a label and attach it to the content area of the dialog

            content_area = gtk_dialog_get_content_area(GTK_DIALOG(dialog));
            label = gtk_label_new("The travel route is as follows: ");
            gtk_container_add(GTK_CONTAINER(content_area), label);

            // The main purpose of this is to show dialog’s child widget, label
            gtk_widget_show_all(dialog);

            // Connecting the "response" signal from the user to the associated callback function
            g_signal_connect(GTK_DIALOG(dialog), "response", G_CALLBACK(travelDirections), NULL);
            totalID = 0;
            traversePath(application, pathIntersections, 0);
        }            /* StatusMessage is updated with name of intersection; x, y now also takes on the values of the
         * converted LatLon coordinate of the location of intersection.
         */
        else {
            std::string statusMessage = findIntersectionFromStreetNames(firstStreet, secondStreet, application, x, y);
            //if the intersection actually exist...
            if (statusMessage != "\0") {
                //display the intersection name in status bar
                application->update_message(statusMessage);
                //get name of canvas
                std::string canvasID = application->get_main_canvas_id();
                //get screen-compatible location of intersection so it zooms in on the correct place.
                ezgl::point2d zoomPoint = (application->get_canvas(canvasID))->get_camera().widget_to_screen(ezgl::point2d(x, y));

                //create a rectangle around the zoomPoint for which the zoomed-in area is defined
                ezgl::point2d leftBottom(x - 0.00006, y - 0.00006);
                ezgl::point2d rightUp(x + 0.00006, y + 0.00006);
                ezgl::rectangle const world(leftBottom, rightUp);

                //set current camera to the rectangle defined above
                (application->get_canvas(canvasID))->get_camera().set_world(zoom_in_world(zoomPoint, world, 1));
                //redraw canvas to update the camera's newly set world.
                (application->get_canvas(canvasID))->redraw();
            } else {
                //if the intersection doesn't exist...
                application->update_message("Streets do not intersect, or incorrect suffix, or incorrect street names");
            }
        }
    }//string does not contain and or &, so now check if name is a POI or a feature...
    else {
        //x, y will hold location of POI or feature
        double x, y;
        //collect a vector of POI with this name
        std::vector<POI_Data> sameNamePOI;
        //this is the global POI's index
        int POIindex;
        //push all POIs with the same name into the sameNamePOI vector
        for (POIindex = 0; POIindex < pointsOfInterest.size(); POIindex++) {
            if (entireString == pointsOfInterest[POIindex].name) {
                sameNamePOI.push_back(pointsOfInterest[POIindex]);
            }
        }
        //if the vector meant to be populated with POIs that match user input is not empty...
        if (!sameNamePOI.empty()) {
            //define the index in sameNamePOI that is the closest
            int closestPOIindex = find_nearest_point_of_interest(sameNamePOI, pos);
            //define the index in the entire pointsOfInterest vector that closestPOIindex actually corresponds
            int actualPOIindex = sameNamePOI[closestPOIindex].index;
            //return the LatLon coordinate that corresponds to the POiindex
            LatLon POILocation = getPointOfInterestPosition(actualPOIindex);
            //do the conversions
            x = lonTox(POILocation.lon());
            y = latToy(POILocation.lat());
            //next block of code zooms in on the POI
            std::string canvasID = application->get_main_canvas_id();
            ezgl::point2d zoomPoint = (application->get_canvas(canvasID))->get_camera().widget_to_screen(ezgl::point2d(x, y));
            ezgl::point2d leftBottom(x - 0.00001, y - 0.00001);
            ezgl::point2d rightUp(x + 0.00001, y + 0.00001);
            ezgl::rectangle const world(leftBottom, rightUp);
            (application->get_canvas(canvasID))->get_camera().set_world(zoom_in_world(zoomPoint, world, 1));
            (application->get_canvas(canvasID))->redraw();
            application->update_message("Click mouse on screen, if you want closest location relative to where you are now.");
            //once this is done, jump to end of function.
            goto finished;
        }//user input was not found in POI list... try features list
        else {
            //traverse the list of features; if there is a match, zoom in on it. Idea similar to POI above.
            for (int featuresIndex = 0; featuresIndex < features.size(); featuresIndex++) {
                if (entireString == features[featuresIndex].name) {
                    LatLon featureLocation = features[featuresIndex].points[0];
                    x = lonTox(featureLocation.lon());
                    y = latToy(featureLocation.lat());
                    std::string canvasID = application->get_main_canvas_id();
                    ezgl::point2d zoomPoint = (application->get_canvas(canvasID))->get_camera().widget_to_screen(ezgl::point2d(x, y));

                    ezgl::point2d leftBottom(x - 0.00001, y - 0.00001);
                    ezgl::point2d rightUp(x + 0.00001, y + 0.00001);

                    ezgl::rectangle const world(leftBottom, rightUp);

                    (application->get_canvas(canvasID))->get_camera().set_world(zoom_in_world(zoomPoint, world, 1));
                    (application->get_canvas(canvasID))->redraw();
                    goto finished;
                }

            }
        }
    }
finished:
    return;
}



gint destroyID = 0;

void travelDirections(GtkWidget *dialog, gint response_id, gpointer user_data) {
    // For demonstration purposes, this will show the enum name and int value of the button that was pressed
    //this keeps running as long as there is instructions
    if (response_id == -3) {
        if (instructions.empty()) {
            label = gtk_label_new("You've reached the destination :3");
            gtk_container_add(GTK_CONTAINER(content_area), label);
            gtk_widget_show_all(dialog);
            destroyID = destroyID - 3;

            if (destroyID == -6) {
                destroyID = 0;
                gtk_widget_destroy(GTK_WIDGET(dialog));
                return;
            }
        } else {
            label = gtk_label_new((instructions.front()).c_str());
            instructions.pop_front();
            gtk_container_add(GTK_CONTAINER(content_area), label);
            gtk_widget_show_all(dialog);
        }

    }

}


//When clicked, triggers the tutorial.
void helpButton(GtkWidget *widget, ezgl::application *application) {

    // get a pointer to the main application window
    window = application->get_object(application->get_main_window_id().c_str());

    // Create the dialog window.
    // Modal windows prevent interaction with other windows in the same application
    dialog = gtk_dialog_new_with_buttons("Tutorial", (GtkWindow*) window, GTK_DIALOG_USE_HEADER_BAR, ("Continue"), GTK_RESPONSE_ACCEPT, NULL);
    // Create a label and attach it to the content area of the dialog
    content_area = gtk_dialog_get_content_area(GTK_DIALOG(dialog));
    label = gtk_label_new("Welcome to our map! Click continue to get a quick tutorial on how to use it :3\n                    Oh and this tutorial bar is draggable... try it right now!");
    gtk_container_add(GTK_CONTAINER(content_area), label);

    // The main purpose of this is to show dialog’s child widget, label
    gtk_widget_show_all(dialog);

    // Connecting the "response" signal from the user to the associated callback function
    g_signal_connect(GTK_DIALOG(dialog), "response", G_CALLBACK(on_dialog_response), NULL);
    totalID = 0;
    return;
}
//create the Find and Load Map buttons, each corresponding to their functions.

void initial_setup(ezgl::application *application) {
    // Update the status bar message
    application->update_message("Search Bar");

    // Create the buttons for find and load
    application->create_button("Find", 7, find_button);
    application->create_button("Load Map", 8, loadMap);
    application->create_button("Help", 9, helpButton);
}

void act_on_key_press(ezgl::application *application, GdkEventKey *event, char *key_name) {

    //std::cout << key_name  << " is pressed " << std::endl;

    std::string keyName(key_name);

    //this only zooms in on one
    if (drawPath && (keyName == "Right" || keyName == "Left")) {
        if (keyName == "Right") {
            num = num + 2;
        }
        if (keyName == "Left") {
            num = num - 2;
        }
        traversePath(application, pathIntersections, num);
    }
}

/* So what is it that I want to do?
 * Given a vector of intersectionIDs, convert their coordinates from LatLon to the correct x,y;
 * Then run this zooming algorithm on it
 * and update to the next element in the vector to do the same thing
 */

void traversePath(ezgl::application *application, std::vector<int> pathIntersections, int intersectionIndex) {

    if (intersectionIndex > (pathIntersections.size() - 1) || intersectionIndex < (0)) return;
    LatLon intersectionLocation = getIntersectionPosition(pathIntersections[intersectionIndex]);
    double x = lonTox(intersectionLocation.lon());
    double y = latToy(intersectionLocation.lat());


    std::string canvasID = application->get_main_canvas_id();
    //get screen-compatible location of intersection so it zooms in on the correct place.
    ezgl::point2d zoomPoint = (application->get_canvas(canvasID))->get_camera().widget_to_screen(ezgl::point2d(x, y));

    //create a rectangle around the zoomPoint for which the zoomed-in area is defined
    ezgl::point2d leftBottom(x - 0.00006, y - 0.00006);
    ezgl::point2d rightUp(x + 0.00006, y + 0.00006);
    ezgl::rectangle const world(leftBottom, rightUp);
    std::cout << x << std::endl;

    //set current camera to the rectangle defined above
    (application->get_canvas(canvasID))->get_camera().set_world(zoom_in_world(zoomPoint, world, 1));
    //redraw canvas to update the camera's newly set world.

    (application->get_canvas(canvasID))->redraw();


}

//add this when streets fixed
//OSM Classification - successfully colors highways and associated ramps but heavily hinders performance so was not implemented

//void buildHighwayClassification() {
//    const OSMWay* wayTagsPtr;
//    std::pair<std::string, std::string> tagPair;
//    unsigned wayCount = getNumberOfWays();
//    unsigned tagCount;
//
//    for (unsigned wayIndex = 0; wayIndex < wayCount; wayIndex++) {
//        wayTagsPtr = getWayByIndex(wayIndex);
//        tagCount = getTagCount(wayTagsPtr);
//        for (unsigned tagIndex = 0; tagIndex < tagCount; tagIndex++) {
//            tagPair = getTagPair(wayTagsPtr, tagIndex);
//            if (tagPair.first == "highway") {
//                std::pair<OSMID, std::string>node(wayTagsPtr->id(), tagPair.second);
//                OSMHighways.insert(node);
//           
//            }
//        }
//    }
//}

//This method was to be implemented in drawStreets() and would utilize the above 'buildHighwayClassification' map, it would
//successfully color motor/highways and associated ramps, large and minor roads with chosen color and widthsvbut heavily hinders performance due to a boost library run time error
//so it was not included in the working final version. None of our code relies on the Boost library so the error was very strange

//        bool isHighway = false;
//        bool isLargeRoad = false;
//        bool isSmallRoad = false;
//        bool isMinorAndZoomedOut = false;
//
//// determine if segment is important enough to set color for highways
//            InfoStreetSegment wayInfo = getInfoStreetSegment(streetStreetSegments[j]);
//            std::unordered_map<OSMID, std::string> :: iterator it = OSMHighways.find(wayInfo.wayOSMID);
//
//            //using .find for road links, junctions, and ramps to be colored and thickened in the same manner
//            if (it != OSMHighways.end()) {
//                if (it->second.find("motorway", 0) != -1 || wayInfo.speedLimit >= 80) {
//                    isHighway = true;
//                }
//                else if (it->second.find("primary", 0) != -1 || it->second.find("secondary", 0) != -1 || it->second.find("trunk", 0) != -1 || wayInfo.speedLimit >= 60) {
//                    isLargeRoad = true;
//                } else {
//                    isSmallRoad = true;
//                }
//            }
//
//        if (isHighway) {
//            g.set_color(ezgl::ORANGE);
//            width = 0.002 / new_height;
//            g.set_line_width(width);
//        }
//
//        else if (isLargeRoad) {
//            g.set_color(ezgl::WHITE);
//            width = 0.001 / new_height;
//            g.set_line_width(width);
//        }
//        else
//            g.set_color(ezgl::WHITE);