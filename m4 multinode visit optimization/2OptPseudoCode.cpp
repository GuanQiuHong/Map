/*struct DeliveryInfo {
    //Specifies a delivery order (input to your algorithm).
    //
    //To satisfy the order the item-to-be-delivered must have been picked-up 
    //from the pickUp intersection before visiting the dropOff intersection.

    DeliveryInfo(unsigned pick_up, unsigned drop_off, float weight)
        : pickUp(pick_up), dropOff(drop_off), itemWeight(weight) {}

    //The intersection id where the item-to-be-delivered is picked-up.
    unsigned pickUp;

    //The intersection id where the item-to-be-delivered is dropped-off.
    unsigned dropOff;

    // Weight of the item in pounds (lb)
    float itemWeight;
}*/

/* Have a vector of pickAndDrops; each element is a pair variable: the first is, whether it's a pickup or dropoff; the second is, its delivery number with respect to vector<deliveries> 
 * We can check that a base solution is valid by considering the state of the truck as it traverses the 
 * In the 2-edge operation, SWAP two elements of this pickAndDrops vector.
 * 
 * 
 * 
 * Pickup indices stores what every courier subpath has picked up, in terms of delivery number. 
 * How is the pickup indices vector for every courier subpath helpful?
 * When you rearrange elements of the pickAndDrops vector
 */


/* void func(argument 1, argument 2, argument 3) {
 * for (int i = 0; i< 100; i++) {
 * std::cout << "lol" << std::endl;
 * }
 * 
 * 
 * }
 * 
 * example of when this is used: multi-start algorithm. you pass a different depot into the function, and it runs all the
 * depot paths at the same time.
 * 
 * while (int i = 0; i < depots.size(); i++) {
 * std::thread th1(multi_start, std::ref(returnedPath), argument 2, depot[i]) threaded fxns must return void
 * std::thread th2(multi_start, std::ref(returnedPath1), argument 2, depot[i+1]) 
 * ...
 * std::thread th8(multi_start, std::ref(returnedPath8), argument 2, depot[i+7])
 * 
 * //whichever thread finishes first.. update the depot it tries
 * 
 * 
 * th1.join();
 * th1.doNextDepot
 * 
 * }
 * 
 * th1.join();
 * th2.join();

 * getTimeForPaths...
 * 
 * if (timeForPathOne < timeForPathTwo) chooseOne;
 *
 * 
 * std::ref to get the value you wanna modify: e.g. 
 * 
 * std::thread th1(func, argument 1, argument 2, std::ref(argument 3))
 * 
 * & in the prototype of the function
 * 
 * 
 * std::vector<CourierSubpath> TwoOptSwap(std::vector<CourierSubpath> oldPath, unsigned i, unsigned k, const float right_turn_penalty, const float left_turn_penalty) {
    
    std::vector<CourierSubpath> newPath;
    /*copy untouched subpaths in oldPath to new newPath
     * need to add route[i] to route[k] in reverse order:
     */
   
   /* CourierSubpath begin_newpath = oldPath[i-1];
    unsigned begin_newpathID = begin_newpath.end_intersection;
   
    CourierSubpath midpath = oldPath[k];
    unsigned midpathID = midpath.end_intersection;
   
    CourierSubpath end_path = oldPath[i];
    unsigned end_pathID = end_path.end_intersection;
   
   
    //take route[0] to route [i-1] and add them in order (Existing order) to new route
    for(unsigned j=0; j < i; j++) {
        newPath.push_back(oldPath[j]);
    }
   
    //take route[i] to route[k] and add them in reverse order to new route
    std::vector<unsigned> partial_path = find_path_between_intersections(begin_newpathID, midpathID, right_turn_penalty, left_turn_penalty);
    std::vector<unsigned> partial_path2 = find_path_between_intersections(midpathID, end_pathID, right_turn_penalty, left_turn_penalty);
   
    CourierSubpath modifiedPath;
    modifiedPath.start_intersection = begin_newpathID;
    modifiedPath.end_intersection = midpathID;
    modifiedPath.subpath = partial_path;
   
    newPath.push_back(modifiedPath);
   
    CourierSubpath modifiedEndPath;
    modifiedEndPath.start_intersection = midpathID;
    modifiedEndPath.end_intersection = end_pathID;
    modifiedEndPath.subpath = partial_path2;
   
    newPath.push_back(modifiedEndPath);
   
    //take route[k+1] to end and add them in order to new route
    for(unsigned n = k+1; n < oldPath.size(); n++) {
        newPath.push_back(oldPath[n]);
    }
}
 */





























