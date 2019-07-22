# Map-application--ECE297

A mapping application that:
- visually displays various city informations: intersections, buildings, natural landmarks/parks
- powerful search function that visually displays the point of interest entered in the UI
- Calculates and highlights the shortest streets path between two intersections in less than a second*, left and right arrow key to travel along path
- Finds near-minimum distance path when visiting multiple intersections, under special conditions**.

*: Dijkstra's algorithm execution time for two nodes on opposing edges of city of Toronto
**: Must visit some intersections before others, travel route limited by imposed restrictions; details in m4 multinode => m4 handout

Second year University of Toronto Computer Engineering project
