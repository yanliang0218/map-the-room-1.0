/**********************************************************************************

Name: 
Xuezheng Chen 32470387, 
Connor McAllister 65908436
File: utils.h 
Purpose: Contains the helper functions and self-defined data structures for the entire project. 
Description: The helper functions and self-defined data structures that is going to be shared by all team members in the project.

**********************************************************************************/

/*********************************** INCLUDES ***********************************/

#pragma once
#include <vector>
#include <cmath>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Polygon.h>


/***************************** CONSTANTS DEFINITIONS ******************************/

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/***************************** METHOD DEFINITIONS ******************************/

namespace utils {

/* Data structures */

using Map = std::vector<std::vector<int>>;

// Hinted by Gen AI
/*
Purpose: Store the real coordinate of a point on the map
*/
struct MapPoint {

public:

    MapPoint();
    MapPoint(double x, double y);
    double x, y; // world x and y

    // Public methods
    std::string toString();
    void print();

};

/*
Purpose: Store the row and column number of a cell on the discrete occupancy map
*/
struct MapIndex {

public:

    MapIndex();
    MapIndex(int row, int col);
    int row, col; // row and column indices

    void print();
    std::string toString();
    
    bool operator==(const MapIndex& other_index) const;

};

/*
Purpose: Store the pose of the robot
*/
struct Pose {

public:

    double x;
    double y;
    double theta;

};

/*
Purpose: Store the closest frontier point and the list of all frontiers
*/
struct FrontierResult {

public:

    geometry_msgs::Point frontier_point;
    geometry_msgs::Polygon frontier_list;

};

/* Utility functions */

/*
Description: An utility function to clamp the value between the min and max.
                std::clamp is introduced after C++ 17, this exist in case C++ 11 is used.
Arguments:  - val: The input value.
            - min: The minimum value.
            - max: The maximum value.
Returns: The clamped value between min and max.
*/
template <typename T>
T clamp(T val, T min, T max) {
    
    T val_to_return;

    if (val < min) {
        val_to_return = min;
    } else if (val > max) {
        val_to_return = max;
    } else { 
        val_to_return = val;
    }

    return val_to_return;

}

double wrapAngle(double diff);

/*
Function: discreteMapBresenhamLineSearch
Description: Bresenham's line algorithm to find points in a discrete map. 
Arguments: The starting point and end point in a discrete map. The number of rows and columns of the map.
Returns: All points in the line between the starting point and the end point.
*/
std::vector<MapIndex> discreteMapBresenhamLineSearch(int x0, int y0, int x1, int y1, int num_row, int num_column);

//Convert the OccupancyGrid ROS message to a 2D Map
Map fromOccupancyGridMsg(const nav_msgs::OccupancyGrid& msg);

}