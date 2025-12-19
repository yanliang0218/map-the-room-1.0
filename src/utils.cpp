/**********************************************************************************

Name: 
Xuezheng Chen 32470387, 
Connor McAllister 65908436
File: utils.cpp
Purpose: Contains the implementation of helper functions for the entire project.  
Description:  The helper function that is going to be shared by all team members in the project.

**********************************************************************************/

/*********************************** INCLUDES ***********************************/

#include "utils.h"

/***************************** FUNCTIONS DEFINITIONS ******************************/

namespace utils {

/*
Function: MapPoint default constructor
Arguments: none
Behaviour: initializes empty MapPoint with default (-1, -1) x y*/
MapPoint::MapPoint() {
    this->x = -1;
    this->y = -1;
}

/*
Function: MapPoint default constructor
Arguments: double x y inputs
Behaviour: initializes empty MapPoint with x y*/
MapPoint::MapPoint(double x, double y) {
    this->x = x;
    this->y = y;
}

/*
Function: toString
Arguments: none
Returns: this as formatted string */
std::string MapPoint::toString() {
    return "(" + std::to_string(x) + ", " + std::to_string(y) + ")";
}

/*
Function: MapIndex default constructor
Arguments: none
Beahviour: initializes default row col values of -1 */
MapIndex::MapIndex() {
    this->row = -1;
    this->col = -1;
}

/*
Function: MapIndex constructor
Arguments: integer row and col indices
Behaviour: initializes row and col from inputs */
MapIndex::MapIndex(int row, int col) {
    this->row = row;
    this->col = col;
}

/*
Function: operator==
Arguments: other MapIndex instance
Returns: bool indicating if row and col fields match */
bool MapIndex::operator==(const MapIndex& other_index) const {
    return row == other_index.row && col == other_index.col;
}

/*
Function: toString
Arguments: none
Returns: this as formatted string */
std::string MapIndex::toString() {
    return "(" + std::to_string(row) + ", " + std::to_string(col) + ")";
}

/*
Function: discreteMapBresenhamLineSearch
Description: Bresenham's line algorithm to mark free cells. 
Arguments:  - x0, y0: The row and column number of the starting point.
            - x1, y1: The row and column number of the end point.
            - num_row, num_column: The row and column number of the entire map, used as grid boundaries.
Returns: All points in the line between the starting point and the end point will be returned.
            The returned points will be in pairs of (row_number, column_number), equivalent to (y position, x position)
*/
std::vector<MapIndex> discreteMapBresenhamLineSearch(int x0, int y0, int x1, int y1, int num_row, int num_column) {

    std::vector<MapIndex> pointsInBetween;

    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int err = dx - dy; // the accumulated error of how much the pixel path has drifted from the ideal geometric line
    int sx, sy;

    if (x0 < x1){
        sx = 1;
    } else{
        sx = -1;
    }
    if (y0 < y1){
        sy = 1;
    } else{
        sy = -1;
    }

    int x = x0;
    int y = y0;

    while (true) {

        if (x != x1 || y != y1) {

            // Clamp to grid boundaries to avoid out-of-bounds
            int cx = clamp(x, 0, num_column - 1);
            int cy = clamp(y, 0, num_row - 1);

            pointsInBetween.push_back(MapIndex(cy, cx));
            
        } else {
            break;
        }

        int e2 = 2 * err;

        // This means the line is closer to the next x coordinate
        if (e2 > -dy) { 
            err -= dy;
            x += sx;
        }
        
        // This means the line is closer to the next y coordinate
        if (e2 < dx) { 
            err += dx;
            y += sy;
        }
    }

    return pointsInBetween;

}

/*
Function: wrapAngle
Description: Clamp the difference of angles to within (-PI, PI) 
                while keep the angles identical.
Arguments:  - target_theta: the desired heading of the robot
            - current_theta: the current heading of the robot
Returns: The wrapped angle within (-PI, PI)
*/
double wrapAngle(double diff) {



    // Guarding against NaN
    if (std::isnan(diff)) {
        return 0.0;
    }

    // If positive, decrease the difference by 2 * M_PI to smaller than M_PI
    while (diff > M_PI) {
        diff -= 2 * M_PI;
    }

    // If negative, increase the difference by 2 * M_PI to greater than - M_PI
    while (diff < - M_PI) {
        diff += 2 * M_PI;
    }
    
    return diff;

}

/*
Function: fromOccupancyGridMsg
Arguments:  - grid: The ROS message of data type nav_msgs::OccupancyGrid.
Returns: The 2D discrete map converted from ROS message.
*/
Map fromOccupancyGridMsg(const nav_msgs::OccupancyGrid& msg) {

    // Extract info
    auto data_num_column = msg.info.width;
    auto data_num_row = msg.info.height;
    auto& data = msg.data;

    Map reconstructedGrid{data_num_row, std::vector<int>(data_num_column, -1)};
    
    // Reconstruct map by converting a 1D vector of int to 2D matrix of int
    for (int i = 0; i < data_num_row; i++) {

        for (int j = 0; j < data_num_column; j++) {

            reconstructedGrid[i][j] = data[i * data_num_column + j];

        }

    }

    return reconstructedGrid;

}

}