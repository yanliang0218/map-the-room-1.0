/**********************************************************************************

Name: Xuezheng Chen 
Student ID: 32470387 
File: occupancy_grid.h 
Purpose: Contains the OccupancyGrid class declaration. 
Description: The OccupancyGrid class build, maintain, and update an occupancy grid.

**********************************************************************************/

/*********************************** INCLUDES ***********************************/

#pragma once
#include <fstream>
#include <ros/ros.h>
#include <cmath>
#include <vector>
#include <utility>
#include <algorithm>
#include <sensor_msgs/LaserScan.h>
#include <kobuki_msgs/BumperEvent.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Polygon.h>
#include <tf/tf.h>
#include "utils.h"

/***************************** CLASS DEFINITIONS ******************************/

struct LaserPoint {

public:

    double x, y; // world x and y
    bool is_occupied; 

};    

class OccupancyGrid {

private:

    // The size and resolution of the grid map
    int num_row;
    int num_column;
    double resolution;
    utils::MapPoint map_origin;

    // The coordinate and orientation of robot
    double robot_coord_x, robot_coord_y, robot_theta;

    // Stores the value of the map and the fronters
    utils::Map grid_occupancy_data;
    utils::Map grid_visit_data;

    // The transform matrix between the occupancy map frame and robot's /odom frame
    tf::Transform transform;

    // The offset from the center of the robot to its rim
    double robot_offset;

    // Helper function for laser message
    void markLineFree(int x0, int y0, int x1, int y1);
    std::vector<LaserPoint> preprocessLaserScan(const sensor_msgs::LaserScan::ConstPtr& msg);
    void smoothMap();
    utils::Map averageSmooth();
    utils::Map majoritySmooth();

public:

    // Constructor
    OccupancyGrid(int map_height, int map_width, double cell_resolution);

    // Getter functions
    int getNumRows();
    int getNumColumns();
    double getResolution();
    utils::MapPoint getMapOrigin();
    double getRobotOffset();
    utils::Map gridGridOccupancyData();
    utils::Map getGridVisitData();
    tf::Transform getTransform();
    std::vector<utils::MapPoint> getFrontiers();
    double getRobotX();
    double getRobotY();
    double getRobotTheta();

    // Setter functions
    void setCell(const int row, const int col, const int val);
    void setGridData(const utils::Map new_map);
    void updateVisitCell();
    void setRobotX(const double new_x);
    void setRobotY(const double new_y);
    void setRobotTheta(const double new_theta);

    // Update occupancy map and robot status with messages from subscribed nodes
    void updateFromLaserData(const sensor_msgs::LaserScan::ConstPtr& msg);
    void updateFromBumperData(const kobuki_msgs::BumperEvent::ConstPtr& msg);
    void updateFromOdomData(const nav_msgs::Odometry::ConstPtr& msg);

    /*
    Function: findClosestFrontier
    Arguments: None 
    Returns: Two ROS messages, one is the closest frontier point, one is the list of all frontiers.
    */
    utils::FrontierResult findClosestFrontier();

    /*
    Function: toRosMsgs
    Arguments: None 
    Returns: The ROS message type of occupancy map and occupancy visit map are stored in a vector.
    */
    std::vector<nav_msgs::OccupancyGrid> toRosMsgs();
    // Helper function for ROS message conversion
    nav_msgs::OccupancyGrid toOccupancyGridMsg(std::string frame_id, utils::Map data_to_convert);

    /*
    Function: printMap
    Arguments: the discrete map  
    Behaviour: Iterates through the map and print out the value of each cell.
    */
    void printMap(const utils::Map& map);

};