/**********************************************************************************

Name: Xuezheng Chen 
Student ID: 32470387 
File: occupancy_grid_node.h 
Purpose: Contains the main function and helper functions for the occupancy grid node. 
Description:  Build and update an occupancy grid,
                subscribe to bumper and laser scanner for sensor data, 
                publish the occupancy grid in the "occupancy_grid" topic.

**********************************************************************************/

/*********************************** INCLUDES ***********************************/

#pragma once
#include <chrono>
#include <tf/transform_broadcaster.h>
#include "occupancy_grid.h"
#include "utils.h"
#include "node.h"

/***************************** CLASS DEFINITIONS ******************************/

namespace ROS {

/*
Class: OccupancyGridNode
Purpose: Build a ROS node to obtain info from the simulator, build the occupancy grid and publish the grid for downstream tasks
*/
class OccupancyGridNode : public ROS::Node { // Inherited from an abstract class

private:

    // // A handle is required by ROS to intialize a node, required for every node
    // ros::NodeHandle nh;

    // Subscribe to topics to obtain message
    ros::Subscriber odom_subscriber;
    ros::Subscriber laser_subscriber;
    ros::Subscriber bumper_subscriber;

    // Publish message to a topic
    ros::Publisher grid_publisher;
    ros::Publisher grid_visit_publisher;
    ros::Publisher frontier_publisher;
    ros::Publisher frontier_list_publisher;

    // Initialize a grid object
    OccupancyGrid grid;

    // Callback methods triggered after receiving messages
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    ros::Time bumper_last_press_time;
    void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

public:
    // Constructor
    OccupancyGridNode();

    // Publish the occupancy grid
    void publishMapToOdomTransform();

    /*
    Function: spin
    Arguments: None
    Purpose: The while loop to keep the node running.
    Notes: Override the method of the abstract class
    */
    void spin() override;
    
    std::vector<nav_msgs::OccupancyGrid> getGridMsg();

    // Save the occupancy map as a PGM file
    void saveAsPGM(const nav_msgs::OccupancyGrid& grid, const std::string& filename_base);

};

}
