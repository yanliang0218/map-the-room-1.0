/**********************************************************************************

Name: Xuezheng Chen 
Student ID: 32470387 
File: occupancy_grid_node.cpp 
Purpose: Contains the main function and helper functions for the occupancy grid node. 
Description:  Build and update an occupancy grid,
                subscribe to bumper and laser scanner for sensor data, 
                publish the occupancy grid in the "occupancy_grid" topic.

**********************************************************************************/

/*********************************** INCLUDES ***********************************/

#include "occupancy_grid_node.h"

/***************************** CLASS IMPLEMENTATIONS ******************************/

namespace ROS {

// The idea of making the node a class is adapted from Gen AI
/*
Description: Default Constructor of the class. 
                Building a discrete occupancy grid with length, width equal to 15 meters, with 0.05 m resolution 
Arguments:  None
Behavior: Construct the subscribers and publishers. 
            Subscribes to the bumper and laser scanner.
            Publish occupancy grid in the topic "occupancy_grid", it tells whether a cell is unknown, free, or occupied.
            Publish occupancy visit grid in the topic "occupancy_visit_map", it tells how many times a cell is visited.
            Publish the closes frontier in the topic "frontier", it gives the next target frontier point to explore.
*/
OccupancyGridNode::OccupancyGridNode() : grid(15, 15, 0.05) {

    // Initialize subscibers
    // Subscibe to /odom topic to get robot's current position 
    odom_subscriber = nh.subscribe("/odom", 10, &OccupancyGridNode::odomCallback, this);
    // Subscibe to /scan topic to get laser data to build occupancy map 
    laser_subscriber = nh.subscribe("/scan", 10, &OccupancyGridNode::laserCallback, this);
    // Subscibe to bumper topic to get robot's bumper data and built fine-grained map in case collision happens 
    bumper_subscriber = nh.subscribe("mobile_base/events/bumper", 10, &OccupancyGridNode::bumperCallback, this); // The use of this is adapted from Gen AI
    
    // Initialize publishers
    // Publish the occupancy data
    grid_publisher = nh.advertise<nav_msgs::OccupancyGrid>("occupancy_grid", 10);
    // Publish how many times a cell is visited
    grid_visit_publisher = nh.advertise<nav_msgs::OccupancyGrid>("occupancy_visit_map", 10);
    // Publish the closest frontier (a free cell with at least one of its eight neighboring cells is an unknown cell)
    frontier_publisher = nh.advertise<geometry_msgs::Point>("frontier", 10);
    // Publish the full list of frontiers
    frontier_list_publisher = nh.advertise<geometry_msgs::Polygon>("frontier_list", 10);

    // Start timer to make sure not updating bumper message too frequently
    bumper_last_press_time = ros::Time(0); // Adapted from Gen AI

}

/*
Function: odomCallback
Arguments:  - msg: Odom message.
Behavior: The callback function for odom message, called when odom message is passed to the node.
                Update robot's x, y, theta with the message.
*/
void OccupancyGridNode::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {   
	grid.updateFromOdomData(msg);
}

/*
Function: laserCallback
Arguments:  - msg: Laser message.
Behavior: The callback function for laser message, called when laser message is passed to the node.
            Update the occupancy grid with laser message.
*/
void OccupancyGridNode::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) { 
	grid.updateFromLaserData(msg);
}

/*
Function: bumperCallback
Arguments:  - msg: Bumper message.
Behavior: The callback function for laser message, called when bumper message is passed to the node.
                Filter the bumper message from frequent switches from 0 to 1 observed when echoing the topic.
                A new press is recorded after at least 0.5s after the last press.
                Update the occupancy grid with bumper event.
*/
void OccupancyGridNode::bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg) {

    // ros::Time now = ros::Time::now();

    // // Bumper message is only published when there is a state change between PRESSED and RELEASED
    // if (msg->state == kobuki_msgs::BumperEvent::PRESSED){  // The use of -> is adapted from Gen AI

	//     // If enough time has passed since last press
    //     if ((now - bumper_last_press_time).toSec() > 0.5){
            
    //         // Update the bumper timer
    //         bumper_last_press_time = now;
    //         grid.updateFromBumperData(msg);

    //     }
    // } 
}

// Adapted from ROS Documentation
/*
Function: publishMapToOdomTransform
Arguments:  None
Behavior: Broadcast the transformation matrix from the occupancy_grid frame to odom frame.
                It can be used to obtain the coordinate of the robot in the occupancy map frame from the coordinate in odom frame.
                The odom transform should be listened from all nodes that subscribe to the /odom topic, 
                to guarantees correct coordinate is used. 
*/
void OccupancyGridNode::publishMapToOdomTransform() { //check odom callback

    // Broadcaster object
    static tf::TransformBroadcaster br;

    // Broadcast the transformation matrix from the occupancy_grid frame to odom frame.
    br.sendTransform(tf::StampedTransform(grid.getTransform(), ros::Time::now(), "occupancy_map", "odom"));

}

/*
Function: getGridMsg
Arguments:  None
Behavior: Returns the most up-to-date grid_occupancy_data and grid_visit_data.
            Primarily used to return message to save the map as .pgm file.
*/
std::vector<nav_msgs::OccupancyGrid> OccupancyGridNode::getGridMsg() {
    return grid.toRosMsgs();
}

/*
Function: spin
Arguments:  None
Behavior: Running the node.
            Count the time.
            Publish the occupancy grid, occupancy visit count and closest frontier.
*/
void OccupancyGridNode::spin() {

    ros::Rate loop_rate(5.0); // 5 message per second
    auto start = std::chrono::system_clock::now();
    double secondsElapsed = 0.0;

    // Run the node loop for at most 10 minutes
    while(ros::ok() && secondsElapsed <= 3000) {

        ros::spinOnce();

        // Convert and publish the occupancy grid
        std::vector<nav_msgs::OccupancyGrid> msgs_to_publish = grid.toRosMsgs(); // Adapted from Gen AI
        grid_publisher.publish(msgs_to_publish[0]);
        grid_visit_publisher.publish(msgs_to_publish[1]);

        // Publish the closest frontier if queried
        // Publishing constantly for now
        utils::FrontierResult frontier_results = grid.findClosestFrontier();
        frontier_publisher.publish(frontier_results.frontier_point);
        frontier_list_publisher.publish(frontier_results.frontier_list);

        // Publish the map->odom transform
        publishMapToOdomTransform();

        // Update the timer
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        loop_rate.sleep();

    }
}

// Adapted from Gen AI
/*
Function: saveAsPGM
// Warning: Every time it will replace the last map.
Arguments:  - grid: The occupancy grid to save.
            - filename: The name of the saved file.
Behavior: Save the occupancy grid map to two files, the PGM file and the YAML metadata file.
            The saved map can be opened by Rviz.
*/
void OccupancyGridNode::saveAsPGM(

    const nav_msgs::OccupancyGrid& grid, 
    const std::string& filename

) {

    // 1. Output PGM file
    std::string pgm_file = filename + ".pgm";
    std::ofstream pgm(pgm_file, std::ios::out | std::ios::binary);

    int width = grid.info.width;
    int height = grid.info.height;

    pgm << "P5\n" << width << " " << height << "\n255\n";

    for (int y = 0; y < height; y++){
        for (int x = 0; x < width; x++){
            int i = x + (height - y - 1) * width;  // flip Y for ROS map format
            int8_t val = grid.data[i];

            uint8_t pixel;
            if (val == 0) pixel = 254;         // free → light
            else if (val == 100) pixel = 0;    // occupied → black
            else pixel = 205;                  // unknown → grey

            pgm.write((char*)&pixel, 1);
        }
    }

    pgm.close();

    // 2. Output YAML metadata file
    std::string yaml_file = filename + ".yaml";
    std::ofstream yaml(yaml_file);

    yaml <<
        "image: " << filename << ".pgm\n"
        "resolution: " << grid.info.resolution << "\n"
        "origin: [" << grid.info.origin.position.x << ", "
                    << grid.info.origin.position.y << ", 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.196\n";

    yaml.close();

    std::cout << "Saved map: " << pgm_file << " and " << yaml_file << std::endl;

}

}

/***************************** FUNCTIONS DEFINITIONS ******************************/

/*
Function: main
Arguments: - argc: Argument Count
            - argv: Argument Vector
Returns: 0 on success, non-zero on fail (int) 
Behavior: The main function of the occupancy grid node. The entry point of the node.
*/
int main(int argc, char** argv) {

    // Initialize node
    ros::init(argc, argv, "occupancy_grid_node");

    // Create an OccupancyGridNode to subscribe, process, and publish
    ROS::OccupancyGridNode node;

    // Running the loop
    node.spin();

    // Obtain the final occupancy map message and save it locally. 
    // Warning: Every time it will replace the last map.
    std::vector<nav_msgs::OccupancyGrid> final_msg = node.getGridMsg();
    node.saveAsPGM(final_msg[0], std::string(getenv("HOME")) + "/map");

    return 0;
    
}
