

/**********************************************************************************
/**********************************************************************************
Name: Liang Yan
Student Name: 33140351 
File: ros_interface.h
Purpose: Contains the ROSInterface class definitions
Description: from occupancy grid, odometry, bumper, frontier topics, we update the internal state of the robot
             such as current pose, current velocities, occupancy grid, visit count grid, bumper states, and nearest frontier point.
             These are then used by the both  global planner and local planner to to plan a global path and compute velocity commands, respectively.
**********************************************************************************/
#pragma once
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <kobuki_msgs/BumperEvent.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>


#include <fstream>
#include <utility>
#include <cmath>
#include <vector>
#include <stdexcept>
#include "dwa_local_planner_class_headers.h"
#include "utils.h"



/***************************** CLASS DECLARATION************************************/
class ROSInterface //this has be merged with robot class!
{
    private:
        //private member for map resolution
        double resolution;

        //private member for ros_interface
        ros::NodeHandle& nh;    

        //subcribers and publishers for ROS topics
        ros::Subscriber odom_sub; 
        ros::Subscriber map_sub;
        ros::Subscriber visit_count_sub;
        ros::Subscriber frontier_sub;
        ros::Subscriber frontier_list_sub;
        ros::Subscriber bumper_sub;
 
        //tranform variable and listener for the transform between the odom frame and the occupancy grid frame
        tf::StampedTransform odom_to_map_transform; 
        tf::TransformListener tf_listener; 

        // Callbacks
        void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
        void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
        void visitCountCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
        void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg);
        void frontierCallback(const geometry_msgs::Point::ConstPtr& msg);
        void frontierListCallback(const geometry_msgs::Polygon::ConstPtr& msg);


        // internal variables for robot state
        utils::Pose pose;
        utils::MapIndex current_position_index;
        double v_curr;
        double omega_curr;
        
        nav_msgs::OccupancyGrid current_grid;
        std::vector<std::vector<int>> current_converted_2D_grid;

        nav_msgs::OccupancyGrid visit_count_grid;   
        std::vector<std::vector<int>> visit_count_converted_2D_grid;
        
        uint8_t bumper[3];
        geometry_msgs::Point current_frontier;
        geometry_msgs::Polygon current_frontier_list;
        
        utils::MapIndex current_frontier_index;
        std::vector<utils::MapIndex> current_frontier_index_list;

        //if we boolean state variables for robot internal states
        bool has_occupancy_grid;
        bool has_visit_count_grid;
        bool transform_ready;
        bool has_bumper_data;
        bool bumper_pressed_and_needs_reset;
        bool has_frontier;
        bool has_frontier_list;

 
    public:
        //constructor
        ROSInterface(ros::NodeHandle& nh, double resolution);

        //calculates the current distance to destination
        double distance_to_frontier() const;

        //getter for map resoliution 
        double get_resolution() const;

        //getter for the node handle
        ros::NodeHandle& getNodeHandle();

        //getter for the robot current pose
        utils::Pose get_pose() const;

        //getter for map index for the current robot postion
        utils::MapIndex get_current_position_index() const;

        //getters for the current linear and angular velocity
        double get_v_curr() const;
        double get_omega_curr() const;

        //getters for the current raw occupancy and visit count grids
        //and the same grids converted into 2D vectos
        const nav_msgs::OccupancyGrid& get_current_grid() const;
        const std::vector<std::vector<int>>& get_current_converted_2D_grid() const;
        const nav_msgs::OccupancyGrid& get_visit_count_grid() const;
        const std::vector<std::vector<int>>& get_visit_count_converted_2D_grid() const;

        //getter for the current bumper state
        const uint8_t* get_bumper() const;

        //getters for the current frontier and frontier list in 
        //real coordinates and map indices
        geometry_msgs::Point get_current_frontier() const;
        utils::MapIndex get_current_frontier_index() const;
        const geometry_msgs::Polygon& get_current_frontier_list() const;
        const std::vector<utils::MapIndex>& get_current_frontier_index_list() const;

        //getter and setter for boolean  indicators
        bool get_has_occupancy_grid() const;
        bool get_has_visit_count_grid() const;
        bool get_has_bumper_data() const;
        bool get_bumper_pressed_and_needs_reset() const;
        void set_bumper_pressed_and_needs_reset(bool value);
        bool get_has_frontier() const;
        bool get_has_frontier_list() const;

};
