/**********************************************************************************
Name: Liang Yan
Student Name: 33140351 
File: map_the_room/dwa_local_planner.h
Purpose: Contains the PathTracker class definition. 
Description: performs the dwa pipeline - evaluates each trajectory and 
             scores for each trajectory and return the best scoring trajectory
**********************************************************************************/
#pragma once
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <kobuki_msgs/BumperEvent.h>


#include "occupancy_grid.h"
#include "occupancy_grid_node.h"
//#include "navigation_environment.h"
#include "grid.h"
#include "path.h"
#include "utils.h"

#include "dwa_local_planner_class_headers.h"
#include "ros_interface.h"



/***************************** CLASS DECLARATION************************************/

/*
Function: constructor for the DWAlocalPlanner class
Arguments: none
Behaviour: initializes ros_interface, and individual dwa modules, and tunable dwa parameters
*/
class DWALocalPlanner {
public:
    // ===== Default constructor =====
    DWALocalPlanner()
        : ros_interface(nullptr),
          sampler(DynamicWindowSampler::Limits(), 10, 10),
          traj_generator(2.0, 0.1),
          obs_eval(0.05, 0.2),
          path_tracker(1.0),
          traj_evaluator(TrajectoryEvaluator::Weights(), 1.0),
          dt_control(0.1),
          has_path(false),
          error_threshold_for_reactive_rotation(5.0),
          kp_for_rotate_for_angle(2.0),
          angle_to_rotate_if_left_or_right_bumper_pressed(20.0),
          angle_to_rotate_if_front_bumper_pressed(40.0),
          backoff_linear_velocity_if_bumper_pressed(-0.1),
          backoff_duration_if_bumper_pressed(1.0),
          distance_to_consider_frontier_reached(0.5)
    {
        // No ROS initialization because ros_interface == nullptr
    }

    private:
         //private member to hold the current real path from A* planner
        std::vector<utils::MapPoint> current_path; \

        //private members to hold every module in dwa pipeline
        //see dwa_local_planner_class_headers.h for detailed description of each module
        ROSInterface* ros_interface;
        DynamicWindowSampler sampler;
        TrajectoryGenerator traj_generator;
        ObstacleEvaluator obs_eval;
        PathTracker path_tracker;
        TrajectoryEvaluator traj_evaluator;

        //main loop control duration per cycle - synced with occupancy grid node
        double dt_control;

        //boolean flag for whether we have a current path
        bool has_path;

        //tunable parameters needed by the dwa pipeline
        double error_threshold_for_reactive_rotation;
 
        //in rotate_for_angle method, in the last 1/4 of the total angle to turn 
        //angular velocity is swtiched from a constant command to proportional
        //to the angle remaining to turn
        double kp_for_rotate_for_angle;
        double angle_to_rotate_if_left_or_right_bumper_pressed;
        double angle_to_rotate_if_front_bumper_pressed;

        double backoff_linear_velocity_if_bumper_pressed;
        double backoff_duration_if_bumper_pressed;

        //threshold to the frontier to consider the frontier has been reached
        double distance_to_consider_frontier_reached;

         //private member to hold the last angular command before dwa local minimum 
         double last_cmd_angular;
        
        //publisher for command velocities
        ros::Publisher cmd_pub;

        //publisher for real path for visualization and debugging
        ros::Publisher real_path_pub;
    


 
    public:
         //tunable parameters passed to the class, and private members of ros_interface, robot dynamic limits
        //weights for each individual score for each trajectory
        DWALocalPlanner(ROSInterface* ros_interface_ptr,
                        double resolution,
                        double radius_robot,
                        const DynamicWindowSampler::Limits& limits,
                        const TrajectoryEvaluator::Weights& weights,
                        int N_v,
                        int N_omega,
                        double horizon,
                        double dt_trajectory,
                        double k_multiplier_lookahead,
                        double dt_control,
                        double radius_multiplier_distance_frontier_reached,
                        double error_threshold_for_reactive_rotation,
                        double kp_for_rotate_for_angle,
                        double angle_to_rotate_if_left_or_right_bumper_pressed,
                        double angle_to_rotate_if_front_bumper_pressed,
                        double backoff_linear_velocity_if_bumper_pressed,
                        double backoff_duration_if_bumper_pressed
                    );  
        
    
        // Main loop called by a ROS timer
        //dt_control is synced with the occupancy grid node update rate
        void Run_DWA();

        //calculate a cmd vel from each main loop cycle at dt_control step size
        geometry_msgs::Twist computeCommand();

        //rotate for specified angle
        void rotate_for_angle(float angle_in_deg);

        //recovery procedures from bumping into obstacles
        void bumper_recovery();

        //getter for ros_interface object to retrieve robot states, and grids
        ROSInterface* getROSInterface() const;

        //getter for the threshold distance to the frontier to consider it has been reached
        double get_distance_to_consider_frontier_reached() const;

        //getter for the has path boolean indicator
        bool get_has_path() const;

        //setter for the has path method
        void set_has_path(bool need_new_path);

        //getter for the current real path
        const std::vector<utils::MapPoint>& get_current_path() const;

        //setter for the current real path from global planning node to th local planner object
        void set_current_path(const std::vector<utils::MapPoint>& new_path);

        //helper function to visualize the real path
        void publishRealPath(const std::vector<utils::MapPoint>& path, const ros::Publisher& path_pub);

        //getter method for the path publisher
        const ros::Publisher& getPathPublisher() const;

};

