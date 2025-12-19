/***********************************************************************************************************************************************************
* Course: MECH 524
* Student Name: Connor McAllister
* Student Number: 65908436
* Date:
*
* Source File: navigation:environment.h
*
* Purpose:
* 
* Description:
* 
*************************************************************************************************************************************************************/


/************************************************************************* INCLUDES *************************************************************************/
#pragma once
#include <vector>
#include <memory>
#include <queue>

#include "grid.h"
#include "path.h"
#include "dwa_local_planner.h"
#include "dwa_local_planner_class_headers.h"
#include "ros_interface.h"

#include "ros/ros.h"

#include "utils.h"

/************************************************************************* STRUCTS **************************************************************************/


struct RobotParameters {
    double radius_robot;
    double resolution;
    DynamicWindowSampler::Limits limits;
    TrajectoryEvaluator::Weights weights;
    int N_v;
    int N_omega;
    int horizon;
    double dt_trajectory;
    double k_multiplier_lookahead;
    double dt_control;
    double radius_multiplier_distance_frontier_reached;
    double main_loop_rate;
    double distance_to_consider_frontier_reached;
  
    double minimum_clearance;
    double path_resolution;

    double error_threshold_for_reactive_rotation;
    double kp_for_rotate_for_angle;
    double angle_to_rotate_if_left_or_right_bumper_pressed;
    double angle_to_rotate_if_front_bumper_pressed;
    double backoff_linear_velocity_if_bumper_pressed;
    double backoff_duration_if_bumper_pressed;

    // Default constructor
    RobotParameters() 
        : radius_robot(0.0), resolution(0.0), limits{}, weights{},
          N_v(0), N_omega(0), horizon(0), dt_trajectory(0.0),
          k_multiplier_lookahead(0.0), dt_control(0.0),
          radius_multiplier_distance_frontier_reached(0.0),
          main_loop_rate(0.0), distance_to_consider_frontier_reached(0.0),
          minimum_clearance(0.0), path_resolution(0.0),
          error_threshold_for_reactive_rotation(0.0),
          kp_for_rotate_for_angle(0.0),
          angle_to_rotate_if_left_or_right_bumper_pressed(0.0),
          angle_to_rotate_if_front_bumper_pressed(0.0),
          backoff_linear_velocity_if_bumper_pressed(0.0),
          backoff_duration_if_bumper_pressed(0.0)
    {}

    RobotParameters(double radius, double res,
                    DynamicWindowSampler::Limits lim, TrajectoryEvaluator::Weights w,
                    int Nv, int Nw, int horiz, double dt_traj, double k, double dt_ctrl, double radius_multiplier, 
                    double error_thr, double kp_rotate, double angle_left_right, double angle_front, double backoff_velocity, double backoff_duration,
                    double path_res)

        :radius_robot(radius), resolution(res), limits(lim), weights(w), N_v(Nv), N_omega(Nw), horizon(horiz),
        dt_trajectory(dt_traj), k_multiplier_lookahead(k), dt_control(dt_ctrl), radius_multiplier_distance_frontier_reached(radius_multiplier),
        error_threshold_for_reactive_rotation(error_thr), kp_for_rotate_for_angle(kp_rotate), angle_to_rotate_if_left_or_right_bumper_pressed(angle_left_right),
        angle_to_rotate_if_front_bumper_pressed(angle_front), backoff_linear_velocity_if_bumper_pressed(backoff_velocity),
        backoff_duration_if_bumper_pressed(backoff_duration), path_resolution(path_res)

    {
        main_loop_rate = 1.0 / dt_control;
        distance_to_consider_frontier_reached = radius_multiplier_distance_frontier_reached * radius_robot;
        minimum_clearance = radius_robot*1.70;
        ROS_INFO("minimum clearance set to %f", minimum_clearance);
        //minimum_clearance = radius_robot*1.0;

    }
};

/*------------------------------------------------------------------ CompareFrontiers Struct -------------------------------------------------------------------
Purpose: provide sorting logic for frontiers based on proximity to current location
Notes: defined inline to allow use in NavigationEnvironment class*/


struct CompareFrontiers {
    
    Mapping::GlobalMapGrid* global_map;
    utils::MapIndex current_location;

    CompareFrontiers(){
        this->current_location = utils::MapIndex();
        this->global_map = nullptr;
    }

    CompareFrontiers(Mapping::GlobalMapGrid* global_map, utils::MapIndex current_location){
        this->global_map = global_map;
        this->current_location = current_location;
    }
    bool operator()(const utils::MapIndex& frontier1, const utils::MapIndex& frontier2) {
        if(global_map == nullptr) {
            return false;
        }
        return this->global_map->getEuclideanDistance(current_location, frontier1) > this->global_map->getEuclideanDistance(current_location, frontier2);
    }
};

/************************************************************************* CLASSES **************************************************************************/

/*------------------------------------------------------------------ NavigationRobot class -------------------------------------------------------------------
    Purpose:
         */

    class NavigationRobot {

        private: 

            Mapping::GlobalMapGrid* global_map;            
            

            //double minimum_clearance;
           struct RobotParameters Robot_Params;

            std::string id;
            Mapping::RobotLocalMap local_map;
            Mapping::DiscretePath discrete_path;
            Mapping::RealPath real_path;
            utils::MapIndex destination;
            DWALocalPlanner local_planner;
            std::vector<utils::MapPoint> global_path;

        public:

        std::unique_ptr<Mapping::PathPlanner> planner;
            std::unique_ptr<Mapping::PathInterpolator> interpolator;

            NavigationRobot(std::string id, Mapping::GlobalMapGrid* global_map, 
                Mapping::PathPlanner::PlannerType planner, Mapping::PathInterpolator::InterpolatorType interpolator, 
                RobotParameters Robot_Param, ROSInterface* ros_interface_ptr);
           
            // Public Methods
            void updateLocalMap();
            void generateNewGlobalPath(utils::MapIndex current_loc, utils::MapIndex new_frontier);
            void computeDWACommand();
            std::vector<utils::MapPoint> getRobotRealPath();
            DWALocalPlanner& getLocalPlanner();
            struct RobotParameters getRobotParams();
            utils::MapIndex getNearestAccessibleNode(utils::MapIndex idx); 
            utils::MapIndex getDestination() const;
            Mapping::RobotLocalMap getLocalMap() const;
            Mapping::GlobalMapGrid* getGlobalMap() const;

    
    };

    /*-------------------------------------------------------------- NavigationEnvironment class ------------------------------------------------------------
    Purpose:
         */

    class NavigationEnvironment {
        
        private:
           
            // @Stephen: Add ROS subscribers here
            ROSInterface ros_interface;
            // boolean indicating if environment has been set up to match parameters dictated by mapping ROS node
            bool initialized, has_robot;
            // Global Map
            Mapping::GlobalMapGrid global_map;
            // Robots within given environment - use unique_ptr to allow internal unique_ptr use
            std::unique_ptr<NavigationRobot> robot;
            // List of frontiers, sorted by proximity to robot at last update
            std::priority_queue<utils::MapIndex, std::vector<utils::MapIndex>, CompareFrontiers> frontiers;
        public:

            // Constructors
            NavigationEnvironment(ros::NodeHandle& nh, double resolution);
            void updateEnvironment(std::vector<std::vector<int>> occupancy, std::vector<std::vector<int>> visits);

            // Public Methods
            void addRobot(std::string id, Mapping::PathPlanner::PlannerType planner_type, Mapping::PathInterpolator::InterpolatorType interpolator_type, RobotParameters Robot_Params, ros::NodeHandle& nh);
            void updateEnvironment();
            void updateFrontiers();
            void reSortFrontiers();
            utils::MapIndex getNearestFrontier();
            void mapSurroundings(std::string robot_id, Mapping::PathPlanner::PlannerType planner_type, Mapping::PathInterpolator::InterpolatorType interpolator_type, RobotParameters robot_params);
            void updateRobotPath();


 };
