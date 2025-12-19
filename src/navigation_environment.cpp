/***********************************************************************************************************************************************************
* Course: MECH 524
* Student Name: Connor McAllister
* Student Number: 65908436
* Date: 
*
* Source File: navigation_environment.cpp
*
* Purpose:
* Define and implement NavigationEnvironment and NavigationRobot classes
* 
* Description:
* This file defines NavigationRobot and NavigationEnvironment objects. The 
* NavigationEnvironment class represents the room or area that is being mapped, 
* and owns the global map and all robots. In its current state, this application
* is designed to map with one robot-> The NavigationRobot class contains all
* data and behaviour pertaining to an individual robot that plans its own path.
* 
*************************************************************************************************************************************************************/


/************************************************************************* INCLUDES *************************************************************************/
#include "navigation_environment.h"
#include "ros/ros.h"

#include "path.h"
#include "grid.h"

#include <vector>
#include <queue>
#include <memory>
#include <cstdlib>

using namespace utils;

/************************************************************************* CLASSES **************************************************************************/


/*----------------------------------------------------------------- NavigationRobot class --------------------------------------------------------------------
Purpose:
    This class contains all information pertiaining to an individual robot-> 
    Each robot instance contains its own physical properties, such as minumum 
    obstacle clearance. This is used to define accessible nodes within its 
    own local map, and generate its own discrete and real global paths within
    the environment.
    */


/*
Function: NavigationRobot constructor
Arguments:
    - id: unique string id used to identify robot
    - Global Map: pointer to global map to reference 
    - PathPlanner type from Mapping::PathPlanner::PlannerType enum
    - PathInterpolator type from Mapping::PathInterpolator::InterpolatorType enum
    - minimum clearance relative to odometer
    - approximate path resolution for position and velocity commands 
Behaviour:
    - initializes internal global_map, id and minimum clearance fields
    - Creates new internal PathInterpolator and PathPlanner objects to match inputs
    - Creates new internal local_map, discrete_path and real_path instances corresonding to global map
Notes:
    unique_ptr is used to create PathPlanner and PathInterpolator instances, such that different planner and interpolator
    types can be used polymorphically when creating a NavigationRobot instance */
NavigationRobot::NavigationRobot(std::string id, Mapping::GlobalMapGrid* global_map, Mapping::PathPlanner::PlannerType planner_type, Mapping::PathInterpolator::InterpolatorType interpolator_type, 
    RobotParameters Robot_Params, ROSInterface* ros_interface_ptr)
    : id(id),
      Robot_Params(Robot_Params),
      global_map(global_map),
      local_map(global_map, Robot_Params.minimum_clearance),
      discrete_path(global_map, &local_map),
      local_planner(ros_interface_ptr, Robot_Params.resolution, Robot_Params.radius_robot,
                    Robot_Params.limits, Robot_Params.weights,
                    Robot_Params.N_v, Robot_Params.N_omega,
                    Robot_Params.horizon, Robot_Params.dt_trajectory,
                    Robot_Params.k_multiplier_lookahead,
                    Robot_Params.dt_control,
                    Robot_Params.radius_multiplier_distance_frontier_reached,
                    Robot_Params.error_threshold_for_reactive_rotation,
                    Robot_Params.kp_for_rotate_for_angle,
                    Robot_Params.angle_to_rotate_if_left_or_right_bumper_pressed,
                    Robot_Params.angle_to_rotate_if_front_bumper_pressed,
                    Robot_Params.backoff_linear_velocity_if_bumper_pressed,
                    Robot_Params.backoff_duration_if_bumper_pressed),
      destination({0,0})
{

    // Instantate path planner of appropriate type
    switch(planner_type) {

        case(Mapping::PathPlanner::PlannerType::ASTAR) : {
            this->planner = std::make_unique<Mapping::AStarPlanner>(this->global_map, &this->local_map);
            break;
        }

        default : {
            throw std::invalid_argument("PathPlanner type is not defined in PathPlanner::PlannerType");
        }
    }

    // Create PathInterpolator corresponding to type
    switch(interpolator_type) {

        case(Mapping::PathInterpolator::InterpolatorType::LINEAR) : {
            this->interpolator = std::make_unique<Mapping::LinearInterpolator>(Robot_Params.path_resolution); 
            break;
        }

        default : {
            throw std::invalid_argument("PathInterpolator type is not defined in PathInterpolator::InterpolatorType");
        }
    }

    // create new real path corresponding to discrete path and interpolator 
    this->real_path = Mapping::RealPath(&this->discrete_path, this->interpolator.get());
}

/* 
Function: updateLocalMap
Arguments: vector of utils::MapPoint instances containing coordinates of newly occupied indices
Behaviour: runs a bush fire update of local map instance */
void NavigationRobot::updateLocalMap() {
    this->local_map.bushFireUpdate();

}

/*
Function: generateNewFlobalPath
Arguments: utils::MapIndex structs containing current locaiton and nearest frontier to map to
Behaviour: updates internal DiscretePath and RealPath instances with new global path */
void NavigationRobot::generateNewGlobalPath(utils::MapIndex current_loc, utils::MapIndex new_frontier) {


    // Get nearest accessible point for the robot
    this->destination = new_frontier;


    // Update discrete path with new values from planner
     

    std::vector<utils::MapIndex> robot_path = this->planner->generatePath(current_loc, this->destination);
    this->discrete_path.updatePath(robot_path);
    this->discrete_path.refinePath();
    this->real_path.updatePath();

    //publish the real path to RVIZ for visualization
    this->getLocalPlanner().publishRealPath(this->real_path.getPathCoordinates(), this->getLocalPlanner().getPathPublisher());
    //print whether destination is accessible in local map
    this->local_map.printAccess(new_frontier.row, new_frontier.col);
}

/*
Function: getRobotRealPath
Arguments: none
Returns: RealPath vector stored within this */
std::vector<utils::MapPoint> NavigationRobot::getRobotRealPath() {
    return this->real_path.getPathCoordinates();
}

/*
Function: computeDWACommand
Arguments: none
Behaviour: runs DWA within local planner  */
void NavigationRobot::computeDWACommand() {
    this->local_planner.Run_DWA();
}

/*
Function: getLocalPlanner
Arguments: none
Returns: reference to robot local planner */
DWALocalPlanner& NavigationRobot::getLocalPlanner() {
    //returning a pointer for consistent updating of local planner properties
    return this->local_planner; 
}

/*
Function: getRobotParams
Arguments: none 
Returns: RobotParams struct of this */
struct RobotParameters NavigationRobot::getRobotParams() {
    return this->Robot_Params;
}

/*
Function: getNearestAccessibleNode
Arguments: node of interest
Returns: nearest accessible node, determined by local map of this */
utils::MapIndex NavigationRobot::getNearestAccessibleNode(utils::MapIndex idx) {
    return this->local_map.getNearestAccessibleNode(idx);
}

/*
Function: get updated destination from searching for nearest accessible node of a nearest frontieran an
Arguments: node of interest
Returns: nearest accessible node, determined by local map of this 
*/
utils::MapIndex NavigationRobot::getDestination() const {
    return this->destination;
}


/*
Function: getLocalMap
Arguments: none
Returns: copy of local map of this 
Notes: returns copy to avoid editing outside this. Only valid for instant check, can become outdated. */
Mapping::RobotLocalMap NavigationRobot::getLocalMap() const {
    return this->local_map;
}

/*
FunctionL getGlobalMap
Arguments: none
Returns: pointer to global map of this */
Mapping::GlobalMapGrid*NavigationRobot::getGlobalMap() const {
    return this->global_map;
}


/*-------------------------------------------------------------- NavigationEnvironment class ------------------------------------------------------------*/

/*
Function: NavigationEnvironment constructor
Arguments: none
Behaviour: Inititalizes empty navigation environment */
NavigationEnvironment::NavigationEnvironment(ros::NodeHandle& nh, double resolution) 
    : ros_interface(nh, resolution ), // Default resolution, will be updated when robot is added
      initialized(false),
      has_robot(false),
      robot(nullptr)
{
}

/*
Function: addRobot
Arguments: 
    - string id for robot
    - PathPlanner type from Mapping::PathPlanner::PlannerType enum
    - PathInterpolator type from Mapping::PathInterpolator::InterpolatorType enum
    - robot clearance relative to odometer
    - approximate path resolution for position and velocity commands
Behaviour: initializes new robot instance within navigation environment */
void NavigationEnvironment::addRobot(std::string id, Mapping::PathPlanner::PlannerType planner_type,
     Mapping::PathInterpolator::InterpolatorType interpolator_type,RobotParameters Robot_Params ,ros::NodeHandle& nh) {
    // Create robot with make_unique to allow internal use of unique_ptr
    if(initialized && !has_robot){
        this->robot = std::make_unique<NavigationRobot>(id, &this->global_map, planner_type, interpolator_type, Robot_Params, &this->ros_interface);
        has_robot = true;
    }
    else if(initialized) {
        ROS_ERROR("Attempted to add robot before intialization");
    }
    else {
        ROS_ERROR("Invalid addRobot: environment already has a robot");
    }
}


/*
Function: updateEnvironment
Arguments: ros messages DEV 
Behaviour: parses ROS messages, updates global map and local maps of all robots */
void NavigationEnvironment::updateEnvironment() {
    

    std::vector<std::vector<int>> occupancy = this->ros_interface.get_current_converted_2D_grid();
    std::vector<std::vector<int>> visits = this->ros_interface.get_visit_count_converted_2D_grid();
    double map_resolution = this->ros_interface.get_resolution();

    int num_rows = this->ros_interface.get_current_grid().info.height;
    int num_cols = this->ros_interface.get_current_grid().info.width;

    ROS_INFO("updateEnvironment: map dimensions = %d x %d, resolution = %f", num_rows, num_cols, map_resolution);

    // Validate input dimensions
    if(visits.size() != num_rows) {
        throw std::runtime_error("New occupancy and visit arrays must have matching dimensions");
    }

    for(std::vector<int>& row : occupancy) {
        if(row.size() != num_cols) {
            throw std::runtime_error("Updated occupancy vector must be rectangular");
        }
    }

    for(std::vector<int>& row : occupancy) {
        if(row.size() != num_cols) {
            throw std::runtime_error("Updated occupancy vector must be rectangular");
        }
    }

    for(std::vector<int>& row : visits) {
        if(row.size() != num_cols) {
            throw std::runtime_error("Updated occupancy vector must be rectangular");
        }
    }
    
    // Set up environment using info from ROS message, if not previously initialized
    if(!initialized) {
        this->global_map = Mapping::GlobalMapGrid(num_rows, num_cols, map_resolution);
        initialized = true;

    }


    // Update global map instance and get vector of updated indices
    global_map.updateMap(occupancy, visits);
    

    // Update robot's local map and global list of frontiers if robot is present
    if(has_robot && initialized) {
        robot->updateLocalMap();
        updateFrontiers();
    }



}

/*
Function: updateFrontiers
Arguments: list of all available frontiers, current robot location
Behaviour: updates intiernal frontiers priority queue, sorting based on frontier proximity*/
void NavigationEnvironment::updateFrontiers() {
    // @Stephen: get current location and list of frontiers from louis, add here

    std::vector<utils::MapIndex> new_frontiers = this->ros_interface.get_current_frontier_index_list();
    ROS_INFO("Number of frontiers: %lu", new_frontiers.size());


    std::priority_queue<utils::MapIndex, std::vector<utils::MapIndex>, CompareFrontiers> new_sorted_frontiers{CompareFrontiers(&this->global_map, this->ros_interface.get_current_position_index())};
    for(utils::MapIndex& idx : new_frontiers) {
        new_sorted_frontiers.push(this->robot->getNearestAccessibleNode(idx));
    }
    
    this->frontiers.swap(new_sorted_frontiers);

}

/*
Function: reSortFrontiers
Arguments: current location of robot
Behaviour: Updates priority queue of frontiers to evaluate proximity to new location */
void NavigationEnvironment::reSortFrontiers() {

    // Create new priority queue for current location
    // @Stephen: get current location from louis
    std::priority_queue<utils::MapIndex, std::vector<utils::MapIndex>, CompareFrontiers> new_sorted_frontiers{CompareFrontiers(&this->global_map, this->ros_interface.get_current_position_index())};

    // Move all frontiers to new priority queue
    while(this->frontiers.size() != 0) {
        new_sorted_frontiers.push(this->frontiers.top());
        this->frontiers.pop();
    }

    this->frontiers.swap(new_sorted_frontiers);
}

/*
Function: getNearestFrontier
Arguments: none
Behaviour: gets nearest frontier from frontiers, pops from queue */
utils::MapIndex NavigationEnvironment::getNearestFrontier() {

    // Validate non empty frontiers
    if(frontiers.size() == 0) 
    {
        throw std::logic_error("getNearestFrontier method requires frontiers to be added to NavigationEnvironment, frontiers is currently empty");
    }

    utils::MapIndex frontier = this->frontiers.top();
    this->frontiers.pop();
    return frontier;
}

/*
Functions: updateRobotPath
Arguments: none
Behaviour: updates robot's global path to nearest accessible frontier */
void NavigationEnvironment::updateRobotPath() {

    int path_size = 0;
    int num_frontiers = this->frontiers.size();
    utils::MapIndex current_frontier, current_position;

    while(path_size == 0) {

        
        current_frontier = getNearestFrontier();
        current_position = this->ros_interface.get_current_position_index();


        this->robot->generateNewGlobalPath(this->ros_interface.get_current_position_index(), current_frontier);


        if(robot->getRobotRealPath().empty()) {
            ROS_INFO("No accessible path to frontier at %s, generating path to next frontier", current_frontier.toString().c_str());
        }

        // Continue if current location is too close to robot (region is already mapped)
        if(this->global_map.getEuclideanDistance(current_frontier, current_position) < this->robot->getLocalPlanner().get_distance_to_consider_frontier_reached()){
            ROS_INFO("Nearest frontier at %s is too close to current location, area already mapped. Getting new frontier", current_frontier.toString().c_str());
            continue;
        }

        if(this->frontiers.empty()) {
            ROS_INFO("No valid path exists to current frontiers, getting new frontiers from mapping node");

            updateFrontiers();
            
            
            // If no new frontiers are added to the list compared to beginning of method call, all accessible frontiers are mapped and program is complete
            path_size = this->robot->getRobotRealPath().size();
            if(num_frontiers == this->frontiers.size()) {
                ROS_INFO("All remaining frontiers are inaccessible with current robot-> Room mapping is complete.");
                exit(EXIT_SUCCESS);
            }
        }

        num_frontiers = this->frontiers.size();
        path_size = this->robot->getRobotRealPath().size();
    }

    //robot->getLocalPlanner().publishRealPath(this->robot->getRobotRealPath(), this->robot->getLocalPlanner().getPathPublisher());

}

/*
Function: mapSurroundings
Arguments: none
Behaviour: Runs main mapping loop after NavigationEnvironment has been set up*/
void NavigationEnvironment::mapSurroundings(std::string robot_id, 
                                            Mapping::PathPlanner::PlannerType planner_type,
                                            Mapping::PathInterpolator::InterpolatorType interpolator_type,
                                            RobotParameters robot_params) 
{
    ROS_INFO("Environment mapping initiated");

    // Set loop rate from robot params
    double main_loop_rate = robot_params.main_loop_rate;
    ros::Rate loop_rate(main_loop_rate); 

    // Define start time
    auto start = std::chrono::system_clock::now();
    double secondsElapsed = 0.0;
    
    // Wait for map to be initialized before adding robot
    ROS_INFO("Waiting for occupancy grid to initialize...");
    ROS_INFO("ros::ok() = %s, initialized = %s", ros::ok() ? "true" : "false", this->initialized ? "true" : "false");

    // Set up global_map, run until initialized
    while(ros::ok() && !this->initialized) {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
        ROS_INFO("Initial spin successful");
        
        // Check if callbacks have received valid grid data
        if(this->ros_interface.get_current_grid().info.height > 0 && 
           this->ros_interface.get_current_grid().info.width > 0 && 
           this->ros_interface.get_visit_count_grid().info.height > 0 && 
           this->ros_interface.get_visit_count_grid().info.width > 0)
           {

            ROS_INFO("Valid occupancy grid received: %d x %d", 
                     this->ros_interface.get_current_grid().info.height,
                     this->ros_interface.get_current_grid().info.width);

             ROS_INFO("Valid visit grid received: %d x %d", 
                     this->ros_interface.get_visit_count_grid().info.height,
                     this->ros_interface.get_visit_count_grid().info.width);

            
            //set initial occupancy grid and visit grid to environment
            
            updateEnvironment();
        } 
        
        else {
            if (this->ros_interface.get_visit_count_grid().info.height <=0 ||
                this->ros_interface.get_visit_count_grid().info.width <=0)
              {
                ROS_WARN("Visit count grid not yet received or invalid");
              }
            ROS_INFO("Waiting for valid grid data...");
        }
        
        ros::Duration(0.1).sleep();
    }


    ROS_INFO("Map initialized, now adding robot with valid global map");
    
    // Ensure global map is properly initialized
    if (this->global_map.getRows() <= 0 || this->global_map.getCols() <= 0) {
        ROS_ERROR("Global map is not properly initialized! Dimensions: %d x %d",
                this->global_map.getRows(), this->global_map.getCols());
        throw std::runtime_error("Cannot add robot: global map is invalid.");
    }

    // Update sensor data and add robot to environment
    ros::spinOnce();

    addRobot(robot_id, planner_type, interpolator_type, robot_params, this->ros_interface.getNodeHandle());
    ROS_INFO("Robot added to environent");
    ROS_INFO("Starting initial 360 degree scan");
    
    // Pivot 306 degrees to scan surroundings and update maps
    this->robot->getLocalPlanner().rotate_for_angle(360.0);
    updateEnvironment();
    

    // Generate initial path before entering main loop and process callbacks
    ros::spinOnce();  
    updateRobotPath();
    utils::MapIndex initial_frontier = this->robot->getDestination();
    utils::MapIndex initial_location = this->ros_interface.get_current_position_index();
    ROS_INFO("Initial location is (%d, %d)", initial_location.row, initial_location.col);    
    ROS_INFO("Generating initial global path to frontier: %s", initial_frontier.toString().c_str());
    std::vector<utils::MapPoint> global_path = robot->getRobotRealPath();

    if (global_path.empty()) 
    {
        ROS_ERROR("Initial global path is empty!");
    }
    else
    {
        ROS_INFO("Initial global path generated with %lu points", global_path.size());
    }

   
	ROS_INFO("destination is (%d, %d), accessibility is %d", this->robot->getDestination().row, this->robot->getDestination().col, this->robot->getLocalMap().isAccessible(this->robot->getDestination().row, this->robot->getDestination().col));

    this->robot->getLocalPlanner().set_current_path(this->robot->getRobotRealPath());
    this->robot->getLocalPlanner().set_has_path(true);

    std::vector<utils::MapPoint> verify_path = this->robot->getLocalPlanner().get_current_path();

    // Run main loop for 50 minutes, stop manually when mapped
    while(ros::ok() && secondsElapsed <=  3000) {


        // Update environment and sensor data
        ros::spinOnce();       
        updateEnvironment();

        // Get euclidean distance to destination for evaluation later
        double distance_to_end = this->global_map.getEuclideanDistance(this->ros_interface.get_current_position_index(), this->robot->getDestination());
        double threshold = this->robot->getLocalPlanner().get_distance_to_consider_frontier_reached(); 
        ROS_INFO("Distance to frontier = %f, threshold = %f", distance_to_end, threshold);

        //checking for bumper rising edge event
        bool bumper_event = this->ros_interface.get_bumper_pressed_and_needs_reset();
        if (bumper_event) {
            ROS_WARN("Bumper event detected, resetting ros internal state to false but keeping the cached state in NavigationEnvironment");\
            this->ros_interface.set_bumper_pressed_and_needs_reset(false);
        }

        // When frontier is reached or collision occurs, generate path to next frontier
        if(distance_to_end <= threshold || bumper_event) {

            if (distance_to_end <= threshold) {
                
                ROS_INFO("Distance to frontier (%f) is less than threshold (%f), bumper pressed is %s", distance_to_end, threshold, this->ros_interface.get_bumper_pressed_and_needs_reset() ? "true" : "false");
                ROS_INFO("Frontier reached, generating new global path");
            }

            else
            {
                ROS_WARN("Bumper pressed, executing recovery behaviour and generating new global path");
            }
            
            // brief pause to ensure robot has fully stopped
            ros::Duration(0.5).sleep(); 

            // update sensor data and environment, generate new global path
            ros::spinOnce();
            this->robot->getLocalPlanner().rotate_for_angle(360.0);
            updateEnvironment();            
            updateRobotPath();
            ROS_INFO("Moving to new frontier at %s", robot->getDestination().toString().c_str());
            ROS_INFO("New global path generated with %lu points", this->robot->getRobotRealPath().size());

            // Update global path within robot local planner
            this->robot->getLocalPlanner().set_current_path(this->robot->getRobotRealPath());
            this->robot->getLocalPlanner().set_has_path(true);   

            bumper_event = false;
            
        }

        // Generate DWA trajectories
        robot->computeDWACommand();
        
        // Update elapsed time
        auto current = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed = current - start;
        secondsElapsed = elapsed.count();
        
        // Sleep to maintain loop rate
        loop_rate.sleep();
    }
}

int main(int argc, char** argv) {

    //Initialize node
    ros::init(argc, argv, "path_planning_node");

    // Create a PathPlanning Node to subscribe, process, and publish
    ros::NodeHandle nh;

    // Create weights struct using parameterized constructor
    TrajectoryEvaluator::Weights weights;
    weights.clearance_weight = 0.75;
    weights.heading_weight = 0.17;
    weights.velocity_weight = 0.08;
    
    // Create limits struct explicitly
    DynamicWindowSampler::Limits limits;
    limits.v_x_min = 0.0;
    limits.v_x_max = 0.4; // 0.65
    limits.omega_min = -M_PI/5; // pi
    limits.omega_max = M_PI/5;
    limits.a_x_max = 0.4; // 0.8
    limits.a_theta_max = 2.25; // 3

    // radius multipler can't be too small, otherwise there could a deadzone where
    //nearly all trajectories graze unknown, and rotating in place increaes heading error
    //so stopping is the best option
    RobotParameters Turtlebot_Params(0.1, 0.05, limits, weights,
                                 20, 20, 2.0, 0.1, 1.0, 0.2, 3.5,  
                                 5.0, 0.25, 15.0, 45.0, 0.3, 1.0,
                                 0.05);

    // RobotParameters(double radius, double res,DynamicWindowSampler::Limits lim, TrajectoryEvaluator::Weights w,
    //                 int Nv, int Nw, int horiz, double dt_traj, double k, double dt_ctrl, double radius_multiplier, 
    //                 double error_thr, double kp, double angle_left_right, double angle_front, double backoff_velocity, double backoff_duration,
    //                 double path_res)
  
    NavigationEnvironment environment(nh, Turtlebot_Params.resolution);
    ROS_INFO("environment constructed");

    // Run main application loop
    environment.mapSurroundings("TurtleBot", Mapping::PathPlanner::PlannerType::ASTAR, 
                                Mapping::PathInterpolator::InterpolatorType::LINEAR, Turtlebot_Params);
    return EXIT_SUCCESS;

}
