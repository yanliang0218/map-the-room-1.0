/********************************************************************************
Name: Liang Yan
File: dwa_local_planner_class_headers
Purpose: Contains header files for DWA local planner classes.
**********************************************************************************/

/***************************** INCLUDES ******************************************/
#pragma once
#include <utility>
#include <stdexcept>
#include <iostream>
#include <vector> 
#include <queue>
#include <cmath>

#include "utils.h"

//using namespace utils;


//TODO: #include "utils.h" when compiling together with utils.cpp

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif 



/***************************** CLASS DECLARATIONS**********************************/

/***************************** Dynamic Window Sampler Class************************/

/**********************************************************************************
Name: Liang Yan
Student Number: 33140351 
Purpose: Contains the DynamicWindowSampler class declaration. 
Description: This class contains the necessary member variables and methods to generate samples of possible
             linear and angular velocities at every control cycle based on the robot's current velocity and
             its kinematic constraints obtained from Turtlebot 2 technical specifications.

**********************************************************************************/

class DynamicWindowSampler {

    public:
        // struct for robot kinematic constraints
        //defined before private members to enable use of Limits in defining private limits
        struct Limits {
            double v_x_min;       // Minimum linear velocity (m/s)
            double v_x_max;       // Maximum linear velocity (m/s)
            double omega_min;     // Minimum angular velocity (rad/s)
            double omega_max;     // Maximum angular velocity (rad/s)
            double a_x_max;       // Max linear acceleration (m/s^2)
            double a_theta_max;   // Max angular acceleration (rad/s^2)
        };

        
        // data structure for computed thresholds for sampling
        struct Window {
            double v_low;          // Minimum linear velocity threshold in dynamic window
            double v_high;         // Maximum linear velocity threshold in dynamic window
            double omega_low;      // Minimum angular velocity threshold in dynamic window
            double omega_high;     // Maximum angular velocity threshold in dynamic window
        };


    private:
        //private member struct Limits for robot kinematic constraints
        //const so the private limits become immutable after construction
        const Limits limits;

        // sample size for linear and angular velocities
         //const so the private N_v and N_omega become immutable after construction
        const int N_v, N_omega;

    public:

        // constructor
        //const Limits& limits: reference to a read-only Limits struct  
        DynamicWindowSampler(const Limits& limits,int N_v, int N_omega);

        //compute reachable velocities for the next control cycle based on current vel and kinematic constraints in Limits_
        Window compute_window(double v_x_curr, double omega_curr, double dt_control) const;

        //sample N_v × N_omega velocities from the window with fixed increments, where the Ns are the sample space dimensions
        std::vector<std::pair<double, double>> sampler(Window window) const;

        //method for printing (v,omega) samples
        //for visualizing 
        void print_samples(const std::vector<std::pair<double, double>>& samples) const;

        //getters for kinematic constraints
        const Limits& get_limits() const;
        int get_N_v() const;
        int get_N_omega() const;

};
/***************************** Trajectory Generator Class************************/
/**********************************************************************************
Name: Liang Yan
Student Number: 33140351 
File: trajector_generator.h
Purpose: Contains the TrajectoryGenerator class declaration
Description: For every (v,omega) sample in the current dynamic window, if one holds this pair constant for a predefined
             prediction horizon, a trajectory in the shape of an arc can be used to approximate the robot's path within 
             this short prediction horizon if this pair were to be chosen. Thus, to evaluate the merit of each (v, omega) 
             is to evaluate the trajectory it would lead to. Each trajectory is then discretized into poses evenly spaced 
             by a step in time i.e. dt_trajectory. These poses are then used in the next step for trajectory evaluation.
             This program therefore aims to generate such discretized poses for a given trajectory. led to by a given 
             (v, omega) pair.

**********************************************************************************/
class TrajectoryGenerator
{
    private:
        //private member variable for how long into the future a trajectory is generated for 
        double horizon; 
        //private member for the resolution in time step of discrete points  of a given trajectory
        double dt_trajectory;
    
    public:

        // //struct to contain a pose defined as x position, y position, and the angle theta
        // struct Pose 
        // {
        //     double x;
        //     double y;
        //     double theta;
        // };
        
        //constructor for a TrajectoryGenerator class object
        TrajectoryGenerator (double horizon, double dt_trajectory);

        //generate discretized trajectory for a given (v,omega) pair
        std::vector<utils::Pose> simulate_trajectory (const utils::Pose& start, double v, double omega);
        
        //getters for private variables
        double get_horizon();
        double get_dt_trajectory();

        //prints a given trajectory using '*'
        void print_traj(std::vector<utils::Pose> traj);

};


/***************************** Obstacle Evaluator Class***************************/
/**********************************************************************************
Name: Liang Yan
Student Number: 33140351 
File: obstacle_evaluator.h
Purpose: Contains the ObstacleEvaluator class declaration. 
Description: for every pose on the discretized trajectory, we evalute frist if it is on an obstacle or unknown cell, 
             if so, we autocally set the minimum clearance to any obstacle of that pose to zero; if not, we calculate
             the minimum distance to any obstacle or unknown cell. This process is repeated for every single pose on 
             a given trajectory, and we loop to find the minimum clearance to any obstacle out of all poses on this 
             particular trajectory. This is then used as an indicator for how close this given trajectory is from 
             obstacles or unknown cells, i.e. how much comfort is there in aoviding obstacles and unknown cells
             if a given (v,omega) is selected, thereby evaluating how safe a given (v,omega) is.


**********************************************************************************/
class ObstacleEvaluator {
private:

    // stored copy of the current map -> change into array!
    std::vector<std::vector<int>> occupancy_grid;  
    // map to store precomputed clearance to any obstacle or unknown cell for each cell in the occupancy grid
    // TODO:how to integrate?
    std::vector<std::vector<double>> clearance_grid;

    //resolution of the occupancy grid in meters/cell
    double resolution;
    //dimension of the occupanc grid in number of row and columns
    size_t num_rows;
    size_t num_cols;
    //radius of the robot in meters
    double radius_robot;

public:

    // Constructor
    ObstacleEvaluator(double resolution, double radius_robot);

    // Update grid if map callback sends a new one
    void set_grid(const std::vector<std::vector<int>>& occupancy_grid);

    // Update clearancegrid from an updated occupancy grid
    // Used for testing -> discuss if integrating with Connor's code!
    void compute_clearance_grid();

    // Compute minimum clearance along a trajectory
    double compute_clearance(const std::vector<utils::Pose>& traj) const;

    // Collision or stepping into unknown space check (hit = true)
    bool on_obstacle_or_unknown(const utils::Pose& p) const; 
    

    //TODO:getters for private members
    const std::vector<std::vector<int>>& get_occupancy_grid() const;

    const std::vector<std::vector<double>>& get_clearance_grid() const; 
    
    double get_resolution() const; 
    
    size_t get_num_rows() const; 

    size_t get_num_cols() const;

    double get_radius_robot() const;
 
};

/********************************Path Tracker*************************************/
/**********************************************************************************
Name: Liang Yan
Student Number: 33140351 
File: path_tracker.cpp
Purpose: Contains the PathTracker class definition. 
Description: From the end pose of a given trajectory, we march forward a small distance to identify where the path is leading 
            ahead. Comparing this with the end pose tells us how much the given trajectory is deviating from the prescribed path,
             hence path tracker. This will lead to another indicator for how closely any given trajectory tracks our path.
**********************************************************************************/

/***************************** CLASS DECLARATION *********************************/



class PathTracker
{
    private:
        //private field for storing a local copy of the current real path
        //Connor code:std::vector<MapPoint> getPathCoordinates()
        std::vector<utils::MapPoint>path;

        //from the end pose of a given trajectory, a distance that we want to imaginarily march forward on the path
        //from the closest point on the path to the end of the trajectory 
        //to identify where the path is leading a small distance ahead. 
        //Comparing this with the end pose tells us how much the given trajectory is deviating from the prescribed path
        double lookahead;

        //the scaling multiplier in tuning the maximum lookahead distance
        double k_multiplier_lookahead;

    public:

        //constructor
        PathTracker(double k_multiplier_lookahead);

        //sets the path everytime a new path is received
        void set_path(const std::vector<utils::MapPoint>& path);
        
		//sets the current linear velocity from odometry
        void set_v_x_curr(double v_x_curr);

        //find the closest point on the path to the end of the trajectory, then march forward till a lookahead distance has been reached
        //from the end of the trajcetory
        utils::MapPoint find_target_on_path(const utils::Pose& pose);

        //getter for the current real path
        const std::vector<utils::MapPoint>& get_path() const;

        //getter for lookahead 
        double get_lookahead() const;

        //getter for the k_multiplier for lookahead
		double get_k_multiplier_lookahead() const;

        //setter for lookahead to dynamically calculate lookahead distance based on current linear velocity
        //TODO: change here for later when running together
        void set_lookahead(double resolution, double k_multiplier_lookahead, double v_x_curr, double v_max, double horizon, double robot_radius);

};


/*****************************Trajectory Evlaluator Class********************************/
/****************************************************************************************
Name: Liang Yan
Student Name: 33140351
File: trajectory_evaluator.h
Purpose: Contains the class TrajectoryEvaluator definition.
Description: evalulate every trajectory based on alignment with path, being able to stop before hitting obstacles or unknown space (bool criterion for automatic reject)
             ,leaving sufficient clearance from obstacles and unknown space, and finally maximizing linear velocity. A weighted sum of all three scores indicate the
             meriit of given (v,omega) pair.
**********************************************************************************/

/***************************** CLASS DECLARATION *********************************/
class TrajectoryEvaluator
{
public:
    //struct to hold weights for heading, clearance, and velocity respectively that will
    // be used when evaluating score for individual trajectories
    struct Weights
    {
        double clearance_weight;
        double heading_weight;
        double velocity_weight;
        
        // Default constructor with valid weights that sum to 1.0
        Weights() : clearance_weight(0.3), heading_weight(0.3), velocity_weight(0.4) {}
        
        // Parameterized constructor
        Weights(double c, double h, double v) : clearance_weight(c), heading_weight(h), velocity_weight(v) {}
    };

    //struct for all scores and the Boolean admissibily check for a given trajectory
    struct TrajectoryScores
    {
        double heading_score;
        double clearance_score;
        double velocity_score;
        double total_score;

        //the admissibility check involves 
        //1. the trajectory cannot involve unknown and occupied cells
        //2. the Turtlebot has to be able to stop before hitting any obstacle or unknown cell  
        //   when at the point on the trajectory that has the minimum clearance
        bool admissibility;
    };

private:
    //individual weights for heading, clearance, and velocity respectively when evaluating an individual trajectory

    Weights weights;

    //maximum braking deceleration for the Turtlebot 2
    double braking_decel;

public:

    //constructor
    TrajectoryEvaluator(Weights weights, double braking_decel);

    //the evalutor that scores each trajectory based on maximizing clearance, minimizing heading error, maximizing 
    //velocity, as well as passing the Boolean admissibility check
    TrajectoryScores evaluate(const std::vector<utils::Pose>& traj, const utils::MapPoint& target,
        double v, double v_x_max, double longest_dimension_of_room, const ObstacleEvaluator& obs);

    //getters for private members
    double get_heading_weight();
    double get_clearance_weight();
    double get_velocity_weight();
    double get_braking_decel();

	void print_trajectory_scores(const TrajectoryScores& scores) const;

};
