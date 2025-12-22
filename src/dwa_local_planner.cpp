/**********************************************************************************
Name: Liang Yan
Student Name: 33140351 
File: map_the_room/dwa_local_planner.h
Purpose: Contains the DWALocalPlanner class definition. 
Description: evaluates sample trajectories and compute command. also contains 
             bumper recovery logic, and rotate_for_angle maneuver
**********************************************************************************/

/*********************************** INCLUDES ***********************************/


#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <kobuki_msgs/BumperEvent.h>

#include "dwa_local_planner.h"


using namespace std;
using namespace utils;

/***************************** CLASS DEFITION************************************/
/*
Function: construct a local planner object
Arguments: a ros pointer to a ros_interface object initialized in global planner module, and a series of tunable parameters
*/
DWALocalPlanner::DWALocalPlanner(ROSInterface* ros_interface_ptr, double resolution, double radius_robot,const DynamicWindowSampler::Limits& limits, const TrajectoryEvaluator::Weights& weights,
                                int N_v, int N_omega, double horizon, double dt_trajectory, double k_multiplier_lookahead, 
                                double dt_control, double radius_multiplier_distance_frontier_reached, 
                                double error_threshold_for_reactive_rotation, double kp_for_rotate_for_angle, double angle_to_rotate_if_left_or_right_bumper_pressed,
                                double angle_to_rotate_if_front_bumper_pressed, double backoff_linear_velocity_if_bumper_pressed, double backoff_duration_if_bumper_pressed)
    : ros_interface(ros_interface_ptr),
      sampler(limits, N_v, N_omega),
      traj_generator(horizon, dt_trajectory),
      obs_eval(resolution, radius_robot),
      path_tracker(k_multiplier_lookahead),
      traj_evaluator(weights, limits.a_x_max),
      dt_control(dt_control),
      has_path(false),
      error_threshold_for_reactive_rotation(error_threshold_for_reactive_rotation),
      kp_for_rotate_for_angle(kp_for_rotate_for_angle),
      angle_to_rotate_if_left_or_right_bumper_pressed(angle_to_rotate_if_left_or_right_bumper_pressed),
      angle_to_rotate_if_front_bumper_pressed(angle_to_rotate_if_front_bumper_pressed),
      backoff_linear_velocity_if_bumper_pressed(backoff_linear_velocity_if_bumper_pressed),
      backoff_duration_if_bumper_pressed(backoff_duration_if_bumper_pressed),
      last_cmd_angular(0.0)
{
    this->cmd_pub = ros_interface_ptr->getNodeHandle().advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
    ROS_INFO("DWA Local Planner publishing to: /cmd_vel_mux/input/navi");

    //this->real_path_pub = ros_interface_ptr->getNodeHandle().advertise<visualization_msgs::Marker>("real_path_markers", 1);
    this->real_path_pub = ros_interface_ptr->getNodeHandle().advertise<nav_msgs::Path>(
    "/real_path", 1, true);
    
    double robot_radius = this->obs_eval.get_radius_robot();
    
    this->distance_to_consider_frontier_reached = radius_multiplier_distance_frontier_reached * robot_radius;

    ROS_INFO("kp set to %f in DWA constructor", this->kp_for_rotate_for_angle);
}

/***************************** CALLBACK DEFINITIONS *****************************/

/*
Function: Run_DWA
Arguments: none
Behaviour: container for running dwa pipeline in main loop
*/
void DWALocalPlanner::Run_DWA()
{
    //computeCommand gets called! dt_control 
    geometry_msgs::Twist cmd = computeCommand(); 

    this->cmd_pub.publish(cmd);
}


/*
Function: computeCommand
Arguments: none
Behaviour: runs through dwa pipeline, evaluates individual scores, selects best scoring trajectory 
	   and sets command velocity to the (v,omega) that led to it
*/
geometry_msgs::Twist DWALocalPlanner::computeCommand()
{
    if (!this->ros_interface->get_has_occupancy_grid() || !this->ros_interface->get_has_frontier() 
    ||!this->ros_interface->get_has_frontier_list()|| !this->ros_interface->get_has_visit_count_grid() || !this->has_path)
    {
        if (!this->ros_interface->get_has_occupancy_grid()) {
            ROS_WARN("No occupancy grid received yet.");
        }
        else if (!this->ros_interface->get_has_frontier()) {
            ROS_WARN("No frontier received yet.");
        }
         else if (!this->ros_interface->get_has_frontier_list()) {
            ROS_WARN("No frontier received yet.");
        }
        else if (!this->ros_interface->get_has_visit_count_grid()) {
            ROS_WARN("No visit count grid received yet.");
        }
        else if (!this->has_path) {
            ROS_WARN("No path available yet.");
        }
        //if we do not have an occupancy grid or a frontier, return zero velocities
        geometry_msgs::Twist cmd;
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        return cmd;
    }

    else
    {
        //ROS_INFO("All necessary data received, proceeding with DWA computation.");
    }
    
    //look into!
    if (this->ros_interface->get_bumper()[0] == kobuki_msgs::BumperEvent::PRESSED || 
        this->ros_interface->get_bumper()[1] == kobuki_msgs::BumperEvent::PRESSED || 
        this->ros_interface->get_bumper()[2] == kobuki_msgs::BumperEvent::PRESSED)
    {
        ROS_WARN("Bumper pressed! Initiating recovery behavior.");
        this->bumper_recovery();
        ROS_INFO("NOTICE: exiting bumper receovery");

    }
    
    //declares a twist object to contain command velocity
    geometry_msgs::Twist cmd;

    //initilizes best score to -1
    double best_score = -1.0;

    //using ros_interface methods to get current robot states, and the current occupancy grid
    double v_curr = this->ros_interface->get_v_curr();
    //ROS_INFO("DWA: 1.setting current linear velocity v_curr = %f", v_curr);

    double omega_curr = this->ros_interface->get_omega_curr();
    //ROS_INFO("DWA: 2.setting current angular velocity omega_curr = %f", omega_curr);

    Pose pose = this->ros_interface->get_pose();
    //ROS_INFO("DWA: 3.setting current pose");

    std::vector<std::vector<int>>occupancy_grid =this->ros_interface->get_current_converted_2D_grid();

    this->obs_eval.set_grid(occupancy_grid);
    //ROS_INFO("DWA: 4.setting occupancy grid for obstacle evaluation");

    //get maximum linear velocity from the sampler object's limits
    double v_x_max = this->sampler.get_limits().v_x_max;
    //ROS_INFO("DWA: 5.setting maximum linear velocity v_x_max = %f", v_x_max);

    //calculate longest dimension of the room from grid
    double longest_dimension_of_room = std::max(occupancy_grid.size(), occupancy_grid[0].size()) * this->ros_interface->get_resolution();
    //ROS_INFO("DWA: 6.setting longest dimension of room = %f", longest_dimension_of_room);
    // 1. Compute dynamic window bounds
    DynamicWindowSampler:: Window window = this->sampler.compute_window(v_curr, omega_curr, this->dt_control);
    //ROS_INFO("DWA: 7.computing dynamic window: v_min = %f, v_max = %f, omega_min = %f, omega_max = %f", 
               //window.v_low, window.v_high, window.omega_low, window.omega_high);

    // 2. Sample velocities
    std::vector<std::pair<double, double>> samples = this->sampler.sampler(window); //v omega pairs sampled
    //ROS_INFO("DWA: 8.sampling %zu (v, omega) pairs from dynamic window", samples.size());
    
    // Check if path is empty before evaluation
    if (this->current_path.empty()) {
        ROS_ERROR("DWA: current_path is empty, cannot run trajectory evaluation. Sending zero velocity command.");
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        return cmd;
    }
    
    // Set path once before the loop (not inside it)
    this->path_tracker.set_path(this->current_path);
    //ROS_INFO("DWA 9: Path set in path tracker with %zu points", this->current_path.size());
    int admissible_trajectory_count = 0;
    // for every (v,omega) sample
    for (auto sample : samples) {

        //ROS_WARN("9.sampling for loop reached, evaluating vel samples");
        //ROS_WARN("Evaluating sample: v = %f, omega = %f", sample.first, sample.second);

        auto v = sample.first;
        auto omega = sample.second;

        // 3. Generate the trajectory for the current (v,omega) pair
        std::vector<Pose>  traj =  this->traj_generator.simulate_trajectory(pose, v, omega); //trajectory_generator gets called

        double test_clearance = this->obs_eval.compute_clearance({traj[0]});
        if (test_clearance == 0.0) {
            ROS_WARN("DWA: Robot's current position has zero clearance! Robot might be in obstacle/unknown space.");
        }
        
        // 4. Find target on the current path as reference for heading error computation
        Pose last_pose = traj.back();
        utils::MapPoint target = this->path_tracker.find_target_on_path(last_pose); 
        //ROS_INFO("DWA: 10. target is at (%f, %f)", target.x, target.y);


        // 5. Evaluate score 
        TrajectoryEvaluator::TrajectoryScores scores = this->traj_evaluator.evaluate(traj, target, v, v_x_max, longest_dimension_of_room, this->obs_eval);

        //automatic reject from braking/admissibility criterion
        if (!scores.admissibility) 
        {
            continue;
        }
        else {
            admissible_trajectory_count++;
            //ROS_WARN("DWA 11: Trajectory with v = %f, omega = %f passed admissibility check.", v, omega);
        }

        //update best score and command
        if (scores.total_score > best_score) { 
            //ROS_INFO("DWA: 12.sampling: best score is updated from %f to %f", best_score, scores.total_score);
            best_score = scores.total_score;
            cmd.linear.x  = v;
            cmd.angular.z = omega;
        }
    }

    //no admissible trajectory found -> should set a default! Look into this!
	if (best_score < 0 || admissible_trajectory_count <= 50) { 
        ROS_WARN("DWA 13: No or two few admissible trajectory found, setting command velocities to default.");
         cmd.linear.x = 0.2;
        cmd.angular.z = (this->last_cmd_angular >= 0) ? 0.2 : -0.2;
        ros::Time start_time = ros::Time::now();
        ros::Rate rate(20); // 20 Hz, adjust as needed
        while (ros::ok() && (ros::Time::now() - start_time).toSec() < 0.2) {
            this->cmd_pub.publish(cmd);
            rate.sleep();
        }
       
    }


    static ros::Time last_log_time = ros::Time(0);
    ros::Time now = ros::Time::now();
    if ((now - last_log_time).toSec() >= 1.0) {
         
        ROS_INFO("pausing to allow time for sensor processing");
        ros::Duration(0.2).sleep(); // Small sleep to ensure log ordering
        ROS_INFO("DWA:14.Finished computing command velocities: linear.x = %f, angular.z = %f", cmd.linear.x, cmd.angular.z);
        last_log_time = now;
    }

    if (admissible_trajectory_count == 0) {
        ROS_WARN("DWA: No admissible trajectories found in this cycle.");
        cmd.linear.x = 0.0;
        // Rotate in last known direction
        cmd.angular.z = (this->last_cmd_angular >= 0) ? 0.2 : -0.2;
    } else {
        ROS_INFO("DWA: Found %d admissible trajectories in this cycle.", admissible_trajectory_count);
    }

    this->last_cmd_angular = cmd.angular.z;
    return cmd;
}


/*
Function: rotate_for_angle
Arguments: angle__in_deg
Behaviour: rotates for an user input angle
*/
void DWALocalPlanner::rotate_for_angle(float angle_in_deg){  

    ROS_WARN("Starting rotation for angle: %f degrees", angle_in_deg);

    float target_angle_in_radians = angle_in_deg * M_PI / 180.0;
    float accumulated_angle_rotated = 0.0; 

    ros::Duration(0.1).sleep();
    float previous_theta = this->ros_interface->get_pose().theta; 

    ros::Rate rate(1.0 / this->dt_control);
    geometry_msgs::Twist rotate_cmd;

    // Minimum rotational speed threshold to ensure movement
    const float min_omega = 0.1; // rad/s, absolute minimum
    const float max_omega = this->sampler.get_limits().omega_max; 

    // in radians
    const float errorThreshold = this->error_threshold_for_reactive_rotation * M_PI / 180.0; 

    while(ros::ok() && std::fabs(accumulated_angle_rotated) < std::fabs(target_angle_in_radians)){

        // Get updated pose from callbacks first
       
        ros::spinOnce();

        float current_theta = this->ros_interface->get_pose().theta;
        float diff = utils::wrapAngle(current_theta - previous_theta);

        accumulated_angle_rotated += diff;
        previous_theta = current_theta;

        // Remaining 
        float remaining_angle_to_rotate = target_angle_in_radians - accumulated_angle_rotated;

        if(std::fabs(remaining_angle_to_rotate) < errorThreshold){
            break; // If error is accepted, then break
        }

        float omega = 0.0;

        // Rotational speed is proportional to the remaining distance with gain
        if (remaining_angle_to_rotate <= 1/4 * target_angle_in_radians && remaining_angle_to_rotate >= -1/4 * target_angle_in_radians) {
            // Need to rotate counter-clockwise
            omega = this->kp_for_rotate_for_angle * remaining_angle_to_rotate;
        } 
        else 
        {
           
            omega = 0.45;
        }
        
        // Clamp to maximum angular velocity
        if(std::fabs(omega) > max_omega){
            omega = (omega < 0) ? -max_omega : max_omega;
        }

        // Enforce minimum angular velocity to ensure movement
        if(std::fabs(omega) < min_omega){
            omega = (omega < 0) ? -min_omega : min_omega;
        }

        //ROS_INFO("Commanding omega = %f rad/s, remaining = %f rad, max_omega = %f", omega, remaining_angle_to_rotate, max_omega);

        rotate_cmd.linear.x = 0.0;
        rotate_cmd.angular.z = omega;
        
        ROS_INFO("In rotate for angle: Publishing rotate command: angular.z = %f rad/s", rotate_cmd.angular.z);
        this->cmd_pub.publish(rotate_cmd);

        
        rate.sleep();
    }
    // stop rotation
    rotate_cmd.linear.x = 0.0;
    rotate_cmd.angular.z = 0.0;
    this->cmd_pub.publish(rotate_cmd);

    ros::Duration(0.1).sleep();

    return;
}


/*
Function: bumper_recovery 
Arguments: none
Behaviour: detects whether left, front, right bumper is fired; backs off, and rotate to the opposite side
*/
void DWALocalPlanner::bumper_recovery(){

    //determining rotation angle based on which bumper is pressed
    float rotate_angle;
    const uint8_t* bumper = this->ros_interface->get_bumper();

    if(bumper[0] == kobuki_msgs::BumperEvent::PRESSED)
    {  // Left bumper
        rotate_angle = this->angle_to_rotate_if_left_or_right_bumper_pressed;
    } 
    
    else if (bumper[2] == kobuki_msgs::BumperEvent::PRESSED)
    {  // Right bumper
        rotate_angle = - this->angle_to_rotate_if_left_or_right_bumper_pressed;  
    } 

    else if (bumper[1] == kobuki_msgs::BumperEvent::PRESSED)
    {
	//using a random seed -> any integer divided by 2 is either 0 or 1 -> binary state   
        int random = rand() % 2;

        if(random == 0)
        {
	    
            rotate_angle = this->angle_to_rotate_if_front_bumper_pressed;
        } 

        else 
        {
	    // Right bumper
            rotate_angle = -this->angle_to_rotate_if_front_bumper_pressed;
        }
    }

    ros::Rate rate(1.0 / this->dt_control);

    geometry_msgs::Twist cmd;

    cmd.linear.x = -this->backoff_linear_velocity_if_bumper_pressed;           
    cmd.angular.z = 0.0;

    ros::Time backup_start = ros::Time::now();
    double backup_duration = this->backoff_duration_if_bumper_pressed; 

    while(ros::ok() && (ros::Time::now() - backup_start).toSec() < backup_duration)
    { // whether or not to keep backup 
        this->cmd_pub.publish(cmd);
        ros::spinOnce();
        rate.sleep();
    }
    
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    this->cmd_pub.publish(cmd);

    // Forced Rotate
    this->rotate_for_angle(rotate_angle);
    ROS_INFO("NOTICE: exiting bumper recovery");

    
    return;
}


/*
Function: getter for the ros interface object
Arguments: none
Behaviour: returns  the private ros interface object
*/
 ROSInterface* DWALocalPlanner::getROSInterface() const
 {
    return this->ros_interface;
 }

 /*
Function: getter current distance to frontier
Arguments: none
Behaviour: returns the boolean indicator has_path
*/
double DWALocalPlanner::get_distance_to_consider_frontier_reached() const
{
    return this->distance_to_consider_frontier_reached;
}

/*
Function: getter for boolean indicator of whether the robot has a path
Arguments: none
Behaviour: returns the boolean indicator has_path
*/
bool DWALocalPlanner::get_has_path() const
{
    return this->has_path;
}

/*
Function: setter for boolean indicator of whether the robot has a path
Arguments: bool has_path 
Behaviour: sets the boolean indicator has_path to true when the global planner sets a realPath to local planner
*/
void DWALocalPlanner::set_has_path(bool has_path)
{
    this->has_path = has_path;
}

/*
Function: getter for boolean indicator of whether the robot has a path
Arguments: none
Behaviour: get the current real path 
*/
const std::vector<utils::MapPoint>& DWALocalPlanner::get_current_path() const
{
    ROS_INFO("DWA get_current_path: returning path with %lu points", this->current_path.size());
    return this->current_path;
}
/*
Function: setter for boolean indicator of whether the robot has a path
Arguments: none
Behaviour: set the current real path 
*/
void DWALocalPlanner::set_current_path(const std::vector<utils::MapPoint>& new_path)
{
    ROS_INFO("DWA set_current_path: received path with %lu points", new_path.size());
    this->current_path = new_path;
    ROS_INFO("DWA set_current_path: internal current_path now has %lu points", this->current_path.size());
}



/*
Function: publishRealPath 
Arguments: path: the real path stored as a vector of MapPoint (metric coordinates)
           path_pub: the ROS publisher for visualizing path 
Behaviour: Publishes real path directly from metric coordinates
*/
void DWALocalPlanner::publishRealPath(const std::vector<utils::MapPoint>& path, const ros::Publisher& path_pub)
{
    nav_msgs::Path path_msg;
    path_msg.header.frame_id = "occupancy_map";
    path_msg.header.stamp = ros::Time::now();

    for (const auto& point : path)
    {
        geometry_msgs::PoseStamped pose;
        pose.header = path_msg.header;

        pose.pose.position.x = point.x;
        pose.pose.position.y = point.y;
        pose.pose.position.z = 0.3;

        pose.pose.orientation.w = 1.0;

        path_msg.poses.push_back(pose);
    }

    path_pub.publish(path_msg);
}
/*
Function: return the marker publisher object for publishing real path markers
Arguments: none
Behaviour: Returns the ROS publisher for visualization markers of the real path
*/
 const ros::Publisher& DWALocalPlanner::getPathPublisher() const
 {
    return this->real_path_pub;
 }
