/**********************************************************************************
Name: Liang Yan
Student Name: 33140351 
File: ros_interface.cpp
Purpose: Contains the ROSInterface class definitions
Description: from occupancy grid, odometry, bumper, frontier topics, we update the internal state of the robot
             such as current pose, current velocities, occupancy grid, visit count grid, bumper states, and nearest frontier point.
             These are then used by the both  global planner and local planner to to plan a global path and compute velocity commands, respectively.

**********************************************************************************/
/*********************************** INCLUDES ***********************************/

#include "ros_interface.h"



using namespace std;
using namespace utils;

/*
Description: constructor of ROSInterface class
Arguments:  node handle, map resolution
Behavior: - sets kinematic constraints and instantiates a DynamicWindowSampler object
*/
/***************************** CLASS DEFINITION************************************/
ROSInterface::ROSInterface(ros::NodeHandle& nh, double resolution)
    : nh(nh), resolution(resolution), transform_ready(false)
{

    this->resolution = resolution;

    //1) Set up ROS transform and subscriptions
    this->transform_ready = false;

    try{
        // Wait up to 5 seconds for transform to be available
        if (this->tf_listener.waitForTransform("occupancy_map", "odom", ros::Time(0), ros::Duration(5.0))) 
        {
            //if transform becomes available, lookup and set it to odom_to_map_transform
            this->tf_listener.lookupTransform("occupancy_map", "odom", ros::Time(0), this->odom_to_map_transform);
            this->transform_ready = true;
            ROS_INFO("Transform from odom to occupancy_map loaded successfully");
        } 
        else {
            ROS_WARN("Transform not available at startup, will try in first callback");
        }
    } 
    catch (tf::TransformException &ex) 
    {
        ROS_WARN("Transform lookup failed at startup: %s", ex.what());
    }

    // Subscribe to /odom and tell ROS to call current ROSInterface object -> odomCallback(...) when updating the odom information
    //i.e. current pose, and velocities
    this->odom_sub = nh.subscribe("/odom",1,&ROSInterface::odomCallback,this);

    // Subscribe to /occupancy_grid and tell ROS to call current ROSInterface object -> mapCallback(...) when updating the occupancy grid
    this->map_sub  = nh.subscribe("/occupancy_grid",1,&ROSInterface::mapCallback,this);
    //Similar logic the visit count map
    this->visit_count_sub = nh.subscribe("/occupancy_visit_map",1,&ROSInterface::visitCountCallback,this);

    // Subscribe to /frontier and tell ROS to call current ROSInterface object -> frontierCallback(...) when updating the nearest frontier point
    this->frontier_sub = nh.subscribe("/frontier",1,&ROSInterface::frontierCallback,this);

    // Subscribe to /frontier_list i.e. a list of all current frontiers
    // and tell ROS to call current ROSInterface object -> frontierListCallback(...) when updating the nearest frontier point
    this->frontier_list_sub = nh.subscribe("/frontier_list",1,&ROSInterface::frontierListCallback,this);

    // Subscribe to /bumper and tell ROS to call current ROSInterface object -> bumperCallback(...) when bumper is pressed
    this->bumper_sub = nh.subscribe("/mobile_base/events/bumper",1,&ROSInterface::bumperCallback,this);


    //2) Initialize internal variables
    this->bumper[0] = kobuki_msgs::BumperEvent::RELEASED;
    this->bumper[1] = kobuki_msgs::BumperEvent::RELEASED;
    this->bumper[2] = kobuki_msgs::BumperEvent::RELEASED;

    //currently does not have occupancy grid or path at the moment of instantiation/startup
    this->has_occupancy_grid = false;
    this->has_visit_count_grid = false;
    this->has_bumper_data = false;
    this->bumper_pressed_and_needs_reset = false;
    this->has_frontier = false;
    this->has_frontier_list = false;

}

/***************************** METHOD DEFINITIONS *****************************/
/*
Description: odomCallBack
Arguments:  nav_msgs::Odometry::ConstPtr& msg
Behavior: - receives odometruy information and transforming 
            from odometry frame to occupancy grid frame
*/
void ROSInterface::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{

    if (!this->transform_ready) {
        try 
        {
            this->tf_listener.lookupTransform("occupancy_map", "odom", ros::Time(0), this->odom_to_map_transform);
            this->transform_ready = true;
            ROS_INFO("Transform loaded on first odom callback");
        } 
        catch (tf::TransformException &ex) 
        {
            //If transform is still not available, skip this callback, wait for next one
            ROS_WARN("Transform still not available: %s", ex.what());
            return;  
        }
    }
    
    //Lookup the most recent transform from odom to occupancy_map, 
    //but in our project, the same transform persists throughout runtime
    try {
        this->tf_listener.lookupTransform("occupancy_map", "odom", ros::Time(0), this->odom_to_map_transform);
    }
    catch (tf::TransformException &ex) {
        ROS_WARN("Transform lookup failed in odomCallback: %s", ex.what());
        return;
    }
    
    // Extract position from odometry. this is now using origin as robot's spawn point -> odom frame
    tf::Vector3 current_pos_in_odom_frame(msg->pose.pose.position.x, 
                            msg->pose.pose.position.y, 
                            msg->pose.pose.position.z);
    
    
    // Transform from odom frame to occupancy frame, with origin at the bottom-left corner of the world
    tf::Vector3 current_pos_in_occupancy_grid_frame = this->odom_to_map_transform * current_pos_in_odom_frame;
    
    // Store transformed pose
    this->pose.x = current_pos_in_occupancy_grid_frame.x();
    this->pose.y = current_pos_in_occupancy_grid_frame.y();
    
    // Transform orientation
    tf::Quaternion q_odom;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, q_odom);
    tf::Quaternion q_map = this->odom_to_map_transform.getRotation() * q_odom;
    
    double roll, pitch, yaw;
    tf::Matrix3x3(q_map).getRPY(roll, pitch, yaw);

    //store current theta of robot. no coordinate shifting needed here
    this->pose.theta = yaw;

    //convert current position to MapIndex
    this->current_position_index = utils::MapIndex(static_cast<int>(std::round(this->pose.y / this->resolution)), static_cast<int>(std::round(this->pose.x / this->resolution)));
   
    // Store velocities (no transform needed)
    this->v_curr = msg->twist.twist.linear.x; 
    this->omega_curr = msg->twist.twist.angular.z;
    ROS_INFO("odomCallback executed");
} 

/*
Description: mapCallBack
Arguments:  OccupancyGrid::ConstPtr& msg
Behavior: - receives the row major raw occupancy grid and convert it 
            to a 2D vector where (0,0) of the occupancy grid represents
            the bottom left corner
            from odometry frame to occupancy grid frame
*/
void ROSInterface::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) 
{
    
    //sets the occupancy grid stored in obstacle_evaluator
    this->current_grid = *msg;
    this->current_converted_2D_grid = utils::fromOccupancyGridMsg(*msg);

    //indicate that we have an occupancy grid   
    this->has_occupancy_grid = true;
}

/*
Description: visitCountCallback 
Arguments:  nav_msgs::OccupancyGrid::ConstPtr& msg
Behavior:  receive the raw visit count grid and 
           convert it to a 2D vector
*/
void ROSInterface::visitCountCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    this->visit_count_grid = *msg;  
    this->visit_count_converted_2D_grid = utils::fromOccupancyGridMsg(*msg);

    this->has_visit_count_grid = true;
}

/*
Description: stores bumper state into bumper state
Arguments:  kobuki_msgs::BumperEvent::ConstPtr& msg
Behavior:  receives bumper state into bumper state
*/
void ROSInterface::bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
    this->bumper[msg->bumper] = msg->state;
    this->has_bumper_data = true;
    ROS_INFO("bumperCallback executed");
}

/*
Description: receive and store the current frontier
Arguments: geometry_msgs::Point::ConstPtr& msg
Behavior:  sets the frontier location to the private frontier
           member and also convert the raw frontier to a MapIndex
*/
void ROSInterface::frontierCallback(const geometry_msgs::Point::ConstPtr& msg)
{
    // Implementation for frontierCallback
    this->current_frontier = *msg;
    this->current_frontier_index = utils::MapIndex(static_cast<int>(std::round(msg->y / this->resolution)), static_cast<int>(std::round(msg->x / this->resolution)));
    this->has_frontier = true;
}
/*
Description: receive and store the current list of frontier
Arguments: geometry_msgs::Polygon::ConstPtr& msg
Behavior:  sets the frontier location to the private frontier
           member and also convert the raw frontier to a MapIndex
*/
void ROSInterface::frontierListCallback(const geometry_msgs::Polygon::ConstPtr& msg)
{
    // Implementation for frontierCallback
    this->current_frontier_list = *msg;
    
    this->current_frontier_index_list.clear();  
    for (const auto& point : msg->points) {
      
        int row = static_cast<int>(std::round(point.y / this->resolution));
        int col = static_cast<int>(std::round(point.x / this->resolution));
        //ROS_INFO("Frontier index: row: %d, col: %d",  row, col);
        this->current_frontier_index_list.push_back(utils::MapIndex(row, col));     
    }
    this->has_frontier_list = true;
    
}

/*
Description: get the distance from the robot's current pose
             to the frontier
Arguments: none
Behavior: calculates the Euclidean distance from the curent pose to 
          the frontier
*/
double ROSInterface::distance_to_frontier() const
{
  utils::Pose robot_pose = this->pose;
  geometry_msgs::Point frontier = this->current_frontier;
  double distance_to_frontier = std::hypot(robot_pose.x - frontier.x, robot_pose.y - frontier.y);
  return distance_to_frontier;
} 

//getter method for resolution
double ROSInterface::get_resolution() const 
{
    return this->resolution;
}

//getter method for NodeHandle
ros::NodeHandle& ROSInterface::getNodeHandle()
{
    return nh;
}

//getter method for pose
utils::Pose ROSInterface::get_pose() const 
{
    return this->pose;
}

//getter method for current position index
utils::MapIndex ROSInterface::get_current_position_index() const 
{
    return this->current_position_index;
}

//getter method for current linear velocity
double ROSInterface::get_v_curr() const 
{
    return this->v_curr;
}

//getter method for current angular velocity
double ROSInterface::get_omega_curr() const 
{
    return this->omega_curr;
}

//getter method for current occupancy grid
const nav_msgs::OccupancyGrid& ROSInterface::get_current_grid() const 
{
    return this->current_grid;
}

//getter method for current converted 2D occupancy grid
const std::vector<std::vector<int>>& ROSInterface::get_current_converted_2D_grid() const 
{
    return this->current_converted_2D_grid;
}



//getter method for current visit count grid
const nav_msgs::OccupancyGrid& ROSInterface::get_visit_count_grid() const 
{
    return this->visit_count_grid;
}

//getter method for current converted 2D visit count grid
const std::vector<std::vector<int>>& ROSInterface::get_visit_count_converted_2D_grid() const 
{
    return this->visit_count_converted_2D_grid;
}

//getter method for bumper data
const uint8_t* ROSInterface::get_bumper() const
{
    return this->bumper;
}

//getter method for current frontier
geometry_msgs::Point ROSInterface::get_current_frontier() const 
{
    return this->current_frontier;
}

//getter method for current frontier index
utils::MapIndex ROSInterface::get_current_frontier_index() const 
{
    return this->current_frontier_index;
}

//getter method for current frontier list
 const geometry_msgs::Polygon& ROSInterface::get_current_frontier_list() const 
{
    return this->current_frontier_list;
}

//getter method for current frontier index list
const std::vector<utils::MapIndex>& ROSInterface::get_current_frontier_index_list() const 
{
    return this->current_frontier_index_list;
}

//getter method for indicator variables
bool ROSInterface::get_has_occupancy_grid() const 
{
    return this->has_occupancy_grid;
}

//getter method for if we have a visit count grid
bool ROSInterface::get_has_visit_count_grid() const 
{
    return this->has_visit_count_grid;
}

//getter method for if we have a bumper data
bool ROSInterface::get_has_bumper_data() const 
{
    return this->has_bumper_data;
}

bool ROSInterface::get_bumper_pressed_and_needs_reset() const
{
    return this->bumper_pressed_and_needs_reset;
}


void ROSInterface::set_bumper_pressed_and_needs_reset(bool value)
{
    this->bumper_pressed_and_needs_reset = value;
}

//getter method for if we ahve a frontier point
bool ROSInterface::get_has_frontier() const 
{
    return this->has_frontier;
}


//getter method for if we ahve a frontier list
bool ROSInterface::get_has_frontier_list() const 
{
    return this->has_frontier_list;
}




