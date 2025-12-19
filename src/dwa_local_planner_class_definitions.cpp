
/**********************************************************************************
Name: Liang Yan
Student Number: 33140351 
Name: dynamic_window_sampler.cpp
Purpose: Contains the DynamicWindowSampler class definitions. 
**********************************************************************************/
/*********************************** INCLUDES ***********************************/
#include <iostream>
#include "dwa_local_planner_class_headers.h"

using namespace std;
using namespace utils;



/***************************** DynamicWindowSampler Class ********************************

/*
Description: constructor of DynamicWindowSampler class
Arguments:  limits.v_x_min: minimum linear velocity, set to 0 m/s
            limits.v_x_max: maximim linear velocity, 0.65 m/s from Turtebot2 specifications
            limits.omega_min: minimum linear velocity, -180 deg/s from Turtebot2 specifications
            limits.omega_max: maximum linear velocity, 180 deg/s from Turtebot2 specifications
            limits.a_x_max: maximum linear acceleration, tunable
            limits.a_theta_max: maximum angular acceleration, tunable
        
            N_v: number of linear velocity samples, tunable
            N_omega： number of angular velocity samples, tunable

Behavior: - sets kinematic constraints and instantiates a DynamicWindowSampler object
*/

//limits passed by const reference to avoid unnecessary copying of data,
//and to prevent modification of the original struct in constructor
DynamicWindowSampler::DynamicWindowSampler(const Limits& limits,int N_v, int N_omega)
   //sets private variables (initializer list, left private, right from arguments)
    : limits(limits),     
      N_v(N_v),
      N_omega(N_omega)
                                 
{
        //input validation when in debugging mode for kinematic constraints and sample sizes
        if (this->N_v <= 0)
        {
            throw std::invalid_argument("N_v must be > 0");
        }

        if (this->N_omega <= 0)
        {
            throw std::invalid_argument("N_omega must be > 0");
        }

        if (this->limits.v_x_min > this->limits.v_x_max)
        {
            throw std::invalid_argument("v_x_min must be <= v_x_max");
        }
        
        if (this->limits.omega_min > this->limits.omega_max)
        {
            throw std::invalid_argument("omega_min must be <= omega_max");
        }
            
        if (this->limits.a_x_max < 0.0)
        {
            throw std::invalid_argument("linear acceleration limits must be >= 0");
        }
          
        if (this->limits.a_theta_max < 0.0)
        {
           throw std::invalid_argument("angular acceleration limits must be >= 0");
        }
}

/*
Adapted from Fox, Dieter et al (1997) The Dynamic Window Approach to Collision Avoidance

Description: compute dynamic window, i.e. thresholds of reachable linear and angular velocities based on current state and limits_
Arguments:  - v_curr: current linear velocity
            - omega_curr: current angular velocity
            - dt_control: time step of main control loop
Behavior: - linear vel lower threshold = max of the min vx and the current vel - max accel*dt_control; uses max to find the more contraining one
          - linear vel higher threshold = min of the max vx and the current vel + max accel*dt_control; uses min to find the more contraining one
          - angular velocity window follows the same logic
          - returns the thresholds for sampling
*/
//dt_control is the time step size of the main control loop!
//dt control dictates for how long can the robot accelerate in the next control cycle for which the dynamic window is being computed 
DynamicWindowSampler::Window DynamicWindowSampler::compute_window(double v_x_curr, double omega_curr, double dt_control) const
{

    //instance of struct Window to hold velocity and acceleration thresholds
    DynamicWindowSampler::Window window;

    //linear vel lower threshold 
    //this is bound by either the mininum reachable velocity, 
    //or how much can the robot decelerate during one control cycle from current velocity
    //the larger value is the more contraining for a lower threhold
    window.v_low = std::max(this->limits.v_x_min, v_x_curr - this->limits.a_x_max * dt_control); //braking would just be the same

    //linear vel higher threshold 
    //this is bound by either the maximum reachable velocity, 
    //or how much can the robot accelerate during one control cycle from current velocity
    //the smaller value is the more contraining for a higher threhold
    window.v_high = std::min(this->limits.v_x_max, v_x_curr + this->limits.a_x_max * dt_control);

    //angular velocity thresholds follow the same logic
    window.omega_low = std::max(this->limits.omega_min, omega_curr - this->limits.a_theta_max * dt_control);
    window.omega_high = std::min(this->limits.omega_max, omega_curr + this->limits.a_theta_max * dt_control);

    return window;
}

/*
Adapted from Fox, Dieter et al (1997) The Dynamic Window Approach to Collision Avoidance

Description: generate even spaced values of v and omega within ranges from the dynamic window
Arguments:  - window: linear and angular velocity thresholds
            - N_v: number of linear velocity samples
            - N_omega: number of angular velocity samples
            
Behavior: -samples sizes are N_v for linear velocity, and N_omega for angular velocity. generate evenly spaced samples of both and permute all(v,omega) samples        
*/
std::vector<std::pair<double, double>> DynamicWindowSampler::sampler(DynamicWindowSampler::Window window) const
{
    //Vector of pairs to hold samples of (v,omega)
    std::vector<std::pair<double, double>> samples;

    //reserve space in the samples vector for the velocity samples; size is N_v * N_omega
    samples.reserve(this->N_v * this->N_omega);


    //evenly spaced linear velocity samples

    for (int i = 0; i < this->N_v; i++)
    {       
        //container for individual v sample
        double v_i;
   
        //defensive for division by 0
        if (this->N_v == 1)
        {
            //default to half of the high and low thresholds
            v_i = (window.v_high + window.v_low)/2;
        }
        else
        {
            //if not division by zero, compute increments of v with fixed step size
            v_i = window.v_low + i * (window.v_high - window.v_low) / (this->N_v - 1);    
        }
        
        
        //double for-loop ensures every (v, omega) combination is covered
        for (int j = 0; j < this->N_omega; j++)
        {
            //container for individual omega sample
            double omega_j;

            //defensive for division by 0
            if (this->N_omega == 1)
            {
                omega_j = (window.omega_high + window.omega_low)/2;
            }
            else{
                omega_j = window.omega_low + j * (window.omega_high - window.omega_low) / (this->N_omega - 1); 
            }

            //push this specific (v,omega) pair to the vector of samples
            samples.emplace_back(v_i, omega_j);
        }

    }
    return samples;
}

//method for printing generated (v,omega) samples for visual feedback
//numerical test in test.cpp
void DynamicWindowSampler::print_samples(const std::vector<std::pair<double, double>>& samples) const
{
    for (const std::pair<double,double>& sample : samples) 
    {
        cout << "v: " << sample.first << ", omega: " << sample.second << endl;
    }

}


//getter for minimum linear velocity
const DynamicWindowSampler::Limits& DynamicWindowSampler::get_limits() const
{
    return this->limits; 
}


//getter for sample size for linear velocity
int DynamicWindowSampler::get_N_v() const   
{
    return this->N_v;
}

//getter for sample size for angular velocity
int DynamicWindowSampler::get_N_omega() const
{
    return this->N_omega;
}


/***************************** Trajectory Generator Class **********************/

/***************************** CLASS DEFINITION ********************************/

/*
Description: constructor of class TrajectoryGenerator
Arguments:  - horizon: prediction horizon for a given trajectory (arc)
            - dt_trajectory: discretization time step for each point on a given  trajectory 
Behavior: - sets the horiton_ and dt_trajectory_ backing fields and instantiates a TrajectoryGenerator object
*/
TrajectoryGenerator::TrajectoryGenerator(double horizon, double dt_trajectory)
{
    //checking for positive horizon and dt_trajectory values
    if(dt_trajectory <= 0){
        throw std::invalid_argument("dt_trajectory must be non-negative");
    }
    if(horizon <= 0){
        throw std::invalid_argument("horizon must be non-negative");
    }
    //set horizon and dt_trajectory values
    this->horizon = horizon; // default 2s
    this->dt_trajectory = dt_trajectory; //default 0.1s
}

/*
Description: trajectory generator for a given (v,omega) .
Arguments:  - start: start pose on the trajectory
            - v: the linear velocity in the given (v,omega) pair being evaluated
            - omega: the angu;ar velocity in the given (v,omega) pair being evaluated
Behavior: generate discretized trajectory for a given (v,omega) pair using the unicycle kinematics model (shown below) 
*/
std::vector<utils::Pose> TrajectoryGenerator::simulate_trajectory (const utils::Pose& start, double v, double omega)
{
    //vector of poses that hold the trajectory
    std::vector<utils::Pose> traj;

    //reserves (horizon/step_size +1) to accomodate the initial pose
    traj.reserve(static_cast<int>(this->horizon / this->dt_trajectory) + 1);

    //assigns initial pose to a pose variable for iterative updating bellow
    utils::Pose p = start; 

    //pushes initial pose onto the vector
    traj.push_back(p);

    //looping through each time step within the prediction horizon; 
    //loops till one dt_trajectory before reaching horizon, predict the next horizon
    //this last horizon contains the pose at the end 
    for (int i = 0; i < this->horizon/this->dt_trajectory; i++)
    {   
        //starting from the start pose, calculat the x and y components of the linear velocity v
        double v_x_component = v * std::cos(p.theta);
        double v_y_component = v * std::sin(p.theta);

        //in the following dt_trajectory, the x coordinate of the robot will have moved by v_x * dt_trajectory, and y v_y * dt_trajectory
        p.x +=  v_x_component * this->dt_trajectory;
        p.y += v_y_component * this->dt_trajectory;
        p.theta += omega * this->dt_trajectory;

        //push this next pose onto the trajectory
        traj.push_back(p);
    }

    return traj;
}

//getter for horizon_
double TrajectoryGenerator::get_horizon()
{
    return this->horizon;
}

//getter for dt_trajectory
double TrajectoryGenerator::get_dt_trajectory()
{
    return this->dt_trajectory;
}


/*
Adapted from OpenAI ChatGPT
Description: helper function trajectory printer
Arguments:  - traj: vecetor of poses that define the trajectory genenrated by a (v,omega) pair
Behavior: prints the trajectory generated by a given (v,omega) pair in a simple grid using cout
          this will come handy for visualizing individual trajectories when debugging.
*/
void TrajectoryGenerator::print_traj(std::vector<utils::Pose> traj)
{
    if (traj.empty()) return;

    //initialize boundary values for display
    double xmin = traj[0].x, xmax = traj[0].x;
    double ymin = traj[0].y, ymax = traj[0].y;

    //find boundary values by looping through the trajectory
    for (auto& p : traj) 
    {
        xmin = std::min(xmin, p.x);
        xmax = std::max(xmax, p.x);
        ymin = std::min(ymin, p.y);
        ymax = std::max(ymax, p.y);
    }

    //set grid resolution
    //each grid cell ~ 1.5 mm
    double resolution = 0.0015;  

    //allow some padding around edges
    //2 cm padding to provide extra space for the boundary points
    double padding = 0.02;   

    //apply padding to boundaries
    xmin -= padding;
    ymin -= padding;
    xmax += padding;
    ymax += padding;

    //convert bounds into discrete grid size - total number of rows & columns
    //casting to int for discretization
    int rows = static_cast<int>((ymax - ymin) / resolution) + 1;
    int cols = static_cast<int>((xmax - xmin) / resolution) + 1;

    //create grid
    //populate with "." initially
    std::vector<std::string> grid(rows, std::string(cols, '.'));

    //plot trajectory points
    for (size_t k = 0; k < traj.size(); k++) 
    {
        
        //compute row and column numbers of a given point on the trajectory
        int r = static_cast<int>((traj[k].y - ymin) / resolution);
        int c = static_cast<int>((traj[k].x - xmin) / resolution);

        //flip y for to shift origin from bottom left corner
        //put * for points on the trajectory
        if (r >= 0 && r < rows && c >= 0 && c < cols)
        {
            grid[rows - r - 1][c] = '*';
        }
            
    }

    //DEBUG:print the entire grid to console
   /* for (auto& row : grid)
    {
        std::cout << row << std::endl;
    }*/
        
}


/***************************** Obstacle Evaluator Class *************************/
/***************************** CLASS DEFINITION *********************************/
/*
Assisted by OpenAI ChatGPT
Description: constructor of class ObstacleEvaluator
Arguments: - resolution: resolution of the occupancy grid in meters/cell
		   - radius_robot: radius of the robot in meters    
Behavior: - sets the occupancy_grid private field to the current occupancy grid and instantiates an ObstacleEvaluator object
*/

 ObstacleEvaluator::ObstacleEvaluator(double resolution, double radius_robot)
 {
    //sets private occupancy grid with correct dimension from one received from map callback

//    this->occupancy_grid = occupancy_grid;
   
//    this->num_rows = occupancy_grid.size();
//    this->num_cols = occupancy_grid.empty() ? 0 : occupancy_grid[0].size();

//     //initialize clearance grid with max double values
//     this->clearance_grid.assign(num_rows,std::vector<double>(num_cols, std::numeric_limits<double>::max()));
	//DEBUG:cout << "clearance grid set to max double" << endl;

    //sets resolution and robot radius
    if (resolution <= 0.0)
    {
        throw std::invalid_argument("resolution must be positive");
    }

    if (radius_robot <= 0.0) {
        throw std::invalid_argument("radius_robot must be positive");
    }

    this->resolution = resolution;
    this->radius_robot = radius_robot;

    //compute the clearance grid upon contruction
    //this->compute_clearance_grid();

    //DEBUG:cout<<"clearance_grid_computed";
    
   
    //for (int i = 0; i < this->num_rows; i++)
    //{
    //    for (int j = 0; j < this->num_cols; j++)
    //    {
    //        cout<<clearance_grid[i][j]<<" ";
    //    }
    //    cout<<endl;
    //}
 }


/*
Assisted by OpenAI ChatGPT
Description: used in mapCallback to update occupancy grid
Arguments: - occupancy_grid: the current occupancy grid  
Behavior: - updates occupancy grid everytime mapCallback sends an updated occupancy grid
*/
void ObstacleEvaluator::set_grid(const std::vector<std::vector<int>>& occupancy_grid)
{
    //std::cout <<" in obs_eval: received new occupancy grid" << std::endl;

    this->occupancy_grid = occupancy_grid;

    this->num_rows = occupancy_grid.size();
    this->num_cols = occupancy_grid.empty() ? 0 : occupancy_grid[0].size();

    //std::cout << " in obs_eval: num_rows: " << this->num_rows << ", num_cols: " << this->num_cols << std::endl;
    //std::cout << " in obs_eval: about to allocate clearance_grid..." << std::endl;

    this->clearance_grid.assign(num_rows,std::vector<double>(num_cols, std::numeric_limits<double>::max()));
    
    //std::cout << " in obs_eval: clearance_grid allocated successfully" << std::endl;

    //compute clearance grid every time  agrid is received
    this->compute_clearance_grid();
}




/*
Description: used in mapCallback to update clearance_grid occupancy grid
Arguments: - occupancy_grid: the current occupancy grid  
Behavior: - compute the minimum distance with the bushfire algorithm to obstacles for each cell
            and update the clearance_grid accordingly.
*/
void ObstacleEvaluator::compute_clearance_grid()
{
    //std::cout<<"in compute clearance 1: computing clearance grid"<<endl;
    //container for starting points i.e. sources
    std::queue<std::pair<int, int>> sources;
    
    //std::cout<<"in compute clearance 2: finding sources"<<endl;
    //std::cout<<"in compute clearance 3: num_rows: "<<this->num_rows<<", num_cols: "<<this->num_cols<<endl;

    //iterating over the occupancy grid to find all occupied and unknown cells as sources
    for(int i = 0; i < this->num_rows; i++) 
    {
        for(int j = 0; j < this->num_cols; j++) 
        {
            //treating both occupied and unknown cells as sources
            //push them onto the queue for process one by one
            if(this->occupancy_grid[i][j] == 100 || this-> occupancy_grid[i][j] == -1) 
            { 
                this->clearance_grid[i][j] = 0.0;
                sources.push(std::make_pair(i,j));
                //std::cout << "in compute clearance 3a: found source cell (" << i << "," << j << ")" << std::endl;
            }

        }
    }
    
    //std::cout << "in compute clearance 4: found " << sources.size() << " obstacle/unknown cells, starting bushfire..." << std::endl;
    
    //variables to hold index of current cell being processed
    int current_row, current_col;
    //int cells_processed = 0;

    //iterating over all sources until queue is empty
    while(sources.size() != 0) {
        
        // cells_processed++;
        // if (cells_processed % 10000 == 0) {
        //     //std::cout << "in compute clearance 5: Processed " << cells_processed << " cells, queue size: " << sources.size() << std::endl;
        // }

        // Get current cell
        current_row = sources.front().first;
        current_col = sources.front().second;

        //std::cout << "in compute clearance 6: processing cell (" << current_row << "," << current_col << ") " << std::endl;

        //container for newly computed distance to neighbor cells
        double new_distance;

        //since it is being processed, pop it from the queue
        sources.pop();

        // Iterate through neighbors
        //std::cout << "in compute clearance 7: iterating through neighbors of cell (" << current_row << "," << current_col << ")" << std::endl;
        for(int i = current_row - 1; i <= current_row + 1; i++) 
        {
            for(int j = current_col - 1; j <= current_col + 1; j++)
            {   
                //skip out of bounds 
                if (i < 0 || i >= num_rows || j < 0 || j >= num_cols) 
                {
                   continue;
                }

                // Skip central node i.e. the current node being expanded for its neighbors
                if(i == current_row && j == current_col) 
                {
                    continue;
                }

                // Compute new distance to diagonal neighbor
                if(i != current_row && j != current_col) 
                {
                    //if diagonal, increment by hypotenuse of one cell i.e. diagonal resolution
                    //further subtract radius of robot to get clearance from robot boundary to obstacle
                    new_distance = this->clearance_grid[current_row][current_col] + std::sqrt(2) * this->resolution;
                }
                else {
                    //if not diagonal, increment by one cell resolution i.e. horizontal or vertical neighbor
                    //further subtract radius of robot to get clearance from robot boundary to obstacle
                    new_distance = this->clearance_grid[current_row][current_col] + this->resolution;
                }

                // if new distance is smaller, update and push neighbor to queue for processing its neighbors
                // and potentially updating its neighbors distances to be shorter
                if(new_distance < this->clearance_grid[i][j]) 
                {
                    this->clearance_grid[i][j] = new_distance;
                    sources.push({i, j});
                }
            }

        }

    }
    
    //std::cout << "in obs_eval: bushfire completed"<<std::endl;
           
}

/*
Description: Check if a single pose is already inside an obstacle or the pose is in unknown space
Arguments: - Pose p: the pose on a given trajectory led to by a given (v, omega) pair currently evaluated
Behavior: - Check if a single pose is already inside an obstacle or the pose is in unknown space
*/
bool ObstacleEvaluator::on_obstacle_or_unknown(const utils::Pose& p) const
{
    // Get the value occupancy grid at a given (x, y)
    // Using round to map from world coordinates to grid indices 
    // e.g. x=0.22m with res=0.1m -> division = 2.2 -> round = 2 -> index 2 corresponds to 0.2m in world coordinates
    int col = static_cast<int>(std::round(p.x/this->resolution));
    int row = static_cast<int>(std::round(p.y/this->resolution));

    // Bound check 
    if (row < 0 || col < 0 || row >= num_rows || col >= num_cols)
    {
        //DEBUG:cout<<"(" << p.x << "," << p.y<<")"<<" out of bounds"<<endl;
        return true;  
    }

    int cell_value =  this->occupancy_grid[row][col]; 

    // Unknown (-1) or occupied (100) means collision
    // if unknown, we don't know if this point is safe, so mark as collision to be safe
    // TODO: discuss with Loise about value to consider occupied with smoothing
    if ((cell_value == -1) || (cell_value == 100))
    {
        return true;
    }

    //if Free (0), pose is not on an obstacle
    return false;
}

/*
Description: computes the minimum clearance to any obstacle out of all points on a given trajectory
Arguments: - traj: the current trajectory being evaluated
Behavior: - looping through all discretized poses on a give trajectory, first evaluate if any pose is on an obstacle (on the inflated map) or unknown
          - if so, minimum clearance is automatically 0.
          - if else, keep looping until the minimum clearance has been found
*/
double ObstacleEvaluator::compute_clearance(const std::vector<utils::Pose>& traj) const
{
    //initialize the minimum clearance to a very large value
    double min_clearance = std::numeric_limits<double>::max(); 

    for (const utils::Pose& p : traj)
    //looping through all poses on a given traj
    {
        //cout<<"pose"<<p.x<<","<<p.y<<endl;
        // First check if pose collides or is in unknown space
        if (this->on_obstacle_or_unknown(p))
        {
			//DEBUG:cout << "point" <<p.x<<","<<p.y<<"on obstacle or unknown space, returned 0" << endl;
            //returns 0 clearance for later automatic rejection in trajectory evaluator
            return 0.0;
            
        }

        //if not, compute distance from current pose to nearest obstacle
        //clearance reduced by robot radius 
        int col = static_cast<int>(std::round(p.x/this->resolution));
        int row = static_cast<int>(std::round(p.y/this->resolution));

        double d = std::numeric_limits<double>::max();

        if ( row >=0 && row < this->num_rows && col >=0 && col < this->num_cols) 
        {
            //get clearance from robot boundary to nearest obstacle
            d = this->clearance_grid[row][col] - this->radius_robot;
            //DEUG: cout<<"clearance updated to"<<d<<endl;  
        }


        //update the minimum clearance found so far
        if (d < min_clearance) {
            min_clearance = d;
        }
            
    }

    //std::cout<<"min clearance ="<<min_clearance<<endl;
    return min_clearance;
    //return min_clearance;
}

//getters for private members

const std::vector<std::vector<int>>& ObstacleEvaluator::get_occupancy_grid() const 
{
    return this->occupancy_grid;
}

const std::vector<std::vector<double>>& ObstacleEvaluator::get_clearance_grid() const 
{
    return this->clearance_grid;
}

double ObstacleEvaluator::get_resolution() const 
{
    return this->resolution;
}
size_t ObstacleEvaluator::get_num_rows() const 
{
    return this->num_rows;
}

size_t ObstacleEvaluator::get_num_cols() const 
{
    return this->num_cols;
}

double ObstacleEvaluator::get_radius_robot() const 
{
    return this->radius_robot;
}


/***************************** Path Tracker Class *******************************/

/***************************** CLASS DEFINITION *********************************/
/*
Description: constructor of class PathTracker
Arguments:  - resolution: resolution of the occupancy grid
            - k_multiplier_lookahead: lookahead distance multiplier for manual tuning
            - v_x_curr: current linear velocity of the robot
            - v_max: maximum linear velocity of the robot
            - horizon: prediction horizon for a given trajectory (arc)
			- robot_radius: radius of the robot
			- path: the current real path given by AStar planner
Behavior:   - instantiates a path tracker object while dynamically computing the lookaehead distance
			 based on current velocity and robot parameters
*/
PathTracker::PathTracker(double k_multiplier_lookahead)
{
    //input validation
    if (k_multiplier_lookahead < 0.0) {
        throw std::invalid_argument("lookahead multiplier must be non-negative");
    }
    this->k_multiplier_lookahead = k_multiplier_lookahead;

}


/*
Description: set lookahead distance
Arguments:- see above
Behavior:- dynamically compute the lookaehead distance based on current velocity and robot parameters
*/
void PathTracker::set_lookahead(double resolution, double k_multiplier_lookahead, double v_x_curr, double v_max, double horizon, double robot_radius)
{
	//the minimum lookahead distance should be at least the map resolution or twice the robot radius to avoid oscillations
	//if the lookahead is small than the resolution, slight deviations from the path cause the robot to think it is way off the path
    //causing the robot to oscillate while repeatedly trying to correct for the heading error that seems to never go away
	//in vast majority of cases, twice the robot radius is larger than the map resolution, so this will be an effective minimum lookahead distance
    double min_lookahead = std::max(resolution, 2.0 * robot_radius);
    //DEBUG:cout<<"min lkahd= "<<min_lookahead<<endl;

	//the maximum lookahead distance should be limited by the distance the robot can travel within the horizon at max speed
    //but also scaled by a tunable multipler to avoid being too small or too large
	//REPORT: horizon should be set to constant value across the system to avoid inconsistencies in scoring trajectories
    double max_lookahead = horizon*v_max;
    //DEBUG:cout << "max lkahd= " << max_lookahead << endl;

    //set default lookahead based on the  k_multiplier*current velocity
	//using absolute value of velocity to avoid negative lookahead in case of backward motion
    double default_lookahead = k_multiplier_lookahead *std::fabs(v_x_curr);

    //extra validation for v_x_curr
    if (v_x_curr<0 || v_x_curr > v_max)
    {
        throw std::invalid_argument("current velocity is bounded by 0 and v_max");
    }
    //DEBUG:cout<<"v_x_curr= "<<v_x_curr<<endl;
    //DEBUG:cout << "default lkahd= " << default_lookahead << endl;

	//clamp lookahead within min and max bounds
	this->lookahead = utils::clamp(default_lookahead, min_lookahead, max_lookahead);
 }	

/*
Description: - to evaluate if a given trajectory leads the robot close or on the input path
Arguments:  - pose: the current pose being evaluated, this will be the end pose of every trajectory generated
            - lookahead: from the end pose of a given trajectory, a distance that we want to imaginarily march forward on the path
                         to identify where the path is leading a small distance ahead. Comparing this with the end pose tells us 
                         how much the given trajectory is deviating from the prescribed path
            
Behavior:   - from the end pose of a given trajectory, first identify the point on the path closest to the end of the trajectory. Then travel
              along the path while accumulating distance marched forward till we reach the lookahead distance. The point on the path we stopped 
              at will be compard with the heading of end of the trajectory to determine how much a given trajectory is following our prescribed 
              path.         
*/
utils::MapPoint PathTracker::find_target_on_path(const utils::Pose& pose)
{
    //default case - if path is empty, just return the pose
    if (this->path.empty())
        return {pose.x, pose.y};

    //1. Find closest point on path to robot current pose

    //initializes the index of the map node on the path closest to a given pose to 0
    //initializes the distance of the map node on the path closest to a given pose to a very large value
    size_t index_closest_pathpoint_to_traj_end = 0;
    double dist_closest_pathpoint_to_traj_end = std::numeric_limits<double>::max();

    //for every map point on the path 
    for (size_t i = 0; i<this->path.size();i++)
    {
        //compute x and y distances to get the distance from the current path point to the end of the trajectory
        //note again that pose here is going to be the end pose of a given trajectory being evaluated
        double dx = this->path[i].x - pose.x;
        double dy = this->path[i].y - pose.y;
        double distance = std::hypot(dx, dy);

        //update the minimum distance and the index
        if (distance<dist_closest_pathpoint_to_traj_end)
        {
            dist_closest_pathpoint_to_traj_end = distance;
            index_closest_pathpoint_to_traj_end = i;
        }
    }

    // 2. March forward until lookahead distance reached

    //initializes an accumulated forward distance to keep track of how far we've looked ahead
    double accum_forward_dist = 0.0;    

    //start from the identified closest point 
    //looping through path points ahead and accumulate distance ahead
    for (size_t i = index_closest_pathpoint_to_traj_end; i<this->path.size()-1;i++)
    {
        //repeat the same Pythagorean procedure for every path point reached
         double dx = this->path[i+1].x - this->path[i].x;
         double dy = this->path[i+1].y - this->path[i].y;
         double seg_accum_forward_dist = std::hypot(dx,dy);

        //increment the accumulated forward distance
         accum_forward_dist += seg_accum_forward_dist;

         //when the lookahead distance has been reached, return the current path point
         if (accum_forward_dist >= this->lookahead)
         {
            //DEBUG::cout<<"ran"<<endl;
            return this->path[i+1];
         }
    }
   
    //if the for loop exits without reaching the forward distance
    //this means that the trajectory is so close to the end of the path
    //that the lookahead cannot be reached before the end of the path is reached
    //in this case, just return the last point of the path
    return this->path.back();
}

//getter for the current real path
const std::vector<utils::MapPoint>& PathTracker::get_path() const
{
    return this->path;
}

//getter fot the lookahead distance
double PathTracker::get_lookahead() const
{
    return this->lookahead;
}    

//getter for the k_multiplier for lookahead
double PathTracker::get_k_multiplier_lookahead() const
{
    return this->k_multiplier_lookahead;
}

/*
Description: set updated path in when receiving path from AStar planner
Arguments:  - path: updated current path
Behavior:   - sets the current path to the path_backing field
*/
void PathTracker::set_path(const std::vector<utils::MapPoint>& path)
{
    if (path.empty())
    {
        throw std::invalid_argument("path cannot be empty");
    }

    this->path = path;
}



/***************************** Trjactory Evaluator Class*************************/

/***************************** CLASS DEFINITION ********************************/
/*
Description: constructor of class TrajectoryEvaluator
Arguments:  - heading_weight: weight assigned to minimizing heading error i.e. keep on the path
            - clearance_weight: weight assigned to maximizing clearance from obstacles
            - velocity_weight: weight maximizing velocity selected
            - braking_decel: maximum braking deceleration - set equal to maximum linear acceleration

Behavior:   - sets the weights and braking deceleration
*/

TrajectoryEvaluator::TrajectoryEvaluator(TrajectoryEvaluator::Weights weights, double braking_decel)
{
    //input validation against individual weights that are not between 0 and 1 (inclusive)
    if (!((weights.heading_weight >= 0.0 && weights.heading_weight <= 1.0) &&
        (weights.clearance_weight >= 0.0 && weights.clearance_weight <= 1.0) &&
        (weights.velocity_weight >= 0.0 && weights.velocity_weight <= 1.0)))
    {
        throw std::invalid_argument("individual weights must be between 0 and 1 inclusive");
    }

    //input validation against sum of weights not equal to 1 (allow small epsilon)
    double sum_weights = weights.heading_weight + weights.clearance_weight + weights.velocity_weight;
    if (std::fabs(sum_weights - 1.0) > 1e-6)
    {
        throw std::invalid_argument("weights must sum to 1");
    }

    //setting the individual weights as private members
    this->weights.heading_weight = weights.heading_weight;
    this->weights.clearance_weight = weights.clearance_weight;
    this->weights.velocity_weight = weights.velocity_weight;

    //input validation against negative braking deceleration
    if ((braking_decel < 0))
    {
        throw std::invalid_argument("braking_decel must be non-negative");
    }
    //setting the individual weights as private members
    this->braking_decel = braking_decel;
}



/*****************************Trajectory Evlaluator Class******************************** /
/*
Description: computes the individual scores, total score, and the Boolean admissibillity criterion for the given trajectory
Arguments:  - traj: the current trajectory being evaluated
            - target: the point on the path that is a lookahead distance ahead of the closest point on the path to the end
                      the end of the trajectory. this point is used as a reference point to measure the ideal heading that
                      the Turtlebot has to take on in order to stay on the path after it goes down this given trajectory
            - v: linear component of the (v,omega) pair that led to this particular trajectory being evaluated
            - v_x_max: maximum translational velocity of Turtlebot 2, used to normalize velocity for fair scoring
            - longest_dimension of room: length of the longest straight line possibly drew in the room
              if the room is square, it will be sqrt(2)*side_length; this is used to normalize clearance score
            - obs: an ObstacleEvaluator class object used to compute clearance

Behavior:   - the method first evaluates the minimum clearance to any obstacle or unknown cell out of any discretized point
              on the trajectory. If this clearance is 0, the total score is defaulted to -1, and the Boolean admissibility to false,
              thereby rejecting the trajectory automatically. It then calculated the difference between the ideal heading from the
              end point on the trajectory to the target point on the path, and calculates the difference from this angle to the current
              heading of the robot. This is the heading error for keeping on the path. This value is bound within 0-180 deg, and normalized
              by division  over 180 deg. This is then subtracted from 1, so that the smaller the heading error, the higher the heading
              score. Next, the linear velocity and the maximum braking deceleration are used to compute the stopping distance. If this
              stopping distance exceeds the minimum clearance previously calculated, a score of -1 is automatically assigned, and the
              Boolean admissibility check is set to false.

*/
TrajectoryEvaluator::TrajectoryScores TrajectoryEvaluator::evaluate(const std::vector<utils::Pose>& traj,
    const utils::MapPoint& target, double v, double v_x_max, double longest_dimension_of_room, const ObstacleEvaluator& obs)
{
    //initialize the struct of scores for this particular trajectory
    TrajectoryScores scores;

    //1. compute clearance score and check for collision against obstacles or unknown space

    //compute the minimum clearance of this particular trajectory
    double clearance = obs.ObstacleEvaluator::compute_clearance(traj);
    //std::cout<<"clearance ="<<clearance<<endl;
    
    //sets the clearance score, normalized by the longest dimension of the given room
    if (longest_dimension_of_room <= 0)
    {
        throw invalid_argument("longest dimension must be positive");
    }
        
    //guarding against if the clearance returns very high value
    scores.clearance_score = std::min(1.0, clearance / longest_dimension_of_room);

	//DEBUG:cout << "clearance= " << clearance << endl;

    //initialize the Boolean admissibility check to true
    scores.admissibility = true;

    //If this clearance is or less than zero (minus robot radius), the total score is defaulted to -1, and return
    //Boolean admissibility to false, thereby rejecting the trajectory automatically
    if (clearance <= 0.0) {
        scores.total_score = -1.0;
        scores.admissibility = false;
        return scores;
    }

    //2. compute heading score 

    //get the last pose on this given trajectory, used to calculate the heading error
    if (traj.empty())
    {
        throw std::invalid_argument("trajectory should not be empty!");
    }
    
    utils::Pose end_traj = traj[traj.size()-1];

    //calculate the target heading from the last pose on the trajectory to the target point on the path
    double target_theta = atan2(target.y - end_traj.y, target.x - end_traj.x);

    //get the current heading of the robot when at the last pose on the path
    double current_theta = end_traj.theta;

    //the heading error is the absolute value difference between the two 
    double heading_error = std::fabs(utils::wrapAngle(target_theta-end_traj.theta));

    //the smaller the heading error, the higher the score, so 1 - heading error/pi
    scores.heading_score = 1.0 - (heading_error / M_PI);

    //3. compute velocity score (before scaling)
    //this is just the linear velocity normalized by the maximum linear velocity
    scores.velocity_score = std::fabs(v) / v_x_max;

    //4. compute braking distance and check against clearance
    //stopping distance from constant deceleration

    //double stopping_dist = (v * v) / (2 * this->braking_decel);

    double stopping_dist = (v * v) / (2.25 * this->braking_decel);
    

    //if the stopping distance is beyond the clearance
    if (stopping_dist > clearance)
    {
        //set total score to -1, Boolean admissibility check to false,  return
        scores.total_score = -1.0;
        scores.admissibility = false;
        return scores;
    }

    //sum individual scores to get the total score component of scores
    scores.total_score = scores.heading_score * this->weights.heading_weight + scores.clearance_score * this->weights.clearance_weight
    + scores.velocity_score * this->weights.velocity_weight;
     
    //return the scores struct (including the total score, Boolean admissibility, and individual scores
    return scores;

}

//getter for heading weight
double TrajectoryEvaluator::get_heading_weight()
{
    return this->weights.heading_weight;
}

//getter for clearance weight
double TrajectoryEvaluator::get_clearance_weight()
{
    return this->weights.clearance_weight;
}

//getter for velocity weight
double TrajectoryEvaluator::get_velocity_weight()
{
    return this->weights.velocity_weight;
}

//getter for braking deceleration
double TrajectoryEvaluator::get_braking_decel()
{
    return this->braking_decel;
}

void TrajectoryEvaluator::print_trajectory_scores(const TrajectoryEvaluator::TrajectoryScores& scores) const
{
    cout<<"clearance score ="<<scores.clearance_score<<endl;
    cout<<"heading score =" << scores.heading_score << endl;
    cout<<"velcoity score =" << scores.velocity_score << endl;
    cout<<"total score =" << scores.total_score << endl;
    cout<<"admissibility"<<scores.admissibility<<endl;
}



