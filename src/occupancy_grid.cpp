/**********************************************************************************

Name: Xuezheng Chen 
Student ID: 32470387 
File: occupancy_grid.cpp 
Purpose: Contains the OccupancyGrid class. 
Description: The OccupancyGrid class build, maintain, and update an occupancy grid.

**********************************************************************************/

/*********************************** INCLUDES ***********************************/

#include "occupancy_grid.h"

/***************************** CLASS IMPLEMENTATIONS ******************************/

/* Struct utils::MapPoint */

/*
Function: print
Arguments:  None
Behavior: Print the map point.
*/
void utils::MapPoint::print() {
    std::cout << "(" << x << ", " << y << ")" << std::endl;
}


/* Class OccupancyGrid */

/*
Class: OccupancyGrid
Description: Constructor of the class.
Arguments:  - map_height: The height of the map in m
            - map_width: The width of the map in m
            - resolution: The side length of a grid cell in m
Behavior: Initializes an OccupancyGrid object and create two grid with the given dimensions.
            One grid is the occupancy map, the other is grid visit map.
*/
OccupancyGrid::OccupancyGrid(int map_height, int map_width, double cell_resolution) {

    resolution = cell_resolution;
    num_row = static_cast<int>(std::round(map_height / resolution));
    num_column = static_cast<int>(std::round(map_width / resolution));

    // Offset of points to calculate for bumper event, 
    // the offset should be the distance between robot center and bumpers
    robot_offset = 0.1; 

    // According to <nav_msgs/OccupancyGrid.h> convention
    // Occupancy probabilities are in the range [0, 100]
    // 0 stands for a known empty cell while 100 stands for an occupied cell
    // -1 stands for an unknown cell
    // Generates an empty grid with all cells unknown
    grid_occupancy_data = utils::Map(num_row, std::vector<int>(num_column, -1));

    // Generates an grid with all cells never visited
    grid_visit_data = utils::Map(num_row, std::vector<int>(num_column, 0));

    // Set the origin of the maps relative to the origin of Gazebo world coordinate 
    map_origin = utils::MapPoint{- num_column * resolution / 2, - num_row * resolution / 2};

    // Initialize robot coordinate and orientation
    // For project construction purposes, we set the robot to start at the origin of Gazebo world coordinate, which is the center of the occupancy map. 
    // However, since the origin of the occupancy map coordinate is in the "lower-left" corner of the occupancy map, we manually offset the robot coordinate.
    // Moreover, this is just an initialization of the two variables, it cannot actually move the robot anywhere, it only stores the robot coordinate for further calculations.
    robot_coord_x = - map_origin.x;
    robot_coord_y = - map_origin.y; 
    robot_theta = 0;

    // A transformation matrix is initialized with the odom(robot)'s coordinate relative to the occupancy map coordinate system.
    // Set the origin of odom frame with respect to the occupancy grid frame
    transform.setOrigin(tf::Vector3(robot_coord_x, robot_coord_y, 0.0));
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, robot_theta);   
    transform.setRotation(q);

}

/*
Function: getNumRows
Arguments:  None
Returns: Number of rows
*/
int OccupancyGrid::getNumRows() {
    return num_row;
}

/*
Function: getNumColumns
Arguments:  None
Returns: Number of columns
*/
int OccupancyGrid::getNumColumns() {
    return num_column;
}

/*
Function: getResolution
Arguments:  None
Returns: Resolution
*/
double OccupancyGrid::getResolution() {
    return resolution;
}

/*
Function: getMapOrigin
Arguments:  None
Returns: utils::Map origin in the Gazebo world coordinate
*/
utils::MapPoint OccupancyGrid::getMapOrigin() {
    return map_origin;
}

/*
Function: getRobotOffset
Arguments:  None
Returns: Number of offset in meters from the center of the robot
*/
double OccupancyGrid::getRobotOffset() {
    return robot_offset;
}

/*
Function: gridGridOccupancyData
Arguments:  None
Returns: The entire grid map.
*/
utils::Map OccupancyGrid::gridGridOccupancyData() {
    return grid_occupancy_data;
}

/*
Function: getGridVisitData
Arguments:  None
Returns: The grid visit map that counts how many times a cell is visited.
*/
utils::Map OccupancyGrid::getGridVisitData() {
    return grid_visit_data;
}

/*
Function: getTransform
Arguments:  None
Returns: The transformation matrix.
*/
tf::Transform OccupancyGrid::getTransform() {
    return transform;
}

/*
Function: getRobotX
Arguments:  None
Returns: The current x coordinate of the robot.
*/
double OccupancyGrid::getRobotX() {
    return robot_coord_x;
}

/*
Function: getRobotY
Arguments:  None
Returns: The current y coordinate of the robot.
*/
double OccupancyGrid::getRobotY() {
    return robot_coord_y;
}

/*
Function: getRobotTheta
Arguments:  None
Returns: The current theta angle of the robot.
*/
double OccupancyGrid::getRobotTheta() {
    return robot_theta;
}

/*
Function: setRobotX
Arguments: - new_x: the new x coordinate
Behavior: Set the robot_coord_x to new_x
*/
void OccupancyGrid::setRobotX(const double new_x) {
    robot_coord_x = new_x;
}

/*
Function: setRobotY
Arguments: -  new_y: the new y coordinate
Behavior: Set the robot_coord_y to new_y
*/
void OccupancyGrid::setRobotY(const double new_y) {
    robot_coord_y = new_y;
}

/*
Function: setRobotTheta
Arguments: - new_theta: the new theta
Behavior: Set the robot_theta to new_theta
*/
void OccupancyGrid::setRobotTheta(const double new_theta) {
    robot_theta = new_theta;
}

/*
Function: setCell
Arguments:  - row: The row number of the cell to edit.
            - col: The column number of the cell to edit.
            - val: The new value of the cell
Behavior: Set the cells in (row, col) entered to desired val.
*/
void OccupancyGrid::setCell(const int row, const int col, const int val) {
    grid_occupancy_data[row][col] = val;
}

/*
Function: setGridData
Arguments:  - new_map: The new occupancy map to replace the old one
Behavior: Set the occupancy grid to the entered one.
*/
void OccupancyGrid::setGridData(const utils::Map new_map) {
    grid_occupancy_data = new_map;
}

// Adapted from Gen AI
/*
Function: preprocessLaserScan
Description: Preprocess the laser scan message to obtain valid points only. 
                Project laser points to the world coordinate system.
                Mark occupied cell and free cell at the laser point.
Arguments: - msg: Laser message.
Returns: Valid laser points in world coordinate system
*/
std::vector<LaserPoint> OccupancyGrid::preprocessLaserScan(

    const sensor_msgs::LaserScan::ConstPtr& msg

) {

    const double max_clear_distance = 2.0;
    std::vector<LaserPoint> points;

    for (size_t i = 0; i < msg->ranges.size(); ++i) {

    double r = msg->ranges[i];

    // Treat NaN / invalid as NO INFORMATION
    if (std::isnan(r) ||
        r < msg->range_min ||
        r > msg->range_max) {
        continue;  // do nothing
    }

    // Angle in laser frame
    double angle = msg->angle_min + i * msg->angle_increment;

    // Clamp distance used for ray end
    double r_to_use = std::min(r, max_clear_distance);

    // Transform to robot frame
    double r_x = r_to_use * std::cos(angle);
    double r_y = r_to_use * std::sin(angle);

    // Transform to world frame
    double world_x = robot_coord_x +
                     (r_x * std::cos(robot_theta) - r_y * std::sin(robot_theta));
    double world_y = robot_coord_y +
                     (r_x * std::sin(robot_theta) + r_y * std::cos(robot_theta));

    bool is_occupied = (r <= max_clear_distance);

    points.push_back(LaserPoint{world_x, world_y, is_occupied});

    }

    return points;

}

// Adapted from Gen AI
/*
Function: markLineFree
Description: Bresenham's line algorithm to mark free cells. 
Arguments:  - x0, y0: The row and column number of the starting point (robot).
            - x1, y1: The row and column number of the end point (occupied cell).
Behavior: All cells in the line between the robot's current position and occupied cell will be marked free cell.
*/
void OccupancyGrid::markLineFree(int x0, int y0, int x1, int y1) {

    std::vector<utils::MapIndex> pointsInBetween = utils::discreteMapBresenhamLineSearch(x0, y0, x1, y1, getNumRows(), getNumColumns());

    for (utils::MapIndex& point : pointsInBetween) {
            
        if (grid_occupancy_data[point.row][point.col] != 100) {

            setCell(point.row, point.col, 0); // free

        }

    }

}

// Adapted from Gen AI
/*
Function: updateFromLaserData
Arguments:  - msg: Laser message.
Behavior:   1. Send the laser message to member function preprocessLaserScan.
                The member function will process the laser scan message and returns
                valid laser points in world coordinate system.
            2. Update the occupancy grid with preprocessed point coordinates, mark both occupied cells
            3. Call member function markLineFree to use Bresenham algorithm to mark all unknown cells between 
                robot current position and occupied cells as free cells.
*/
void OccupancyGrid::updateFromLaserData(const sensor_msgs::LaserScan::ConstPtr& msg) {

    std::vector<LaserPoint> points = preprocessLaserScan(msg);

    int robot_col = utils::clamp(static_cast<int>(robot_coord_x / resolution), 0, num_column - 1);
    int robot_row = utils::clamp(static_cast<int>(robot_coord_y / resolution), 0, num_row - 1);

    for (const auto& p : points){

        // Find the coordinate of the occupied cell
        int gx = utils::clamp(static_cast<int>(p.x / resolution), 0, num_column - 1);
        int gy = utils::clamp(static_cast<int>(p.y / resolution), 0, num_row - 1);

        if (grid_occupancy_data[gy][gx] != 100) {

            // Mark obstacle and free cell
            if (p.is_occupied == true) {
                setCell(gy, gx, 100);
            } else if (p.is_occupied == false) {
                setCell(gy, gx, 0);
        }

        }

        // Use Bresenham algorithm to mark the cell as free cells if laser not bounce back.
        markLineFree(robot_col, robot_row, gx, gy);

    }

    // smoothMap();

}

// Adapted from Gen AI
/*
Function: smoothMap
Arguments:  None
Behavior: Smooth the entire occupancy grid map with 3X3 filter. 
*/
void OccupancyGrid::smoothMap() {

    // TODO: If free, can try store each cell as a class or a struct, every laser scan can give the cell 1 mark to its celltype,
    // Every cell need to be scanned two or three times, and assigned the same type before the cell is actually labelled.
    // That makes every cell confirmed multiple times before making a decision. 

    int rows = num_row;
    int cols = num_column;
    utils::Map original = gridGridOccupancyData();
    utils::Map smoothed = original;

    // Iterate through every row
    for (int r = 0; r < rows; r++) {

        // Iterate through every column
        for (int c = 0; c < cols; c++) {

            int count_unknown = 0;
            int count_free = 0;
            int count_occ = 0;

            // Convolve the 3X3 kernel and set the value of the center to the majority of 8 surrounding cells
            // Cell value = -1 is negligible at this stage
            for (int dr = -1; dr <= 1; dr++) {

                for (int dc = -1; dc <= 1; dc++) {

                    int rr = utils::clamp(r + dr, 0, rows - 1);
                    int cc = utils::clamp(c + dc, 0, cols - 1);
                    int v = original[rr][cc];

                    if (v == -1) {
                        count_unknown++;
                    } else if (v == 0) {
                        count_free++;
                    } else if (v == 100) {
                        count_occ++;
                    }

                }
            }
            
            // Majority decision
            if (count_occ > count_free && count_occ > count_unknown) {
                smoothed[r][c] = 100;
            } else if (count_free > count_occ && count_free > count_unknown) {
                smoothed[r][c] = 0;
            } else {
                smoothed[r][c] = -1;
            }

        }
    }

    // Reset current occupancy grid map
    setGridData(smoothed);

}

// Adapted from Gen AI
/*
Function: updateFromBumperData
Description: The bumperCallback function already verified the PRESSED state and a valid press interval in the OccupancyGridNode.
Arguments:  - msg: Bumper message.
Behavior: Verify which bumper is pressed, mark adjacent 5 cells as occupied.
*/
void OccupancyGrid::updateFromBumperData(const kobuki_msgs::BumperEvent::ConstPtr& msg) {

    // Determine which bumper was pressed,
    // and add offset angle to change from the direction robot is facing to left/right. 
    // CCW is positive angle rotation.
    double delta_theta = 0.0;

    // delta_theta is the angle of the line connecting robot center and bumper
    // sigma_theta is the angle perpendicular to delta_theta
    // Bumber has a 60 degree angle between the robot front direction
    if (msg->bumper == kobuki_msgs::BumperEvent::LEFT) {
        delta_theta = robot_theta + M_PI / 3.0;
    } else if (msg->bumper == kobuki_msgs::BumperEvent::CENTER) {
        delta_theta = robot_theta;
    } else if (msg->bumper == kobuki_msgs::BumperEvent::RIGHT) {
        delta_theta = robot_theta - M_PI / 3.0;
    }
    
    double sigma_theta = delta_theta + M_PI / 2.0;

    // Compute a line of 5 obstacle cells in that direction
    std::vector<std::pair<int, int>> obstacle_points;

    for (int i = -2; i <= 2; i++) {

        // Convert offset from meters → cells
        double world_x = robot_coord_x + robot_offset * std::cos(delta_theta)
                         + i * resolution * std::cos(sigma_theta);

        double world_y = robot_coord_y + robot_offset * std::sin(delta_theta)
                         + i * resolution * std::sin(sigma_theta);

        int gx = static_cast<int>(world_x / resolution);
        int gy = static_cast<int>(world_y / resolution);  

        if (gx >= 0 && gx < num_column && gy >= 0 && gy < num_row) {
            obstacle_points.push_back(std::make_pair(gy, gx)); // row (y), col (x)
        }

    }

    // Mark 5 cells as occupied
    for (const auto& point : obstacle_points) {
        setCell(point.first, point.second, 100);
    }

}

/*
Function: updateVisitCell
Arguments:  None
Behavior: Update the cell visited data according to current robot coordinate.
*/
void OccupancyGrid::updateVisitCell() {

    int col = static_cast<int>(robot_coord_x / resolution);
    int row = static_cast<int>(robot_coord_y / resolution);
    grid_visit_data[row][col] += 1;

}

/*
Function: updateFromOdomData
Arguments:  - msg: Odom message.
Behavior: Update the robot position and orientation in the occupancy grid frame with /odom message.
                Also update the visit cell count if the robot is moving on the cell. 
*/
void OccupancyGrid::updateFromOdomData(const nav_msgs::Odometry::ConstPtr& msg) {

    // The transformation matrix transform the position of the robot in odom coordinate system to occupancy map coordinate system
    // The transformation matrix is used for uniformity with other nodes that may use robot's position
    tf::Vector3 robot_coord_vector_in_odom(msg->pose.pose.position.x, msg->pose.pose.position.y, 0.0);
    tf::Vector3 robot_coord_vector_in_occupancy_grid = transform * robot_coord_vector_in_odom;

    // Update current robot position
    setRobotX(robot_coord_vector_in_occupancy_grid[0]);
    setRobotY(robot_coord_vector_in_occupancy_grid[1]);
    setRobotTheta(tf::getYaw(msg->pose.pose.orientation));

    // Update visit cell if the robot is on a cell and it is moving, i.e. linear velocity greater than 0.001
    // This is to avoid keep updating visit count when a robot is stationary
    const auto& robot_lin_vel = msg->twist.twist.linear;

    if (
        std::abs(robot_lin_vel.x) >= 0.05 || 
        std::abs(robot_lin_vel.y) >= 0.05 || 
        std::abs(robot_lin_vel.z) >= 0.05
    ) {

        updateVisitCell();

    }

}

/*
Function: getFrontiers
Description: Getter function of the frontiers based on the current grid map.
                A cell is considered a frontier if it is a free cell and,
                at least one of its eight neighboring cells is an unknown cell.
Arguments:  None
Returns: All detected frontiers.
*/
std::vector<utils::MapPoint> OccupancyGrid::getFrontiers() {
    // Initialize a new frontier every time
    std::vector<utils::MapPoint> frontiers;

    // Loop through all cells to find the cell satisfying the requirement. 
    for (int row = 0; row < num_row - 1; row++) {

        for (int col = 0; col < num_column - 1; col++) {

            if (grid_occupancy_data[row][col] == 0) {

                bool is_frontier = false;

                // Search for its neighboring cells
                for (int temp_row = row - 1; temp_row <= row + 1 && !is_frontier; temp_row++) {

                    for (int temp_col = col - 1; temp_col <= col + 1 && !is_frontier; temp_col++) {

                        // Skip invalid neighbors that goes beyond the map
                        if (
                            temp_row < 0 || 
                            temp_col < 0 || 
                            temp_row > num_row - 1 || 
                            temp_col > num_column - 1
                        ) {
                            continue;
                        }

                        if (grid_occupancy_data[temp_row][temp_col] == -1) {

                            frontiers.push_back(utils::MapPoint{col*resolution, row*resolution});
                            // No need to check other neighbors when the cell is already confirmed a frontier
                            is_frontier = true;  

                        }
                    }
                }
            }
        }
    }

    return frontiers;

}

/*
Function: findClosestFrontier
Description: Calls member function getFrontiers to obtain all frontiers in the occupancy map.
                Find the frontier closest to robot's current position, 
                but greater than a thresold to avoid obtaining its current position.
                If there is no frontier greater than the threshold, return the frontier closest to the thresold.
                The list of all frontiers is also returned.
Arguments:  None.
Returns: The struct of two ROS messages.
            One is the point of the closest frontier to explore,
            the second is the list of all frontiers.
*/
utils::FrontierResult OccupancyGrid::findClosestFrontier() {

    // Always get the up-to-date frontiers before finding the closest frontier
    std::vector<utils::MapPoint> frontiers = getFrontiers();
    
    utils::MapPoint closest_frontier;
    utils::MapPoint closest_frontier_greater_than_offset;

    bool has_offset_frontier = false;

    double offset_distance = 0.3; // only consider frontiers that are at least 0.3m away from the robot
    double closest_distance = 0;; // initialize it with zero
    double closest_distance_greater_than_offset = std::numeric_limits<double>::infinity(); // initialize it with a huge number
    double robot_x = robot_coord_x;
    double robot_y = robot_coord_y;

    // Initialize an empty polygon message to store all frontiers
    geometry_msgs::Polygon poly;

    // Iterate through all frontiers and calculate the distance to robot current position
    for (const auto& frontier : frontiers) {

        double distance = std::hypot(robot_x - frontier.x, robot_y - frontier.y);

        // To find the frontier that is at least offset_distance away but cloest to it
        if (distance >= offset_distance && distance < closest_distance_greater_than_offset) {

            closest_distance_greater_than_offset = distance;
            closest_frontier_greater_than_offset = frontier;
            has_offset_frontier = true;

        }

        // To find the frontier that is smaller than offset_distance but closest to it
        if (distance < offset_distance && distance > closest_distance) {

            closest_distance = distance;
            closest_frontier = frontier;

        }

        // Add the frontier to the polygon message
        // Polygon message only accepts Point32 type
        geometry_msgs::Point32 p;
        p.x = frontier.x;
        p.y = frontier.y;
        p.z = 0.0;
        poly.points.push_back(p);

    }

    // If there is a frontier that is at least offset_distance away, return that one
    utils::MapPoint result;
    if (has_offset_frontier) {
        result = closest_frontier_greater_than_offset;
    } else {
        result = closest_frontier;
    }

    // Creates a Point message of the closest frontier for publication
    geometry_msgs::Point msg;
    msg.x = result.x;
    msg.y = result.y;
    msg.z = 0.0;

    // Construct the utils::FrontierResult struct to return both messages
    utils::FrontierResult frontier_result;
    frontier_result.frontier_point = msg;
    frontier_result.frontier_list = poly;

    return frontier_result;
}

/*
Function: toOccupancyGridMsg
Description: Convert a map to an OccupancyGrid message for publication in the topic.
Arguments:  - frame_id: The id of the frame, should use the same one if want messages to have the same coordinate system.
            - data_to_convert: The mao data to convert to a OccupancyGrid message.
Returns: The occupancy grid map in the data type of nav_msgs::OccupancyGrid.
*/
nav_msgs::OccupancyGrid OccupancyGrid::toOccupancyGridMsg(

    std::string frame_id,
    utils::Map data_to_convert

) {

    // Adapted from Gen AI
    nav_msgs::OccupancyGrid msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame_id;
    msg.info.resolution = resolution;
    msg.info.width = num_column;
    msg.info.height = num_row;
    // The map is offset w.r.t the Gazebo world coordinate so that Gazebo origin is always in the center of the map
    msg.info.origin.position.x = map_origin.x;
    msg.info.origin.position.y = map_origin.y;
    msg.info.origin.orientation.w = 1.0;    

    // Adapted from Gen AI
    // flatten 2D vector to 1D for ROS message
    msg.data.resize(num_row * num_column);
    for(int i = 0; i < num_row; i++) {

        for(int j = 0; j < num_column; j++) {

            msg.data[i * num_column + j] = data_to_convert[i][j];

        }
    }

    return msg;

}

/*
Function: toRosMsgs
Description: Convert occupancy grid map and occupancy visit map to a OccupancyGrid message for publication in the topic.
Arguments: None
Returns: The occupancy grid map and occupancy visit map both in the data type of nav_msgs::OccupancyGrid.
*/
std::vector<nav_msgs::OccupancyGrid> OccupancyGrid::toRosMsgs() {

    // smoothMap();

    nav_msgs::OccupancyGrid grid_occupancy_data_msg = toOccupancyGridMsg("occupancy_map", grid_occupancy_data);

    nav_msgs::OccupancyGrid grid_visit_data_msg = toOccupancyGridMsg("occupancy_map", grid_visit_data);

    return std::vector<nav_msgs::OccupancyGrid>{grid_occupancy_data_msg, grid_visit_data_msg};

}

/*
Function: printMap
Arguments: - map: the map to print.
Behavior: Iterate through the entire map and prints every cell
*/
void OccupancyGrid::printMap(const utils::Map& map) {

    for (const auto& row : map) {

        for (const auto& cell : row) {

            std::cout << cell << " ";

        }

        std::cout << std::endl;

    }
}
