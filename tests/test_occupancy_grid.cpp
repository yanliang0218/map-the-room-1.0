/**********************************************************************************

Name: Xuezheng Chen 
Student ID: 32470387 
File: test_occupancy_grid.cpp 
Purpose: Contains the unit tests and main function of the OccupancyGrid class. 
Description: The unit testing file of the OccupancyGrid class.
                In this file, unit testings are conducted on important methods of the OccupancyGrid class.
                ATTENTION: grid.toRosMsg() testing needs an active ROS simulation so need to run `roslaunch turtlebot_gazebo turtlebot_world.launch` before compiling.

**********************************************************************************/

/*********************************** INCLUDES ***********************************/

#include "occupancy_grid.h"
#include <gtest/gtest.h>

/***************************** FUNCTIONS DEFINITIONS ******************************/

// Adapted from Gen AI
/*
Description: Test constructor.
Arguments:  None.
Returns: See the description in main().
*/
TEST(OccupancyGridTest, ConstructorInitializesCorrectly) {

    OccupancyGrid grid(5, 5, 0.05); // 5x5 m, 5cm resolution

    EXPECT_EQ(grid.getNumRows(), 100);
    EXPECT_EQ(grid.getNumColumns(), 100);
    EXPECT_DOUBLE_EQ(grid.getResolution(), 0.05);

    // Check all cells initialized to -1 (unknown)
    auto data = grid.gridGridOccupancyData();
    for (auto& row : data) {
        for (auto& cell : row) {
            EXPECT_EQ(cell, -1);
        }
    }

}

// Adapted from Gen AI
/*
Description: Test toRosMsg(). ATTENTION: Needs an active ROS simulation to test.
Arguments:  None.
Returns: See the description in main().
*/
TEST(OccupancyGridTest, ToRosMsgProducesCorrectMessage) {

    OccupancyGrid grid(5, 5, 0.05);

    std::vector<nav_msgs::OccupancyGrid> msgs = grid.toRosMsgs();
    auto msg = msgs[0];

    EXPECT_EQ(msg.info.width, 100);
    EXPECT_EQ(msg.info.height, 100);
    EXPECT_NEAR(msg.info.resolution, 0.05, 1e-3); // Floating point numbers will have rounding errors, cannot use EXPECT_EQ
    EXPECT_EQ(msg.data.size(), 100*100);

    // Check first and last cells are correct
    EXPECT_EQ(msg.data[0], -1);
    EXPECT_EQ(msg.data[msg.data.size()-1], -1);

}

/*
Description: Test setCell() and gridGridOccupancyData().
Arguments:  None.
Returns: See the description in main().
*/
TEST(OccupancyGridTest, SetCellUpdatesCorrectly) {

    OccupancyGrid grid(1, 1, 0.1); // 1x1 m, 10cm resolution
    grid.setCell(0, 0, 100);       // set occupied
    auto data = grid.gridGridOccupancyData();
    EXPECT_EQ(data[0][0], 100);

}

/*
Description: Test updateFromBumperData() and gridGridOccupancyData().
Arguments:  None.
Returns: See the description in main().
*/
TEST(OccupancyGridTest, UpdatesFromBumperDataCorrectly) {

    OccupancyGrid grid(5, 5, 0.05);
    int grid_x = 9; // The coordiante of the 10th row
    int grid_y = 9; // The coordinate of the 10th column
    double grid_coord_x = grid_x * grid.getResolution();
    double grid_coord_y = grid_y * grid.getResolution();

    grid.setRobotY(grid_coord_x);
    grid.setRobotX(grid_coord_y);
    grid.setRobotTheta(M_PI/2); //90 degree, aligned with y axis

    // For right button press, convert robot heading direction to robot heading to the bumper
    double delta_theta = grid.getRobotTheta() - M_PI / 3.0;

    // Adapted from Gen AI
    // (row, column) convention
    double dx = grid.getRobotOffset() * std::cos(delta_theta);
    double dy = grid.getRobotOffset() * std::sin(delta_theta);
    double gx = - grid.getResolution() * std::sin(delta_theta); // cos(theta + 90) = -sin(theta)
    double gy = grid.getResolution() * std::cos(delta_theta);

    std::vector<std::pair<int,int>> expected_occupied;

    // Manually calculate occupied pairs based on given bumper message
    for (int k = -2; k <= 2; k++) {

        int px = static_cast<int>((grid_coord_x + dx + k * gx) / grid.getResolution()); 
        int py = static_cast<int>((grid_coord_y + dy + k * gy) / grid.getResolution());

        expected_occupied.emplace_back(std::make_pair(py, px));

    }

    // Create a dummy bumper message for testing
    kobuki_msgs::BumperEvent::Ptr msg(new kobuki_msgs::BumperEvent);
    msg->bumper = kobuki_msgs::BumperEvent::RIGHT;
    msg->state = kobuki_msgs::BumperEvent::PRESSED;

    grid.updateFromBumperData(msg);

    auto data = grid.gridGridOccupancyData();

    // Create a list to check the location of the occupied cell
    // Check that some cells became 100 (occupied)
    std::vector<std::pair<int, int>> found_occupied;
    for (int i=0; i<data.size(); i++){
        for (int j=0; j<data[i].size(); j++){
            if (data[i][j] == 100){
                found_occupied.push_back(std::make_pair(i, j)); 
            }            
        }      
    }

    // Sort BOTH vectors so ordering does not matter
    std::sort(found_occupied.begin(), found_occupied.end());
    std::sort(expected_occupied.begin(), expected_occupied.end());
    // Remove identical pairs for comparison
    expected_occupied.erase(
        std::unique(expected_occupied.begin(), expected_occupied.end()),
        expected_occupied.end()
    );

    // Check size
    EXPECT_EQ(found_occupied.size(), expected_occupied.size());

    // Check every instance
    EXPECT_EQ(found_occupied, expected_occupied);

}

/*
Description: Test updateFromLaserData() and gridGridOccupancyData().
Arguments:  None.
Returns: See the description in main().
*/
TEST(OccupancyGridTest, UpdateFromLaserDataMarksObstacles) {

    // Create a testing occupancy grid
    OccupancyGrid grid(10, 10, 0.1);
    grid.setRobotX(0.5);
    grid.setRobotY(0.5);
    grid.setRobotTheta(0.0);

    // Adapted from Gen AI
    // Create a dummy laser message for testing
    sensor_msgs::LaserScan::Ptr msg(new sensor_msgs::LaserScan());
    // Angle includes positive x axis
    msg->angle_min = -M_PI / 4;
    msg->angle_max = M_PI / 4;
    msg->angle_increment = M_PI / 8;
    msg->range_min = 0.1;
    msg->range_max = 5.0;
    msg->ranges = {1.0, 1.0, 1.0, 1.0, 1.0}; // five points, each returns at 1.0 m

    grid.updateFromLaserData(msg);

    auto data = grid.gridGridOccupancyData();
    int gx = static_cast<int>((grid.getRobotX() + 1.0) / grid.getResolution());
    int gy = static_cast<int>(grid.getRobotY() / grid.getResolution());
    EXPECT_EQ(data[gy][gx], 100); 

}

/*
Description: Test updateFromOdomData(), updateVisitCell(), and gridGridOccupancyData().
Arguments:  None.
Returns: See the description in main().
*/
TEST(OccupancyGridTest, UpdateFromOdomDataUpdatesRobotStatusAndVisitCell) {

    // Create a testing occupancy grid
    OccupancyGrid grid(10, 10, 0.1);
    utils::MapPoint map_origin = grid.getMapOrigin();

    // Create a dummy laser message for testing
    nav_msgs::Odometry::Ptr msg(new nav_msgs::Odometry());
    msg->header.stamp = ros::Time::now();
    msg->header.frame_id = "odom";
    msg->child_frame_id = "base_link";

    // Place robot at map origin
    msg->pose.pose.position.x = map_origin.x;
    msg->pose.pose.position.y = map_origin.y;
    msg->pose.pose.position.z = 0.0;

    // Set robot orientation to 45 degrees
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, M_PI / 4.0);
    msg->pose.pose.orientation.x = q.x();
    msg->pose.pose.orientation.y = q.y();
    msg->pose.pose.orientation.z = q.z();
    msg->pose.pose.orientation.w = q.w();

    // Set a small non-zero linear velocity to trigger visit update
    msg->twist.twist.linear.x = 0.1;  // greater than 1e-3
    msg->twist.twist.linear.y = 0.0;
    msg->twist.twist.linear.z = 0.0;

    // Record initial visit count at expected cell
    int col = static_cast<int>((msg->pose.pose.position.x - map_origin.x) / grid.getResolution());
    int row = static_cast<int>((msg->pose.pose.position.y - map_origin.y) / grid.getResolution());

    int initial_visit = grid.getGridVisitData()[row][col];

    grid.updateFromOdomData(msg);

    const auto& robot_x = grid.getRobotX();
    const auto& robot_y = grid.getRobotY();
    const auto& robot_theta = grid.getRobotTheta();

    // Verify robot pose update, use near for double
    EXPECT_NEAR(robot_x, 0.0, 1e-3);
    EXPECT_NEAR(robot_y, 0.0, 1e-3);
    EXPECT_NEAR(robot_theta, M_PI / 4.0, 1e-3);

    // Adapted from Gen AI
    // Verify visit cell incremented
    // This tests whether the counter is increased or not
    int updated_visit = grid.getGridVisitData()[row][col];
    EXPECT_GT(updated_visit, initial_visit);

    // This makes sure a stationary robot does not increase visit count
    msg->twist.twist.linear.x = 0.0;
    grid.updateFromOdomData(msg);
    EXPECT_EQ(grid.getGridVisitData()[row][col], updated_visit);

}

/*
Description: Test findClosestFrontier() and getFrontiers().
                The first test case is when the second frontier is smaller than the offset distance
Arguments:  None.
Returns: See the description in main().
*/
TEST(OccupancyGridTest, findClosestFrontierCorrectly1) {

    // First test case is when the second frontier is smaller than the offset distance
    // Create a testing occupancy grid
    OccupancyGrid grid(10, 10, 0.1);
    const auto& resolution = grid.getResolution();

    // Set up a two frontiers by assigning value to the occupancy grid
    int row1 = 5;
    int col1 = 5;
    int row2 = 35;
    int col2 = 35;
    grid.setCell(row1, col1, 0);
    grid.setCell(row2, col2, 0);
    std::vector<utils::MapPoint> ground_truth_frontiers{
        utils::MapPoint{col1*resolution, row1*resolution}, 
        utils::MapPoint{col2*resolution, row2*resolution}
    };
    const auto& ground_truth_closest_frontier = ground_truth_frontiers[0];

    // Set robot position, the robot is always closer to the second frontier
    grid.setRobotX((col2 - 1) * resolution);
    grid.setRobotY((row2 - 1) * resolution);

    // Get frontier, check size, and check value
    std::vector<utils::MapPoint> frontiers = grid.getFrontiers();
    EXPECT_EQ(frontiers.size(), ground_truth_frontiers.size());

    for (int i = 0; i < ground_truth_frontiers.size(); i++) {

        EXPECT_NEAR(frontiers[i].x, ground_truth_frontiers[i].x, 1e-3);
        EXPECT_NEAR(frontiers[i].y, ground_truth_frontiers[i].y, 1e-3);

    }

    // Check closest frontier
    geometry_msgs::Point closest_frontier_msg = grid.findClosestFrontier().frontier_point;
    EXPECT_NEAR(closest_frontier_msg.x, ground_truth_closest_frontier.x, 1e-3);
    EXPECT_NEAR(closest_frontier_msg.y, ground_truth_closest_frontier.y, 1e-3);

}

/*
Description: Test findClosestFrontier() and getFrontiers().
                The second test case is when the second frontier is greater than the offset distance
Arguments:  None.
Returns: See the description in main().
*/
TEST(OccupancyGridTest, findClosestFrontierCorrectly2) {

    // Create a testing occupancy grid
    OccupancyGrid grid(10, 10, 0.1);
    const auto& resolution = grid.getResolution();

    // Set up a two frontiers by assigning value to the occupancy grid
    int row1 = 5;
    int col1 = 5;
    int row2 = 35;
    int col2 = 35;
    grid.setCell(row1, col1, 0);
    grid.setCell(row2, col2, 0);
    std::vector<utils::MapPoint> ground_truth_frontiers{
        utils::MapPoint{col1*resolution, row1*resolution}, 
        utils::MapPoint{col2*resolution, row2*resolution}
    };
    const auto& ground_truth_closest_frontier = ground_truth_frontiers[1];

    // Set robot position, the robot is always closer to the second frontier
    grid.setRobotX((col2 - 8) * resolution);
    grid.setRobotY((row2 - 8) * resolution);

    // Get frontier, check size, and check value
    std::vector<utils::MapPoint> frontiers = grid.getFrontiers();
    EXPECT_EQ(frontiers.size(), ground_truth_frontiers.size());

    for (int i = 0; i < ground_truth_frontiers.size(); i++) {

        EXPECT_NEAR(frontiers[i].x, ground_truth_frontiers[i].x, 1e-3);
        EXPECT_NEAR(frontiers[i].y, ground_truth_frontiers[i].y, 1e-3);

    }

    // Check closest frontier
    geometry_msgs::Point closest_frontier_msg = grid.findClosestFrontier().frontier_point;
    EXPECT_NEAR(closest_frontier_msg.x, ground_truth_closest_frontier.x, 1e-3);
    EXPECT_NEAR(closest_frontier_msg.y, ground_truth_closest_frontier.y, 1e-3);

}

/*
Description: The entry point of unit testing for occupancy_grid class.
Arguments: - argc: Argument Count
            - argv: Argument Vector
Returns: In the console output, returns test result for each individual test block:
            - RUN: The test block starts to run.
            - OK: The test block passed.
            - FAILED: The test block failed the test.
*/
int main(int argc, char **argv) {

    // ROS initialization for testing
    ros::init(argc, argv, "occupancy_grid_test"); 
    ros::NodeHandle nh;

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();

}