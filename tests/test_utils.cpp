/**********************************************************************************

Name: Xuezheng Chen 
Student ID: 32470387 
File: test_utils.cpp 
Purpose: Contains the unit tests and main function of the utils file. 
Description: The unit testing file of the utility functions.
**********************************************************************************/

/*********************************** INCLUDES ***********************************/

#include "utils.h"
#include "occupancy_grid.h"
#include <gtest/gtest.h>

/***************************** FUNCTIONS DEFINITIONS ******************************/

// Adapted from Gen AI
/*
Description: Test utils::fromOccupancyGridMsg(). 
Arguments:  None.
Returns: See the description in main().
*/
TEST(UtilsTest, fromOccupancyGridMsgProducesCorrectData) {

    // Initialize grid
    OccupancyGrid grid(5, 5, 0.05);
    grid.setCell(0, 0, 100);
    grid.setCell(0, 1, 0);
    grid.setCell(1, 0, 100);

    // Create a fake message
    nav_msgs::OccupancyGrid grid_occupancy_data_msg = grid.toOccupancyGridMsg("occupancy_map", grid.gridGridOccupancyData());

    // Decode the message and reconstruct the utils::Map
    utils::Map reconstructedMap = utils::fromOccupancyGridMsg(grid_occupancy_data_msg);

    EXPECT_EQ(reconstructedMap.size(), grid.getNumRows());
    EXPECT_EQ(reconstructedMap[0].size(), grid.getNumColumns());

    // Check first three cells are correct
    EXPECT_EQ(reconstructedMap[0][0], 100);
    EXPECT_EQ(reconstructedMap[0][1], 0);
    EXPECT_EQ(reconstructedMap[1][0], 100);

}

/*
Description: The entry point of unit testing for utils file.
Arguments: - argc: Argument Count
            - argv: Argument Vector
Returns: In the console output, returns test result for each individual test block:
            - RUN: The test block starts to run.
            - OK: The test block passed.
            - FAILED: The test block failed the test.
*/
int main(int argc, char **argv) {

    // ROS initialization for testing
    ros::init(argc, argv, "utils_test"); 
    ros::NodeHandle nh;

    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();

}