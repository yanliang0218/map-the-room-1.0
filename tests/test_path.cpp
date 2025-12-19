/***********************************************************************************************************************************************************
* Course: MECH 524
* Student Name: Connor McAllister
* Student Number: 65908436
* Date: December 2nd 2025
*
* Source File: test_pathj.cpp
*
* Purpose:
* Implement unit testing for Path class
* 
* Description:
* This file implements unit testing for the Path class and related objects,
* including PathPlanners, PathInterpolators, DiscretePaths and RealPaths
* using gtest unit testing library 
*
*************************************************************************************************************************************************************/


/************************************************************************* INCLUDES *************************************************************************/

#include <gtest/gtest.h>
#include "grid.h"
#include "path.h"

/************************************************************************* TESTS ****************************************************************************/

/* ---------------------------------------------------------------- AStarPlanner Class ---------------------------------------------------------------------*/

// Setup test fixture
class AStarPlannerTests : public testing::Test {

    protected:

        Mapping::GlobalMapGrid global_map;
        Mapping::RobotLocalMap robot_map;
        std::vector<std::vector<int>> new_occupancy;
        std::vector<std::vector<int>> new_visits;
        Mapping::AStarPlanner planner;

        void SetUp() override {

            global_map = Mapping::GlobalMapGrid(5, 5, 0.1);
            robot_map = Mapping::RobotLocalMap(&global_map, 1e-5);
            planner = Mapping::AStarPlanner(&global_map, &robot_map, 0.5);
        }
};

/*
Test: StartEqualsGoal
Purpose: Verify edge case of start and end point matching
Expectation: Returnssingle element path equal to start/end point*/
TEST_F(AStarPlannerTests, StartEqualsGoal) {
    
    utils::MapIndex idx(1,1);

    std::vector<utils::MapIndex> path = planner.generatePath(idx, idx);

    EXPECT_EQ(path.size(), 1);
    EXPECT_EQ(path[0], idx);
    

}

/*
Test: StraightLinePath
Purpose: validate AStarPlanner behaviour without obstacles
Expectation: returns straight path for start and end points located horizontally and diagonally */
TEST_F(AStarPlannerTests, StraightLinePath) {

    // Generate diagonal path
    utils::MapIndex start1(0, 0);
    utils::MapIndex end1(4, 4);
    std::vector<utils::MapIndex> expected_1 = {utils::MapIndex(0, 0), utils::MapIndex(1, 1), utils::MapIndex(2, 2), utils::MapIndex(3, 3), utils::MapIndex(4, 4)};
    std::vector<utils::MapIndex> path1 = planner.generatePath(start1, end1);

    // Generate linear path
    utils::MapIndex start2(0, 2);
    utils::MapIndex end2(4, 2);
    std::vector<utils::MapIndex> expected_2 = {utils::MapIndex(0, 2), utils::MapIndex(1, 2), utils::MapIndex(2, 2), utils::MapIndex(3, 2), utils::MapIndex(4, 2)};
    std::vector<utils::MapIndex> path2 = planner.generatePath(start2, end2);

    EXPECT_EQ(path1, expected_1);
    EXPECT_EQ(path2, expected_2);
}

/*
Test: UnreachableGoal
Purpose: Validate AStarPlanner behaviour in the event of an unreachable goal
Expectation: returns empty path */
TEST_F(AStarPlannerTests, UnreachableGoal) {

    // Define x variable to improve new_occupancy readability
    int x = 100;
    new_occupancy = {
            {0, 0, x, 0, 0},
            {0, 0, x, 0, 0},
            {0, 0, x, x, x},
            {0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0}
        };

    // Define empty 2D visit vector
    new_visits = std::vector<std::vector<int>>(5, std::vector<int>(5, 0));

    // Update global and local maps
    std::vector<utils::MapIndex> updates = global_map.updateMap(new_occupancy, new_visits);
    robot_map.bushFireUpdate(updates);

    std::vector<utils::MapIndex> path = planner.generatePath(utils::MapIndex(4, 0), utils::MapIndex(0, 4));

    // Expect empty path return
    EXPECT_EQ(path.size(), 0) << "Path Size: " << path.size();
}

/*
Test: */
TEST_F(AStarPlannerTests, SimpleTurnTest) {

    // Define x variable to improve new_occupancy readability
    int x = 100;
    new_occupancy = {
            {0, 0, x, 0, 0},
            {0, 0, x, 0, 0},
            {x, x, x, 0, 0},
            {0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0}
        };
    new_visits = std::vector<std::vector<int>>(5, std::vector<int>(5, 0));

    std::vector<utils::MapIndex> updates = global_map.updateMap(new_occupancy, new_visits);
    robot_map.bushFireUpdate(updates);

    std::vector<utils::MapIndex> expected_path = {
        utils::MapIndex(4, 0), utils::MapIndex(3, 1), utils::MapIndex(3, 2) , utils::MapIndex(2, 3),
        utils::MapIndex(1, 3), utils::MapIndex(0, 3)
    };

    std::vector<utils::MapIndex> path = planner.generatePath(utils::MapIndex(4, 0), utils::MapIndex(0, 3));

    EXPECT_EQ(path, expected_path);
    
    
}


/*
Test: ProioritizeUnvisited
Purpose: Test visit sensitivity functionality within AStarPlanner
Expectation: in two paths of equal length, least visited direction will be prioritized */
TEST_F(AStarPlannerTests, PrioritizeUnvisited) {

    // Define x variable to improve new_occupancy readability
    int x = 100;
    new_occupancy = {
            {0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0},
            {0, 0, x, 0, 0},
            {0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0}
        };

    // Set left side as visited, expect prioritization of right side
    new_visits = {
            {0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0},
            {1, 1, 0, 0, 0},
            {0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0}
        };

    // Update local and global maps with new obstacles and visits
    std::vector<utils::MapIndex> updates = global_map.updateMap(new_occupancy, new_visits);
    robot_map.bushFireUpdate(updates);

    std::vector<utils::MapIndex> expected_path_right = {utils::MapIndex(0, 2), utils::MapIndex(1, 2), utils::MapIndex(2, 3), utils::MapIndex(3, 2), utils::MapIndex(4, 2)};
    std::vector<utils::MapIndex> path_right = planner.generatePath(utils::MapIndex(0, 2), utils::MapIndex(4, 2));

    // Set right side as visited, expect prioitization of left side
    new_visits = {
            {0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0},
            {0, 0, 0, 1, 1},
            {0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0}
        };

    // Update local and global maps with new obstacles and visits
    updates = global_map.updateMap(new_occupancy, new_visits);
    robot_map.bushFireUpdate(updates);

    std::vector<utils::MapIndex> expected_path_left = {utils::MapIndex(0, 2), utils::MapIndex(1, 2), utils::MapIndex(2, 1), utils::MapIndex(3, 1), utils::MapIndex(4, 2)};
    std::vector<utils::MapIndex> path_left = planner.generatePath(utils::MapIndex(0, 2), utils::MapIndex(4, 2));

    EXPECT_EQ(path_left, expected_path_left);
    EXPECT_EQ(path_right, expected_path_right);
}

/* -------------------------------------------------------------------- DiscretePath Class --------------------------------------------------------------------*/

/*
Test: RefinePathSimpleTurn
Purpose: validate RefinePath functionality within DiscretePath
Expectation: Path is refined to three points for a simple turn:
    - start index
    - corner point
    - end index */
TEST(DiscretePathTests, RefinePathSimpleTurn) {

    Mapping::GlobalMapGrid global_map = Mapping::GlobalMapGrid(5, 5, 0.1);
    Mapping::RobotLocalMap robot_map = Mapping::RobotLocalMap(&global_map, global_map.getResolution() / 4);

    // Define x variable to improve new_occupancy readability
    int x = 100;
    std::vector<std::vector<int>> new_occupancy = {
            {0, 0, x, 0, 0},
            {0, 0, x, 0, 0},
            {x, x, x, 0, 0},
            {0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0}
        };
    std::vector<std::vector<int>> new_visits = std::vector<std::vector<int>>(5, std::vector<int>(5, 0));

    std::vector<utils::MapIndex> updates = global_map.updateMap(new_occupancy, new_visits);
    robot_map.bushFireUpdate(updates);

    std::vector<utils::MapIndex> input_path = {
        utils::MapIndex(4, 0), utils::MapIndex(3, 1), utils::MapIndex(3, 2), utils::MapIndex(3, 3), utils::MapIndex(2, 3),
        utils::MapIndex(1, 3), utils::MapIndex(0, 3)
    };

    std::vector<utils::MapIndex> refined_path = {utils::MapIndex(4, 0), utils::MapIndex(2, 3), utils::MapIndex(0, 3)};

    Mapping::DiscretePath path = Mapping::DiscretePath(&global_map, &robot_map, input_path);

    path.refinePath();

    EXPECT_EQ(path.getPathCoordinates(), refined_path);
}

/* -------------------------------------------------------------------- PathSmoother Class --------------------------------------------------------------------*/

/*
Test: SingleTurnInterpolate
Purpose: Validate behaviour of linear interpolator
Expectation: Output of three point path matches values obtained via hand calculations */
TEST(LinearInterpolatorTests, SingleTurnInterpolate) {

    std::vector<utils::MapPoint> input_path = {utils::MapPoint(0, 1), utils::MapPoint(2, 0), utils::MapPoint(2, 2)};

    
    Mapping::LinearInterpolator interpolator(0.5);
    std::vector<utils::MapPoint> path = interpolator.interpolate(input_path);


    // Segment 1:
    // total distance = sqrt(5)
    // Expected 4 points, evenly spaced by absolute distance 0.559
    // dx = + 2 / 4
    // dy = -1 / 4

    // Segment 2:
    // total distance = 2
    // Expected 4 points, evenly spaced by absolute distance 0.5
    // dx = 0
    // dy = 0.5

    std::vector<double> expected_x = {0, 0.5, 1, 1.5, 2, 2, 2, 2, 2};
    std::vector<double> expected_y = {1, 0.75, 0.5, 0.25, 0, 0.5, 1, 1.5, 2};

    EXPECT_EQ(path.size(), 8) << "Path Size: " << path.size();

    for(int i = 0; i < 8; i++) {
        EXPECT_DOUBLE_EQ(path[i].x, expected_x[i]) << "Expected x: " << expected_x[i] << " | Actual x: " << path[i].x;
        EXPECT_DOUBLE_EQ(path[i].y, expected_y[i]) << "Expected y: " << expected_y[i] << " | Actual y: " << path[i].y;
    }

}