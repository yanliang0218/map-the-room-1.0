/***********************************************************************************************************************************************************
* Course: MECH 524
* Student Name: Connor McAllister
* Student Number: 65908436
* Date: November 28th 2025
*
* Source File: test_grid.cpp
*
* Purpose:
* Implement unit testing for Grid class
* 
* Description:
* This file implements the gtest library to test and validate the behaviour of the 
* Grid template class, all derived members, and all grid node types
* 
*************************************************************************************************************************************************************/


/************************************************************************* INCLUDES *************************************************************************/

#include <gtest/gtest.h>
#include <climits>
#include <cmath>
#include <memory>
#include <limits>
#include "grid.h"
#include "path.h"
#include "utils.h"

/************************************************************************* TESTS ****************************************************************************/

/* --------------------------------------------------------------------- Grid Class ------------------------------------------------------------------------*/


/*
Test: ConstructsCorrectSize
Purpose: validate grid constructor dimensions
Expectation: getRows and getCols returns same dimensions as constructor input */
TEST(GridTest, ConstructsCorrectSize) {
    Mapping::Grid<int> grid(3,4);
    EXPECT_EQ(grid.getRows(), 3);
    EXPECT_EQ(grid.getCols(), 4);
}

/*
Test: ConstructorSizeExceptions
Purpose: validate input validation in grid constructor
Expectation: Invalid constructor indices throw invald argument exception */
TEST(GridTest, ConstructorSizeExceptions) {
    EXPECT_THROW(Mapping::Grid<int> grid(-1,1), std::invalid_argument);
    EXPECT_THROW(Mapping::Grid<int> grid(1,-1), std::invalid_argument);

}

/*
Test: GetNodeOutOfBounds
Purpose: validate inability to access out of bounds nodes
Expectation: Exception throw on invalid getNode inputs */
TEST(GridTest, GetNodeOutOfBounds) {
    Mapping::Grid<int> grid(3,4);

    EXPECT_THROW(grid.getNode(3,3), std::invalid_argument);
    EXPECT_THROW(grid.getNode(2,4), std::invalid_argument);
    EXPECT_THROW(grid.getNode(-1,2), std::invalid_argument);
    EXPECT_THROW(grid.getNode(2,-1), std::invalid_argument);
}


/*
Test: IsValidIndexWithinBounds
Purpose: Verify isValidIndex for valid indices
Expectation: returns true for valid indices */
TEST(GridTest, IsValidIndexWithinBounds) {

    Mapping::Grid<int> grid(3,4);

    EXPECT_EQ(grid.isValidIndex(1,2), true);
    EXPECT_EQ(grid.isValidIndex(0,0), true);
    EXPECT_EQ(grid.isValidIndex(2,3), true);
}

/*
Test: IsInalidIndexWithinBounds
Purpose: Verify isValidIndex for invalid indices
Expectation: returns false for valid indices */
TEST(GridTest, IsValidIndexOutOfBounds) {

    Mapping::Grid<int> grid(3,4);

    EXPECT_EQ(grid.isValidIndex(-1,-1), false);
    EXPECT_EQ(grid.isValidIndex(3,4), false);
    EXPECT_EQ(grid.isValidIndex(10,10), false);
}


/* -------------------------------------------------------------------- MapGrid Class -----------------------------------------------------------------------*/


/*
Test: IndexToCoordinateConversion
Purpose: Validate ability to convert inidces to real world coordinates
Expectation: Returns real world coordinates that match hand calculations */
TEST(GlobalMapGridTest, IndexToCoordinateConversion) {
    Mapping::GlobalMapGrid grid(10, 10, 1e-3);
    utils::MapIndex index(3,4);

    EXPECT_EQ(grid.getRealCoordinates(index).x, 3e-3);
    EXPECT_EQ(grid.getRealCoordinates(index).y,  4e-3);
}

/*
Test: CoordinateToIndexConversion
Purpose: Validate Real world coordinate to integer index converison
Expectation: Returns same indices as determined via hand calculation */
TEST(GlobalMapGridTest, CoordinateToIndexConversion) {

    Mapping::GlobalMapGrid grid(10, 10, 1e-3);
    utils::MapPoint point(4e-3, 6e-3);

    EXPECT_EQ(grid.getIndices(point).row, 6);
    EXPECT_EQ(grid.getIndices(point).col, 4);

}

/*
Test: MapGridNodalDefaults
Purpose: ensure appropriate default construction of GlobalMapGrid
Expectation: all nodes within GlobalMapGird have default occupancy and visit count parameters*/
TEST(GlobalMapGridTest, MapGridNodalDefaults) {

    Mapping::GlobalMapGrid grid(10, 10, 1e-3);

    // Verify default values within nodes
    for(int i = 0; i < 10; i++) {
        for(int j = 0; j < 10; j++) {
            EXPECT_EQ(grid.getNode(i, j).getOccupancy(), -1);
            EXPECT_EQ(grid.getNode(i, j).getVisits(), 0);
        }
    }
    
}

/*
Test: UpdatesFromVector
Purpose: Ensure proper map updates from vector input, as received from mapping ROS node
Expectation: visit count and occupancy data within grid matches values provided by vectors */
TEST(GlobalMapGridTest, UpdatesFromVector) {

    // Initialize Grid and vectors
    Mapping::GlobalMapGrid grid(5,5, 1e-3);

    std::vector<std::vector<int>> new_occupancy = {
        {1, 2, 3, 4, 5},
        {6, 7, 8, 9, 10},
        {2, 5, 99, 88, 0},
        {-1, -1, 100, 100, 100},
        {-1, -1, 100, 100, 100}
    };

    std::vector<std::vector<int>> new_visits = {
        {0, 1, 0, 0, 0},
        {0, 0, 1, 0, 0}, 
        {0, 0, 1, 53, 0}, 
        {0, 33, 0, 0, 0},
        {2, 3, 2, 1, 1}
    };


    // Update grid
    std::vector<utils::MapIndex> updates = grid.updateMap(new_occupancy, new_visits);

    // Validate proper nodal parameters within grid
    int row = 0;
    for(int i = 0; i < 5; i++) {
        for(int j = 0; j < 5; j++)
        {
            EXPECT_EQ(grid.getNode(i, j).getOccupancy(), new_occupancy[i][j]);
            EXPECT_EQ(grid.getNode(i, j).getVisits(), new_visits[i][j]);
        }
    }

    // Validate newly occupied indices returned by update
    std::vector<utils::MapIndex> expected_updates = {utils::MapIndex(3, 2), utils::MapIndex(3, 3), utils::MapIndex(3, 4), utils::MapIndex(4, 2), utils::MapIndex(4, 3), utils::MapIndex(4, 4)};
    EXPECT_EQ(updates, expected_updates);



}



/* -------------------------------------------------------------------- GlobalMapNode Class -----------------------------------------------------------------------*/

/*
Test: SetOccupancyInputValidation
Purpose: verify input validation for occupancy field
Expectation: no exception thrown on valud values, exception thrown on values out of bounds [-1, 100]*/
TEST(GlobalMapNodeTest, SetOccupancyInputValidation) {
    Mapping::GlobalMapNode node;

    // Expect no throw for common inputs
    EXPECT_NO_THROW(node.setOccupancy(-1));
    EXPECT_NO_THROW(node.setOccupancy(0));
    EXPECT_NO_THROW(node.setOccupancy(100));


    // expect throw for inputs out of bounds
    EXPECT_THROW(node.setOccupancy(-2), std::invalid_argument);
    EXPECT_THROW(node.setOccupancy(101), std::invalid_argument);

}

/*
Test: SetVisitsInputValidation
Purpose: verify input validation for vists field
Expectation: no exception thrown on valud values, exception thrown on values out of bounds [0, inf]*/
TEST(GlobalMapNodeTest, SetVisitsInputValidation) {
    Mapping::GlobalMapNode node;

    // Expect no throw for values greater than or equal to zero
    EXPECT_NO_THROW(node.setVisits(0));
    EXPECT_NO_THROW(node.setVisits(INT_MAX));


    EXPECT_THROW(node.setVisits(-1), std::invalid_argument);
    EXPECT_THROW(node.setVisits(INT_MIN), std::invalid_argument);
}

/* -------------------------------------------------------------------- SearchNode Class --------------------------------------------------------------------*/

/*
Test: SetGCostInputValidation
Purpose: verify input validation of setGCost to indicate errors within search algorithms
Expectation: thow invalid argument on negative input */
TEST(SearchNodeTest, SetGCostInputValidation) {
    Mapping::SearchNode node(1,2);

    EXPECT_THROW(node.setGCost(-1), std::invalid_argument);
}

/*
Test: SetHCostInputValidation
Purpose: verify input validation of setHCost to indicate errors within search algorithms
Expectation: thow invalid argument on negative input */
TEST(SearchNodeTest, SetHCostInputValidation) {
    Mapping::SearchNode node(1,2);

    EXPECT_THROW(node.setHCost(-1), std::invalid_argument);
}

/*
Test:SetParentInputValidation
Purpose: ensure parent node is adjacent to current node, allowing for continuous path
Expectation: exception throw on non-adjacent parent node set */
TEST(SearchNodeTest, SetParentInputValidation) {

    Mapping::SearchNode node(10, 10);
    std::unique_ptr<Mapping::SearchNode> parent;

    // Iterate through possible neighbors, validate input
    for(int row = 8; row <= 12; row++) {
        for(int col = 8; col <= 12; col++) {

            parent = std::make_unique<Mapping::SearchNode>(row, col);

            if(std::abs(row - 10) > 1 || std::abs(col - 10) > 1) {
                EXPECT_THROW(node.setParent(parent.get()), std::logic_error);
            }
            else {
                EXPECT_NO_THROW(node.setParent(parent.get()));
            }

            // Set parent to nullptr to avoid pointers to deleted instances
            node.setParent(nullptr);

        }
    }
}

/*
Test: RestsToDefaults
Purpose: ensure reset function resets nodal defaults
Expectation: reset creates infinite GCost, zero HCost, no parent node */
TEST(SearchNodeTest, ResetsToDefaults) {
    
    Mapping::SearchNode node(10,10);
    Mapping::SearchNode parent = Mapping::SearchNode(9,11);
    
    // Set parameters to non-defaults
    node.setParent(&parent);
    node.setGCost(100);
    node.setHCost(100);

    // Reset to defaults
    node.reset();

    // Expect default values
    EXPECT_EQ(node.getGCost(), std::numeric_limits<double>::infinity());
    EXPECT_EQ(node.getHCost(), 0);
    EXPECT_FALSE(node.hasParent());


}

/* -------------------------------------------------------------------- SearchGrid Class --------------------------------------------------------------------*/

/*
Test: Constructs default nodes
Purpose: Ensure search grid initialization constructs nodal defaults
Expectation: all nodes within default grid contain default values */
TEST(SearchGridTests, ConstructsDefaultNodes) {

    Mapping::SearchGrid grid(10,10);

    // Validate all nodal defaults
    for(int i = 0; i < 10; i++) {
        for(int j = 0; j < 10; j++) {
            EXPECT_DOUBLE_EQ(grid.getNode(i, j)->getGCost(), std::numeric_limits<double>::infinity());
            EXPECT_DOUBLE_EQ(grid.getNode(i, j)->getHCost(), 0);
        }
    }
}

/*
Test: GetNeighborsCenter
Purpose: test getNeighbors method in centre of grid
Expectation: returns 8 adjacent nodes */
TEST(SearchGridTests, GetNeighborsCenter) {

    Mapping::SearchGrid grid(10,10);

    std::vector<Mapping::SearchNode*> neighbors = grid.getNeighbors(5,5);

    // Expect 8 neighbors
    EXPECT_EQ(neighbors.size(), 8);

    // Expect row and col indices to be within 1 of central node
    for(int i = 0; i < neighbors.size(); i++) {
        EXPECT_LE(std::abs(neighbors[i]->getRow() - 5), 1);
        EXPECT_LE(std::abs(neighbors[i]->getCol() - 5), 1);
    }

}

/*
Test: GetNeighborsEdge
Purpose: test getNeighbors along edge of grid
Expectation: returns 5 adjacent nodes, all within grid bounds for all four edges */
TEST(SearchGridTests, GetNeighborsEdge) {

    Mapping::SearchGrid grid(10,10);

    // run getNeighbors at each edge
    std::vector<Mapping::SearchNode*> neighbors_left = grid.getNeighbors(5, 0);
    std::vector<Mapping::SearchNode*> neighbors_right = grid.getNeighbors(5, 9);
    std::vector<Mapping::SearchNode*> neighbors_top = grid.getNeighbors(0, 5);
    std::vector<Mapping::SearchNode*> neighbors_bottom = grid.getNeighbors(9, 5);

    // verify sizes
    EXPECT_EQ(neighbors_left.size(), 5);
    EXPECT_EQ(neighbors_right.size(), 5);
    EXPECT_EQ(neighbors_top.size(), 5);
    EXPECT_EQ(neighbors_bottom.size(), 5);

    // verify valid indices
    for(int i = 0; i < 5; i++) {
        EXPECT_TRUE(grid.isValidIndex(neighbors_left[i]->getRow(), neighbors_left[i]->getCol()));
        EXPECT_TRUE(grid.isValidIndex(neighbors_right[i]->getRow(), neighbors_right[i]->getCol()));
        EXPECT_TRUE(grid.isValidIndex(neighbors_top[i]->getRow(), neighbors_top[i]->getCol()));
        EXPECT_TRUE(grid.isValidIndex(neighbors_bottom[i]->getRow(), neighbors_bottom[i]->getCol()));
    }
}

/*
Test: GetNeighborsEdge
Purpose: test getNeighbors in corners of grid
Expectation: returns 3 adjacent nodes, all within grid bounds */
TEST(SearchGridTests, GetNeighborsCorner) {

    Mapping::SearchGrid grid(10,10);

    // Run getNeighbors at each corner
    std::vector<Mapping::SearchNode*> neighbors_TL = grid.getNeighbors(0, 0);
    std::vector<Mapping::SearchNode*> neighbors_TR = grid.getNeighbors(0, 9);
    std::vector<Mapping::SearchNode*> neighbors_BL = grid.getNeighbors(9, 0);
    std::vector<Mapping::SearchNode*> neighbors_BR = grid.getNeighbors(9, 9);

    // Verify sizes
    EXPECT_EQ(neighbors_TL.size(), 3);
    EXPECT_EQ(neighbors_TR.size(), 3);
    EXPECT_EQ(neighbors_BL.size(), 3);
    EXPECT_EQ(neighbors_BR.size(), 3);

    // Verify all valid indices
    for(int i = 0; i < 3; i++) {
        EXPECT_TRUE(grid.isValidIndex(neighbors_TL[i]->getRow(), neighbors_TL[i]->getCol()));
        EXPECT_TRUE(grid.isValidIndex(neighbors_TR[i]->getRow(), neighbors_TR[i]->getCol()));
        EXPECT_TRUE(grid.isValidIndex(neighbors_BL[i]->getRow(), neighbors_BL[i]->getCol()));
        EXPECT_TRUE(grid.isValidIndex(neighbors_BR[i]->getRow(), neighbors_BR[i]->getCol()));
    }

}

/* -------------------------------------------------------------------- RobotLocalMap Class --------------------------------------------------------------------*/

// Setup test fixture
class RobotLocalMapTests : public testing::Test {

    public:
        RobotLocalMapTests() = default;

    protected:

        // Test variables
        std::unique_ptr<Mapping::GlobalMapGrid> global_map;
        Mapping::RobotLocalMap robot_map;
        double expected_distance;

        void SetUp() override {

            // Intialize test variables
            global_map = std::make_unique<Mapping::GlobalMapGrid>(5, 5, 0.1);
            double radius = std::sqrt(3) * global_map->getResolution();
            
            // Create array of new occupancy values
            int x = 100;
            std::vector<std::vector<int>> new_occupancy = {
                {0, 0, 0, 0, 0},
                {0, x, 0, 0, 0},
                {0, 0, x, 0, 0},
                {0, 0, 0, 0, 0},
                {0, 0, 0, 0, 0}
            };
            
            // Create array of zero visits
            std::vector<std::vector<int>> new_visits(5, std::vector<int>(5, 0));


            // Update global and local maps
            std::vector<utils::MapIndex> sources = global_map->updateMap(new_occupancy, new_visits);
            robot_map = Mapping::RobotLocalMap(global_map.get(), radius);

        }

};

/*
Test: BushFireInitializesCorrectly
Purpose: test proper initialization of RobotLocalMap from GlobalMapGrid containing obstacles
Expectation: nodal accessibility within local map matches distances obtained from hand calculations */
TEST_F(RobotLocalMapTests, LocalMapInitializesCorrectly) {

    double r2 = std::sqrt(2);
    double r5 = 1 + std::sqrt(2);
    double r8 = std::sqrt(8);
    
    std::vector<double> expected_nodal_distance = {
        r2, 1,  r2, r5, r8,
        1,  0,  1,  r2, r5,
        r2, 1,  0,  1,  2,
        r5, r2, 1,  r2, r5,
        r8, r5, 2,  r5, r8
    };

    for(int i = 0; i < 5; i++) {
        for(int j = 0; j < 5; j++) {

            expected_distance = expected_nodal_distance[5 * i + j] * global_map->getResolution();
            EXPECT_NEAR(robot_map.getNode(i, j), expected_distance , 1e-9);
        }
    }
}

/*
Test: LocalMapUpdatesCorrectly
Purpose: test proper update workflow of RobotLocalMap
Expectation: nodal accessibility within local map matches distances obtained from hand calculations */
TEST_F(RobotLocalMapTests, BushFireUpdatesCorrectly) {

    double r2 = std::sqrt(2);
    double r5 = 1 + std::sqrt(2);
    double r8 = std::sqrt(8);

    // Array of expected values
    std::vector<double> expected_nodal_distance = {
        r2, 1,  r2, 1,  0,
        1,  0,  1,  r2, 1,
        r2, 1,  0,  1,  r2,
        r5, r2, 1,  0,  1,
        r8, r5, r2, 1,  r2 
    };
    
    // Array of updated points
    std::vector<utils::MapIndex> updated_points = {utils::MapIndex(0,4), utils::MapIndex(3,3)};

    robot_map.bushFireUpdate(updated_points);

    for(int i = 0; i < 5; i++) {
        for(int j = 0; j < 5; j++) {

            expected_distance = expected_nodal_distance[5 * i + j] * global_map->getResolution();
            EXPECT_NEAR(robot_map.getNode(i, j), expected_distance , 1e-9);
        }
    }

}

/*
Test: CheckAccessWithinClearance
Purpose: validate isAccessible method
Expectation: returns false if distance to nearest obstacle is less than minimum clearance */
TEST_F(RobotLocalMapTests, CheckAccessWithClearance) {

    std::vector<bool> expected_result = {
        0, 0, 0, 1, 1,
        0, 0, 0, 0, 1,
        0, 0, 0, 0, 0,
        1, 0, 0, 0, 1,
        1, 1, 0, 1, 1
    };


    for(int i = 0; i < 5; i++) {
        for(int j = 0; j < 5; j++) {

            EXPECT_EQ(robot_map.isAccessible(i, j), expected_result[5 * i + j]) << "Error at (" << i << ", " << j << "): Real = " 
            << robot_map.getNode(i,j) << " Cutoff: " << robot_map.getMinClearance() << std::endl;
        }
    }
}

/*
Test: GetNearestAccessbileNode
Purpose: Test getNearestAccessibleNode for mapping to inaccessible frontiers
Expectation: Returns nearest accessible node within robot_map, determined via hand calculations */
TEST_F(RobotLocalMapTests, GetNearestAccessibleNode) {

    EXPECT_EQ(robot_map.getNearestAccessibleNode(utils::MapIndex(1, 0)), utils::MapIndex(3, 0));
    EXPECT_EQ(robot_map.getNearestAccessibleNode(utils::MapIndex(1, 2)), utils::MapIndex(0, 3));

}