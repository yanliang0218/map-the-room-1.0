/**********************************************************************************
Name: Liang Yan
Student Number: 33140351
Name: test.cpp
Purpose: contains all unit tests for dwa_local_planner functionalities
**********************************************************************************/

#include "pch.h"
#include <gtest/gtest.h>
#include <vector>
#include <algorithm> 
#include "dwa_local_planner_class_headers.h"
using namespace std;


/**************************************Google Test Main Function********************************************/
int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

/************************************************************************************************************/
/**************************************Dynamic Window Sampler Tests******************************************/
/************************************************************************************************************/
/*
Adapted from OpenAI ChatGPT
*/
class SamplerTest : public ::testing::Test
{
protected:
    /*
    Define kinematic constraints:
    {v_x_min = 0, v_x_max =0.65 (from Turtlebot2 spec),
    omega_min = -M_PI, omega_max = M_PI (both from Turtlebot 2 spec),
    a_x_max = 0.8 m/s^2, a_theta_max = 3.0 rad/s^2 (both reasonable values)}
    */
    DynamicWindowSampler::Limits limits = { 0.0, 0.65, -M_PI, M_PI, 0.8, 3.0 };

    //using unique_ptr to automatically delete old objects if a new one is intialized
    std::unique_ptr<DynamicWindowSampler> sampler = nullptr;
    std::unique_ptr<TrajectoryGenerator> traj_generator = nullptr;

    void SetUp() override
    {
		//instantiating new DynamicWindowSampler object with defined limits and 2*2 samples
        sampler = std::make_unique<DynamicWindowSampler>(limits, 2, 2);
    }

    void TearDown() override
    {
      
    }
};


//Helper function to check if a vector contains a specific (v, omega) pair
bool pair_equal(const std::pair<double, double> pair_ref, std::pair<double, double> pair_to_compare) 
{
    bool result = true;
    if (abs(pair_ref.first - pair_to_compare.first) > 1e-9 || abs(pair_ref.second - pair_to_compare.second) > 1e-9) 
    {
        result = false;
	}
    
    return result;
}


//Adapted from OpenAI ChatGPT for template of all unit tests
//Description: fundamenal test: ensure that the computed window is within the defined limits
TEST_F(SamplerTest, WindowWithinLimits)
{
    //current v_x = 0.3 m/s, omega = 0.0 rad/s, control cycle dt = 0.1 s
    DynamicWindowSampler:: Window window = sampler->compute_window(0.3, 0.0, 0.1);

    //computed window lower linear bound should be >= min linear velocity limit 
    EXPECT_GE(window.v_low, limits.v_x_min);
    // computed window upper linear bound should be <= max linear velocity limit
    EXPECT_LE(window.v_high, limits.v_x_max);

    //similarly for angualr velocity bounds
    EXPECT_GE(window.omega_low, limits.omega_min);
    EXPECT_LE(window.omega_high, limits.omega_max);
}

//Description: ensure that the computed window matches hand calculated values
TEST_F(SamplerTest, WindowMatchHandCalcValues)
{
    //current v_x = 0.3 m/s, omega = 0.0 rad/s, control cycle dt = 0.1 s
    DynamicWindowSampler::Window window = sampler->compute_window(0.3, 0.0, 0.1);

    //computed window lower linear bound should be >= min linear velocity limit 
    EXPECT_NEAR(window.v_low, 0.22, 1e-9);
    // computed window upper linear bound should be <= max linear velocity limit
    EXPECT_NEAR(window.v_high, 0.38, 1e-9);

    //similarly for angualr velocity bounds
    EXPECT_NEAR(window.omega_low, -0.3, 1e-9);
    EXPECT_NEAR(window.omega_high, 0.3, 1e-9);
}


//Description: ensure that the computed window matches hand calculated
//values at zero current velocity, i.e. robot starting from rest
TEST_F(SamplerTest, InitialWindowAtZeroVel)
{
    //current v_x = 0.3 m/s, omega = 0.0 rad/s, control cycle dt = 0.1 s
    DynamicWindowSampler::Window window = sampler->compute_window(0.0, 0.0, 0.1);

    //computed window lower linear bound should be >= min linear velocity limit 
    EXPECT_NEAR(window.v_low, 0.0, 1e-9);
    // computed window upper linear bound should be <= max linear velocity limit
    EXPECT_NEAR(window.v_high, 0.08, 1e-9);

    //similarly for angualr velocity bounds
    EXPECT_NEAR(window.omega_low, -0.3, 1e-9);
    EXPECT_NEAR(window.omega_high, 0.3, 1e-9);
}

//Adapted from OpenAI ChatGPT for method of testing if samples contain expected (v, omega) pairs
//Description: ensure that the computed samples match hand calculated values
TEST_F(SamplerTest, SamplesMatchHandCalcValues)
{

    //current v_x = 0.3 m/s, omega = 0.0 rad/s, control cycle dt = 0.1 s
    DynamicWindowSampler::Window window = sampler->compute_window(0.3, 0.0, 0.1);

    std::vector<std::pair<double, double>>samples = sampler->sampler(window);

    //computed window lower linear bound should be >= min linear velocity limit 
	for (int i = 0; i < samples.size(); i++) 
    {
        if (i == 0) 
        {
            EXPECT_TRUE(pair_equal(samples[i], std::make_pair(0.22, -0.3)));
        }
        else if (i == 1) 
        {
            EXPECT_TRUE(pair_equal(samples[i], std::make_pair(0.22, 0.3)));
        }
        else if (i == 2) {
            EXPECT_TRUE(pair_equal(samples[i], std::make_pair(0.38, -0.3)));
        }
        else if (i == 3) 
        {
            EXPECT_TRUE(pair_equal(samples[i], std::make_pair(0.38, 0.3)));
        }
    } 
}
/************************************************************************************************************/
/**************************************Trajectory Generator Tests********************************************/
/************************************************************************************************************/

class TrajGenTest : public ::testing::Test
{
protected:
    /*
    Define kinematic constraints:
    {v_x_min = 0, v_x_max =0.65 (from Turtlebot2 spec),
    omega_min = -M_PI, omega_max = M_PI (both from Turtlebot 2 spec),
    a_x_max = 0.8 m/s^2, a_theta_max = 3.0 rad/s^2 (both reasonable values)}
    */
    DynamicWindowSampler::Limits limits = { 0.0, 0.65, -M_PI, M_PI, 0.8, 3.0 };;

    //using unique_ptr to automatically delete old objects if a new one is intialized
    std::unique_ptr<DynamicWindowSampler> sampler = nullptr;

    std::unique_ptr<TrajectoryGenerator> traj_generator = nullptr;

    void SetUp() override
    { 
        //these are shared by all tests in this class, so put here

        //instantiating new DynamicWindowSampler object with defined limits and 3*3 samples
        sampler = std::make_unique<DynamicWindowSampler>(limits, 3, 3);

        //instantiating new TrajectoryGenerator object with a prediction horizon of 0.5s and dt_trajectory of 0.1s
        traj_generator = std::make_unique<TrajectoryGenerator>(0.5, 0.1);
    }

    void TearDown() override
    {
        
    }
};

//Description: ensure that the computed trajectories all start the same start pose
TEST_F(TrajGenTest, StartAtStartPoseTest)
{

    //current v_x = 0.3 m/s, omega = 0.0 rad/s, control cycle dt = 0.1 s
    DynamicWindowSampler::Window window = sampler->compute_window(0.0, 0.0, 0.1);

    std::vector<std::pair<double, double>>samples = sampler->sampler(window);

    for (const auto& sample : samples) 
    {
        //initial pose at origin facing along x axis
        TrajectoryGenerator::Pose start = { 0.0, 0.0, 0.0 };    

		//generate trahectory for the given (v, omega) pair
        std::vector<TrajectoryGenerator::Pose> traj = traj_generator->simulate_trajectory(start, sample.first, sample.second);

        //all poses in the trajectory should start at the origin
        {
            EXPECT_NEAR(traj[0].x, 0.0, 1e-9);
            EXPECT_NEAR(traj[0].y, 0.0, 1e-9);
        }
	}

}


//Description: ensure that the computed trajectories all stay at start pose under zero velocity
TEST_F(TrajGenTest, ZeroVelTest)
{
	//for zero linear velocity and zero angular velocity
    std::pair<double, double > sample_at_rest = std::make_pair(0.0, 0.0);

    //initial pose at origin facing along x axis
    TrajectoryGenerator::Pose start = { 0.0, 0.0, 0.0 };

    //generate trahectory for the given (v, omega) pair
    std::vector<TrajectoryGenerator::Pose> traj = traj_generator->simulate_trajectory(start, sample_at_rest.first, sample_at_rest.second);
    for (size_t i = 0; i < traj.size(); i++)
    //all poses in the trajectory should stay at start pose
    {
        EXPECT_NEAR(traj[i].x, 0.0, 1e-9);
        EXPECT_NEAR(traj[i].y, 0.0, 1e-9);
        EXPECT_NEAR(traj[i].theta, 0.0, 1e-9);
    }   
}

//Description: ensure that the computed trajectories with a zero angular velocity are straight lines along the x axis
TEST_F(TrajGenTest, StraightLineWhenOmegaIsZeroTest)
{
	//sample (v, omega) pair with non-zero linear velocity and zero angular velocity
	std::pair<double, double > sample_straight_line = std::make_pair(0.3, 0.0);

	//initial pose at origin facing along x axis
    TrajectoryGenerator::Pose start = { 0.0, 0.0, 0.0 };

	//generate trahectory for the given (v, omega) pair
    std::vector<TrajectoryGenerator::Pose> traj = traj_generator->simulate_trajectory(start, sample_straight_line.first, sample_straight_line.second);

	for (auto const& pose : traj)
    {
		//all poses in the trajectory should lie on the x axis and its direction should not change, 
        //hence y = 0 and theta = 0
        EXPECT_NEAR(pose.y, 0.0, 1e-9);
        EXPECT_NEAR(pose.theta, 0.0, 1e-9);
    }

}

//Description: ensure that the computed trajectories with a zero linear velocity are in-place rotations
TEST_F(TrajGenTest, WhenVIsNonZeroOnlyRotateTest)
{
    //sample (v, omega) pair with zero linear velocity and non-zero angular velocity
    std::pair<double, double > sample_in_place_rotate = std::make_pair(0.0, M_PI/4);

    //initial pose at origin facing along x axis
    TrajectoryGenerator::Pose start = { 0.0, 0.0, 0.0 };

    //generate trahectory for the given (v, omega) pair
    std::vector<TrajectoryGenerator::Pose> traj = traj_generator->simulate_trajectory(start, sample_in_place_rotate.first, sample_in_place_rotate.second);

    for (auto const& pose : traj)
    {
        //all poses in the trajectory should only be changing in theta 
        //hence x = 0 and y = 0
        EXPECT_NEAR(pose.x, 0.0, 1e-9);
        EXPECT_NEAR(pose.y, 0.0, 1e-9);
    }

}
/****************************************************************************
Adapted from OpenAI ChatGPT 
Description: ensure that the computed trajectories with non-zero linear velocity and zero angular velocity 
             are straight lines along the heading direction
****************************************************************************/
TEST_F(TrajGenTest, StraightLineDiagonalWhenThetaNonZero)
{
    TrajectoryGenerator::Pose start{ 0.0, 0.0, M_PI / 10};
    double v = 0.2, omega = 0.0;

    auto traj = traj_generator->simulate_trajectory(start, v, omega);

    for (size_t i = 0; i < traj.size(); i++)
    {
        double t = i * traj_generator->get_dt_trajectory();

        EXPECT_NEAR(traj[i].x, v * t * std::cos(M_PI / 10), 1e-6);
        EXPECT_NEAR(traj[i].y, v * t * std::sin(M_PI / 10), 1e-6);
		//DEBUG: std::cout << "Theta at step " << i << ": " << traj[i].theta << std::endl;
        EXPECT_NEAR(traj[i].theta, M_PI / 10, 1e-6);
    }
}


/*********************************************************************************
Assited by OpenAI ChatGPT 
Description: ensure that the computed trajectories with non-zero linear velocity and zero angular velocity 
             are straight lines along the heading direction
*********************************************************************************/
TEST_F(TrajGenTest, CurveFollowsUnicycleModel)
{
    TrajectoryGenerator::Pose start{ 0.0, 0.0, M_PI / 10 };
    double v = 0.2, omega = 0.1;

    auto traj = traj_generator->simulate_trajectory(start, v, omega);
    //DEBUG: traj_generator->print_traj(traj);

    for (size_t i = 0; i < traj.size(); i++)
    {

        double t = i * traj_generator->get_dt_trajectory();

        //starting from the start pose, calculat the x and y components of the linear velocity v
        double v_x_component = v * std::cos(start.theta);
        double v_y_component = v * std::sin(start.theta);

		//skip first iteration since robot is already at start pose
        if (i > 0) 
        {
            start.x += v_x_component * traj_generator->get_dt_trajectory();
            start.y += v_y_component * traj_generator->get_dt_trajectory();
            start.theta += omega * traj_generator->get_dt_trajectory();
        }

        EXPECT_NEAR(traj[i].x, start.x, 1e-6);
        EXPECT_NEAR(traj[i].y, start.y, 1e-6);
        EXPECT_NEAR(traj[i].theta, start.theta, 1e-6);

    }
}

/************************************************************************************************************/
/**************************************Obstacle Evaluator Tests**********************************************/
/************************************************************************************************************/

class ObsEvalTests : public ::testing::Test
{
protected:
    /*
    Define kinematic constraints:
    {v_x_min = 0, v_x_max =0.65 (from Turtlebot2 spec),
    omega_min = -M_PI, omega_max = M_PI (both from Turtlebot 2 spec),
    a_x_max = 0.8 m/s^2, a_theta_max = 3.0 rad/s^2 (both reasonable values)}
    */
    DynamicWindowSampler::Limits limits = limits = { 0.0, 0.65, -M_PI, M_PI, 0.8, 3.0 }; ;

    //using unique_ptr to automatically delete old object when a new one is intialized
    std::unique_ptr<DynamicWindowSampler> sampler = nullptr;
    std::unique_ptr<TrajectoryGenerator> traj_generator = nullptr;
    std::unique_ptr<ObstacleEvaluator> obs_evaluator = nullptr;

    void SetUp() override
    {
        //these are shared by all tests in this test class, so put here
        
        //instantiating new DynamicWindowSampler object with defined limits and 3*3 samples
        sampler = std::make_unique<DynamicWindowSampler>(limits, 3, 3);

        //instantiating new TrajectoryGenerator object with a prediction horizon of 2s and dt_trajectory of 0.1s
        traj_generator = std::make_unique<TrajectoryGenerator>(2.0, 0.1);

    }
    void TearDown() override
    {

    }
};

/**********************************************************************************
Assited by OpenAI ChatGPT 
Description: ensure that in a 5*5 occupancy grid with all free cells, 
             all trajectories generated from all (v, omega) give minimum clearance 
             as max double - robot radius; except for those that go out of bounds
             which should give clearance of 0.0
**********************************************************************************/
TEST_F(ObsEvalTests, AllOpenCells)
{
	//take a 5*5 occupancy grid with all free cells
    std::vector<std::vector<int>>occupancy_grid = {
        {0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0}
    };

    //instantiating new ObstacleEvaluator object with occupancy grid, resolution of 0.5m/cell, robot radius of 0.1m
    obs_evaluator = std::make_unique<ObstacleEvaluator>(0.5, 0.1);
    obs_evaluator->set_grid(occupancy_grid);

	//initial pose at origin facing along x axis
    TrajectoryGenerator::Pose start{0.0, 0.0, 0.0 };

    //current v_x = 0.0 m/s, omega = 0.0 rad/s, control cycle dt = 0.1 s, compute dynamic window
    DynamicWindowSampler::Window window = sampler->compute_window(0.0, 0.0, 0.05);

	//take all (v, omega) samples from the computed window
    std::vector<std::pair<double, double>>samples = sampler->sampler(window);
    
	//for every (v, omega) sample
    for (const auto& sample : samples)
    {
		//initialize out_of_bounds flag i.e default to that 
		//all poses on the trajectory generated by this sample 
        //are within bounds of the occupancy grid
        bool out_of_bounds = false;

        //generate trahectory for the given (v, omega) pair
        std::vector<TrajectoryGenerator::Pose> traj = traj_generator->simulate_trajectory(start, sample.first, sample.second);

        //iterating through each pose on a given trajectory
        for (const auto& pose : traj) 
        {
            //check if any pose in the trajectory is out of bounds of the occupancy grid
            if (pose.x < 0 || pose.x >= obs_evaluator->get_num_cols() * obs_evaluator->get_resolution() ||
                pose.y < 0 || pose.y >= obs_evaluator->get_num_rows() * obs_evaluator->get_resolution())
            {
				//if any pose is out of bounds, set flag for this trajectory to true and break
                out_of_bounds = true;
                break;
            }
		}

        if (out_of_bounds) 
        {
            //if any pose is out of bounds, expect clearance to be 0.0
			//DEBUG:cout << "Trajectory goes out of bounds for sample (" << sample.first << "," << sample.second << ")" << endl;
            EXPECT_NEAR(0.0, obs_evaluator->compute_clearance(traj), 1e-9);
        }
        else 
        {
			//cout << "Trajectory within bounds for sample (" << sample.first << "," << sample.second << ")" << endl;
            //DEBUG:if all poses are within bounds, expect clearance to be large value (max double - robot radius)
            constexpr double expected_clearance = std::numeric_limits<double>::max();
            EXPECT_NEAR(expected_clearance, obs_evaluator->compute_clearance(traj), obs_evaluator->get_radius_robot());
		}
        
    }
}

/**********************************************************************************
Description: ensure that in a 5*5 occupancy grid with all occupied cells,
             all trajectories generated from all (v, omega) give minimum clearance of 0.0
**********************************************************************************/
TEST_F(ObsEvalTests, AllOccupiedOrUnknownCells)
{
    //take a 5*5 occupancy grid with all occupied cells
    std::vector<std::vector<int>>occupancy_grid = {
        {100, 100, 100, 100, 100},
        {100, 100, 100, 100, 100},
        {100, 100, 100, 100, 100},
        {100, 100, 100, 100, 100},
        {100, 100, 100, 100, 100}
    };

    //instantiating new ObstacleEvaluator object with occupancy grid, resolution of 0.5 m/cell, robot radius of 0.01 m
    obs_evaluator = std::make_unique<ObstacleEvaluator>(0.5, 0.01);
    obs_evaluator->set_grid(occupancy_grid);

    //initial pose at origin facing along x axis
    TrajectoryGenerator::Pose start{ 0.0, 0.0, 0.0 };

    //current v_x = 0.0 m/s, omega = 0.0 rad/s, control cycle dt = 0.1 s, compute dynamic window
    DynamicWindowSampler::Window window = sampler->compute_window(0.0, 0.0, 0.05);

    //take all (v, omega) samples from the computed window
    std::vector<std::pair<double, double>>samples = sampler->sampler(window);

    //for every (v, omega) sample
    for (const auto& sample : samples)
    {
        //generate trahectory for the given (v, omega) pair
        std::vector<TrajectoryGenerator::Pose> traj = traj_generator->simulate_trajectory(start, sample.first, sample.second);
		//expect clearance to be 0.0 since all cells are occupied
        EXPECT_NEAR(0.0, obs_evaluator->compute_clearance(traj), 1e-9);
    }

    //now repeat for all unknown cells
    occupancy_grid = {
    {-1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1}
    };

    //for every (v, omega) sample
    for (const auto& sample : samples)
    {
        //generate trahectory for the given (v, omega) pair
        std::vector<TrajectoryGenerator::Pose> traj = traj_generator->simulate_trajectory(start, sample.first, sample.second);
        //expect clearance to be 0.0 since all cells are occupied
        EXPECT_NEAR(0.0, obs_evaluator->compute_clearance(traj), 1e-9);
    }

}

/**********************************************************************************
Description: ensure that in a 5*5 occupancy grid with the starting pose occupied
             all trajectories generated from all (v, omega) give minimum clearance of 0.0
**********************************************************************************/
TEST_F(ObsEvalTests, InitialPoseIsOccupied)
{
    //take a 5*5 occupancy grid with all free cells
    std::vector<std::vector<int>>occupancy_grid = {
        {-1, -1, -1, -1, -1},
        {-1, -1, -1, -1, -1},
        {0, -1, -1, -1, -1},
        {0, 0, -1, -1, -1},
        {100, 0, 0, -1, -1}
    };

    //instantiating new ObstacleEvaluator object with occupancy grid, resolution of 0.5m/cell, robot radius of 0.01m
    obs_evaluator = std::make_unique<ObstacleEvaluator>(0.5, 0.01);
    obs_evaluator->set_grid(occupancy_grid);



    //initial pose at origin facing along x axis
    TrajectoryGenerator::Pose start{ 0.0, 0.0, 0.0 };

    //current v_x = 0.0 m/s, omega = 0.0 rad/s, control cycle dt = 0.1 s, compute dynamic window
    DynamicWindowSampler::Window window = sampler->compute_window(0.0, 0.0, 0.05);

    //take all (v, omega) samples from the computed window
    std::vector<std::pair<double, double>>samples = sampler->sampler(window);

    //for every (v, omega) sample
    for (const auto& sample : samples)
    {
        //generate trahectory for the given (v, omega) pair
        std::vector<TrajectoryGenerator::Pose> traj = traj_generator->simulate_trajectory(start, sample.first, sample.second);
        //expect clearance to be 0.0 since all cells are occupied
        EXPECT_NEAR(0.0, obs_evaluator->compute_clearance(traj), 1e-9);
    }
}

//Asissted from OpenAI ChatGPT
//Description: ensure the correct propogation of clearance values from a single obstacle in the occupancy grid
TEST_F(ObsEvalTests, SingleObstacleCorrectClearanceGrid)
{
    std::vector<std::vector<int>>occupancy_grid = {
        {0,0,0},
        {0,100,0},
        {0,0,0}
    };

	//clearly the clearance grid should look like:  where 1.41 represents sqrt(2)
    // 1.41  1    1.41   
    // 1     0    1      
    // 1.41  1    1.41  
   
   //instantiating new ObstacleEvaluator object with occupancy grid, resolution of 1.0m/cell, robot radius of ~0.0m
    obs_evaluator = std::make_unique<ObstacleEvaluator>(1.0, 1e-20);
    obs_evaluator->set_grid(occupancy_grid);


    auto clearance_grid_to_test = obs_evaluator->get_clearance_grid();

	for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++) 
        {
            if (i==1 && j==1) 
            {
                EXPECT_NEAR(clearance_grid_to_test[i][j], 0.0, 1e-5);
            } 
			else if ( i == 1 || j == 1 )
            {
                EXPECT_NEAR(clearance_grid_to_test[i][j], 1.0, 1e-5);
			}
            else
            {
                EXPECT_NEAR(clearance_grid_to_test[i][j], std::sqrt(2), 1e-5);
            }
            
        }

    }

}
//Description: ensure the correct propogation of clearance values from three obstacles in the occupancy grid
TEST_F(ObsEvalTests, ThreeObstacleCorrectClearanceGrid)
{
    std::vector<std::vector<int>>occupancy_grid = {
        {100,0,0},
        {0,100,0},
        {0, 0, 100}
    };

    //clearly the clearance grid should look like:  where 1.41 represents sqrt(2)
    // 0     1    1.41   
    // 1     0    1      
    // 1.41  1    0 

   //instantiating new ObstacleEvaluator object with occupancy grid, resolution of 1.0m/cell, robot radius of ~0.0m
    obs_evaluator = std::make_unique<ObstacleEvaluator>(1.0, 1e-20);
    obs_evaluator->set_grid(occupancy_grid);

    auto clearance_grid_to_test = obs_evaluator->get_clearance_grid();

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            if (i == 1 && j == 1)
            {
                EXPECT_NEAR(clearance_grid_to_test[i][j], 0.0, 1e-5);
            }
            else if (i == 1 || j == 1)
            {
                EXPECT_NEAR(clearance_grid_to_test[i][j], 1.0, 1e-5);
            }
            else 
            {   
                if ( (i == 0 && j ==0) || (i ==2 && j ==2) )
                {
                    EXPECT_NEAR(clearance_grid_to_test[i][j], 0.0, 1e-5);
                }
                else 
                {
                    EXPECT_NEAR(clearance_grid_to_test[i][j], std::sqrt(2), 1e-5);
                }
            }
        }
    }
}


/***************************************************************************
//Description: ensure the correct propogation of clearance values whilw considering effect of the robot's radius
               from a single obstacle in the occupancy grid
***************************************************************************/
TEST_F(ObsEvalTests, SingleObstacleRadiusEffect)
{
    std::vector<std::vector<int>>occupancy_grid = {
        {0,0,0},
        {0,100,0},
        {0,0,0}
    };

    //clearly the clearance grid should look like:  where 1.41 represents sqrt(2)
    // 1.41-0.1  1-0.1    1.41-0.1  
    // 1-0.1     0        1-0.1      
    // 1.41-0,1  1-0.1    1.41-0.1

   //instantiating new ObstacleEvaluator object with occupancy grid, resolution of 1.0 m/cell, robot radius of 0.1 m
    obs_evaluator = std::make_unique<ObstacleEvaluator>(1.0, 0.1);
    obs_evaluator->set_grid(occupancy_grid);

    auto clearance_grid_to_test = obs_evaluator->get_clearance_grid();

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            if (i == 1 && j == 1)
            {
                EXPECT_NEAR(clearance_grid_to_test[i][j], 0.0, 1e-5);
            }
            else if (i == 1 || j == 1)
            {
                EXPECT_NEAR(clearance_grid_to_test[i][j], 1.0 - obs_evaluator->get_radius_robot(), 1e-5);
            }
            else
            {
                EXPECT_NEAR(clearance_grid_to_test[i][j], std::sqrt(2) - obs_evaluator->get_radius_robot(), 1e-5);
            }

        }

    }

}

/***************************************************************************
//Description: ensure the correct computation of minimum clearance along a straight trajectory
			   towards a single obstacle in the occupancy grid
***************************************************************************/
TEST_F(ObsEvalTests, StraightTrajCorrectMinClearance)
{
	//occupancy grid with a single obstacle at (2,1) m
    std::vector<std::vector<int>>occupancy_grid = {
        {0,  0,    0,   -1},
		{0,  100,  0,   -1},
        {0,  0,    0,   -1},
        {0,  0,    0,    0}
    };

   //instantiating new ObstacleEvaluator object with occupancy grid, resolution of 1 m/cell, robot radius of 0.1 m
    obs_evaluator = std::make_unique<ObstacleEvaluator>(1.0, 0.1);
    obs_evaluator->set_grid(occupancy_grid);

    //initial pose at origin facing along x axis
    TrajectoryGenerator::Pose start{ 0.0, 0.0, 0.0 };

	//with a linear velocity of 1.0 m/s and zero angular velocity, the robot should move straight along x axis
    std::pair<double, double> sample = {1.0, 0.0};   
    
    //generate trahectory for the given (v, omega) pair
    std::vector<TrajectoryGenerator::Pose> traj = traj_generator->simulate_trajectory(start, sample.first, sample.second);

	//since the prediction horizon is 2s, the robot should move to x = 2.0 m at the end of the trajectory
	//therefore, the minimum clearance along the trajectory should be 1.0 m - robot radius to the obstacle at (2,1)m
    EXPECT_NEAR(1.0 - obs_evaluator->get_radius_robot(), obs_evaluator->compute_clearance(traj), 1e-9);
}


/***************************************************************************
//Description: ensure the correct computation of minimum clearance along a set curved trajectory
			   in an occupancy grid with obstacles and unknown cells blocking one side of the start
               pose -> in this case, the min_clearance should be 0.0 since one of the trajectories
               will eventually collide with an obstacle
***************************************************************************/
TEST_F(ObsEvalTests, CurvedTrajCorrectMinClearance)
{
    std::vector<std::vector<int>>occupancy_grid =
    {
        {0,    -1,   0,    0},
        {0,    0,    100,  0},
        {0,    0,    0,    0},
        {0,    100,  0,   100}
    };

    //instantiating new ObstacleEvaluator object with occupancy grid, resolution of 1 m/cell, robot radius of 0.1 m
    obs_evaluator = std::make_unique<ObstacleEvaluator>(1.0, 0.1);
    obs_evaluator->set_grid(occupancy_grid);

    //initial pose at origin facing along x axis
    TrajectoryGenerator::Pose start{ 0.0, 0.0, 0.0 };

    //with a linear velocity of 1.0 m/s and zero angular velocity, the robot should move straight along x axis
    std::pair<double, double> sample = { 1.0, 0.0 };

    //current v_x = 0.0 m/s, omega = 0.0 rad/s, control cycle dt = 0.1 s, compute dynamic window
    DynamicWindowSampler::Window window = sampler->compute_window(0.0, 0.0, 0.05);

    //take all (v, omega) samples from the computed window
    std::vector<std::pair<double, double>>samples = sampler->sampler(window);

	double min_clearance_of_all_trajectories = std::numeric_limits<double>::max();
	for (const auto& sample : samples)
    {
        //generate trahectory for the given (v, omega) pair
        std::vector<TrajectoryGenerator::Pose> traj = traj_generator->simulate_trajectory(start, sample.first, sample.second);

        //since the prediction horizon is 2s, the robot should move to x = 2.0 m at the end of the trajectory
        //therefore, the minimum clearance along the trajectory should be 1.0 m - robot radius to the obstacle at (2,1)m
		if (obs_evaluator->compute_clearance(traj) < min_clearance_of_all_trajectories)
        {
            min_clearance_of_all_trajectories = obs_evaluator->compute_clearance(traj);
        }
    }

    EXPECT_NEAR(0.0,min_clearance_of_all_trajectories, 1e-9);
}


/************************************************************************************************************/
/**************************************Path Tracker Tests****************************************************/
/************************************************************************************************************/

class PathTrackerTests : public ::testing::Test
{
protected:
    /*
    Define kinematic constraints:
    {v_x_min = 0, v_x_max = 0.65 (from Turtlebot2 spec),
    omega_min = -M_PI, omega_max = M_PI (both from Turtlebot 2 spec),
    a_x_max = 0.8 m/s^2, a_theta_max = 3.0 rad/s^2 (both reasonable values)}
    */
    DynamicWindowSampler::Limits limits = { 0.0, 0.65, -M_PI, M_PI, 0.8, 3.0 };

    //using unique_ptr to automatically delete old objects when a new one is intialized
    std::unique_ptr<TrajectoryGenerator> traj_generator = nullptr;
    std::unique_ptr<ObstacleEvaluator> obs_evaluator = nullptr;
    std::unique_ptr<PathTracker> path_tracker = nullptr;

    void SetUp() override
    {
        //instantiating new TrajectoryGenerator object with a prediction horizon of 2s and dt_trajectory of 0.1s
        //this is shared by all tests in this test class, thus put here
        traj_generator = std::make_unique<TrajectoryGenerator>(2.0, 0.1);
    }

    void TearDown() override
    {

    }
};

/**********************************************************************************
Description: ensure that the dynamically computed lookahead is correct
**********************************************************************************/
TEST_F(PathTrackerTests, CorrectLookaheadSetter)
{
    //result does not depend on occupancy grid, so giving a placeholder value
    std::vector<std::vector<int>> occupancy_grid = { {0, 0} };

    //result does not depend on the path either, so giving a placeholder value
    std::vector<MapPoint> path = { {0, 0} };

    //instantiating obstacle evaluator with resolution of 1m/cell, and a robot radius of 0.25m
    obs_evaluator = std::make_unique<ObstacleEvaluator>(1.0, 0.25);
    obs_evaluator->set_grid(occupancy_grid);

    
    //instantiating a path tracker with above parameters, with a k_multiplier of 3.0，and v_x_curr = 0.6
    path_tracker = std::make_unique<PathTracker>(3.0);
    path_tracker->set_lookahead(obs_evaluator->get_resolution(), path_tracker->get_k_multiplier_lookahead(),0.6, limits.v_x_max, traj_generator->get_horizon(), obs_evaluator->get_radius_robot());
    path_tracker->set_path(path);



    /*****************************************************************************
    min_lookahead = max(resolution, 2.0 * robot_radius) = max(1.0, 2.0 * 0.25) = 1.0 m
    max_lookahead = horizon * v_max = 2.0*0.65 = 1.3 m
    default_lookahead = 3 * std::abs(0.6) = 1.8 m
    this->lookahead = clamp(1.8, 1.0,1.3) = 1.3 m
    *****************************************************************************/
    EXPECT_NEAR(1.3, path_tracker->get_lookahead(), 1e-6);

    //instantiating a path tracker with above parameters, with a k_multiplier of 2.0，and v_x_curr = 0.6
    path_tracker = std::make_unique<PathTracker>(2.0);
    path_tracker->set_lookahead(obs_evaluator->get_resolution(), path_tracker->get_k_multiplier_lookahead(), 0.6, limits.v_x_max, traj_generator->get_horizon(), obs_evaluator->get_radius_robot());
    path_tracker->set_path(path);
    //this->lookahead = clamp(1.2, 1.0, 1.3) = 1.2 m
    EXPECT_NEAR(1.2, path_tracker->get_lookahead(), 1e-6);

    //instantiating a path tracker with above parameters, with a k_multiplier of 0.5，and v_x_curr = 0.6
    path_tracker = std::make_unique<PathTracker>(0.5);
    path_tracker->set_lookahead(obs_evaluator->get_resolution(), path_tracker->get_k_multiplier_lookahead(), 0.6, limits.v_x_max, traj_generator->get_horizon(), obs_evaluator->get_radius_robot());
    path_tracker->set_path(path);
    //this->lookahead = clamp(0.3, 1.0, 1.3) = 1 m
    EXPECT_NEAR(1.0, path_tracker->get_lookahead(), 1e-6);
}


/**********************************************************************************
Description: ensure that for a predefined current pose, current v_x, and real path, 
             the path_tracker can find the correct target, with which the DWA local 
             planner later calculates the heading error
**********************************************************************************/
TEST_F(PathTrackerTests, CorrectTarget)
{
    //result does not depend on occupancy grid, so giving a placeholder value
    std::vector<std::vector<int>>occupancy_grid = { {0, 0} };
   
    //a real path
    std::vector<MapPoint> path = {
        {1.0, 1.0},
        {2.0, 2.0},
        {3.0, 3.0},
        {4.0, 4.0},
	};
   
    //instantiating obstacle evaluator with resolution of 1m/cell, and a robot radius of 0.25m
    obs_evaluator = std::make_unique<ObstacleEvaluator>(1.0, 0.25);
    obs_evaluator->set_grid(occupancy_grid);

	//instantiating a path tracker with above parameters, with a k_multiplier of 1.0 and a current v_x of 0.0
    path_tracker = std::make_unique<PathTracker>(1.0);
    path_tracker->set_lookahead(obs_evaluator->get_resolution(), path_tracker->get_k_multiplier_lookahead(), 0.0, limits.v_x_max, traj_generator->get_horizon(), obs_evaluator->get_radius_robot());
    path_tracker->set_path(path);

	//DEBUG:cout << "lookahead= " << path_tracker->get_lookahead() << endl;

	TrajectoryGenerator::Pose pose = { 0.0, 0.0, 0.0 };

    MapPoint target = path_tracker->find_target_on_path(pose);
    
    //DEBUG:cout<<"target point is "<<target.x<<","<<target.y<<endl;

    EXPECT_NEAR(2.0, target.x, 1e-6);
    EXPECT_NEAR(2.0, target.y, 1e-6);
}

/************************************************************************************************************/
/**************************************Trajectory Evaluator Tests********************************************/
/************************************************************************************************************/
class TrajectoryEvaluatorTests : public ::testing::Test
{
protected:
    /*
    Define kinematic constraints:
    {v_x_min = 0, v_x_max = 0.65 (from Turtlebot2 spec),
    omega_min = -M_PI, omega_max = M_PI (both from Turtlebot 2 spec),
    a_x_max = 0.8 m/s^2, a_theta_max = 3.0 rad/s^2 (both reasonable values)}
    */
    DynamicWindowSampler::Limits limits = { 0.0, 0.65, -M_PI, M_PI, 0.8, 3.0 };

    TrajectoryEvaluator::Weights weights = {0.8,0.1,0.1};


    //using unique_ptr to automatically delete old objects when a new one is intialized
    std::unique_ptr<DynamicWindowSampler> sampler = nullptr;
    std::unique_ptr<TrajectoryGenerator> traj_generator = nullptr;
    std::unique_ptr<ObstacleEvaluator> obs_evaluator = nullptr;
    std::unique_ptr<PathTracker> path_tracker = nullptr;
    std::unique_ptr<TrajectoryEvaluator> traj_eval = nullptr;

    void SetUp() override
    {
        //instantiating new DynamicWindowSampler object with defined limits and 3*3 samples
        sampler = std::make_unique<DynamicWindowSampler>(limits, 3, 3);

        //instantiating new TrajectoryGenerator object with a prediction horizon of 2s and dt_trajectory of 0.1s
        traj_generator = std::make_unique<TrajectoryGenerator>(2.0, 0.1);

        traj_eval = std::make_unique<TrajectoryEvaluator>(weights, limits.a_x_max);
    }

    void TearDown() override
    {

    }
};


/**********************************************************************************
Assited by OpenAI ChatGPT
Description: ensure that the that the scores for a simple hand-calculable case are correct
**********************************************************************************/
TEST_F(TrajectoryEvaluatorTests, HandCalculableCase)
{
    // Simple clearance grid
    std::vector<std::vector<int>> occupancy_grid =
    {
        {0,  100,0},
        {0,  0,  0},
        {0,  0,  -1}
    };

    // Simple path
    std::vector<MapPoint> path = {
        {0,0},
        {1,1}
    };

    TrajectoryGenerator::Pose start = { 0,0,0 };

	std::pair<double, double> v_omega_curr = { 0.0,0.0 };

    // a trajectory pf only the start pose
    auto traj = traj_generator->simulate_trajectory(start, v_omega_curr.first, v_omega_curr.second);

	double robot_radius = 0.1;
	double resolution = 1.0;

    obs_evaluator = std::make_unique<ObstacleEvaluator>(resolution, robot_radius);
    obs_evaluator->set_grid(occupancy_grid);

    double k_multiplier_lookahead = 1.0;
    
    // simple lookahead config
    path_tracker = std::make_unique<PathTracker>(k_multiplier_lookahead);
    path_tracker->set_lookahead(obs_evaluator->get_resolution(), path_tracker->get_k_multiplier_lookahead(), v_omega_curr.first, limits.v_x_max, traj_generator->get_horizon(), obs_evaluator->get_radius_robot());
    path_tracker->set_path(path);

    /*****************************************************************************
    min_lookahead = max(resolution, 2.0 * robot_radius) = max(1.0, 2.0 * 0.1) = 1.0 m
    max_lookahead = horizon * v_max = 2.0*0.65 = 1.3 m
    default_lookahead = 1 * std::abs(0.5) = 0.5 m
    this->lookahead = clamp(0.5, 1.0,1.3) = 1.0 m
    *****************************************************************************/

    MapPoint target = path_tracker->find_target_on_path(start);

    traj_eval = std::make_unique<TrajectoryEvaluator>(weights, limits.a_x_max);

    double longest_dim = std::sqrt(2)*obs_evaluator->get_num_rows(); // arbitrary but easy

    auto scores = traj_eval->evaluate(
        traj, target, v_omega_curr.first, limits.v_x_max, longest_dim, *obs_evaluator);
   
    //Expected Results:

	// clearance = 1-0.1 → normalized = (1-0.1) / (sqrt(2)*3)
    EXPECT_NEAR(scores.clearance_score, 0.9 / (std::sqrt(2) * 3), 1e-6);

    // heading = 45 deg off
    EXPECT_NEAR(scores.heading_score, 0.75, 1e-6);

    // velocity score = 0
    EXPECT_NEAR(scores.velocity_score, 0.0, 1e-6);

    // admissibility true (no collision, braking OK)
    EXPECT_TRUE(scores.admissibility);

    // total = weighted sum = 0.8*(1-0.1)/ longest_dim + 0.1*0.75 + 0.1*0 = 0.8 + 0.01 = 0.81
    EXPECT_NEAR(scores.total_score, 0.8 * 0.9/(std::sqrt(2)*3) + 0.1 * 0.75, 1e-6);
}


/**********************************************************************************
Assited by OpenAI ChatGPT
Description: a known case where the lowest scoring trajectory must be -1 (automatic rejection)
**********************************************************************************/
TEST_F(TrajectoryEvaluatorTests, LowestScoreHasToBeNegOne)
{
    // Simple clearance grid
    std::vector<std::vector<int>> occupancy_grid =
    {
        {0,  100,0},
        {0,  0,  0},
        {0,  0,  -1}
    };

    // Simple path
    std::vector<MapPoint> path = {
        {0,0},
        {1,1}
    };

    TrajectoryGenerator::Pose start = {0.0,0.0,0.0};

    std::pair<double, double> v_omega_curr = {0.0,0.0};

    DynamicWindowSampler::Window window = sampler->compute_window(v_omega_curr.first, v_omega_curr.second,0.1);
    std::vector<std::pair<double, double>> samples = sampler->sampler(window);

    //instantiating obstacle evaluator with resolution of 1m/cell, and a robot radius of 0.25m
    obs_evaluator = std::make_unique<ObstacleEvaluator>(1.0, 0.25);
    obs_evaluator->set_grid(occupancy_grid);

    //instantiating a path tracker with above parameters, with a k_multiplier of 1.0，and v_x_curr = 0.0
    path_tracker = std::make_unique<PathTracker>(1.0);
    path_tracker->set_lookahead(obs_evaluator->get_resolution(), path_tracker->get_k_multiplier_lookahead(), v_omega_curr.first, limits.v_x_max, traj_generator->get_horizon(), obs_evaluator->get_radius_robot());
    path_tracker->set_path(path);
 

    MapPoint target = path_tracker->find_target_on_path(start);

	double min_score = std::numeric_limits<double>::max();
    for (auto sample : samples)
    {
        std::vector<TrajectoryGenerator::Pose>traj = traj_generator->simulate_trajectory(start, sample.first, sample.second);
        TrajectoryEvaluator::TrajectoryScores scores = traj_eval->evaluate(traj,
            target, sample.first, limits.v_x_max, obs_evaluator->get_num_rows() * sqrt(2), *obs_evaluator);

		//if the robot keeps heading straight, it will eventually collide with the obstacle at (1,1)
        //so the lowest score of all trajectories must be -1 -> automatic rejection
        if (scores.total_score < min_score)
        {
            min_score = scores.total_score;
		}
        //DEBUG:traj_eval->print_trajectory_scores(scores);
        //DEBUG:cout<<endl;
    }
    EXPECT_NEAR(min_score,-1,1e-9);
}
