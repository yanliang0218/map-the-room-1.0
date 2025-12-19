/***********************************************************************************************************************************************************
* Course: MECH 524
* Student Name: Connor McAllister
* Student Number: 65908436
* Date: October 29th, 2025
*
* Source File: path.h
*
* Purpose:
* Define Path<T> and declare related objects.
* 
* Description:
* This file contains an inline definition for Path<T> such that template
* functionality can be retained. Additionally, declarations are provided
* for derived members and related objects.
* 
*************************************************************************************************************************************************************/


/************************************************************************* INCLUDES *************************************************************************/
#pragma once
#include "grid.h"
#include "utils.h"

#include <vector>
#include <array>
#include <queue>
#include <ctime>


/************************************************************************** FUNCTIONS **************************************************************************/


    


/*********************************************************************** STRUCTURES **************************************************************************/

namespace Mapping {

    struct CompareSearchNodePtrs {
        bool operator()(const SearchNode* node1, const SearchNode* node2);
    };

/************************************************************************* CLASSES **************************************************************************/

/*--------------------------------------------------------------- PathInterpolaor class --------------------------------------------------------------------*/

    class PathInterpolator {
        protected:
            // Resolution of interpolated path
            double path_resolution;

            // Protected Methods
            std::vector<utils::MapPoint> joinPaths(const std::vector<std::vector<utils::MapPoint>>& paths);

        public:

            // Enum containing derived members
            enum class InterpolatorType {
                LINEAR
            };

            // Constructors
            PathInterpolator();
            PathInterpolator(double path_resolution);

            // Public Methods
            virtual std::vector<utils::MapPoint> interpolate(const std::vector<utils::MapPoint>& input_path) = 0;
    };

    /*
/*--------------------------------------------------------------- LinearInterpolator class --------------------------------------------------------------------*/

class LinearInterpolator : public PathInterpolator {

        private:

            // Private Methods
            std::vector<utils::MapPoint> linearInterpolate(utils::MapPoint start, utils::MapPoint end, bool inclusive = true);

        public:

            // Constructors
            LinearInterpolator();
            LinearInterpolator(double path_resolution);

            // Public Methods
            std::vector<utils::MapPoint> interpolate(const std::vector<utils::MapPoint>& input_path) override;
    };
    
    


    class DiscretePath {
        private:

            // Path coordinates
            std::vector<utils::MapIndex> path_coordinates;
            // Pointer to local map
            RobotLocalMap* local_map;
            // Pointer to global map
            GlobalMapGrid* global_map;
            // time of last update
            time_t time_stamp;

        public:
            
            // Constructors
            DiscretePath(GlobalMapGrid* global_map, RobotLocalMap* local_map);
            DiscretePath(GlobalMapGrid* global_map, RobotLocalMap* local_map, const std::vector<utils::MapIndex>& inputPath);

            // Getter Methods
            time_t getTimeStamp() const;

            // Public Methods
            std::vector<utils::MapIndex> getPathCoordinates();
            std::vector<utils::MapPoint> getRealCoordinates();
            void refinePath();
            void updatePath(const std::vector<utils::MapIndex>& new_path);
    };

    /*
    Class: RealPath
    Purpose: Represent a series or coordinates for robot traversal in real space*/
    class RealPath  {
        
        private:

            // Path coordinates
            std::vector<utils::MapPoint> path_coordinates;
            // Reference to DiscretePath instance from which this is derived
            DiscretePath* discrete;
            // interpolator used to covert discrete path to RealPath
            PathInterpolator* interpolator;
            // time of last sync with discrete path
            time_t time_stamp;

        public:

            // Constructors
            RealPath();
            RealPath(DiscretePath* discrete, PathInterpolator* interpolator);

            // Public Methods
            std::vector<utils::MapPoint> getPathCoordinates();
            void updatePath();
    };




 
/*---------------------------------------------------------------------- PathPlanner class ----------------------------------------------------------------------------
    Notes:
        When adding derived members, will require adding a field to 
        PlannerType enum to allow implementation in NavigationEnvironment
        class */
    class PathPlanner {
        
        protected:
            // Store metadata for running search algorithm
            SearchGrid search_grid;
            // Reference to local map, for getting accessibility data
            RobotLocalMap* local_map;
            // Reference to external global map, for getting visit count data
            GlobalMapGrid* global_map;

            // Public Methods
            std::vector<utils::MapIndex> pathFromEndNode(SearchNode* end_node, utils::MapIndex start);
            

        public:

            // Enum of derived classes
            enum class PlannerType {
                ASTAR
            };
            
            // Constructors
            PathPlanner(GlobalMapGrid* global_map, RobotLocalMap* local_map);

            // Public Methods
            virtual std::vector<utils::MapIndex> generatePath(utils::MapIndex start, utils::MapIndex end) = 0;
            void printClosedNodes();
            RobotLocalMap* getLocalMap() const;

    };

/*---------------------------------------------------------------------- AStarPlanner class ----------------------------------------------------------------------------*/

    class AStarPlanner : public PathPlanner {
        private:

            // Parameter indicating relative weight of number of visits path cost
            double visit_sensitivity;

        public:
            
            // Constructors
\
            AStarPlanner(GlobalMapGrid* global_map, RobotLocalMap* local_map, double visitSensitivity = 0);

            // Public Methods
            std::vector<utils::MapIndex> generatePath(utils::MapIndex start, utils::MapIndex end) override;
    };
}