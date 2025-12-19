/***********************************************************************************************************************************************************
* Course: MECH 524
* Student Name: Connor McAllister
* Student Number: 65908436
* Date:
*
* Source File: grid.h
*
* Purpose:
* Define Grid<NodeType> class and declare derived members
* 
* Description:
* This file contains an inline definition of Grid<NodeType>, such that template
* functionality can be retained. This class exists to provide an abstract structure
* for derived members that represent a 2D discrete map within a unique scope.
* 
*************************************************************************************************************************************************************/


/************************************************************************* INCLUDES *************************************************************************/
#pragma once
#include "utils.h"
#include <vector>
#include <memory>
#include <stdexcept>
#include <cmath>
#include <string>
#include <algorithm>
#include <queue>
// #include "nav_msgs/OccupancyGrid.h"
// #include "ros/ros.h"

using namespace std;

/************************************************************************* CLASSES **************************************************************************/

/*
Namespace: mapping
Purpose: 
    - Contains classes pertaining to various nodes and mapping grids. 
    - Distinguishes between ROS node and Grid node types */
namespace Mapping {


    /*-------------------------------------------------------------------- GlobalMapNode CLASS --------------------------------------------------------------------
    Purpose: 
        Represent a Node in a GlobalMapGrid, containing parameters to represent 
        occupancy and number of visits */
    class GlobalMapNode {

        private:
            // Number of times this has been visited
            int num_visits;
            // Occupancy determined by map_the_room node (-1 = unknown, 100 = occupied, 0 = free)
            int occupancy;

        public:

            // enum indicating occupancy states
            enum Occupancy {
                OCCUPIED = 100,
                FREE = 0,
                UNKNOWN = -1
            };

            // Constructors
            GlobalMapNode();

            // Getter Methods
            int getOccupancy() const;
            int getVisits() const;
            bool isOccupied() const;
            bool isUnknown() const;
            bool isFree() const;

            // Setter methods
            void setOccupancy(int newOccupancy);
            void setVisits(int newVisits);
            

            // Public methods
            void reset();
    };

    /*-------------------------------------------------------------------- SearchNode Class --------------------------------------------------------------------
    Purpose: 
        Represent a discrete Node in a SearchGrid, containing parameters 
        commonly seen in searching algorithms */

    class SearchNode {

        private:
            // x and y coordintes within grid
            int row, col;
            // Cost of traversing current planned path up to this
            double gCost;
            // Heuristic estimate
            double hCost;
            // Node most recently traversed in path plan
            SearchNode* parent;
            // Indication of whether node can no longer be evaluated
            bool closed;

            
            
            

        public:

            // Constructors
            SearchNode();
            SearchNode(int row, int col);

            // Getter Methods
            int getRow() const;
            int getCol() const;
            double getGCost() const;
            double getHCost() const;           
            double getTotalCost() const;
            bool hasParent() const;
            SearchNode* getParent() const;
            bool isClosed() const ;
            
            // Setter Methods
            void setParent(SearchNode* newParent);
            void setHCost(double newHCost);
            void setGCost(double newGCost);

            // Public Methods
            void reset();
            void closeNode();
            std::string stringIndices() const;
        
            

    };

    /*-------------------------------------------------------------------- Grid Class --------------------------------------------------------------------
    Purpose: provide base class for discrete grids used to represent 2D space 
    Notes: Defined inline to allow template implementation */
    template <typename node_type>
    class Grid {
        
        protected:
            // 2D vector of grid Nodes
            std::vector<std::vector<node_type>> node_grid;
            // number of rows and columns within grid
            int num_rows, num_cols;

        public:

            /*
            Function: Grid default constructor
            Arguments: none
            Behaviour: initializes empty grid */
            Grid() {

                this->num_rows = 0;
                this->num_cols = 0;
            }

            /*
            Function: Grid Constructor
            Arguments: integer values for number of rows and columns
            Behaviour: initializes num_rows and num_cols in this */
            Grid(int num_rows, int num_cols) {

                if(num_rows <= 0 || num_cols <= 0) {
                    throw std::invalid_argument("Grid must have row and column counts greater than zero");
                }


                for(int i = 0; i < num_rows; i++) {
                    node_grid.push_back({});
                    for(int j = 0; j < num_cols; j++) {
                        node_grid[i].push_back(node_type());
                    }
                }

                this->num_rows = num_rows;
                this->num_cols = num_cols;
            }

            /*
            Function: getRows
            Arguments: none
            Returns: integer number of rows in this */
            int getRows() const {
                return num_rows;
            }

            /*
            Function: getCols
            Arguments: none
            Returns: integer number of columns in this */
            int getCols() const {
                return num_cols;
            }

            /*
            Function: getNode
            Arguments: integer indices for row and col
            Returns: Node at positon (row,col) */
            node_type& getNode(int row, int col) {

                // throw runtime exception if empty grid
                if(node_grid.size() == 0) {
                    throw std::runtime_error("Cannot access nodes from empty grid");
                }

                // throw invalid argument if out of bounds
                if(!isValidIndex(row, col)) {
                    throw std::invalid_argument("getNode method input must be within grid");
                }

                return node_grid[row][col];
            }

            /*
            Function: getNode
            Arguments: utils::MapIndex representing desired node location
            Returns: Node at positon (row,col) */
            node_type& getNode(utils::MapIndex idx)  {

                // throw runtime exception if empty grid
                if(node_grid.size() == 0) {
                    throw std::runtime_error("Cannot access nodes from empty grid");
                }

                // Verify input within bounds
                if(!isValidIndex(idx.row, idx.col)) {
                    throw std::invalid_argument("getNode method input must be within grid");
                }
                return node_grid[idx.row][idx.col];
            }

        /*
        Function: isValidIndex
        Arguments: utils::MapIndex with evaluated indicies
        Returns: boolean indicating if input is valid (row, col) index*/
        bool isValidIndex(int row, int col) const {
            
            //std::cout<< "in isValidIndex for index: "s << utils::MapIndex(row, col).toString() << std::endl;
            // Verify row and column indices within bounds
            bool rowWithinBounds = row >= 0 && row < num_rows;
            bool colWithinBounds = col >= 0 && col < num_cols;

            // Return true if all indices within bounfds
            return rowWithinBounds && colWithinBounds;
        }

        /*
        Function: getEuclideanDistance
        Arguments: row and column of start and end points
        Returns: diagonal (euclidean) distance between start and end points */
        virtual double getEuclideanDistance(utils::MapIndex index1, utils::MapIndex index2) const {
            return std::hypot((double) (index1.row - index2.row), (double) index1.col - index2.col);
        }
    };


    /*-------------------------------------------------------------------- GlobalMapGrid CLASS --------------------------------------------------------------------
    Purpose: 
        This class extends the functionality of the Grid base class, containing
        GlobalMapNode instances. It is meant to represent obstacles and traversal in 
        real space, and is updated as the room is mapped more extensively. */

    class GlobalMapGrid : public Grid<GlobalMapNode> {
        
        private:

            // Resolution for each cell widths
            double resolution;

        public:

            // Constructors
            GlobalMapGrid();
            GlobalMapGrid(int num_rows, int num_cols, double resolution);


            // Getter Methods

            double getResolution() const;

            // Public Methods
            utils::MapPoint getRealCoordinates(utils::MapIndex) const;
            std::vector<utils::MapPoint> getRealCoordinates(const std::vector<utils::MapIndex>& index_path) const;
            void updateMap(const std::vector<std::vector<int>>& occupancy, const std::vector<std::vector<int>>& visits);
            double getEuclideanDistance(utils::MapIndex index1, utils::MapIndex index2) const override;
            utils::MapIndex getIndices(utils::MapPoint real_coordinates);
            utils::MapIndex getIndices(double x, double y);
            std::vector<utils::MapIndex> getOccupiedIndices();

    

            
    };

    /*-------------------------------------------------------------------- SearchGrid CLASS --------------------------------------------------------------------
    Purpose: 
        This class extends the functionality of the Grid base class to provide a
        workspace for PathPlanner instances to operate within.  */
    class SearchGrid : public Grid<SearchNode> {
        
        public:

            // Constructors
            SearchGrid();
            SearchGrid(int num_rows, int num_cols);

            // Getter Methods
            SearchNode* getNode(int row, int col);
            SearchNode* getNode(utils::MapIndex idx);
            std::vector<SearchNode*> getNeighbors(int row, int col);

            // Public Methods
            void reset();
            void printNodes();

            
    };

    

    /*---------------------------------------------------------------- RobotLocalMap class ------------------------------------------------------------
    Purpose:
        Contain localized navigation data pertinent to an individual robot. While 
        the GlobalMapGrid object contains information in real space, RobotLocalMap
        incorporates minimum clearance of the robot's origin, such that accessible
        points can be determined. */

    class RobotLocalMap : public Grid<double> {
        
        private:

            // Minimum clearance
            double minimum_clearance;
            // Reference to global map
            GlobalMapGrid* global_map;
            

        public:
            
            // Constructors
            // RobotLocalMap();
            RobotLocalMap(GlobalMapGrid* global_map, double minimum_clearance);

            // Getters
            double getMinClearance() const;
            double getRobotClearance(int row, int col);
            double getRobotClearance(double x, double y);
            bool isAccessible(int row, int col);
            bool isAccessible(double x, double y);


            // Public Methods
            void bushFireUpdate();
            void printAccess();
            utils::MapIndex getNearestAccessibleNode(utils::MapIndex idx);
            void printAccess(int row, int col);
    };
}