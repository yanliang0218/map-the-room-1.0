/***********************************************************************************************************************************************************
* Course: MECH 524
* Student Name: Connor McAllister
* Student Number: 65908436
* Date: November 15th 2025
*
* Source File: path.cpp
*
* Purpose:
* Implement functionality of Path objects and related members
* 
* Description:
* This file exisits to define Path, PathPlanner and PathInterpolator objects, as
* well as their derived members. Path objects are used to define and ecapsulate
* a Robot's intended path, and has derived members DiscretePath, representing 
* indices in a Discrete occupancy grid, and a RealPath, representing interpolated
* coordinates in real space. DiscretePath will be generated using a derived member
* of PathPlanner, such as AStarPlanner implemented in this file. RealPath objects
* will be generated using a PathInterpolator object, such as LinearInterpolator 
* implemented in this file, and an associated DiscretePath instance.
* 
*************************************************************************************************************************************************************/


/************************************************************************* INCLUDES *************************************************************************/
#include "path.h"
#include "grid.h"

#include <queue>
#include <stack>
#include <iostream>

using namespace std;



/************************************************************************* STRUCTS **************************************************************************/

namespace Mapping {


    /*------------------------------------------------------------ CompareSearchNodePtrs Struct --------------------------------------------------------------
    Purpose: Contains priority logic for SearchNode pointers */
    
    bool CompareSearchNodePtrs::operator()(const SearchNode* node1, const SearchNode* node2) {
        return node1->getTotalCost() > node2->getTotalCost();
    }
/************************************************************************* CLASSES **************************************************************************/


    /*------------------------------------------------------------ DiscretePath Class -----------------------------------------------------------------------
    Purpose:
        Extend the functionality of Path<int> to respresent a path contining
        discrete points in an occupancy grid indexed by row and column */

    /*


    /*
    Function: DiscretePath constructor
    Arguments: pointer to global map
    Behaviour: instantiates empty DiscretePa
    
    th instance */
    DiscretePath::DiscretePath(GlobalMapGrid* global_map, RobotLocalMap* local_map) {
        this->global_map = global_map;
        this->local_map = local_map;
    }

    /*
    Function: DiscretePath constructor
    Arguments: pointer to global map, known path cooridantes
    Behaviour: instantiates DiscretePath instance with known path values, sets timestamp to current time*/
    DiscretePath::DiscretePath(GlobalMapGrid* global_map, RobotLocalMap* local_map, const std::vector<utils::MapIndex>& input_path) {
        this->path_coordinates = input_path;
        this->global_map = global_map;
        this->local_map = local_map;
        this->time_stamp = std::time(nullptr);
    }

    /*
    Function: getTimeStamp
    Arguments: none
    Returns: time stamp of last update*/
    time_t DiscretePath::getTimeStamp() const {
        return time_stamp;
    }

    /*
    Function: getRealCoordinates
    Arguments: none
    Returns: path coordinates in real space, defined using associated GlobalMapGrid */
    std::vector<utils::MapPoint> DiscretePath::getRealCoordinates() {
        return global_map->getRealCoordinates(path_coordinates);
    }

    /*
    Function: getPathCoordinates
    Arguments: none
    Returns: vector of path coordinates */
    std::vector<utils::MapIndex> DiscretePath::getPathCoordinates() {
        return path_coordinates;
    }

    /*
    Function: refinePath
    Arguments: none
    Behaviour: iterates through path removing unnecessary points */
    void DiscretePath::refinePath() {

        std::vector<utils::MapIndex> indices;
        std::vector<utils::MapIndex> refined_path = {path_coordinates[0]};

        // Print input if in debug mode
        #ifdef _DEBUG
        for(int i = 0; i < path_coordinates.size(); i++) {
            std::cout << path_coordinates[i].toString();
        }
        #endif
     
        // Intialize temporary varaibles
        int start_index = 0;
        utils::MapIndex start = path_coordinates[start_index];
        utils::MapIndex end;
        bool isObstructed;

        while(start_index != path_coordinates.size() - 1) {            

            // Iterate backwards from end of path to specified start_index
            for(int i = path_coordinates.size() - 1; i >= start_index; i--) {

                // If no unobstructed shortcut exists, move to next point in planned path
                if(i == start_index) {
                    start_index++;
                    refined_path.push_back(path_coordinates[start_index]);
                    break;
                }

                // Assign end point
                end = path_coordinates[i];

                // Reset obstruction bool
                isObstructed = false;

                // Generate nodal shortcut path
                indices = utils::discreteMapBresenhamLineSearch(start.col, start.row, end.col, end.row, local_map->getRows(), local_map->getCols());
                
                // Print connections if in debug mode
                #ifdef _DEBUG
                std::cout << "Start Index: " << start_index << " | End Index: " << i << std::endl;
                std::cout << "Points: " << path_coordinates[start_index].toString() << "-----" << end.toString() << std::endl;
                #endif

                // Iterate through nodes in shortcut line, marks as obstructed if 
                for(utils::MapIndex& point : indices) {

                    // Print shortcut data if in debug mode 
                    #ifdef _DEBUG
                    std::cout << point.toString() << " | " << local_map->isAccessible(point.row, point.col) << std::endl;
                    #endif

                    // If shortcut is invalid, exit loop
                    if (!local_map->isAccessible(point.row, point.col)) {
                        isObstructed = true;
                        break;
                    }

                }

                // If evaluated shortcut is unobstructed, add end point to refined path
                if(!isObstructed) {
                    refined_path.push_back(path_coordinates[i]);
                    start_index = i;
                    start = path_coordinates[i];
                    break;
                }

            }
        }

        // Update internal path_coordinates parameter
        this->path_coordinates = refined_path;
    }

    /*
    Function: updatePath
    Arguments: pointer to path planner, new start and end indices
    Behaviour: generates new path between start and end, updates internal path vector */
    void DiscretePath::updatePath(const std::vector<utils::MapIndex>& new_path) {

        // update time stamp to current time
        time_stamp = std::time(nullptr);
        path_coordinates = new_path;
    }


    /*------------------------------------------------------------ RealPath Class -----------------------------------------------------------------------
    Purpose:
        Extend the functionality of Path<double> to represent a series of
        (x,y) coordinates within real 2D space, connected to indiidual 
        DiscretePath instance */

    /*
    Functiion:RealPath constructor
    Arguments: none
    Behaviour: initializes empty RealPath instance */
    RealPath::RealPath() {}

    /*
    Function: RealPath constructor
    Arguments: pointers to associated DiscretePath and PathInterpolator instances
    Behaviour: instantiates discrete and interpolator parameters, generates interpolated path from discrete*/
    RealPath::RealPath(DiscretePath* discrete, PathInterpolator* interpolator) {

        this->discrete = discrete;
        this->interpolator = interpolator;

        // set timestamp to signify sync with discrete path
        this->time_stamp = discrete->getTimeStamp();
        // Update path_coordinates from stored DiscretePath and PathInterpolator instances
        updatePath();
    }

    /*
    Function: updatePath
    Arguments: none
    Behaviour: interanlly updates path vector using discrete and interpolator*/
    void RealPath::updatePath() {
        // Update time stamp
        this->time_stamp = discrete->getTimeStamp();
        this->path_coordinates = interpolator->interpolate(discrete->getRealCoordinates());
    }

    /*
    Function: getPathCoordinates
    Arguments: none
    Returns: vector of path coordinates, after updating from DiscretePath instance if necessary */
    std::vector<utils::MapPoint> RealPath::getPathCoordinates() {
        
        // update path if out of sync with associated discrete path
        if(this->time_stamp != discrete->getTimeStamp()) {
            updatePath();
        } 

        return this->path_coordinates;
    }

    /*------------------------------------------------------------ PathInterpolator Class -----------------------------------------------------------------------
    Purpose: Serve as conversion between DiscretePath and RealPath objects, by performing the following
        - Converts Discrete map coordinates into real space
        - Interpolates discrete points as continuous path 
    Notes: creating derived members will require adding associated field to InterpolatorType enum */

    /*
    Function: PathInterpolator default constructor
    Arguments: none
    Behaviour: instantiates empty PathInterpolator instance */
    PathInterpolator::PathInterpolator() {}
    
    /*
    Function: PathInterpolator constructor
    Arguments: interpolateed path resolution
    Behaviour: instantiates and assigns internal fields */
    PathInterpolator::PathInterpolator(double path_resolution) {
        this->path_resolution = path_resolution;
    }

    /*
    Function: joinPaths
    Arguments: 3D jagged vector, representing vector of paths
    Returns: concatenated vector of paths 
    Notes: private helper function, allows for peicewise interpolation*/
    std::vector<utils::MapPoint> PathInterpolator::joinPaths(const std::vector<std::vector<utils::MapPoint>>& paths) {
        
        std::vector<utils::MapPoint> concat;
        int final_size = 0;
        
        // Preallocate vector space to avoid resizing
        for(int i = 0; i < paths.size(); i++) {
            final_size += paths[i].size();
        }
        
        concat.resize(final_size);
        int k = 0;

        for(int i = 0; i < paths.size(); i++) {
            for(int j = 0; j < paths[i].size(); j++) {
                concat[k++] = paths[i][j];
            }
        }

        return concat;
    }


    /*------------------------------------------------------- LinearInterpolator Class -----------------------------------------------------------------
    Purpose:
        Implement peicewise linear interpolation to convert discrete
        nodal path to real world coordinates, from which motion can be 
        generated. */
    
    /*    
    Function: LinearInterpolator default constructor
    Arguments: none
    Behaviour: initializes empty object*/
    LinearInterpolator::LinearInterpolator() : PathInterpolator() {}

    /*
    Function: LinearInterpolator constructor
    Arguments: resolution
    Behaviour: initializes internal parameters (calls base constructor) */
    LinearInterpolator::LinearInterpolator(double path_resolution) : PathInterpolator(path_resolution) {}

    /*
    Function: linearInterpolate
    Arguments: start point, end point, resolution 
    Returns: interpolated vector of coordinates */
    std::vector<utils::MapPoint> LinearInterpolator::linearInterpolate(utils::MapPoint start, utils::MapPoint end, bool inclusive) {
        

        std::vector<utils::MapPoint> interpolated_coords;
        interpolated_coords.push_back(start);

        // get number of points and x y step sizes, forcing minimum of one step
        int num_points = utils::clamp<int>(std::round(std::hypot(end.y - start.y, end.x - start.x) / path_resolution), 1, INT_MAX);
        double dx = (end.x - start.x) / num_points;
        double dy = (end.y - start.y) / num_points;

        // Placeholders for readability
        double x_new = start.x;
        double y_new = start.y;

        // Incrementally interpolate
        for(int i = 0; i < num_points - 1; i++) {

            x_new += dx;
            y_new += dy;
            interpolated_coords.push_back(utils::MapPoint(x_new, y_new));

            // Print interpolation data if in debug mode
            #ifdef _DEBUG
            std::cout << "Interpolating Index: " << i << " Coordinates: " << utils::MapPoint(x_new, y_new).toString() << std::endl;
            #endif
        }

        // Add end point to interpolated path if marked inclusive (defaults to true)
        if(inclusive) {
            interpolated_coords.push_back(end);
        }
        return interpolated_coords;
    }


    /*
    Function: interpolate
    Arguments: none
    Returns: 
        interpolateed, interpolated RealPath instance derived from DiscretePath 
        using quadratic bezier interpolateing */
    std::vector<utils::MapPoint> LinearInterpolator::interpolate(const std::vector<utils::MapPoint>& input_path) {

        // vector of interpolated path segments
        std::vector<std::vector<utils::MapPoint>> all_paths;
        // interpolate, output path
        std::vector<utils::MapPoint> interpolated_path = {};
        
        // Return empty path for empty input
        if(input_path.size() == 0) {
            return interpolated_path;
        }


        for(int i = 0; i < input_path.size() - 1; i++) {

            // Add linear segments to vector of paths, ignoring final point to avoid repeated coordinates
            all_paths.push_back(linearInterpolate(input_path[i], input_path[i + 1], false));

        } 

        // Combine peicewise paths
        return joinPaths(all_paths);
    }

    /*------------------------------------------------------ PathPlanner Class -----------------------------------------------------------------------
    Purpose:
        Provide an abstract interface for the implementation of derived 
        pathplanner types */


    
    /*
    Function: PathPlanner constructor
    Arguments: Reference to global map
    Behaviour: updates internal reference to map, instantuates default SearchGrid */
    PathPlanner::PathPlanner(GlobalMapGrid* global_map, RobotLocalMap* local_map) {
        
        this->global_map = global_map;
        this->local_map = local_map;

        // Initialize SearchGrid to match parameters of LocalMapGrid
        search_grid = SearchGrid(local_map->getRows(), local_map->getCols());
    }
 
    /*
    Function: pathFromEndNode
    Arguments: Pointer to end node within completed searchgrid, integer start indices
    Returns: vector of path indices */
    std::vector<utils::MapIndex> PathPlanner::pathFromEndNode(SearchNode* end_node, utils::MapIndex start) {
        
        // Placeholder variables for iterating through path
        std::stack<SearchNode*> reverse_path;
        SearchNode* current_node = end_node;

        // Empty path to output
        std::vector<utils::MapIndex> forward_path;


        // Iterate through path in reverse order, by accessing each node's parent
        while(current_node->getRow() != start.row || current_node->getCol() != start.col) {

            // Add curent node to path stack
            reverse_path.push(current_node);

            // move to parent
            current_node = current_node->getParent();
        }

        // add final node to path
        reverse_path.push(current_node);

        int num_nodes = reverse_path.size();

        while(true) {
            
            // Access and remove last element in stack
            current_node = reverse_path.top();
            reverse_path.pop();
            
            // Add vector of indices to path
            forward_path.push_back(utils::MapIndex(current_node->getRow(), current_node->getCol()));

            if(reverse_path.size() == 0) { 
                break;
            }
        }

        return forward_path;
    }

    RobotLocalMap* PathPlanner::getLocalMap() const {
        return this->local_map;
    }

    /*
    Function: printClosedNodes
    Arguments: none
    Behaviour: prints 2d array depicting closed nodes, for debugging*/
    void PathPlanner::printClosedNodes() {

        for(int i = 0; i < search_grid.getRows(); i++) {
            for(int j = 0; j < search_grid.getCols(); j++) {

                if(search_grid.getNode(i, j)->isClosed()) {
                    std::cout << "x ";
                }
                else {
                    std::cout << "o ";
                }
            }
            std::cout << std::endl;
        }

        std::cout << std::endl;

    }




    /*-------------------------------------------------------- AStarPlanner Class -------------------------------------------------------------------
    Purpose:
        Implement A* path planning algorithm in derived member of PathPlanner
        to generate a vector of discrete coordinates within an occupancy grid */


    
    /*
    Function: AStarPlanner constructor
    Arguments: global map, visit sensitivity
    Behaviour: instantiates internal SearchGrid and visitSensitivity */
    AStarPlanner::AStarPlanner(GlobalMapGrid* global_map, RobotLocalMap* local_map, double visit_sensitivity) : PathPlanner(global_map, local_map) {
        this->visit_sensitivity = visit_sensitivity;
    }
    
    /*
    Function: generatePath
    Arguments: none
    Returns: vector of (row, column) indices for an A* path within and occupancy grid*/
    std::vector<utils::MapIndex> AStarPlanner::generatePath(utils::MapIndex start, utils::MapIndex end) {

        // Reset search grid for new path
        search_grid.reset();

        // Initialize priority queue of open nodes, sorting by total fCost + hCost. Sord order defined in CompareSearchNodePtrs
        std::priority_queue<SearchNode*, std::vector<SearchNode*>, CompareSearchNodePtrs> open_nodes = std::priority_queue<SearchNode*, std::vector<SearchNode*>, CompareSearchNodePtrs>();
        
        // Return single point if start == end
        if(start == end) {
            return std::vector<utils::MapIndex>({start}) ;
        }

        SearchNode* current_node = search_grid.getNode(start);

        // Set g and h cost parameters of starting node, using euclidean distance as heuristic function
        current_node->setGCost(0);
        current_node->setHCost(search_grid.getEuclideanDistance(start, end));

        // Initialize empty vector of neighbors to output
        std::vector<SearchNode*> neighbors;
        
        // Add start to open nodess
        open_nodes.push(current_node);

        // placeholders for gcost, hcost and total cost of node for comparison
        double current_gcost, current_hcost, current_total_cost;
        
        // loop until target node is reached
        while(true) {

            // If open nodes is empty and goal not reached, goal is inaccessible. Return empty path
            if(open_nodes.size() == 0 && (current_node->getRow() != end.row || current_node->getCol() != end.col)) {
                return std::vector<utils::MapIndex>();
            }
          
            // Get node with lowest total cost and remove from open_nodes
            current_node = open_nodes.top();
            open_nodes.pop();

            if(current_node->getCol() == end.col && current_node->getRow() == end.row) {
                break;
            }
            
            neighbors = search_grid.getNeighbors(current_node->getRow(), current_node->getCol());

            for(SearchNode* neighbor : neighbors) {


                // Do not search neighbor if closed
                if(neighbor->isClosed() || !local_map->isAccessible(neighbor->getRow(), neighbor->getCol())) { 
                    continue;
                }

                // Get h cost estimate 
                current_hcost = search_grid.getEuclideanDistance(utils::MapIndex(neighbor->getRow(), neighbor->getCol()), end);

                // Determine if neighbor is adjacent or diagonal and get updated G cost
                if(neighbor->getRow() != current_node->getRow() && neighbor->getCol() != current_node->getCol()) {
                    current_gcost = current_node->getGCost() + std::sqrt(2) + global_map->getNode(neighbor->getRow(), neighbor->getCol()).getVisits() * visit_sensitivity;
                }
                else {
                    current_gcost = current_node->getGCost() + 1 + global_map->getNode(neighbor->getRow(), neighbor->getCol()).getVisits() * visit_sensitivity;
                }


                // Update node parameters if no parent has been assigned, or if current node as parent provides a more optimal path
                if(neighbor->getGCost() > current_gcost) {
                    // Update parent and cost paramets
                    neighbor->setParent(current_node);
                    neighbor->setGCost(current_gcost);
                    neighbor->setHCost(current_hcost);

                    // Add neighboring node to open nodes
                    open_nodes.push(neighbor);                    
                    
                }
                
            }

            // Close current_node

            current_node->closeNode();
           // std::cout<<"generatePath 11"<<std::endl;

            // Print for debugging
            #ifdef _DEBUG
            if(current_node->hasParent()) {
            std::cout << "Current: " << current_node->stringIndices() << " | Parent: " << current_node->getParent()->stringIndices() << std::endl;
            } 
            else {
                std::cout << "Current: " << current_node->stringIndices() << " | Parent: NONE" << std::endl;
            }
            #endif
        }

        // Generate path through parent node tracing
        std::vector<utils::MapIndex> astar_path = pathFromEndNode(current_node, start);


        // Print closed nodes if in debug mode
        #ifdef _DEBUG
        printClosedNodes();
        #endif

        // Trace path backward from end node
        return astar_path;
    }


}