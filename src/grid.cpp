/***********************************************************************************************************************************************************
* Course: MECH 524
* Student Name: Connor McAllister
* Student Number: 65908436
* Date: 
*
* Source File: grid.cpp
*
* Purpose:
* Define Grid<NodeType> derived members and associated nodes
* 
* Description:
* This file defines derived members of the Grid<NodeType> class defined in 
* grid.h. Grid objects will be used to represent a 2D discrete map with each
* node containing metadata pertinent to its scope.
* 
*************************************************************************************************************************************************************/


/************************************************************************* INCLUDES *************************************************************************/
#include "grid.h"

#include <iostream>
#include <limits>
#include <vector>
#include <queue>

using namespace std;

    /************************************************************************* CLASSES **************************************************************************/
        
namespace Mapping {

    /*-------------------------------------------------------------------- GlobalMapNode CLASS --------------------------------------------------------------------*/

    /*
    Function: GlobalMapNode constructor
    Arguments: integer row and column indecies
    Behaviour: calls base Node constructor using row and col inputs and sets internal variables to defaults using reset()*/
    GlobalMapNode::GlobalMapNode()  {
        reset();
    }

    /*
    Function: reset
    Arguments: none
    Behaviour: resets occupanc y and num_visits to default values*/
    void GlobalMapNode::reset() {
        // set occupancy as unknown
        occupancy = -1;
        // set nVists as unvisited
        num_visits = 0;
    }

    /*
    Function: getOccupancy
    Arugments: none
    Returns: int occupancy of this */
    int GlobalMapNode::getOccupancy() const {
        return occupancy;
    }

    /*
    Function: isOccupied
    Arugments: none
    Returns: boolean indicating whether this is occupied */
    bool GlobalMapNode::isOccupied() const {
        return occupancy == Occupancy::OCCUPIED;
    }

    /*
    Function: isUnknown
    Arguments: none
    Returns: boolean indicating whether this is unknown */
    bool GlobalMapNode::isUnknown() const {
        return occupancy == Occupancy::UNKNOWN;
    }

    /*
    Function: isFree
    Arguments: none
    Returns: boolean indicatig whether this is free */
    bool GlobalMapNode::isFree() const {
        return occupancy == Occupancy::FREE;
    }

    /*
    Function: getVisits
    Argumnets: none
    Returns: int num_visits of this*/
    int GlobalMapNode::getVisits() const {
        return num_visits;
    }

    /*
    Function: setOccupancy
    Arguments: new occupancy value
    Behaviour: updates occupancy of this */
    void GlobalMapNode::setOccupancy(int new_occupancy) {
        
        // Validate input within possible range
        if(new_occupancy < -1 || new_occupancy > 100) {
            throw std::invalid_argument("New occupancy must be between -1 and 100 (inclusive)");
        }
        occupancy = new_occupancy;
    }

    /*
    Function: setVisits
    Arguments: new visit count
    Behaviour: updates nVists of this */
    void GlobalMapNode::setVisits(int newVisits) {

        // Validate input within possible range
        if(newVisits < 0) {
            throw std::invalid_argument("New visit count must be greater than or equal to zero");
        }
        num_visits = newVisits;
    }

    /*-------------------------------------------------------------------- SearchNode class --------------------------------------------------------------------*/

    /*
    Function: reset
    Arguments: none
    Behaviour: 
        - Sets gCost and hCost with -1 
        - Sets parent as nullptr  */
    void SearchNode::reset() {
        
        // Set gCost and hCost as 0
        gCost = std::numeric_limits<double>::infinity();
        hCost = 0;
        closed = false;
        // Set parent as nullptr, indicating no parent node
        parent = nullptr;
    }

    /*
    Function: SearchNode default constructor
    Arguments: row and column indices
    Behaviour: Calls base constructor and sets internal variables to defaults using reset() */
    Mapping::SearchNode::SearchNode() {
    }

    /*
    Function: SearchNode constructor
    Arguments: row and column indices
    Behaviour: Calls base constructor and sets internal variables to defaults using reset() */
    Mapping::SearchNode::SearchNode(int row, int col) {
        
        this->row = row;
        this->col = col;
        reset();
    }

    /*
    Function: stringIndices
    Arguments: none
    Returns: x and y coordinates as formatted string */
    std::string SearchNode::stringIndices() const {
        return "(" + std::to_string(row) + "," + std::to_string(col) + ")";
    }


    /*
    Function: getGCost
    Arguments: None 
    Returns: G cost of this */
    double SearchNode::getGCost() const {
        return gCost;
    }

    /*
    Function: gethCost
    Arguments: None 
    Returns: h cost of this */
    double SearchNode::getHCost() const {
        return hCost;
    }

    /*
    Function: getTotalCost
    Arguments: None 
    Returns: total cost of current Node, sum of H and G cost*/
    double SearchNode::getTotalCost() const {
        return hCost + gCost;
    }


    /*
    Function: hasParent
    Arguments: none
    Returns: boolean value indicating if parent node has been assigned */
    bool SearchNode::hasParent() const { 
        return parent != nullptr;
    }

    /*
    Function: getParent
    Arguments: none
    Returns: Parent node, throws exception if parent not assigned */
    SearchNode* SearchNode::getParent() const {

        // throw logic_error if no parent is assigned
        if(!hasParent()) {
            throw std::logic_error("SearchNode at " + stringIndices() + " cannot use getParent, no parent node assigned");
        }

        return parent;
    }

    /*
    Function: isClosed
    Arguments: none
    Returns: bool indicating if this is closed */
    bool SearchNode::isClosed() const {
        return closed;
    }


    /* 
    Function: setParent
    Arguments: reference to new parent node
    Behaviour: updates internal parent node */
    void SearchNode::setParent(SearchNode* new_parent) {

        // If nullptr is input, do not validate further
        if(new_parent == nullptr) {
            parent = nullptr;
            return;
        }
        
        // Ensure new parent is adjacent to this
        if((std::abs(new_parent->getRow() - row) > 1) || std::abs(new_parent->getCol() - col) > 1) {
            throw std::logic_error("Updated parent must be adjacent to SearchNode at " + stringIndices());
        }

        parent = new_parent;
    }

    /*
    Function: setHCost
    Arguments: new H cost value
    Behaviour: updates hCost from input */
    void SearchNode::setHCost(double newHCost) {

        // Ensure positive H cost input
        if(newHCost < 0){
            throw std::invalid_argument("Updated hCost must be a positive double");
        }
        hCost = newHCost;
    }

    /*
    Function: setGCost
    Arguments: new gCost value
    Behaviour: updates gCost from input*/
    void SearchNode::setGCost(double newGCost) {
        if(newGCost < 0){
            throw std::invalid_argument("Updated gCost must be a positive double, input: " + std::to_string(newGCost));
        }
        gCost = newGCost;
    }

    /*
    Function: getRow
    Arguments: none
    Returns: row index  of this */
    int SearchNode::getRow() const {
        return row;
    }

    /*
    Function: getCol
    Arguments: none
    Returns: column index of this */
    int SearchNode::getCol() const {
        return col;
    }



    /*
    Function: closeNode
    Arugments: none
    Behaviour: closes node, updating closed and open parameters */
    void SearchNode::closeNode() {
        closed = true;
    }

    /*-------------------------------------------------------------------- SEARCHGRID CLASS --------------------------------------------------------------------*/
    /*

    /*
    Function: SearchGrid default constructor
    Arguments:none
    Behaviour: Initializes empty SearchGrid */
    SearchGrid::SearchGrid() : Grid<SearchNode>() {}

    /*
    Function: SearchGrid Constructor
    Arguments: number of rows and cols, pointer to global GlobalMapGrid instance
    Behaviour: initializes node_grid with default SearchNode instances, instantiates fields*/
    SearchGrid::SearchGrid(int num_rows, int num_cols) : Grid<SearchNode>() {


        this->num_cols = num_cols;
        this->num_rows = num_rows;
        

        if(num_rows <= 0 || num_cols <= 0) {
            throw std::invalid_argument("SearchGrid must have row and column counts greater than zero");
        }

        node_grid = std::vector<std::vector<SearchNode>>(num_rows, std::vector<SearchNode>(num_cols, SearchNode()));

        for(int i = 0; i < this->num_rows; i++) {
            for(int j = 0; j < this->num_cols; j++) {
                node_grid[i][j] = SearchNode(i, j);
            }
        }

        
    }


    /*
    Function: getNode
    Arguments: row and column indicies of node of interest
    Returns: pointer to node of interest
    Nodes: used to hide base getNode, would otherwise return std::unique_pointer<SearchNode> */
    SearchNode* SearchGrid::getNode(int row, int col){
        
        // Validate Input
        if(!isValidIndex(row, col)) {
            throw std::invalid_argument("row and column indices out of bounds for getNode method call");
        }


        return &node_grid[row][col];
    }

    /*
    Function: getNode
    Arguments: utils::MapIndex struct containing row and col of node of interest
    Returns: pointer to node of interest
    Nodes: used to hide base getNode, would otherwise return std::unique_pointer<SearchNode> */
    SearchNode* SearchGrid::getNode(utils::MapIndex index){

        // Add input validation in debug mode
        if(!isValidIndex(index.row, index.col)) {
            throw std::invalid_argument("row and column indices out of bounds for getNode method call");
        }

        return &node_grid[index.row][index.col];
    }
    

    /*
    Function: getNeighbors
    Arguments: integer indices x and y
    Returns: vector of Nodes adjacent to Node at (x,y)*/
    std::vector<SearchNode*> SearchGrid::getNeighbors(int row, int col){

            // Empty vector of neighboring nodes
            std::vector<SearchNode*> neighbors;

            // Iterate through all adjacent nodes
            for(int i = row - 1; i <= row + 1; i++) {
                for(int j = col - 1; j <= col + 1; j++) {

                    // If node is not central node and within bounds, append to neighbors
                    if(!(i == row && j == col) && isValidIndex(i,j)) {
                        neighbors.push_back(getNode(i, j));
                    }
                }
            }

        // Return populated neighbors vector
        return neighbors;
    }


    /*
    Function: reset
    Arguments: none
    Behaviour: resets all nodes in SearchGrid to default values*/
    void SearchGrid::reset() {

        for(int i = 0; i < num_rows; i++) {
            for(int j = 0; j < num_cols; j++) {
                getNode(i, j)->reset();
            }
        }
    }

    /*
    Function: printNodes
    Arguments: none
    Behaviour: prints nodal data for debugging */
    void SearchGrid::printNodes() {

        std::cout << "Rows: " << node_grid.size() << std::endl;
        
        for(int i = 0; i < this->num_rows; i++) {
            for(int j = 0; j < this->num_cols; j++) {
                std::cout<< "cols: " << node_grid[i].size();
                std::cout << "(" << i << ", " << j << ") | Gcost = " << getNode(i, j)->getGCost() 
                << " | hCost = " << getNode(i, j)->getHCost() << " | hasParent: " << getNode(i, j)->hasParent()
                << " | isClosed: " << getNode(i, j)->isClosed() << std::endl;
            }
        }
    }
    

    

    /*-------------------------------------------------------------------- GlobalMapGrid CLASS --------------------------------------------------------------------
    Purpose:
        Contain global searching data pertaining to the environment being searched
        by all active robots. This object is meant to act as a link between the 
        program and the real world.

    */

    /*
    Function: GlobalMapGrid default constructor
    Arguments: none
    Behaviour: initializes empty MapGrid */
    GlobalMapGrid::GlobalMapGrid() {}


    /*
    Function: GlobalMapGrid constructor
    Arguments: integer row and column counts, double resolution value
    Behaviour: Sets up node_grid with default GlobalMapNode instances and initializes fields */
    GlobalMapGrid::GlobalMapGrid(int num_rows, int num_cols,  double resolution) : Grid(num_rows, num_cols) {

        // Initialize parameters
        this->resolution = resolution;
    }

    /*
    Function: UpdateMap
    Arguments: OccupancyGrid messages containing occupancy and visit count data
    Behaviour: 
        - parses through incoming messages and Node_map, updating GlobalMapNode fields with new values 
        - Returns vector of newly occupied nodes */
    void GlobalMapGrid::updateMap(const std::vector<std::vector<int>>& occupancy, const std::vector<std::vector<int>>& visits) {

        // Validate input vector dimensions
        

        std::vector<utils::MapIndex> newly_occupied_points;
        

        // Iterate through elements of occupancy and visits
        for(int i = 0; i < num_rows; i++) {
            for(int j = 0; j < num_cols; j++){


                // Check occupancy for equality and update if different

                if(occupancy[i][j] != getNode(i, j).getOccupancy()) {
                    
                    getNode(i, j).setOccupancy(occupancy[i][j]);

                    if(occupancy[i][j] == GlobalMapNode::Occupancy::OCCUPIED) { 
                        newly_occupied_points.push_back(utils::MapIndex(i, j));
                    }

                }

                // Check visits for equality and update if different
                if(visits[i][j] != getNode(i, j).getVisits()) {
                    getNode(i, j).setVisits(visits[i][j]);
                }

            }
        }
    }

    /* 
    Function: getRealCoordinates
    Arguments: x and y coordinates of Node of interest
    Returns: vector containing (x,y) coordinates of node in real space */
    utils::MapPoint GlobalMapGrid::getRealCoordinates(utils::MapIndex index) const {
        
        // Convert to global x,y coordinates
        double xReal = resolution *  index.col;
        double yReal = resolution * index.row;

        return utils::MapPoint(xReal, yReal);
    }



    /*
    Function: getRealCoordinates
    Arguements: path of indexed integer coordinates
    Returns: path of double coordiniates in real space */
    std::vector<utils::MapPoint> GlobalMapGrid::getRealCoordinates(const std::vector<utils::MapIndex>& index_path) const{
        
        std::vector<utils::MapPoint> real_path;
        // iterate through index_path and convert to real coordinates
        for(int i = 0; i < index_path.size(); i++) {
            // Append real coordinates to real path
            real_path.push_back(getRealCoordinates(index_path[i]));
        }

        return real_path;
    }

    /*
    Function: getIndices
    Arguments: x and y coordinates of point to be appriximated
    Returns: utils::MapIndex containing row, col indices of utils::MapPoint */
    utils::MapIndex GlobalMapGrid::getIndices(double x, double y) {

        int row = std::round(y  / resolution);
        int col = std::round(x / resolution);

        if(!isValidIndex(row, col)) {
            throw std::invalid_argument("Coordinate inputs out of bounds");
        }

        return utils::MapIndex(row, col);
    }

    /*
    Function: getIndices
    Arguments: utils::MapPoint from which indices will be approximated
    Returns: utils::MapIndex containing row, col indices of utils::MapPoint */
    utils::MapIndex GlobalMapGrid::getIndices(utils::MapPoint point) {
        
        return getIndices(point.x, point.y);
    }

    /*
    Function: getResolution
    Arguments: none
    Returns: resolution of this */
    double GlobalMapGrid::getResolution() const {
        return resolution;
    }



    /*
    Function: getEuclideanDistance
    Argumnets: integer row and column of two points
    Returns: Euclidean Distance between two points in real space */
    double GlobalMapGrid::getEuclideanDistance(utils::MapIndex index1, utils::MapIndex index2) const {
        double x_diff = (index2.col - index1.col) * resolution;
        double y_diff = (index2.row - index1.row) * resolution;

        return(std::hypot(x_diff, y_diff));
    }



    /*
    Function: getOccupiedIndices
    Arguments: none
    Returns: vector of indices that are occupied within the map */
    std::vector<utils::MapIndex> GlobalMapGrid::getOccupiedIndices() {

        std::vector<utils::MapIndex> occupied;

        // iterate through map, add occupancy to output vector
        for(int i = 0; i < this->num_rows; i++) {
            for(int j = 0; j < this->num_cols; j++) {
                if(getNode(i, j).isOccupied()) {

                    occupied.push_back(utils::MapIndex(i,j));

                }
            }
        }

        return occupied;
    }
    


    /*---------------------------------------------------------------- RobotLocalMap class ------------------------------------------------------------
    Purpose:
        Store accessibility of surrounding enviroment, accounting for robot size
    */

    // /*
    // Function: RobotLocalMap default constructor
    // Arguments: none
    // Behaviour: Iniitalizes empty default constructor for RobotLocalMap */
    // RobotLocalMap::RobotLocalMap() : Grid<double>() {};
       
            
    /*
    Function: RobotLocalMap constructor
    Arguments: pointer to global map, minimum allowable clearance for robot origin
    Behaviour: instantiates this and internal fields */
    RobotLocalMap::RobotLocalMap(GlobalMapGrid* global_map, double radius): Grid<double>(global_map->getRows(), global_map->getCols()) {
        
        // Initialize fields
        this->global_map = global_map;

        node_grid = std::vector<std::vector<double>>(this->num_rows, std::vector<double>(this->num_cols, std::numeric_limits<double>::infinity()));

        // Set minimum clearance, accounting for maximum discretization error
        this->minimum_clearance = radius + global_map->getResolution() / 2;

        
        // Update access with bushFire function
        bushFireUpdate();
        
    }

    

    /*
    Function: bushFire
    Arguments: queue of sources (new obstacles)
    Behaviour: updates raduis and access within this using bushFire algorithm */
    void RobotLocalMap::bushFireUpdate() {

        // Reset distnace to infinity
        for(int i = 0; i < this->num_rows; i++) {
            for(int j = 0; j < this->num_cols; j++) {
                this->node_grid[i][j] = std::numeric_limits<double>::infinity();
            }
        }

        // Placeholder values for iteration and comparison without overwriting
        int current_row, current_col;
        double new_distance;
        std::queue<utils::MapIndex> sources;
        std::vector<utils::MapIndex> updated = this->global_map->getOccupiedIndices();

        // Iterate through inputs and copy into sources
        for(const utils::MapIndex& idx : updated) {
            sources.push(idx);

            // Set distance of obstacle node to zero
            this->node_grid[idx.row][idx.col] = 0.0;
        }


        // The following code is adapted from equations and algorithms provided by OpenAI ChatGPT
        // Prompt: "Explain the step by step process of the bushfire inflation algorithm"

        while(sources.size() != 0) {

            current_row = sources.front().row;
            current_col = sources.front().col;

            sources.pop();


            // Iterate through neighbors
            for(int i = current_row - 1; i <= current_row + 1; i++) {
                for(int j = current_col - 1; j <= current_col + 1; j++) {

                    // Skip central node and out of bounds nodes
                    if(i == current_row && j == current_col || !isValidIndex(i, j)) {
                        continue;
                    }

                    if(i != current_row && j != current_col) {
                        new_distance = getNode(current_row, current_col) + std::sqrt(2) * global_map->getResolution();
                    }
                    else {
                        new_distance = getNode(current_row, current_col) + global_map->getResolution();
                    }

                    if(new_distance < getNode(i, j)) {
                        getNode(i, j) = new_distance;
                        sources.push(utils::MapIndex(i, j));
                    }
                }

                // End of AI assisted code
            }

        }
                
    }

    /*
    Function: getClearance
    Arguments: none
    Returns: clearance field */
    double RobotLocalMap::getMinClearance() const {
        return minimum_clearance;
    }


    /*
    Function: getRobotClearance
    Arguments: row, col indices point of interest
    Returns: Distance to nearest obstacle, accounting for minimum clearance / robot size */
    double RobotLocalMap::getRobotClearance(int row, int col) {

        double clearance = node_grid[row][col] - minimum_clearance;

        // If clearance is negative, collision will occur, treat as zero clearance
        if(clearance < 0) {
            clearance = 0;
        }

        return clearance;
    }

    /*
    Function: getRobotClearance
    Arguments: x, y coordinates of point of interest
    Returns: Distance to nearest obstacle, accounting for minimum clearance / robot size */
    double RobotLocalMap::getRobotClearance(double x, double y) {
        utils::MapIndex idx = global_map->getIndices(x, y);
        return getRobotClearance(idx.row, idx.col);
    }

    /*
    Function: isAccessible
    Arguments: integer row and column indices
    Returns: boolean indicating if node can be accessed */
    bool RobotLocalMap::isAccessible(int row, int col)  {

        if(!isValidIndex(row, col)) {
            throw std::invalid_argument(("isAccessible inputs must be within associated GlobalMapGrid range: " + utils::MapIndex(row, col).toString() + "not within ["
            + std::to_string(getRows()) + ", " + std::to_string(getCols()) + "]"));
        }
        return node_grid[row][col] > minimum_clearance;
        
    }

    /*
    Function: isAccessible
    Arguments: double x and y coordinates
    Returns: boolean indicating if node can be accessed */
    bool RobotLocalMap::isAccessible(double x, double y) {
        utils::MapIndex idx = global_map->getIndices(x, y);
        return isAccessible(idx.row, idx.col);
    }


    /*
    Function: getNearestAccessibleNode
    Arguments: utils::MapIndex of interest
    Returns: index of nearest accessbile node */
    utils::MapIndex RobotLocalMap::getNearestAccessibleNode(utils::MapIndex idx) {

        std::vector<std::vector<bool>> checked_nodes(num_rows, std::vector<bool>(num_cols, false));
        int current_row, current_col;

        std::queue<utils::MapIndex> sources;
        sources.push(idx);

        if(isAccessible(idx.row, idx.col)) {
            return idx;
        }

        while(sources.size() != 0) {


            current_row = sources.front().row;
            current_col = sources.front().col;
            sources.pop();
            checked_nodes[current_row][current_col] = true;


            for(int i = current_row - 1; i <= current_row + 1; i++) {
                for(int j = current_col - 1; j <= current_col + 1; j++) {


                    // Check for valid index, include in own if statement to ensure checking first
                    if(!isValidIndex(i, j)) {
                        continue;
                    }
                    // Skip central node, indices out of bounds, and previously checked nodes
                    else if((current_row == i && current_col == j) || checked_nodes[i][j]) {
                        continue;
                    }

                    // Return first accessible node
                    if(isAccessible(i, j)) {
                        return utils::MapIndex(i, j);
                    }

                    sources.push(utils::MapIndex(i, j));
                }
            }
        }

        throw std::runtime_error("Could not find accessible node within local map");
    }

    /*
    Function: printAccess
    Arguments: none
    Behaviour: prints accessof each node to console */
    void RobotLocalMap::printAccess() {
        
        for(int i = 0; i < num_rows; i++) {
            for(int j = 0; j < num_cols; j++) {
            
                if(isAccessible(i, j)) {
                    std::cout <<"o";
                }
                else {
                    std::cout << "x";
                }
                
            }
            std::cout << std::endl;
        }
        std::cout << std::endl;
    }

    /*
    Function: printAccess
    Arguments: row, column indices
    Behaviour: prints access of a given row column index */
    void RobotLocalMap::printAccess(int row, int col) {
        std::cout << utils::MapIndex(row, col).toString() << " | Obstacle Distance: " << getNode(row, col) << " | Is Accessible: " << isAccessible(row, col) << std::endl;
    }
}

