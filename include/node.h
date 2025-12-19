/**********************************************************************************

Name: Xuezheng Chen 
Student ID: 32470387 
File: node.h 
Purpose: Contains the class declaration of Node abstract class. 
Description: The abstract class is to implement the idea of polymorphism 
                and to be inherited by other node classes for easier node class creation
                and a more standardized behavior.

**********************************************************************************/

/*********************************** INCLUDES ***********************************/

#pragma once
#include <ros/ros.h>

/***************************** CLASS DEFINITIONS ******************************/

namespace ROS {

/*
Class: Node
Purpose: Create an abstract class for all ROS nodes.
            Since it only has abstract methods, only has a header file.
*/
class Node{

protected:

    // A handle is required by ROS to intialize a node, required for every node
    ros::NodeHandle nh;

public:

    virtual void spin() = 0;

};

}