cmake_minimum_required(VERSION 3.0.2)
project(map_the_room)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  kobuki_msgs
  sensor_msgs
  std_msgs
  tf
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs#   nav_msgs#   sensor_msgs#   std_msgs
# )


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES map_the_room
#  CATKIN_DEPENDS geometry_msgs nav_msgs rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Declare a C++ library
add_library(occupancy_grid
  src/occupancy_grid.cpp
)
add_library(utils
  src/utils.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don not collide
add_executable(occupancy_grid_node 
  src/occupancy_grid_node.cpp
)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(
  occupancy_grid_node
  occupancy_grid
  utils
  ${catkin_LIBRARIES}
)

#############
## Testing ##
#############

# Add gtest based cpp test target and link libraries
catkin_add_gtest(occupancy_grid_test tests/test_occupancy_grid.cpp)
catkin_add_gtest(utils_test tests/test_utils.cpp)

if(TARGET occupancy_grid_test)

  target_link_libraries(occupancy_grid_test
   occupancy_grid
   utils
   ${catkin_LIBRARIES}
  )
  
endif()

if(TARGET utils_test)

  target_link_libraries(utils_test
   occupancy_grid
   utils
   ${catkin_LIBRARIES}
  )
  
endif()

