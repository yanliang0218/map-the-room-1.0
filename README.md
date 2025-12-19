# map-the-room

1. This project proposes the development of an autonomous exploration system for a mobile ground robot (TurtleBot platform) operating in a simulated indoor environment (in Gazebo simulation). The goal is to enable the robot to autonomously navigate an unknown space, avoid obstacles, and incrementally build an accurate 2D occupancy map of the entire room with the frontier exploration methodology without prior knowledge of its layout.

2. We are making our work a ROS package suited for easy management. The ROS compatible with ROS1 Kinetic and can be compiled in Ubuntu 16.04. According to ChatGPT, this should be the location of our package in the hierarchy to compile: ![](catkin_ws_structure.png) 

3. This repository only contains the ROS package `map-the-room`, not the catkin_ws, to reproduce the result, you will have to create your own catkin_ws and build the ROS package. The **CMakeLists.txt** is ready to use to build the package. The catkin_ws should be created according to ROS tutorial: https://wiki.ros.org/catkin/Tutorials/create_a_workspace. If you download our repo to other location, move the repo to the location shown in the graph above:   
`mv ~/map-the-room ~/catkin_ws/src/`  
and change your directory to the repo:  
`cd ~/catkin_ws/src/map-the-room`

4. After compilation, run our package `roslaunch map_the_room occupancy_grid.launch` to launch both Gazebo simulation and Rviz visualization. The "map_the_room" is the name of our package, `occupancy_grid.launch` is our custom launch file. The custom launch file includes the world file that we use as the Gazebo simulation environment. Run `roslaunch turtlebot_teleop keyboard_teleop.launch` if you want to activate teleoperate control of the robot. If you do not have custom launch file yet, use the `roslaunch turtlebot_gazebo turtlebot_world.launch` and `roslaunch turtlebot_teleop keyboard_teleop.launch` to initiate default TurtleBot environment. 

5. For Rviz visulization, please set up the software according to the following image:  ![](rviz_explanation.png)

6. A new map will be saved everytime running `roslaunch map_the_room occupancy_grid.launch`, overwriting existing map files. The map file contains the map `map.pgm` and metadata `map.yaml`. The default location to save the map is home/{user_name}.

7. To visualize the saved map, first run the ROS core `roscore`. Secondly, open a new terminal and change directory to where the map.pgm and map.yaml is saved, if saved directly from a simulation, it is in home/{user_name}. Then run `rosrun map_server map_server map.yaml`, this will publish the map to the ROS master. Finally, open another new terminal and run `rosrun rviz rviz` to launch Rviz. The frames are already set up correctly, so click "add" in the lower left, click "By topic", and then "Map". The image above can be helpful.     
