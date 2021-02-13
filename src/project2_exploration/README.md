### Project 2 Part 1
Garret Sugarbaker

#### Installation
No additional Python packages are needed.
Add the `src/project2_exploration` directory to your Catkin workspace.
Source your Catkin workspace's `devel/setup.sh`
Run `catkin_make` in the root directory of your Catkin workspace.
Start roscore by running `roscore` in a new terminal.
Start Gazebo on stage 4 by running `roslaunch turtlebot3_gazebo turtlebot3_stage_4.launch` in another terminal.
Start SLAM mapping with `roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping open_rviz:=false`
Start navigation with `roslaunch project2_exploration turtlebot3_navigation.launch`
Call `rosrun project2_exploration project2_tf_output.py` to begin outputting the current transform.
Call `rosrun project2_exploration project2_go_to_goal.py` to begin navigating to goal positions.


#### Tuning Variables
This script has the robot follow a set of hard-coded positions using the `move_base` server.
|Variable|Description|
|--------|-----------|
|`targets`|Ordered list of tuple positions to have the robot nagivate to in (x,y,z) format.|