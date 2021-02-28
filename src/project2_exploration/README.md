### Project 2
Garret Sugarbaker

#### Error in Part 2
I saw your error you commented in Canvas. That is normally due to mismatched axes but I do not encounter it in my code.\
Perhaps we have different versions of numpy. Hopefully this fixes it: the subarray is now a tuple so there should be no question\
of the shape of that array.

#### Installation for Part 2 & 3
Needs the sklearn Python package.\
Add the `project2_exploration` directory to your Catkin workspace's `src` directory.\
Source your Catkin workspace's `devel/setup.sh`\
Run `catkin_make` in the root directory of your Catkin workspace.\
Start roscore by running `roscore` in a new terminal.\
Start Gazebo on stage 4 by running `roslaunch turtlebot3_gazebo turtlebot3_stage_4.launch` in another terminal.\
Start SLAM mapping with `roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping open_rviz:=false`\
Start frontier mapping with `roslaunch project2_exploration turtlebot3_navigation.launch`

#### Demo Video
Part 2 & 3 https://www.youtube.com/watch?v=qmnhddArPDE 