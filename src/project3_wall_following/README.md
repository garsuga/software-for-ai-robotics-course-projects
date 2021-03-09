### Project 3
Garret Sugarbaker

### Method
I defined a subset of possible states and actions beforehand. The states are strings in the form '###' where each digit is a value from 0-2.\
0 corresponds to being 'close', 1 is 'correct', and 2 is 'far'\
Possible actions are combinations of turning and 'slightly' turning while either moving or remaining still or simply going forward.\
Q-values are derived by supplying predicates to test against states and increasing the q-values for those states on ideal actions.

#### Installation
No extra python packages are used.\
Add the `project3_wall_following` directory to your Catkin workspace's `src` directory.\
Source your Catkin workspace's `devel/setup.sh`\
Run `catkin_make` in the root directory of your Catkin workspace.\
Start roscore by running `roscore` in a new terminal.\
Start Gazebo on the correct stage by running `roslaunch project3_wall_following wallfollow.launch` in another terminal.\
Start SLAM mapping with `roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping open_rviz:=false`\
Start agent navigation with `roslaunch project3_wall_following wallfollow_agent.launch`

#### Demo Video
Part 1 https://www.youtube.com/watch?v=LAWOp4BZNio