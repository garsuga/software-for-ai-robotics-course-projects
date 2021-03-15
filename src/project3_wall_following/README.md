### Project 3
Garret Sugarbaker

### Method (Part 1)
I defined a subset of possible states and actions beforehand. The states are strings in the form '###' where each digit is a value from 0-2.\
0 corresponds to being 'close', 1 is 'correct', and 2 is 'far'\
Three angles are used to represent the environment around the robot.\
'Front' which is 0 degrees, 'front-right' which is 45 degrees, and 'right' which is 90 degrees.\
Possible actions are combinations of turning and 'slightly' turning while either moving or remaining still or simply going forward.\
Q-values are derived by supplying predicates to test against states and increasing the q-values for those states on ideal actions.\
The q-table is generated on the fly during this task but a copy is saved to `q-table.txt` once the script is run.

### Method (Part 2)
Beyond the states in part 1 I added values for 7 more probed angles. With the power of the Q-Learning algorithm\
I was able to get more consistent results by incorporating more data than I did before. The learning process was very quick\
as I built on the already close to working model in part 1 and trained using a consistent reward function. Rewards\
were determined by the frequency of actions which 'moved the robot forwards' while the robot was near the wall.\
Being near the wall was determined by if the robot was not too 'close' on any side and not too 'far' on every side except forward.\
This created very consistent results and the robot can navigate properly.

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
Part 2 https://www.youtube.com/watch?v=RUhg3_99Bjw