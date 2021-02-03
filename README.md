### Project 1 Part 2-3
Garret Sugarbaker

#### Installation
No additional Python packages are needed.
Add the `src/du_garret_sugarbaker` directory to your Catkin workspace.
Source your Catkin workspace's `devel/setup.sh`
Run `catkin_make` in the root directory of your Catkin workspace.
Start roscore by running `roscore` in a new terminal.
Start turtlesim by running `rosrun turtlesim turtlesim_node` in another terminal.
Call `rosrun du_garret_sugarbaker du_garret_sugarbaker.py`


#### Tuning Variables
The script uses global variables to store settings for having the turtle follow a path.
The goal of the script is to have the turtle follow the path in as close to the target time as possible.
|Variable|Description|
|--------|-----------|
|`runtime_seconds`|Desired runtime in seconds to be met.|
|`vertices`|An ordered list of relative positions to have the turtle follow. The turtle 'starts' at the first vertex.|
|`shape_scale`|A scale to be applied to the shape. Vertices components are multiplied by this scalar|
|`tick_rate`|Rate at which velocity updates are published|