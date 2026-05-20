# multi_robot_navigation
Spawning a swarm of robots and utilize the navigation stack using ROS2 Jazzy and Gazebo Harmonic

# Basic launch files - usually do not run them alone

### `world.launch.py`
Starts a Gazebo world (default is an empty world) and RViz

### `spawn_robot.launch.py`
Spawns robots into the already running simulation with a certain name using `name:=robot_x` argument during launch. This launch file can be started in multiple instances with unique names.  
E.g.  
```bash
ros2 launch multi_robot_navigation spawn_robot.launch.py name:=robot_1
ros2 launch multi_robot_navigation spawn_robot.launch.py name:=robot_2
ros2 launch multi_robot_navigation spawn_robot.launch.py name:=robot_N
```

# Launch a swarm of robots

### `spawn_swarm.launch.py`
A single launch file that starts the simulation and RViz (`world.launch.py`) and spawns N robot instances through `spawn_robot.launch.py`. Number of robot instances can be set within the launch files using the following variable:

```python
number_of_robots = 3
```

# Multi-robot map merge

### TL;DR: run it with a single launch file

```bash
ros2 launch multi_robot_navigation multirobot_mapping_slam_toolbox.launch.py
```

### `multirobot_mapping_slam_toolbox.launch.py`
A single launch file that starts the simulation and RViz (`world.launch.py`) and spawns 2 hardcoded robot instances through `spawn_robot.launch.py`. It also starts 2 instances of SLAM toolbox and the `map_merge` node from `multi_robot_map_merge` packge of this repository.

### `multirobot_mapping_cartographer.launch.py` - In progress
Same as the other launch file but starts cartogrpaher instances instead of SLAM toolbox. `map_merge` node doesn't support cartographer inputs at the moment.


# Multi-robot exploration
It's a combination of running multiple instances of navigation stack and an additional exploration node. 

### TL;DR: run it with a single launch file and the exploration node

```bash
ros2 launch multi_robot_navigation multirobot_navigation_slam_toolbox.launch.py
ros2 run multi_robot_explore explore_map2
```

### `multirobot_navigation_slam_toolbox.launch.py`
A single launch file that starts the simulation and RViz (`world.launch.py`) and spawns 2 hardcoded robot instances through `spawn_robot.launch.py`. It also starts 2 instances of SLAM toolbox, 2 instances of navigation stack (Nav2) and the `map_merge` node from `multi_robot_map_merge` packge of this repository.

### Start exploration node
There are 2 nodes within the `multi_robot_explore` packageof this repository:
- `explore_map.py`: it was my original development, it's doing exploration based on both the global map and the separate maps of the 2 robots:
```
/map
/robot_1/map
/robot_2/map
```
It tries to compare local frontiers against the global map and sending goals to robots in their own local map.
- `explore_map2.py`: it's an improved version which only uses the global map. Important to note that it relies on my `map_merge` node that always creates a global map even if the map snippets cannot be merged!

# Remarks
The `multirobot_mapping_slam_toolbox.launch.py` can be compatible with the `m-explore-ros2` project and its `map_merge_py` package's `map_merge` node. Just turn off `multi_robot_map_merge` in the launch file and start `map_merge_py`:
```bash
ros2 run map_merge_py map_merge
```