# multi_robot_navigation
Spawning a swarm of robots and utilize the navigation stack using ROS2 Jazzy and Gazebo Harmonic

# 1. Start the simulation with 2 robots

```bash
ros2 launch multi_robot_navigation spawn_robot.launch.py
```

# 2. Run SLAM on both robots

```bash
ros2 launch multi_robot_navigation multirobot_mapping_cartographer.launch.py
```

### OR

```bash
ros2 launch multi_robot_navigation multirobot_mapping_slam_toolbox.launch.py
```

# 3. Start map merging

```bash
ros2 launch multirobot_map_merge map_merge.launch.py
```

- This requires a static TF since the node doesn't provide any TF between `world --> robot_x/map`

### OR

```bash
ros2 run map_merge_py map_merge
```

- There must be no static TF because this node provides dynamic transformation during map merging `world --> robot_x/`
- Turn off static TF in `multirobot_mapping_xxx.launch.py`:  
  ```python
    #launchDescriptionObject.add_action(static_world_transform_1)
    #launchDescriptionObject.add_action(static_world_transform_2)
  ```