import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch.actions import (DeclareLaunchArgument, EmitEvent, LogInfo,
                            RegisterEventHandler)
import random

def generate_launch_description():

    pkg_multi_robot_navigation = get_package_share_directory('multi_robot_navigation')

    # Add your own gazebo library path here
    gazebo_models_path = "/home/david/gazebo_models"
    os.environ["GZ_SIM_RESOURCE_PATH"] += os.pathsep + gazebo_models_path
    gazebo_models_path, ignore_last_dir = os.path.split(pkg_multi_robot_navigation)
    os.environ["GZ_SIM_RESOURCE_PATH"] += os.pathsep + gazebo_models_path

    number_of_robots = 3

    world_arg = DeclareLaunchArgument(
        'world', default_value='empty.sdf',
        description='Name of the Gazebo world file to load'
    )

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Flag to enable use_sim_time'
    )

    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_multi_robot_navigation, 'launch', 'world.launch.py'),
        ),
        launch_arguments={
        'world': LaunchConfiguration('world'),
        'rviz': 'true',
        }.items()
    )

    robot_spawn_launch = []

    for i in range(number_of_robots):

        robot_spawn_launch.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_multi_robot_navigation, 'launch', 'spawn_robot.launch.py'),
                ),
                launch_arguments={
                'name': f'robot_{i+1}',
                'x': str(random.randint(-20, 20)/2.0),
                'y': str(random.randint(-20, 20)/2.0),
                'yaw': str(random.random() * 3.14),
                }.items()
            )
        )

    launchDescriptionObject = LaunchDescription()

    launchDescriptionObject.add_action(sim_time_arg)
    launchDescriptionObject.add_action(world_arg)
    launchDescriptionObject.add_action(world_launch)

    for robot in robot_spawn_launch:
        launchDescriptionObject.add_action(robot)


    return launchDescriptionObject