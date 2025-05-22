import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from tf_transformations import quaternion_from_euler

def generate_launch_description():

    robot_1 = {'name': 'robot_1',
               'x': -4.5,
               'y': 4.2,
               'yaw': 1.5708}
    robot_1['quaternion'] = quaternion_from_euler(0.0, 0.0, robot_1['yaw'])

    robot_2 = {'name': 'robot_2',
               'x': 4.0,
               'y': -5.5,
               'yaw': 2.3562}
    robot_2['quaternion'] = quaternion_from_euler(0.0, 0.0, robot_2['yaw'])

    pkg_multi_robot_navigation = get_package_share_directory('multi_robot_navigation')

    # Add your own gazebo library path here
    gazebo_models_path = "/home/david/gazebo_models"
    os.environ["GZ_SIM_RESOURCE_PATH"] += os.pathsep + gazebo_models_path
    gazebo_models_path, ignore_last_dir = os.path.split(pkg_multi_robot_navigation)
    os.environ["GZ_SIM_RESOURCE_PATH"] += os.pathsep + gazebo_models_path

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value='mapping.rviz',
        description='RViz config file'
    )

    world_arg = DeclareLaunchArgument(
        'world', default_value='home.sdf',
        description='Name of the Gazebo world file to load'
    )

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Flag to enable use_sim_time'
    )

    static_tf_arg = DeclareLaunchArgument(
        'static_map_tf', default_value='true',
        description='Apply static transform between world and odom frame'
    )

    cartographer_config_dir = PathJoinSubstitution(
        [
            FindPackageShare('multi_robot_navigation'),
            'config',
        ]
    )

    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_multi_robot_navigation, 'launch', 'world.launch.py'),
        ),
        launch_arguments={
        'world': LaunchConfiguration('world'),
        'rviz': 'false',
        }.items()
    )

    robot_1_spawn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_multi_robot_navigation, 'launch', 'spawn_robot.launch.py'),
        ),
        launch_arguments={
        'name': robot_1['name'],
        'x': str(robot_1['x']),
        'y': str(robot_1['y']),
        'yaw': str(robot_1['yaw']),
        'static_tf': 'false',
        }.items()
    )

    robot_2_spawn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_multi_robot_navigation, 'launch', 'spawn_robot.launch.py'),
        ),
        launch_arguments={
        'name': robot_2['name'],
        'x': str(robot_2['x']),
        'y': str(robot_2['y']),
        'yaw': str(robot_2['yaw']),
        'static_tf': 'false',
        }.items()
    )

    # Launch rviz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg_multi_robot_navigation, 'rviz', LaunchConfiguration('rviz_config')])],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    static_world_transform_1 = Node( 
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform',
            namespace=robot_1['name'],
            condition=IfCondition(LaunchConfiguration('static_map_tf')),
            arguments=[str(robot_1['x']),
                       str(robot_1['y']),
                       '0.0',
                       str(robot_1['quaternion'][0]),
                       str(robot_1['quaternion'][1]),
                       str(robot_1['quaternion'][2]),
                       str(robot_1['quaternion'][3]),
                       'world',
                       'robot_1/map'],
    	    parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}])

    static_world_transform_2 = Node( 
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform',
            namespace=robot_2['name'],
            condition=IfCondition(LaunchConfiguration('static_map_tf')),
            arguments=[str(robot_2['x']),
                       str(robot_2['y']),
                       '0.0',
                       str(robot_2['quaternion'][0]),
                       str(robot_2['quaternion'][1]),
                       str(robot_2['quaternion'][2]),
                       str(robot_2['quaternion'][3]),
                       'world',
                       'robot_2/map'],
    	    parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}])

    cartographer_1 = Node(
            package='cartographer_ros',
            executable='cartographer_node',
            output='screen',
            namespace=robot_1['name'],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', "cartographer_1.lua"])

    cartographer_occupancy_1 = Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            output='screen',
            namespace=robot_1['name'],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            arguments=['-resolution', '0.05'])

    cartographer_2 = Node(
            package='cartographer_ros',
            executable='cartographer_node',
            output='screen',
            namespace=robot_2['name'],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', "cartographer_2.lua"])

    cartographer_occupancy_2 = Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            output='screen',
            namespace=robot_2['name'],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            arguments=['-resolution', '0.05'])

    launchDescriptionObject = LaunchDescription()

    launchDescriptionObject.add_action(rviz_config_arg)
    launchDescriptionObject.add_action(sim_time_arg)
    launchDescriptionObject.add_action(static_tf_arg)
    launchDescriptionObject.add_action(world_arg)
    launchDescriptionObject.add_action(world_launch)
    launchDescriptionObject.add_action(robot_1_spawn_launch)
    launchDescriptionObject.add_action(robot_2_spawn_launch)
    launchDescriptionObject.add_action(rviz_node)
    launchDescriptionObject.add_action(static_world_transform_1)
    launchDescriptionObject.add_action(static_world_transform_2)
    launchDescriptionObject.add_action(cartographer_1)
    launchDescriptionObject.add_action(cartographer_occupancy_1)
    launchDescriptionObject.add_action(cartographer_2)
    launchDescriptionObject.add_action(cartographer_occupancy_2)

    return launchDescriptionObject