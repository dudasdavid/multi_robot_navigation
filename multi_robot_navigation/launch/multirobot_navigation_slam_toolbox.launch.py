import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import LifecycleNode
from launch.actions import (DeclareLaunchArgument, EmitEvent, LogInfo,
                            RegisterEventHandler)
from launch_ros.event_handlers import OnStateTransition
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from launch.events import matches_action
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
    #pkg_aws_robomaker_world = get_package_share_directory('aws_robomaker_small_house_world')

    # Add your own gazebo library path here
    gazebo_models_path = "/home/david/gazebo_models"
    os.environ["GZ_SIM_RESOURCE_PATH"] += os.pathsep + gazebo_models_path
    gazebo_models_path, ignore_last_dir = os.path.split(pkg_multi_robot_navigation)
    os.environ["GZ_SIM_RESOURCE_PATH"] += os.pathsep + gazebo_models_path
    #gazebo_models_path, ignore_last_dir = os.path.split(pkg_aws_robomaker_world)
    #os.environ["GZ_SIM_RESOURCE_PATH"] += os.pathsep + gazebo_models_path

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value='navigation.rviz',
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
        'static_map_tf', default_value='false',
        description='Apply static transform between world and odom frame'
    )

    slam_params_file_1 = os.path.join(get_package_share_directory("multi_robot_navigation"), 'config', 'slam_toolbox_mapping_1.yaml')
    slam_params_file_2 = os.path.join(get_package_share_directory("multi_robot_navigation"), 'config', 'slam_toolbox_mapping_2.yaml')

    nav2_navigation_launch_path = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'bringup_launch.py'
    )

    navigation_params_path_1 = os.path.join(
        get_package_share_directory('multi_robot_navigation'),
        'config',
        'navigation_1.yaml'
    )

    navigation_params_path_2 = os.path.join(
        get_package_share_directory('multi_robot_navigation'),
        'config',
        'navigation_2.yaml'
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
    
    map_merger_node = Node(
        package='multi_robot_map_merge',
        executable='map_merge',
        name='map_merge',
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('static_map_tf')),
        parameters=[
            {'match_confidence_threshold': 0.3,
             'map_publish_frequency': 1.0,
             'use_sim_time': LaunchConfiguration('use_sim_time')},
        ])

    start_async_slam_toolbox_node_1 = LifecycleNode(
        parameters=[
          slam_params_file_1,
          {
            'use_lifecycle_manager': False,
            'use_sim_time': LaunchConfiguration('use_sim_time')
          }
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        namespace=robot_1['name'],
        remappings=[
            ("/map", "map"),
            ("/map_metadata", "map_metadata"),
            ("/slam_toolbox/scan_visualization", "slam_toolbox/scan_visualization"),
            ("/slam_toolbox/graph_visualization", "slam_toolbox/graph_visualization"),
        ]
    )

    configure_event_1 = EmitEvent(
        event=ChangeState(
          lifecycle_node_matcher=matches_action(start_async_slam_toolbox_node_1),
          transition_id=Transition.TRANSITION_CONFIGURE
        )
    )

    activate_event_1 = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=start_async_slam_toolbox_node_1,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                LogInfo(msg="[LifecycleLaunch] Slamtoolbox node is activating."),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(start_async_slam_toolbox_node_1),
                    transition_id=Transition.TRANSITION_ACTIVATE
                ))
            ]
        )
    )

    start_async_slam_toolbox_node_2 = LifecycleNode(
        parameters=[
          slam_params_file_2,
          {
            'use_lifecycle_manager': False,
            'use_sim_time': LaunchConfiguration('use_sim_time')
          }
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        namespace=robot_2['name'],
        remappings=[
            ("/map", "map"),
            ("/map_metadata", "map_metadata"),
            ("/slam_toolbox/scan_visualization", "slam_toolbox/scan_visualization"),
            ("/slam_toolbox/graph_visualization", "slam_toolbox/graph_visualization"),
        ]
    )

    configure_event_2 = EmitEvent(
        event=ChangeState(
          lifecycle_node_matcher=matches_action(start_async_slam_toolbox_node_2),
          transition_id=Transition.TRANSITION_CONFIGURE
        )
    )

    activate_event_2 = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=start_async_slam_toolbox_node_2,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                LogInfo(msg="[LifecycleLaunch] Slamtoolbox node is activating."),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(start_async_slam_toolbox_node_2),
                    transition_id=Transition.TRANSITION_ACTIVATE
                ))
            ]
        )
    )

    navigation_launch_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_navigation_launch_path),
        launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'namespace': '/robot_1',
                'params_file': navigation_params_path_1,
                'slam': 'True',
                'use_localization': 'False',
                'use_composition': 'False',
                'autostart': 'True',
                'use_namespace': 'True',
        }.items()
    )

    navigation_launch_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_navigation_launch_path),
        launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'namespace': '/robot_2',
                'params_file': navigation_params_path_2,
                'slam': 'True',
                'use_localization': 'False',
                'use_composition': 'False',
                'autostart': 'True',
                'use_namespace': 'True',
        }.items()
    )

    relay_tf_1 = Node(
        package='topic_tools',
        executable='relay',
        name='relay_tf_1',
        output='screen',
        arguments=['tf', 'robot_1/tf'],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    relay_tf_static_1 = Node(
        package='topic_tools',
        executable='relay',
        name='relay_tf_static_1',
        output='screen',
        arguments=['tf_static', 'robot_1/tf_static'],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    relay_tf_2 = Node(
        package='topic_tools',
        executable='relay',
        name='relay_tf_2',
        output='screen',
        arguments=['tf', 'robot_2/tf'],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    relay_tf_static_2 = Node(
        package='topic_tools',
        executable='relay',
        name='relay_tf_static_2',
        output='screen',
        arguments=['tf_static', 'robot_2/tf_static'],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

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
    launchDescriptionObject.add_action(map_merger_node)
    launchDescriptionObject.add_action(start_async_slam_toolbox_node_1)
    launchDescriptionObject.add_action(configure_event_1)
    launchDescriptionObject.add_action(activate_event_1)
    launchDescriptionObject.add_action(start_async_slam_toolbox_node_2)
    launchDescriptionObject.add_action(configure_event_2)
    launchDescriptionObject.add_action(activate_event_2)
    launchDescriptionObject.add_action(navigation_launch_1)
    launchDescriptionObject.add_action(navigation_launch_2)
    launchDescriptionObject.add_action(relay_tf_1)
    launchDescriptionObject.add_action(relay_tf_static_1)
    launchDescriptionObject.add_action(relay_tf_2)
    launchDescriptionObject.add_action(relay_tf_static_2)

    return launchDescriptionObject