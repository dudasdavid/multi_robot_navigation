import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import random
import tempfile
from tf_transformations import quaternion_from_euler

def patch_and_launch_nodes(context, *args, **kwargs):

    robot_name = LaunchConfiguration('name').perform(context)

    gz_bridge_params_path = os.path.join(
        get_package_share_directory('multi_robot_navigation'),
        'config',
        'gz_bridge.yaml'
    )

    with open(gz_bridge_params_path, 'r') as f:
        config_text = f.read()

    # Replace placeholder in plain text
    patched_text = config_text.replace('__ROBOT_NAME__', robot_name)

    # Save to a temp file
    tmp_gz_bridge = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.yaml')
    tmp_gz_bridge.write(patched_text)
    tmp_gz_bridge.close()

    # Node to bridge /cmd_vel and /odom
    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        namespace=LaunchConfiguration('name'),
        arguments=[
            '--ros-args', '-p',
            f'config_file:={tmp_gz_bridge.name}'
        ],
        output="screen",
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    ekf_params_path = os.path.join(
        get_package_share_directory('multi_robot_navigation'),
        'config',
        'ekf.yaml'
    )

    with open(ekf_params_path, 'r') as f:
        config_text = f.read()

    # Replace placeholder in plain text
    patched_text = config_text.replace('__ROBOT_NAME__', robot_name)

    # Save to a temp file
    tmp_ekf = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.yaml')
    tmp_ekf.write(patched_text)
    tmp_ekf.close()

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        namespace=LaunchConfiguration('name'),
        output='screen',
        parameters=[
            tmp_ekf.name,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
             ]
    )

    return [gz_bridge_node, ekf_node]

def patch_static_transform(context, *args, **kwargs):

    yaw_angle = LaunchConfiguration('yaw').perform(context)
    yaw_quaternion = quaternion_from_euler(0, 0, float(yaw_angle))

    static_world_transform = Node( 
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform',
            namespace=LaunchConfiguration('name'),
            condition=IfCondition(LaunchConfiguration('static_tf')),
            arguments=[LaunchConfiguration('x'),
                       LaunchConfiguration('y'),
                       '0.0',
                       str(yaw_quaternion[0]),
                       str(yaw_quaternion[1]),
                       str(yaw_quaternion[2]),
                       str(yaw_quaternion[3]),
                       'world',
                       PythonExpression(["'", LaunchConfiguration('name'), "/odom'"])],
    	    parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}])

    return [static_world_transform]

def generate_launch_description():

    pkg_multi_robot_navigation = get_package_share_directory('multi_robot_navigation')

    gazebo_models_path, ignore_last_dir = os.path.split(pkg_multi_robot_navigation)
    os.environ["GZ_SIM_RESOURCE_PATH"] += os.pathsep + gazebo_models_path

    name_arg = DeclareLaunchArgument(
        'name', default_value='robot_1',
        description='Name of the robot to spawn'
    )

    static_tf_arg = DeclareLaunchArgument(
        'static_tf', default_value='true',
        description='Apply static transform between world and odom frame'
    )


    x_arg = DeclareLaunchArgument(
        'x', default_value=str(random.randint(-20, 20)/2.0),
        description='x coordinate of spawned robot'
    )

    y_arg = DeclareLaunchArgument(
        'y', default_value=str(random.randint(-20, 20)/2.0),
        description='y coordinate of spawned robot'
    )

    yaw_arg = DeclareLaunchArgument(
        'yaw', default_value=str(random.random() * 3.14),
        description='yaw angle of spawned robot'
    )

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Flag to enable use_sim_time'
    )

    xacro_file = os.path.join(pkg_multi_robot_navigation,
                              "urdf",
                              f"mogi_bot.urdf")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            xacro_file,
            " ",
            "name:=",
            LaunchConfiguration('name'),
            " ",
            "prefix:=",
            LaunchConfiguration('name')
        ]
    )

    # Spawn the URDF model using the `/world/<world_name>/create` service
    spawn_urdf_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-entity", LaunchConfiguration('name'),
            "-string", robot_description_content,
            "-robot_namespace", LaunchConfiguration('name'),
            "-x", LaunchConfiguration('x'), "-y", LaunchConfiguration('y'), "-z", "0.5", "-Y", LaunchConfiguration('yaw')  # Initial spawn position
        ],
        output="screen",
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=LaunchConfiguration('name'),
        output='screen',
        parameters=[
            {'frame_prefix': PythonExpression(["'", LaunchConfiguration('name'), "/'"]),
             'robot_description': robot_description_content,
             'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    # Node to bridge /cmd_vel and /odom
    gz_image_bridge_node = Node(
        package="ros_gz_image",
        executable="image_bridge",
        namespace=LaunchConfiguration('name'),
        arguments=[
            PythonExpression(["'/", LaunchConfiguration('name'), "/camera/image'"])
        ],
        output="screen",
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time'),
             '.camera.image.compressed.jpeg_quality': 50},
        ],
    )

    # Relay node to republish camera_info to /camera_info
    relay_camera_info_node = Node(
        package='topic_tools',
        executable='relay',
        name='relay_camera_info',
        namespace=LaunchConfiguration('name'),
        output='screen',
        arguments=['camera/camera_info', 'camera/image/camera_info'],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    trajectory_node = Node(
        package='mogi_trajectory_server',
        executable='mogi_trajectory_server',
        name='mogi_trajectory_server',
        namespace=LaunchConfiguration('name'),
        parameters=[{'reference_frame_id': PythonExpression(["'", LaunchConfiguration('name'), "/odom'"]),
                     'robot_frame_id': PythonExpression(["'", LaunchConfiguration('name'), "/base_footprint'"]),
                     'trajectory_topic': PythonExpression(["'/", LaunchConfiguration('name'), "/trajectory'"]),
                     'update_rate': 3.0,
                     'publish_rate': 2.0,
                     'use_sim_time': LaunchConfiguration('use_sim_time'),}]
    )

    interactive_marker_twist_server_node = Node(
            package='interactive_marker_twist_server',
            executable='marker_server',
            namespace=LaunchConfiguration('name'),
            parameters=[{'link_name': PythonExpression(["'", LaunchConfiguration('name'), "/base_link'"])}],
            remappings=[('/cmd_vel', PythonExpression(["'/", LaunchConfiguration('name'), "/cmd_vel'"]))])

    opaque_nodes = OpaqueFunction(function=patch_and_launch_nodes)
    opaque_static_transform = OpaqueFunction(function=patch_static_transform)

    launchDescriptionObject = LaunchDescription()

    launchDescriptionObject.add_action(name_arg)
    launchDescriptionObject.add_action(static_tf_arg)
    launchDescriptionObject.add_action(x_arg)
    launchDescriptionObject.add_action(y_arg)
    launchDescriptionObject.add_action(yaw_arg)
    launchDescriptionObject.add_action(sim_time_arg)
    launchDescriptionObject.add_action(spawn_urdf_node)
    launchDescriptionObject.add_action(robot_state_publisher_node)
    launchDescriptionObject.add_action(gz_image_bridge_node)
    launchDescriptionObject.add_action(relay_camera_info_node)
    launchDescriptionObject.add_action(trajectory_node)
    launchDescriptionObject.add_action(interactive_marker_twist_server_node)
    launchDescriptionObject.add_action(opaque_nodes)
    launchDescriptionObject.add_action(opaque_static_transform)

    return launchDescriptionObject
