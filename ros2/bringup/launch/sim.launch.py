import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # pkg_bringup = FindPackageShare("bringup")
    # pkg_ros_gz_sim = FindPackageShare("ros_gz_sim")
    pkg_slam_toolbox = FindPackageShare("slam_toolbox")
    pkg_nav2_bringup = FindPackageShare("nav2_bringup")
    
    # Locate the config file
    # Ensure 'bringup' matches your actual package name where the yaml is stored
    bridge_params = os.path.join(
        FindPackageShare("bringup").find("bringup"),
        "config",
        "gz_bridge.yaml"
    )

    # Start environment
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathSubstitution(FindPackageShare("ros_gz_sim")), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={'gz_args': '-r ros2/description/worlds/empty.sdf'}.items(),
    )
    
    # Sad and futile attempt at making the mimic joints work.
     
    # # 1. Start environment (Bullet Featherstone)
    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [PathSubstitution(FindPackageShare("ros_gz_sim")), "/launch/gz_sim.launch.py"]
    #     ),
    #     launch_arguments={'gz_args': '-r ros2/description/worlds/empty_bullet_featherstone.sdf --physics-engine gz-physics-bullet-featherstone-plugin'}.items(),
    # )

    # Robot State Publisher
    robot_description_content = Command(
        [
            "xacro",
            " ",
            PathSubstitution(FindPackageShare("description")),
            "/urdf/terrence.urdf.xacro",
            " ",
            "sim_mode:=true",
        ]
    )
    
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": robot_description_content, "use_sim_time": True}],
    )

    # Bridge ROS & Gazebo
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[
            {'config_file': bridge_params},
            {'qos_overrides./tf_static.publisher.durability': 'transient_local'}
        ],
        output='screen'
    )

    # Spawn Robot
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description",
            "-name", "terrence",
            "-z", "0.5",
        ],
        output="screen",
    )

    # Joystick and Teleoperation
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{'use_sim_time': True}]
    )

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[
            PathSubstitution(FindPackageShare("control"))
            / "config"
            / "joystick.yaml",
            {'use_sim_time': True}
        ]
    )

    # Controllers
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    terrence_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["terrence_controller"],
    )

    # Delay controller spawners until after the robot is spawned
    delay_terrence_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[terrence_controller_spawner],
        )
    )

    delay_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )
    
    # Calculates the map to odom transform
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathSubstitution(FindPackageShare("slam_toolbox")), "/launch/online_async_launch.py"]
        ),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )
    
    # Launches the navigation stack (planner, controller, behavior trees)
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathSubstitution(FindPackageShare("nav2_bringup")), "/launch/navigation_launch.py"]
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': PathSubstitution([FindPackageShare("nav2_bringup"), 'params', 'nav2_params.yaml'])
        }.items(),
    )

    return LaunchDescription([
        gazebo,
        rsp,
        bridge,
        spawn_entity,
        joy_node,
        teleop_node,
        delay_joint_state_broadcaster_spawner,
        delay_terrence_controller_spawner,
        slam_toolbox,
        nav2_bringup,
    ])