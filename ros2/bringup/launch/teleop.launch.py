import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # 1. Start Gazebo with empty world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathSubstitution(FindPackageShare("ros_gz_sim")), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # 2. Robot State Publisher
    robot_description_content = Command(
        [
            "xacro",
            " ",
            PathSubstitution(FindPackageShare("description")),
            "/urdf/terrence.urdf.xacro",
        ]
    )
    
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": robot_description_content, "use_sim_time": True}],
    )

    # 3. ROS-Gazebo Bridge for /clock
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # 4. Spawn Terrence Robot in Gazebo
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description",
            "-name", "terrence",
            "-z", "0.5", # Drop it from 0.5m to prevent floor clipping
        ],
        output="screen",
    )

    # 5. Teleoperation Setup
    
    # Joy node (reads joystick input)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{'use_sim_time': True}]
    )

    # Teleop node (converts joystick input to velocity commands)
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

    # 6. Controller Spawners

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

    # Delay start of terrence_controller after `spawn_entity`
    delay_terrence_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[terrence_controller_spawner],
        )
    )

    # Delay start of joint_state_broadcaster after `spawn_entity`
    delay_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
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
    ])