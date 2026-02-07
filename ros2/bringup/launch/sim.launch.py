import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # Locate the config file
    # Ensure 'bringup' matches your actual package name where the yaml is stored
    bridge_params = os.path.join(
        FindPackageShare("bringup").find("bringup"),
        "config",
        "gz_bridge.yaml"
    )

    # 1. Start environment
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathSubstitution(FindPackageShare("ros_gz_sim")), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={'gz_args': '-r ros2/description/worlds/empty.sdf'}.items(),
    )

    # 2. Robot State Publisher
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

    # 3. Bridge ROS & Gazebo
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[
            {'config_file': bridge_params},
            {'qos_overrides./tf_static.publisher.durability': 'transient_local'}
        ],
        output='screen'
    )

    # 4. Spawn Robot
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

    # 5. Joystick and Teleoperation
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

    # 6. Controllers
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