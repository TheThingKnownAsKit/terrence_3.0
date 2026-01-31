import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # 1. Start Gazebo (Harmonic) environment
    # We use 'gz_sim.launch.py' from ros_gz_sim instead of gazebo_ros
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

    # 3. Bridge ROS <-> Gazebo
    # Essential for synchronizing time (/clock) so controllers work
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # 4. Spawn Robot
    # 'create' is the new node for spawning in ros_gz_sim
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

    # 5. Controllers
    # We delay this until the spawner exits to ensure the robot exists
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    return LaunchDescription([
        gazebo,
        rsp,
        bridge,
        spawn_entity,
        joint_state_broadcaster,
    ])