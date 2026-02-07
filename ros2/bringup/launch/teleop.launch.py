from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gui",
                default_value="true",
                description="Start RViz2 automatically with this launch file.",
            ),
            # Joy node (gets input)
            Node(
                package='joy',
                executable='joy_node',
                name='joy_node',
                parameters=[{
                    'device_id': 0,
                    'deadzone': 0.05
                }]
            ),
            # Teleop twist controller
            Node(
                package='teleop_twist_joy',
                executable='teleop_node',
                name='teleop_twist_joy_node',
                parameters=[
                    PathSubstitution(FindPackageShare("control"))
                    / "config"
                    / "joystick.yaml"
                ]
            ),
            # Control node
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[
                    PathSubstitution(FindPackageShare("control"))
                    / "config"
                    / "controllers.yaml"
                ],
                output="both",
            ),
            # robot state publisher with robot_description from xacro
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="both",
                parameters=[
                    {
                        "robot_description": Command(
                            [
                                "xacro",
                                " ",
                                PathSubstitution(FindPackageShare("description"))
                                / "urdf"
                                / "terrence.urdf.xacro",
                            ]
                        )
                    }
                ],
            ),
            # RViz2 node
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="log",
                arguments=[
                    "-d",
                    PathSubstitution(FindPackageShare("bringup"))
                    / "config"
                    / "teleop.rviz",
                ],
                condition=IfCondition(LaunchConfiguration("gui")),
            ),
            # spawns things (yes its stupid)
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "--activate"],
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["terrence_controller", "--activate"],
            ),
        ]
    )