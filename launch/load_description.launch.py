from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

DEFAULT_LOG_LEVEL = "info"


def generate_launch_description() -> LaunchDescription:
    # Get default config file.
    description_pkg_share = FindPackageShare(package="maila_description")
    model_description_path = PathJoinSubstitution(
        [description_pkg_share, "urdf", "description.urdf"]
    )

    # Declare arguments commands.
    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level", default_value=DEFAULT_LOG_LEVEL, description="log level"
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="",
        output="both",
        parameters=[
            {
                "robot_description": Command(["xacro ", model_description_path]),
                "publish_frequency": 30.0,
            }
        ],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_log_level_cmd)

    # Add the action to launch the node
    ld.add_action(robot_state_publisher)

    return ld
