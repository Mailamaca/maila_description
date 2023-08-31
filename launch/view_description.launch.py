from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

DEFAULT_LOG_LEVEL = "info"
PKG = "maila_description"


def generate_launch_description() -> LaunchDescription:
    # Get default config file.
    description_pkg_share = FindPackageShare(package=PKG)
    rviz_config_path = PathJoinSubstitution(
        [description_pkg_share, "rviz", "only_robotmodel.rviz"]
    )

    # Declare arguments commands.
    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level", default_value=DEFAULT_LOG_LEVEL, description="log level"
    )
    declare_rviz_argument = DeclareLaunchArgument(
        "rviz", default_value="true", description="Open RViz."
    )
    declare_jsp_argument = DeclareLaunchArgument(
        "jsp", default_value="true", description="Run joint state publisher node."
    )

    # Robot state publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        description_pkg_share,
                        "launch",
                        "load_description.launch.py",
                    ]
                )
            ]
        )
    )

    # Joint state publisher gui
    jsp_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        namespace="",
        name="joint_state_publisher_gui",
        condition=IfCondition(LaunchConfiguration("jsp")),
    )

    # RViz
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_path],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_jsp_argument)
    ld.add_action(declare_rviz_argument)

    # Add the action to launch the node
    ld.add_action(jsp_gui)
    ld.add_action(rviz)
    ld.add_action(rsp)

    return ld
