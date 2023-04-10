from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition, IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get default config file.
    pkg_share = FindPackageShare(package='maila_description')
    model_path = PathJoinSubstitution([pkg_share, 'urdf', 'description.urdf'])
    config_path = PathJoinSubstitution([pkg_share, 'rviz', 'only_robotmodel.rviz'])

    # Create the launch configuration variables
    declare_joint_gui_cmd = DeclareLaunchArgument(
        name='gui',
        default_value='False',
        description='Flag to enable joint_state_publisher_gui'
    )
    declare_rviz_gui_cmd = DeclareLaunchArgument(
        name='rviz',
        default_value='False',
        description='Flag to enable the rviz visualization of the model.'
    )

    # Specify the actions
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', model_path])}]
    )
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', config_path],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_joint_gui_cmd)
    ld.add_action(declare_rviz_gui_cmd)

    # Add the action to launch the node
    ld.add_action(joint_state_publisher_node)
    ld.add_action(joint_state_publisher_gui_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(rviz_node)

    return ld
