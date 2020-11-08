import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():

    xacro_def_path = os.path.join(
        get_package_share_directory('maila_description'), 'urdf',
        'maila.urdf.xacro')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    xacro_path = LaunchConfiguration('xacro_path', default=xacro_def_path)

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'xacro_path',
            default_value=xacro_def_path,
            description='path to urdf.xacro file to publish'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': Command(['xacro', ' ', xacro_path])
            }],
            arguments=[]),
        # arguments=[urdf]),
    ])
