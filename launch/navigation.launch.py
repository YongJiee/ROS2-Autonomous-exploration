import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Use your own package instead of turtlebot3_navigation2
    maze_explorer_dir = get_package_share_directory('maze_explorer')
    
    param_file_name = TURTLEBOT3_MODEL + '.yaml'
    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            maze_explorer_dir,
            'config',
            param_file_name))

    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    slam_launch_file_dir = os.path.join(get_package_share_directory('slam_toolbox'), 'launch')

    rviz_config_dir = os.path.join(
        maze_explorer_dir,
        'rviz',
        'navigation.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        # Launch SLAM Toolbox
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([slam_launch_file_dir, '/online_async_launch.py']),
            launch_arguments={
                'use_sim_time': use_sim_time}.items(),
        ),

        # Launch Nav2 without map
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/navigation_launch.py']),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': param_dir}.items(),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])