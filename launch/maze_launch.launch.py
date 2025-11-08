#!/usr/bin/env python3
"""
Full Maze Navigation Launch
- Launches Gazebo, SLAM Toolbox, Nav2, and RViz
- Adds startup delay between each stage
- World path can be customized via 'world' launch argument
"""

import os
import time
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ===========================================
    # Package directories
    # ===========================================
    pkg_gazebo = get_package_share_directory('maze_explorer')
    pkg_slam = get_package_share_directory('slam_toolbox')
    pkg_nav2 = get_package_share_directory('nav2_bringup')

    # ===========================================
    # Launch arguments
    # ===========================================
    declare_world = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(
            os.path.expanduser('~'),
            'turtlebot3_ws/src/maze_explorer/worlds/maze_walls_full.world'
        ),
        description='Path to Gazebo world file'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ===========================================
    # File paths
    # ===========================================
    slam_params = os.path.join(
        os.path.expanduser('~'),
        'turtlebot3_ws/src/maze_explorer/config/slam_anti_drift.yaml'
    )

    nav2_params = os.path.join(
        os.path.expanduser('~'),
        'turtlebot3_ws/src/maze_explorer/config/nav2_params.yaml'
    )

    # ===========================================
    # Launch descriptions
    # ===========================================

    # 1️⃣ Gazebo world
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'world_launch.py')
        ),
        launch_arguments={'world': world}.items(),
    )

    # 2️⃣ SLAM Toolbox (wait a few seconds for Gazebo to load)
    slam_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_slam, 'launch', 'online_async_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'slam_params_file': slam_params
                }.items(),
            )
        ]
    )

    # 3️⃣ Nav2 Navigation (delay after SLAM)
    nav2_launch = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_nav2, 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'params_file': nav2_params
                }.items(),
            )
        ]
    )

    # 4️⃣ RViz (start last)
    rviz_launch = TimerAction(
        period=15.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_nav2, 'launch', 'rviz_launch.py')
                ),
                launch_arguments={'use_sim_time': use_sim_time}.items(),
            )
        ]
    )

    # ===========================================
    # Launch sequence
    # ===========================================
    return LaunchDescription([
        declare_world,
        declare_use_sim_time,
        gazebo_launch,
        slam_launch,
        nav2_launch,
        rviz_launch,
    ])
