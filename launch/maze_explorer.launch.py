#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    # Get package directories
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_turtlebot3_description = get_package_share_directory('turtlebot3_description')
    
    # Set environment variables
    turtlebot3_model_env = SetEnvironmentVariable(
        name='TURTLEBOT3_MODEL',
        value='burger'
    )
    
    # Add your custom models to GAZEBO_MODEL_PATH
    gazebo_model_path_env = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.environ.get('GAZEBO_MODEL_PATH', '') + ':' + 
              os.path.join(os.environ['HOME'], 'RSE2108_Assigment')
    )
    
    # Use sim time
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Robot description for robot_state_publisher
    urdf_file = os.path.join(
        pkg_turtlebot3_description,
        'urdf',
        'turtlebot3_burger.urdf'
    )
    
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    
    # Launch empty Gazebo world
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'world': os.path.join(pkg_turtlebot3_gazebo, 'worlds', 'empty_world.world')
        }.items()
    )
    
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )
    
    # Robot State Publisher - Critical for TF transforms and mapping
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc
        }]
    )
    
    # Spawn maze after 2 seconds using -database flag
    spawn_maze = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', 'maze02',
                    '-database', 'Maze02',
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '0.0'
                ],
                output='screen'
            )
        ]
    )
    
    # Spawn robot after 3 seconds using -database flag
    # Position adjusted to be safely outside maze walls
    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', 'turtlebot3_burger',
                    '-database', 'turtlebot3_burger',
                    '-x', '3.0',    # Adjust based on your maze layout
                    '-y', '3.0',    # Adjust based on your maze layout
                    '-z', '0.01'    # Just above ground - FIXED from 0.5
                ],
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        turtlebot3_model_env,
        gazebo_model_path_env,
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_maze,
        spawn_robot
    ])