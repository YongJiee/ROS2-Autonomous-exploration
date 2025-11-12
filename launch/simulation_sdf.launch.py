#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Package directories
    maze_explorer_dir = get_package_share_directory('maze_explorer')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')

    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='0.0')
    yaw = LaunchConfiguration('yaw', default='0.0')

    # Path to the SDF maze model
    maze_sdf_path = os.path.join(
        maze_explorer_dir,
        'models',
        'ros2_maze',
        'model.sdf'
    )

    # Gazebo server with empty world
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        )
    )

    # Gazebo client
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # Robot state publisher
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_dir, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Spawn maze model from SDF file
    spawn_maze_cmd = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
            '-entity', 'maze_07',
            '-file', maze_sdf_path,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0'
        ],
        output='screen'
    )

    # Spawn robot with full position control
    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_dir, 'launch', 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
            'z_pose': z_pose,
            'yaw': yaw
        }.items()
    )

    # Add delays for proper spawning sequence
    delayed_spawn_maze = TimerAction(
        period=5.0,
        actions=[spawn_maze_cmd]
    )
    
    delayed_spawn_robot = TimerAction(
        period=8.0,
        actions=[spawn_turtlebot_cmd]
    )

    # Create launch description
    ld = LaunchDescription()
    
    # Add actions in order
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(delayed_spawn_maze)
    ld.add_action(delayed_spawn_robot)

    return ld