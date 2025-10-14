#!/usr/bin/env python3

import os
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def parse_maze_spawn_point(sdf_path):
    """
    Parse SDF file and find a safe spawn point inside the maze.
    Looks for floor/ground elements or calculates from wall positions.
    """
    try:
        tree = ET.parse(sdf_path)
        root = tree.getroot()
        
        # Strategy 1: Look for a floor or ground link (common in mazes)
        for model in root.iter('model'):
            for link in model.iter('link'):
                link_name = link.get('name', '').lower()
                # Floor usually indicates the navigable area
                if 'floor' in link_name or 'ground' in link_name or 'plane' in link_name:
                    pose = link.find('pose')
                    if pose is not None and pose.text:
                        coords = pose.text.strip().split()
                        if len(coords) >= 2:
                            x, y = float(coords[0]), float(coords[1])
                            print(f"Found floor at: ({x:.2f}, {y:.2f})")
                            return str(x), str(y), '0.01'
        
        # Strategy 2: Find walls and calculate interior point
        wall_positions = []
        for model in root.iter('model'):
            for link in model.iter('link'):
                link_name = link.get('name', '').lower()
                if 'wall' in link_name:
                    pose = link.find('pose')
                    if pose is not None and pose.text:
                        coords = pose.text.strip().split()
                        if len(coords) >= 2:
                            wall_positions.append((float(coords[0]), float(coords[1])))
        
        if wall_positions:
            # Calculate the centroid of wall positions
            avg_x = sum(x for x, y in wall_positions) / len(wall_positions)
            avg_y = sum(y for x, y in wall_positions) / len(wall_positions)
            
            # Offset slightly to avoid being exactly on a wall
            spawn_x = avg_x
            spawn_y = avg_y
            
            print(f"Calculated spawn from {len(wall_positions)} walls: ({spawn_x:.2f}, {spawn_y:.2f})")
            return str(spawn_x), str(spawn_y), '0.01'
        
        # Strategy 3: Parse all collision geometries to find open space
        all_poses = []
        for model in root.iter('model'):
            for link in model.iter('link'):
                pose = link.find('pose')
                if pose is not None and pose.text:
                    coords = pose.text.strip().split()
                    if len(coords) >= 2:
                        all_poses.append((float(coords[0]), float(coords[1])))
        
        if all_poses:
            # Use median instead of mean (more robust for finding center)
            all_poses.sort()
            mid_idx = len(all_poses) // 2
            median_x = all_poses[mid_idx][0]
            
            all_poses.sort(key=lambda p: p[1])
            median_y = all_poses[mid_idx][1]
            
            print(f"Using median position: ({median_x:.2f}, {median_y:.2f})")
            return str(median_x), str(median_y), '0.01'
            
    except Exception as e:
        print(f"Error parsing SDF: {e}")
    
    # Default fallback
    print("Using default spawn point (0, 0)")
    return '0.0', '0.0', '0.01'

def generate_launch_description():
    
    # Set TURTLEBOT3_MODEL environment variable
    os.environ['TURTLEBOT3_MODEL'] = 'burger'
    
    # Get package directories
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_maze_explorer = get_package_share_directory('maze_explorer')
    
    # Paths
    maze_sdf = os.path.join(pkg_maze_explorer, 'models', 'Maze02', 'model.sdf')
    world = os.path.join(pkg_turtlebot3_gazebo, 'worlds', 'empty_world.world')
    
    # Automatically calculate spawn point from maze SDF
    spawn_x, spawn_y, spawn_z = parse_maze_spawn_point(maze_sdf)
    
    # Use the model.sdf from turtlebot3_gazebo
    turtlebot3_model_path = os.path.join(
        pkg_turtlebot3_gazebo,
        'models',
        'turtlebot3_burger',
        'model.sdf'
    )
    
    # Launch Gazebo
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )
    
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )
    
    # Spawn maze after 2 seconds
    spawn_maze = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', 'maze02',
                    '-file', maze_sdf,
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '0.0'
                ],
                output='screen'
            )
        ]
    )
    
    # Spawn robot after 3 seconds at calculated position
    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', 'turtlebot3_burger',
                    '-file', turtlebot3_model_path,
                    '-x', spawn_x,
                    '-y', spawn_y,
                    '-z', spawn_z
                ],
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        gzserver,
        gzclient,
        spawn_maze,
        spawn_robot
    ])