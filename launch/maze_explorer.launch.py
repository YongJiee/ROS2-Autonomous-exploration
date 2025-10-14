#!/usr/bin/env python3

import os
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def parse_wall_boxes(sdf_path):
    """
    Extract all wall collision boxes from the SDF file.
    Returns list of (x, y, width, height) tuples.
    """
    walls = []
    try:
        tree = ET.parse(sdf_path)
        root = tree.getroot()
        
        for link in root.iter('link'):
            # Get pose (position)
            pose_elem = link.find('pose')
            if pose_elem is None or not pose_elem.text:
                continue
            
            pose_values = pose_elem.text.strip().split()
            if len(pose_values) < 2:
                continue
            
            x, y = float(pose_values[0]), float(pose_values[1])
            
            # Get collision box size
            collision = link.find('.//collision')
            if collision is not None:
                geometry = collision.find('.//geometry')
                if geometry is not None:
                    box = geometry.find('.//box')
                    if box is not None:
                        size_elem = box.find('size')
                        if size_elem is not None and size_elem.text:
                            size_values = size_elem.text.strip().split()
                            if len(size_values) >= 2:
                                width = float(size_values[0])
                                height = float(size_values[1])
                                walls.append((x, y, width, height))
    except Exception as e:
        print(f"Error parsing walls: {e}")
    
    return walls

def point_in_wall(px, py, walls, buffer=0.2):
    """
    Check if point (px, py) is inside any wall.
    Buffer adds safety margin around walls.
    """
    for wx, wy, width, height in walls:
        half_w = width / 2.0 + buffer
        half_h = height / 2.0 + buffer
        
        if (wx - half_w <= px <= wx + half_w and 
            wy - half_h <= py <= wy + half_h):
            return True
    return False

def find_spawn_point(sdf_path):
    """
    Find a valid spawn point inside the maze:
    1. Calculate maze bounds
    2. Try center
    3. Spiral outward if center is blocked
    """
    walls = parse_wall_boxes(sdf_path)
    
    if not walls:
        print("No walls found, using default (0, 0)")
        return '0.0', '0.0', '0.01'
    
    # Calculate maze bounds
    min_x = min(wx - w/2 for wx, wy, w, h in walls)
    max_x = max(wx + w/2 for wx, wy, w, h in walls)
    min_y = min(wy - h/2 for wx, wy, w, h in walls)
    max_y = max(wy + h/2 for wx, wy, w, h in walls)
    
    center_x = (min_x + max_x) / 2.0
    center_y = (min_y + max_y) / 2.0
    
    print(f"Maze bounds: X[{min_x:.2f}, {max_x:.2f}], Y[{min_y:.2f}, {max_y:.2f}]")
    print(f"Maze center: ({center_x:.2f}, {center_y:.2f})")
    
    # Try center first
    if not point_in_wall(center_x, center_y, walls):
        print(f"✓ Center is clear! Spawning at ({center_x:.2f}, {center_y:.2f})")
        return f'{center_x:.2f}', f'{center_y:.2f}', '0.01'
    
    print("✗ Center is blocked, searching for open space...")
    
    # Spiral search from center
    step = 0.3  # Search step size (adjust if needed)
    max_radius = max(max_x - min_x, max_y - min_y)
    
    for radius in range(1, int(max_radius / step) + 1):
        # Try points in a circle around center
        import math
        num_points = max(8, radius * 4)  # More points for larger circles
        
        for i in range(num_points):
            angle = 2 * math.pi * i / num_points
            test_x = center_x + radius * step * math.cos(angle)
            test_y = center_y + radius * step * math.sin(angle)
            
            # Check if point is within maze bounds
            if (min_x <= test_x <= max_x and 
                min_y <= test_y <= max_y and
                not point_in_wall(test_x, test_y, walls)):
                print(f"✓ Found open space at ({test_x:.2f}, {test_y:.2f})")
                return f'{test_x:.2f}', f'{test_y:.2f}', '0.01'
    
    # Fallback to center if nothing found
    print("⚠ No clear space found, using center anyway")
    return f'{center_x:.2f}', f'{center_y:.2f}', '0.01'

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
    
    # Automatically find valid spawn point
    spawn_x, spawn_y, spawn_z = find_spawn_point(maze_sdf)
    
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
