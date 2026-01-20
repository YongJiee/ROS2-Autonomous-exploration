# ROS2 Autonomous Maze Exploration

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/Python-3.10-blue)](https://www.python.org/)

> Autonomous navigation system for TurtleBot3 to explore and escape unknown mazes using SLAM and Nav2 stack

**🌐 Full Project Details:** [yongjiee.github.io/project/project-4](https://yongjiee.github.io/project/project-4)

---

## 🎯 Overview

This project implements an autonomous navigation system where a TurtleBot3 robot explores unknown mazes, builds real-time maps using SLAM (Simultaneous Localization and Mapping), and successfully finds exits without any pre-programmed path or human intervention.


**Built for:** RSE2108 ROS2 Assignment  
**Platform:** TurtleBot3 Burger in Gazebo Simulation  
**Framework:** ROS2 Humble on Ubuntu 22.04

---

## ✨ Features

### Core Capabilities
- **Real-time SLAM Mapping** - Builds and updates maps continuously using SLAM Toolbox
- **Autonomous Path Planning** - Nav2 stack handles dynamic obstacle avoidance
- **Intelligent Exit Detection** - LiDAR-based recognition with 5-second time-based confirmation
- **Smart Recovery Behaviors** - Multi-directional clearance analysis when stuck
- **Frontier Exploration** - Autonomous goal generation for unknown environment exploration

### Technical Highlights
- Custom Python exploration node with state machine
- Optimized SLAM parameters for maze corridors (0.5s updates, 5cm movement threshold)
- Multi-stage recovery system analyzing 4-direction clearance
- Time-based confirmation preventing false exit detections
- Configurable for different maze sizes and complexities

---

## 🎮 Usage

### Quick Start

**Terminal 1: Launch Simulation**
```bash
cd ~/ros2_assign_ws
source install/setup.bash

# Launch default maze (2.5×2.5m)
ros2 launch maze_explorer maze_world.launch.py world_name:=tb3_maze.world x_pose:=0.0 y_pose:=-0.3
```

**Terminal 2: Start Navigation Stack**
```bash
cd ~/ros2_assign_ws
source install/setup.bash

ros2 launch maze_explorer navigation.launch.py use_sim_time:=true
```

**Terminal 3: Run Exploration Node**
```bash
cd ~/ros2_assign_ws
source install/setup.bash

ros2 run maze_explorer nav_goal_sender
```

The robot will now autonomously explore the maze and attempt to find the exit!

---

### Available Launch Configurations

#### 1. Basic TurtleBot3 Maze (2.5×2.5m)
```bash
ros2 launch maze_explorer maze_world.launch.py \
  world_name:=tb3_maze.world \
  x_pose:=0.0 \
  y_pose:=-0.3
```

#### 2. Large Complex Maze (5×5m)
```bash
ros2 launch maze_explorer maze_world.launch.py \
  world_name:=maze_walls_full.world \
  x_pose:=1.5 \
  y_pose:=1.3
```

#### 3. Custom SDF Maze
```bash
ros2 launch maze_explorer simulation_sdf.launch.py \
  x_pose:=0.25 \
  y_pose:=-1.92
```

---

### Launch File Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `world_name` | string | - | Gazebo world file name (e.g., `tb3_maze.world`) |
| `x_pose` | float | 0.0 | Initial robot X position in meters |
| `y_pose` | float | 0.0 | Initial robot Y position in meters |
| `z_pose` | float | 0.01 | Initial robot Z position in meters |
| `use_sim_time` | bool | true | Use Gazebo simulation time |

---

## ⚙️ Configuration

### SLAM Toolbox Parameters

Edit `config/mapper_params_online_async.yaml`:

```yaml
slam_toolbox:
  ros__parameters:
    # Mapping frequency
    map_update_interval: 0.5  # Update map every 0.5s
    
    # Movement threshold
    minimum_travel_distance: 0.05  # Update after 5cm movement
    
    # Loop closure
    loop_search_maximum_distance: 3.0
    
    # Map resolution
    resolution: 0.05  # 5cm grid cells
```

### Nav2 Parameters

Edit `config/nav2_params.yaml`:

Key parameters you might want to adjust:

```yaml
controller_server:
  ros__parameters:
    # Motion control
    max_vel_x: 0.22  # Max linear velocity (m/s)
    max_vel_theta: 2.0  # Max angular velocity (rad/s)
    
    # Obstacle avoidance
    inflation_radius: 0.55  # Safety buffer around obstacles (m)
    
local_costmap:
  ros__parameters:
    update_frequency: 5.0  # Costmap update rate (Hz)
    width: 3  # Local costmap width (m)
    height: 3  # Local costmap height (m)
```

### Exploration Node Parameters

Edit in `nav_goal_sender.py`:

```python
# Exit detection
EXIT_CONFIRMATION_TIME = 5.0  # Seconds to confirm exit
EXIT_CLEARANCE_THRESHOLD = 0.80  # 80% forward clearance required

# Recovery
STUCK_DETECTION_TIME = 10.0  # Seconds before triggering recovery
MIN_SAFE_CLEARANCE = 0.4  # Minimum safe distance (m)
```

---


## 📂 Project Structure

```
maze_explorer/
├── config/                          # Configuration files
│   ├── mapper_params_online_async.yaml  # SLAM Toolbox config
│   └── nav2_params.yaml             # Nav2 navigation config
├── launch/                          # ROS2 launch files
│   ├── maze_world.launch.py         # Main simulation launcher
│   ├── navigation.launch.py         # Navigation stack launcher
│   └── simulation_sdf.launch.py     # SDF maze launcher
├── maze_explorer/                   # Python package
│   ├── __init__.py
│   └── nav_goal_sender.py          # Exploration node
├── models/                          # Gazebo models
│   └── [maze SDF files]
├── rviz/                           # RViz configuration
│   └── nav2_default_view.rviz
├── worlds/                         # Gazebo world files
│   ├── tb3_maze.world
│   └── maze_walls_full.world
├── CMakeLists.txt
├── package.xml
├── README.md
└── LICENSE
```

---


## 🙏 Acknowledgments

- **ROS2 Community** - For excellent documentation and support
- **Navigation2 Team** - For the robust navigation stack
- **SLAM Toolbox** - For reliable real-time SLAM
- **TurtleBot3** - For the well-designed robot platform
- **RSE2108 Course** - Project assignment and guidance

---
