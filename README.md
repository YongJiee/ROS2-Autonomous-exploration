# ============ TERMINAL 1 ============
cd ~/ros2_assign_ws
source install/setup.bash
ros2 launch maze_explorer maze_explorer.launch.py

# Wait 5-10 seconds for Gazebo to load completely


# ============ TERMINAL 2 ============ (new terminal)
cd ~/ros2_assign_ws
source install/setup.bash
ros2 run maze_explorer explorer
```