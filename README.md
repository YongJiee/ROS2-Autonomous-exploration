export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/yongjie/ros2_assign_ws/install/maze_explorer/share/maze_explorer/models

cd ros2_assign_ws
colcon build --packages-select maze_explorer

# ============ TERMINAL 1 ============
# Spawn World
ros2 launch maze_explorer maze_world.launch.py world_name:=tb3_maze.world x_pose:=0.0  y_pose:=-0.3
ros2 launch maze_explorer maze_world.launch.py world_name:=maze_walls_full.world x_pose:=1.5 y_pose:=1.3

# Spawn Sdf
ros2 launch maze_explorer simulation_sdf.launch.py x_pose:=0.25 y_pose:=-1.92

# ============ TERMINAL 2 ============ (new terminal)
ros2 launch maze_explorer navigation.launch.py use_sim_time:=true

# ============ TERMINAL 3 ============ (new terminal)
ros2 run maze_explorer nav_goal_sender
