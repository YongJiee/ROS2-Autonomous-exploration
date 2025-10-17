# ============ TERMINAL 1 ============
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/yongjie/ros2_assign_ws/install/maze_explorer/share/maze_explorer/models
ros2 launch turtlebot3_gazebo empty_world.launch.py


# ============ TERMINAL 2 ============ (new terminal)
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/yongjie/ros2_assign_ws/install/maze_explorer/share/maze_explorer/models
ros2 run gazebo_ros spawn_entity.py -entity maze02 -database Maze02 \
    -x 0.2 \
    -y -1.0 \
    -z 0.0

ros2 launch maze_explorer navigation.launch.py use_sim_time:=true

colcon build --packages-select maze_explorer

