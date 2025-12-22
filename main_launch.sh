#!/bin/bash

# --- RVIZ + MoveIt Launch ---
gnome-terminal -- bash -c "
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ur5_moveit_config demo.launch.py
exec bash
"

# Wait 3 seconds
sleep 3

# --- UR5 Arm Demo ---
gnome-terminal -- bash -c "
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run ur5_moveit_config ur5_pick_place_cpp_r
exec bash
"




