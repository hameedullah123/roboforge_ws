#!/bin/bash

# Terminal 1
gnome-terminal -- bash -c "
source ~/roboforge_ws/install/setup.bash
ros2 launch roboforge_description gazebo_control.launch.py
exec bash"

# Terminal 2
gnome-terminal -- bash -c "
source ~/roboforge_ws/install/setup.bash
ros2 launch roboforge_moveit_config move_group.launch.py
exec bash"

# Terminal 3
gnome-terminal -- bash -c "
source ~/roboforge_ws/install/setup.bash
ros2 launch roboforge_moveit_config moveit_rviz.launch.py
exec bash"

# Terminal 4
gnome-terminal -- bash -c "
source ~/roboforge_ws/install/setup.bash
ros2 topic echo /joint_states
exec bash"
