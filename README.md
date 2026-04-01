# RoboForge WS

ROS 2 workspace for a 4-DOF robotic arm project with robot description files, MoveIt configuration, simulation launch files, Python hardware bridges, and Arduino control sketches.

## Features

- 4-DOF robotic arm description with URDF, SDF, and mesh assets
- MoveIt 2 configuration for planning and visualization
- ROS 2 launch files for display, simulation, and controller setup
- Python bridge nodes for live control and drag-based interaction
- Arduino sketches for serial and drag-controller integration

## Project Structure

- `src/roboforge_description` - robot description, meshes, URDF, SDF, and controller config
- `src/roboforge_moveit_config` - MoveIt configuration, planning settings, and launch files
- `src/arm_hw_bridge` - Python ROS 2 package for bridge and teleoperation utilities
- `arduino/` - Arduino sketches for hardware-side control
- `run_simulation.sh` - helper script for simulation startup
- `drag_bridge.py`, `live_bridge.py` - standalone bridge/control scripts

## Requirements

- Ubuntu with ROS 2 installed
- MoveIt 2
- Python 3
- `colcon`
- Arduino IDE or Arduino CLI for uploading sketches

## Build

```bash
cd ~/roboforge_ws
colcon build
source install/setup.bash
```

## Main Entry Points

Run the simulation helper:

```bash
./run_simulation.sh
```

Launch the robot display:

```bash
ros2 launch roboforge_description display.launch.py
```

Launch MoveIt demo:

```bash
ros2 launch roboforge_moveit_config demo.launch.py
```

Launch the real-hardware direct drag bridge:

```bash
ros2 launch arm_hw_bridge direct_drag_real.launch.py
```

## Hardware Integration

This repository includes Arduino sketches and ROS 2 bridge code for connecting the robotic arm to real hardware through serial-based control workflows.

## Ignored Files

The repository intentionally excludes generated workspace artifacts such as:

- `build/`
- `install/`
- `log/`
- `__pycache__/`
- `*.pyc`

## Author

Hameedullah
