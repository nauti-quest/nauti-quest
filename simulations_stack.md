# Simulation Stack
This document provides instructions for setting up and running the simulation workspace for the Nauti project. The workspace is designed to work with ROS Noetic and includes various packages and dependencies required for Gazebo simulation.

## System Requirements
- Ubuntu 20.04.6 LTS (Focal Fossa)
- ROS Noetic
- Gazebo Simulator
- Python 3

## Dependencies
Core packages required:
- [nauti_controls](https://github.com/nauti-quest/nauti_controls) - Control algorithms
- [nauti_pilot](https://github.com/nauti-quest/nauti_pilot) - Core pilot functionality
- [nauti_gazebo](https://github.com/nauti-quest/nauti_gazebo) - Gazebo simulation environment
- [nauti_yolo](https://github.com/nauti-quest/nauti_yolo) - Object detection
- [nauti-sm-nav](https://github.com/nauti-quest/nauti_sm_nav) - State machine navigation

## Installation and Build

### 1. Create Workspace
```bash
mkdir -p ~/nauti_ws/src
cd ~/nauti_ws/src
```

### 2. Clone Required Packages
```bash
# Clone core packages
git clone https://github.com/nauti-quest/nauti_controls.git
git clone https://github.com/nauti-quest/nauti_pilot.git
git clone https://github.com/nauti-quest/nauti_gazebo_sims.git
git clone https://github.com/nauti-quest/nauti_yolo.git
git clone https://github.com/nauti-quest/nauti_sm_nav.git

# Setup Gazebo environment
cd nauti_gazebo
# Follow README instructions for additional Gazebo setup

# Switch to simulation branches
cd ../nauti_controls
git checkout simulation
cd ../nauti_pilot
git checkout simulation
```

### 3. Build Workspace
```bash
cd ~/nauti_ws
catkin build
source devel/setup.bash

# Optional: Add to bashrc for automatic sourcing
echo "source ~/nauti_ws/devel/setup.bash" >> ~/.bashrc
```

## Running Simulations

### State Machine Navigation Simulation
Launch the following commands in separate terminal windows in the given order:

1. Launch Gazebo environment:
   ```bash
   roslaunch v2_control sauvc.launch
   ```
   Note: Press play button or spacebar in Gazebo to start simulation

2. Start camera sensors:
   ```bash
   roslaunch v2_control auv_sensors.launch
   ```

3. Launch object detection:
   ```bash
   roslaunch yolo_bbox yolo_detector.launch
   ```

4. Start state machine navigation:
   ```bash
   roslaunch nauti_sm_nav sm_nav.launch
   ```

5. Launch control bridge:
   ```bash
   rosrun v2_control nav_ctrl
   ```

To visualize object detection:
```bash
rqt_image_view  # Select yolo_output topic
```

### Waypoint Navigation Simulation
1. Launch Gazebo environment:
   ```bash
   roslaunch v2_control sauvc.launch
   ```

2. Start camera sensors:
   ```bash
   roslaunch v2_control auv_sensors.launch
   ```

3. Additional waypoint navigation commands will be added here

## Monitoring and Control
- Use `rqt_image_view` to monitor camera and YOLO detection outputs
- The AUV will autonomously navigate towards objects of interest
- Stop simulation by pressing Ctrl+C in all terminal windows

## Troubleshooting
- Ensure all ROS nodes are properly communicating
- Check Gazebo physics simulation is running (not paused)
- Verify camera outputs using rqt_image_view
- Monitor robot state using `rostopic echo` on relevant topics