# Hardware Stack
This document provides instructions for setting up and running the hardware workspace for communication between our code logic and AUV sensors using MAVLink messages. The workspace is designed to work with ROS Noetic and includes various packages and dependencies required for hardware integration.

## System Requirements
- ROS Noetic
- MAVROS (for Pixhawk communication)
- Ethernet cable (for RPi to Laptop/Jetson connection)
- RPi (Raspberry Pi)
- Pixhawk
- Xbox controller (for teleoperation)

## Dependencies
- [nauti_pilot](https://github.com/nauti-quest/nauti_pilot)
- [nauti_yolo](https://github.com/nauti-quest/nauti_yolo)
- [nauti-sm-nav](https://github.com/nauti-quest/nauti_sm_nav)
- [teleop-files](https://github.com/nauti-quest/teleop_files/tree/master) (for usb_cam image)
- [nauti-teleop](https://github.com/nauti-quest/nauti_teleop)
- joy (ROS package for joystick interface)
- mavros
- usb_cam

## Installation and Build
1. Create a catkin workspace:
   ```bash
   mkdir -p ~/auv_ws/src
   cd ~/auv_ws
   ```

2. Clone all required packages into the src directory:
   ```bash
   cd src
   git clone <package-urls>  # Clone all packages listed in Dependencies
   ```

3. Install system dependencies:
   ```bash
   sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras ros-noetic-joy
   sudo apt-get install ros-noetic-usb-cam
   ```

4. Build the workspace:
   ```bash
   cd ~/auv_ws
   catkin_make
   source devel/setup.bash
   ```

## Network Setup
1. Ensure RPi and Laptop/Jetson are connected via ethernet cable
2. Configure static IP addresses for both devices
3. Set up ROS_MASTER_URI and ROS_IP on both devices to establish common ROS master

## Running the AUV

### Basic Teleoperation Mode
1. Launch ROS master (on laptop/RPi):
   ```bash
   roscore
   ```

2. On RPi (via SSH), launch MAVROS:
   ```bash
   source ~/auv_ws/devel/setup.bash
   roslaunch mavros apm.launch
   ```

3. On RPi (new terminal), start the teleop interface:
   ```bash
   source ~/auv_ws/devel/setup.bash
   cd src/nauti_teleop/src/
   python3 px4_interface.py
   ```

4. On Laptop, start joy node:
   ```bash
   source ~/auv_ws/devel/setup.bash
   rosrun joy joy_node
   ```

5. On Laptop (new terminal), launch controller node:
   ```bash
   source ~/auv_ws/devel/setup.bash
   cd src/nauti_teleop/src/
   python3 xbox_controller.py
   ```

### Autonomous Navigation Mode
1. Ensure ROS master is running and MAVROS is connected (steps 1-2 from Teleoperation Mode)

2. On RPi, launch the pilot node:
   ```bash
   source ~/auv_ws/devel/setup.bash
   cd src/nauti_pilot/src/
   python3 pilot.py
   ```

3. On RPi (new terminal), start the camera:
   ```bash
   source ~/auv_ws/devel/setup.bash
   roslaunch usb_cam_files usb_cam.launch
   ```

4. On Laptop, launch YOLO detection:
   ```bash
   source ~/auv_ws/devel/setup.bash
   roslaunch yolo_bbox yolo_detector.launch
   ```
   Note: Verify that yolo_bbox is configured to receive USB camera image inputs from the correct rostopic

5. On Laptop (new terminal), launch state machine navigation:
   ```bash
   source ~/auv_ws/devel/setup.bash
   roslaunch nauti-sm-nav sm-nav.launch
   ```

The AUV will begin lawn-mowing pattern motion after 5 seconds and switch to object detection mode when it identifies any object from the predefined set of objects of interest.

## Troubleshooting
- Ensure all ROS nodes can communicate by checking network configuration
- Verify MAVROS connection status
- Check USB camera feed using `rqt_image_view`
- Monitor robot state using `rostopic echo` on relevant topics
