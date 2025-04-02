# Simulations Stack
This document provides instructions for setting up the simulation workspace for the Nauti project. The workspace is designed to work with ROS Noetic and includes various packages and dependencies required for Gazebo simulation.
## Dependencies
- [nauti_controls](https://github.com/nauti-quest/nauti_controls)
- [nauti_pilot](https://github.com/nauti-quest/nauti_pilot)
- [nauti_gazebo_sims](https://github.com/nauti-quest/nauti_gazebo_sims)
- [nauti_grounding_dino](https://github.com/nauti-quest/nauti_yolo)
## Installation
- Go to your preferred directory to create the workspace and run the following command:
```bash
mkdir -p <parent-directory>/nauti_ws/src
cd <parent-directory>/nauti_ws/src
```
- Clone the required repositories into the `src` directory:
nauti-quest/nauti_controls
```bash
git clone https://github.com/nauti-quest/nauti_controls
```
nauti-quest/nauti_pilot
```bash
git clone https://github.com/nauti-quest/nauti_pilot
```
nauti-quest/nauti_gazebo_sims
```bash
git clone https://github.com/nauti-quest/nauti_gazebo_sims
```
nauti-quest/nauti_grounding_dino
```bash
git clone https://github.com/nauti-quest/nauti_yolo
```
- After cloning the repositories, navigate to nauti-gazebo-sims and setup the package using the README file provided in the package. This will ensure that all the necessary dependencies are installed and configured correctly.
```bash
cd nauti-gazebo-sims
```
- Once all the repositories are cloned and setup, navigate back to the `src` directory of the workspace and checkout to the simulation branch in both the nauti_pilot and nauti_controls package:
```bash
cd nauti_controls
git checkout simulation
cd ../nauti_pilot
git checkout simulation
cd ..
```
- Navigate back to the root folder of the workspace and build the workspace with catkin with the following commands:
```bash
cd ..
catkin build
```
- After building the workspace successfully, source the workspace to make the packages available in your ROS environment:
```bash
source devel/setup.bash
```
- To ensure that the workspace is sourced automatically every time you open a new terminal, add the following line to your `~/.bashrc` file:
```bash
echo "source <parent-directory>/nauti_ws/devel/setup.bash" >> ~/.bashrc
```
- The workspace is now set up and ready to use. You can launch the Gazebo simulation using te followinig command:
```bash
roslaunch v2_control sauvc.launch
```

## Running the state machine navigation simulation:
To run the state machine navigation simulation, multiple terminals are required to be open and is referred below by the terminal number. The following commands should be run in the respective terminals:
- Terminal 1: Launch the Gazebo simulation with the following command:
```bash
roslaunch v2_control sauvc.launch
```
Once the gazebo simulation is launched, press the play button on the bottom left corner of the taskbar in Gazebo or space bar to start the simulation.
- Terminal 2: Launch the auv_sensors for camera output with the following command:
```bash
roslaunch v2_control auv_sensors.launch
```
A window will pop up showing the camera output.
- Terminal 3: Once the camera node is up and running, launch the object detection node with the following command:
```bash
roslaunch yolo_bbox yolo_detector.launch
```
- Terminal 4: Launch the bounding box node for publishing command velocity values based on the detected object with the following command:
```bash
roslaunch nauti_controls bbox_controller.launch
```
- Terminal 5: Launch the bridge node for converting the command velocity to ROS messages for the Gazebo to recognize with the following command:
```bash
rosrun v2_control nav_ctrl 
```
- Once all these commands are up and running, you can observe the state machine navigation at work in the Gazebo simulation. The AUV will navigate towards the object of interest and stop once it reaches a certain distance from the object. The real time bounding box will be displayed in rqt_image_view at the yolo_output topic:
```bash
rqt_image_view
```
- To stop the simulation, press Ctrl+C in all the terminals to terminate the running processes.

## Running the Waypoint navigation simulation:
To run the waypoint navigation simulation, multiple terminals are required to be open and is referred below by the terminal number. The following commands should be run in the respective terminals:
- Terminal 1: Launch the Gazebo simulation with the following command:
```bash
roslaunch v2_control sauvc.launch
```
Once the gazebo simulation is launched, press the play button on the bottom left corner of the taskbar in Gazebo or space bar to start the simulation.
- Terminal 2: Launch the auv_sensors for camera output with the following command:
```bash
roslaunch v2_control auv_sensors.launch
```
A window will pop up showing the camera output.
<fill the rest in here>