# ROS Run Instructions

## File Description and Run Instruction

`catkin_make` first to build the projects.  

### bookle_control
`rosrun bookle_control bookle_control` to control the robot go straight along the line.

### bookle_description
The description and urdf files for the robot.

### bookle_gazebo
`roslaunch bookle_gazebo test.launch` to launch the robot together with the environment in the gazebo simulation

## General Run
Run/launch `bookle_gazebo` and `bookle_control` as descripbed above to open the project.

## GMapping
Download the `git clone https://github.com/ros-perception/slam_gmapping.git` and build the project.  

Run with `roslaunch gmapping slam_gmapping.launch`. This takes input directly from the ros message.

## Bookle Interface
Publish estimate and actual pose with `rosrun bookle_interface bookle_interface`

## Command Velocity
Send cmd_vel to the robot by `rosrun teleop_twist_keyboard teleop_twist_keyboard.py`

