# ROS Run Instructions

## File Description and Run Instruction

`catkin_make` first to build the projects.  

### bookle_control
`rosrun bookle_control bookle_control` to control the robot go straight along the line.

### bookle_description
The description and urdf files for the robot.

### bookle_gazebo
`roslaunch bookle_gazebo test.launch` to launch the robot together with the environment in the gazebo simulation

### laser_transform
`rosrun laser_transform laser_transform_node` to transform the `sensor_msgs::LaserScan` to the pcl conpatible version of `PointCloud2`.  
To change the rotation matrix, edit the variables in the constructor in `laser_transform.h`.

### static_tf_pub.launch
`roslaunch ./static_tf_pub.launch` to publish the `/tf` topic.

## General Run
Run/launch `bookle_gazebo` and `bookle_control` as descripbed above to open the project.

## Velodyne Project
Download the `git clone https://github.com/laboshinl/loam_velodyne.git` and build the project.  

Run with `roslaunch loam_velodyne loam_velodyne.launch`.  

To give input to the loam_velodyne system, in addition to the General Run above, use `laser_transform` to input PointCloud to the system.

## GMapping
Download the `git clone https://github.com/ros-perception/slam_gmapping.git` and build the project.  

Run with `rosrun gmapping slam_gmapping scan:=/bookle/laser/scan`. This takes input directly from the ros message.

