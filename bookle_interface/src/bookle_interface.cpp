#include <ros/ros.h>

#include <gazebo/gazebo.hh>
// #include <gazebo/ModelState.hh>
#include <tf/transform_broadcaster.h>
#include <iostream>

#include "gazebo_msgs/GetModelState.h" 
#include "gazebo_msgs/GetPhysicsProperties.h"
#include "gazebo_msgs/ModelState.h"

const float PI = 3.141593;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "bookle_control");

	ros::NodeHandle* nh = new ros::NodeHandle();
	ros::Subscriber state_sub = nh.subscribe("/gazebo/get_model_state", 1);

	
	while (ros::ok()) {

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
