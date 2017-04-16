#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>

#include <gazebo/gazebo.hh>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <iostream>

#include "gazebo_msgs/SetModelState.h"
#include "gazebo_msgs/GetModelState.h" 
#include "gazebo_msgs/GetPhysicsProperties.h"
#include "gazebo_msgs/ModelState.h"

#define PI 3.1415926
#define SPEED 0.01	// meter per 100ms

void getQuaternionFromRPY(float r, float p, float y, float& qx, float& qy, float& qz, float& qw) {
	tf::Matrix3x3 m;

	tfScalar rt = r, pt = p, yt = y;
	m.setRPY(rt, pt, yt);
	
	tf::Quaternion q;
	m.getRotation(q);
	qx = q.x();
	qy = q.y();
	qz = q.z();
	qw = q.w();
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "bookle_motor_sim");

	ros::NodeHandle* nh = new ros::NodeHandle();
	ros::Rate loop_rate(10);	// 10 hz
	ros::Publisher state_pub = nh->advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1);

	gazebo_msgs::ModelState ms;
	volatile float qx, qy, qz, qw;
	volatile float yaw = PI / 2, pitch = 0, roll = 0;
	getQuaternionFromRPY(roll, pitch, yaw, qx, qy, qz, qw);

	ms.model_name = "bookle";

	ms.pose.position.x = 0.75;
	ms.pose.position.y = -0.75;
	ms.pose.position.z = 0;
	ms.pose.orientation.x = x;
	ms.pose.orientation.y = y;
	ms.pose.orientation.z = z;
	ms.pose.orientation.w = w;

	state_pub.publish(ms);

	while (ros::ok()) {


		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}