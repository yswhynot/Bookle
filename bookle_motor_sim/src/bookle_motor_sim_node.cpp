#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>

#include <gazebo/gazebo.hh>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <math.h>
#include <iostream>

#include "gazebo_msgs/SetModelState.h"
#include "gazebo_msgs/GetModelState.h" 
#include "gazebo_msgs/GetPhysicsProperties.h"
#include "gazebo_msgs/ModelState.h"

#define PI 3.141592
#define LINE_SPEED 0.01		// meter per 100ms
#define ROATION_SPEED 0.02	// rad per 100ms

struct BooklePoint {
	BooklePoint() : x(0), y(0), yaw(PI / 2) {}
	float x, y, yaw;
} target_point, current_point, action_point;

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

void getRPYFromQuaternion(float qx, float qy, float qz, float qw, float& r, float& p, float& y) {
	tf::Quaternion q(qx, qy, qz, qw);
	tf::Matrix3x3 m(q);

	tfScalar rt, pt, yt;
	m.getRPY(rt, pt, yt);
	r = rt;
	p = pt;
	y = yt;
}

void updateActionPoint() {
	// line action
	if((target_point.x - current_point.x) > LINE_SPEED)
		action_point.x = current_point.x + LINE_SPEED;
	else if((current_point.x - target_point.x) > LINE_SPEED)
		action_point.x = current_point.x - LINE_SPEED;
	else
		action_point.x = current_point.x;

	if((target_point.y - current_point.y) > LINE_SPEED)
		action_point.y = current_point.y + LINE_SPEED;
	else if((current_point.y - target_point.y) > LINE_SPEED)
		action_point.y = current_point.y - LINE_SPEED;
	else 
		action_point.y = current_point.y;
	
	// rotation action
	float yaw_diff = target_point.yaw - current_point.yaw;
	// normalize yaw 
	if(yaw_diff > PI)
		yaw_diff -= (2 * PI);
	else if(yaw_diff < (- PI))
		yaw_diff += (2 * PI);

	if(yaw_diff > ROATION_SPEED) {
		action_point.yaw = current_point.yaw + ROATION_SPEED;
	}
	else if(yaw_diff < (-ROATION_SPEED)) {
		action_point.yaw = current_point.yaw - ROATION_SPEED;
	}
	else 
		action_point.yaw = current_point.yaw;

	ROS_INFO("action_point:  %.2f, %.2f, %.2f\n", action_point.x, action_point.y, action_point.yaw);
	ROS_INFO("current_point: %.2f, %.2f, %.2f\n", current_point.x, current_point.y, current_point.yaw);
	ROS_INFO("target_point:  %.2f, %.2f, %.2f\n", target_point.x, target_point.y, target_point.yaw);
}

void TargetCallback(const geometry_msgs::Point::ConstPtr& input) {
	target_point.x = input->x;
	target_point.y = input->y;
	target_point.yaw = input->z;
}

void CurrentCallback(const geometry_msgs::PoseStamped::ConstPtr& input) {
	geometry_msgs::PoseStamped input_pose = *input;
	float r, p, y;
	getRPYFromQuaternion(
		input_pose.pose.orientation.x,
		input_pose.pose.orientation.y,
		input_pose.pose.orientation.z,
		input_pose.pose.orientation.w, 
		r, p, y);

	current_point.x = input_pose.pose.position.x;
	current_point.y = input_pose.pose.position.y;
	current_point.yaw = y;

	// check if target received
	if(target_point.x != 0) 
		updateActionPoint();
}

void updateModelState(gazebo_msgs::ModelState& ms) {
	float qx, qy, qz, qw;
	getQuaternionFromRPY(0, 0, action_point.yaw, qx, qy, qz, qw);

	ms.pose.position.x = action_point.x;
	ms.pose.position.y = action_point.y;
	ms.pose.position.z = 0;
	ms.pose.orientation.x = qx;
	ms.pose.orientation.y = qy;
	ms.pose.orientation.z = qz;
	ms.pose.orientation.w = qw;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "bookle_motor_sim");

	ros::NodeHandle* nh = new ros::NodeHandle();
	ros::Rate loop_rate(10);	// 10 hz
	ros::Publisher state_pub_ = nh->advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1);
	ros::Subscriber current_sub_ = nh->subscribe("/bookle/current_pose", 1, CurrentCallback);
	ros::Subscriber target_sub_ = nh->subscribe("/bookle/target_pose", 1, TargetCallback);

	gazebo_msgs::ModelState ms;
	action_point.x = 0.75;
	// action_point.x = 0.75;
	action_point.y = -0.75;

	ms.model_name = "bookle";
	updateModelState(ms);
	state_pub_.publish(ms);
	ROS_INFO("Init robot pose\n");

	while (ros::ok()) {
		updateModelState(ms);
		state_pub_.publish(ms);

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}