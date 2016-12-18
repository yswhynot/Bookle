#include "bookle_interface/bookle_interface.h"
#include <iostream>

using namespace bookle;

BookleInterface::BookleInterface(ros::NodeHandle& nh) {
	gmapping_pt_pub_ = nh.advertise<geometry_msgs::Point>("bookle/est_point", 1);
	gazebo_pt_pub_ = nh.advertise<geometry_msgs::Point>("bookle/gazebo_point", 1);
	// gazebo_state_sub_ = nh.subscribe("gazebo/model_states", 1, &BookleInterface::StateCallback, this);

	gms.request.model_name = "bookle";
	gms_c = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

}

BookleInterface::~BookleInterface() {
	gmapping_pt_pub_.shutdown();
	gazebo_pt_pub_.shutdown();
	// gazebo_state_sub_.shutdown();
}

void BookleInterface::start() {
	// call service to get model state
	gms_c.call(gms);

	// get tf 
	try {
		tf_listener.lookupTransform("/odom", "/base_footprint", ros::Time(0), tf_transform);
	} catch (tf::TransformException &ex) {
		ROS_ERROR("%s",ex.what());
		return;
	}

	// read and save data
	geometry_msgs::Pose tmp_pose;
	tmp_pose = gms.response.pose;
	gazebo_point.x = tmp_pose.position.x;
	gazebo_point.y = tmp_pose.position.y;
	gazebo_point.z = tmp_pose.position.z;

	geometry_msgs::TransformStamped tmp_msg;
	tf::transformStampedTFToMsg(tf_transform, tmp_msg);
	est_point.x = tmp_msg.transform.translation.x;
	est_point.y = tmp_msg.transform.translation.y;
	est_point.z = tmp_msg.transform.translation.z;

	// publish positions
	gmapping_pt_pub_.publish(est_point);
	gazebo_pt_pub_.publish(gazebo_point);
}

void BookleInterface::StateCallback(const gazebo_msgs::ModelState::ConstPtr& input_state) {
	printf("In state cb\n");
	gazebo_msgs::ModelState state = *input_state;

	try {
		tf_listener.lookupTransform("/odom", "/base_footprint", ros::Time(0), tf_transform);
	} catch (tf::TransformException &ex) {
		ROS_ERROR("%s",ex.what());
		return;
	}

	// read and save data
	geometry_msgs::TransformStamped tmp_msg;
	gazebo_point = state.pose.position;
	tf::transformStampedTFToMsg(tf_transform, tmp_msg);
	est_point.x = tmp_msg.transform.translation.x;
	est_point.y = tmp_msg.transform.translation.y;
	est_point.z = tmp_msg.transform.translation.z;

	// publish positions
	gmapping_pt_pub_.publish(est_point);
	gazebo_pt_pub_.publish(gazebo_point);
}