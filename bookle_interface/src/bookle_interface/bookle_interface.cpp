#include "bookle_interface/bookle_interface.h"

BookleInterface::BookleInterface(ros::NodeHandle& nh) {
	gmapping_pt_pub_ = nh.advertise<geometry_msgs::Point>("bookle/est_point", 5);
	gazebo_pt_pub_ = nh.advertise<geometry_msgs::Point>("bookle/gazebo_point", 5);
	gazebo_state_sub_ = nh.subscribe("gazebo/model_states", 5, &BookleInterface::StateCallback, this);
}

BookleInterface::~BookleInterface() {
	gmapping_pt_pub_.shutdown();
	gazebo_pt_pub_.shutdown();
	gazebo_state_sub_.shutdown();
}

void BookleInterface::StateCallback(const gazebo_msgs::ModelState::ConstPtr& input_state) {
	gazebo_msgs::ModelState state = *input_state;

	try {
		tf_listener.lookupTransform("/odom", "/base_footprint", ros::Time(0), tf_transform);
	} catch (tf::TransformException &ex) {
		ROS_ERROR("%s",ex.what());
		return;
	}

	// read and save data
	geometry_msgs::PoseStamped tmp_msg;
	gazebo_point = state.pose.position;
	tf::poseStampedTFToMsg(tf_transform, tmp_msg);
	est_point = tmp_msg.pose.position;

	// publish positions
	gmapping_pt_pub_.publish(est_point);
	gazebo_pt_pub_.publish(gazebo_point);
}