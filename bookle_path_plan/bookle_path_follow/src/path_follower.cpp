#include <bookle_path_follow/path_follower.h>

#include <algorithm>

namespace bookle {

	PathFollow::PathFollow(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
		// subscriptions
		path_sub_ = nh.subscribe("bookle/planned_path", 1, &PathFollow::PlannedPathCallback, this);

		// publications
		target_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("bookle/target_pose", 1);
		current_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("bookle/current_pose", 1);

	}

	PathFollow::~PathFollow() {
		path_sub_.shutdown();
	}

	void PathFollow::PlannedPathCallback(nav_msgs::Path::ConstPtr input_path) {
		nav_msgs::Path tmp_path = *input_path;


	}

}