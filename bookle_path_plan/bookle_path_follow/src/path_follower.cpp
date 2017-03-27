#include <bookle_path_follow/path_follower.h>

#include <algorithm>

namespace bookle {

	PathFollow::PathFollow(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
		// subscriptions
		path_sub_ = nh.subscribe("bookle/planned_path", 1, &PathFollow::PlannedPathCallback, this);
		pose_sub_ = nh.subscribe("bookle/current_pose", 1, &PathFollow::CurrentPointCallback, this);

		// publications
		target_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("bookle/target_pose", 1);
		// current_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("bookle/current_pose", 1);

	}

	PathFollow::~PathFollow() {
		path_sub_.shutdown();
		pose_sub_.shutdown();
	}

	void PathFollow::PlannedPathCallback(const nav_msgs::Path::ConstPtr& input_path) {
		nav_path = *input_path;

		for(int i = 0; i < nav_path.poses.size(); i++) {
			geometry_msgs::PoseStamped tmp_pos = nav_path.poses[i];
			Point tmp_point = {
				int(tmp_pos.pose.position.x),
				int(tmp_pos.pose.position.y),
				int(tmp_pos.pose.position.z)
			};
			v_path.push_back(tmp_point);
		}
	}

	void PathFollow::CurrentPointCallback(const geometry_msgs::Point::ConstPtr& input_point) {
		geometry_msgs::Point tmp_point = *input_point;

		current_point = (Point) {
			int(tmp_point.x), int(tmp_point.y), int(tmp_point.z)
		};
	}

}