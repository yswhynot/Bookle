#include <path_follow/path_follower.h>

#include <algorithm>

namespace path_follow {
	const int FLOOR_LENGTH = 5;

	inline bool operator<(const PathPoint &a, const PathPoint &b) {
		return a.distance < b.distance;
	}

	PathFollow::PathFollow(ros::NodeHandle& nh, ros::NodeHandle& pnh) : isRandom(true) {
		// subscriptions
		est_pos_sub_ = nh.subscribe("state_est_pos", 1, &PathFollow::EstPoseCallback, this);
		yaw_sub_ = nh.subscribe("tag_detections_yaw", 1, &PathFollow::YawCallback, this);
		path_sub_ = nh.subscribe("planned_path", 1, &PathFollow::PlannedPathCallback, this);

		// publications
		des_pos_pub_ = nh.advertise<geometry_msgs::PoseStamped>("des_pos", 1);

	}

	PathFollow::~PathFollow() {
		est_pos_sub_.shutdown();
		yaw_sub_.shutdown();
		path_sub_.shutdown();
	}

	void PathFollow::EstPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& input_est_pose) {
		est_pos = *input_est_pose;

		ComputeDesPose();
	}

	void PathFollow::PlannedPathCallback(const nav_msgs::Path::ConstPtr& input_path) {
		nav_msgs::Path path_array = *input_path;

		des_path_array.clear();

		for(int i=0; i < path_array.poses.size(); i++) {
			geometry_msgs::PoseStamped tmp_pos = path_array.poses[i];
			PathPoint tmp_point = {
				(int)round(tmp_pos.pose.position.x),
				(int)round(tmp_pos.pose.position.y),
				(int)round(tmp_pos.pose.position.z),
				0
			};
			des_path_array.push_back(tmp_point);
		}

		isRandom = false;
		ComputeDesPose();
	}
	void PathFollow::YawCallback(const std_msgs::Float32MultiArray::ConstPtr& input_yaw) {
		std_msgs::Float32MultiArray yaw_array = *input_yaw;
		for(int i=0; i < yaw_array.data.size(); i++) {
			if(yaw_array.data[i] != 0)
				est_yaw = yaw_array.data[i];
		}
		yaw_int = path_util::getYawEnum(est_yaw);
	}

	void PathFollow::ComputeDesPose() {
		if(isRandom)
			return;

		PathPoint current_point = {
			(int)round(est_pos.pose.position.x),
			(int)round(est_pos.pose.position.y),
			yaw_int,
			0
		};

		// delete the des point if arrived
		if(current_point == des_path_array.front()) {
			ROS_INFO("Arrived at x: %d, y: %d, yaw: %d", des_path_array.front().x, des_path_array.front().y, des_path_array.front().yaw);
			des_path_array.erase(des_path_array.begin());
		}

		// loop in nav path to get the nearest point
		std::vector<PathPoint> path_array_distance;
		for(int i=0; i < des_path_array.size(); i++) {
			PathPoint tmp_point = {
				des_path_array[i].x,
				des_path_array[i].y,
				des_path_array[i].yaw,
				0
			};
			tmp_point.distance = path_util::ManDistance(current_point.x, current_point.y, current_point.yaw, tmp_point.x, tmp_point.y, tmp_point.yaw);

			path_array_distance.push_back(tmp_point);
		}

		// sort path_array on smallest distance
		std::sort(path_array_distance.begin(), path_array_distance.end());

		geometry_msgs::PoseStamped des_pos;
		PathPoint des_point = path_array_distance.front();
		path_array_distance.clear();
		des_pos.pose.position.x = (float)des_point.x;
		des_pos.pose.position.y = (float)des_point.y;
		des_pos.pose.position.z = path_util::getYawFloat(des_point.yaw);
		
		// publish
		des_pos_pub_.publish(des_pos);

		ROS_INFO("Next pose at x: %f, y: %f, yaw: %f", (float)des_point.x, (float)des_point.y, (float)des_point.yaw);
	}
}