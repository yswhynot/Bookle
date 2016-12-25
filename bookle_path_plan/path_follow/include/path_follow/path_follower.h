#ifndef PATH_FOLLOWER_H
#define PATH_FOLLOWER_H

#include <path_follow/path_util.h>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32MultiArray.h>

#include <vector>

namespace path_follow {
	struct PathPoint {
		int x;
		int y;
		int yaw;
		int distance;
		bool operator==(const PathPoint& a) const {
			return (x == a.x && y == a.y && yaw == a.yaw);
		}
	};

	class PathFollow {
	public:
		PathFollow(ros::NodeHandle& nh, ros::NodeHandle& pnh);
		~PathFollow();

	private:
		void EstPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& input_est_pose);
		void PlannedPathCallback(const nav_msgs::Path::ConstPtr& input_path);
		void YawCallback(const std_msgs::Float32MultiArray::ConstPtr& input_yaw);
		void ComputeDesPose();

	private:
		ros::Subscriber est_pos_sub_;
		ros::Subscriber yaw_sub_;
		ros::Subscriber path_sub_;
		ros::Publisher des_pos_pub_;

		geometry_msgs::PoseStamped est_pos;
		std::vector<PathPoint> des_path_array;
		float est_yaw;
		int yaw_int;

		bool isRandom;
	};
}

#endif