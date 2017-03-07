#ifndef PATH_FOLLOWER_H
#define PATH_FOLLOWER_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32MultiArray.h>

#include <vector>

namespace bookle {
	struct iPoint {
		int x, y, theta;
	};
	struct fPoint {
		float x, y, theta;
	}

	class PathFollow {
	public:
		PathFollow(ros::NodeHandle& nh, ros::NodeHandle& pnh);
		~PathFollow();

	private:
		void PlannedPathCallback(nav_msgs::Path::ConstPtr input_path);

	private:
		ros::Subscriber path_sub_;
		ros::Publisher target_pose_pub_;
		ros::Publisher current_pose_pub_;

		nav_msgs::Path path;

	};
}

#endif