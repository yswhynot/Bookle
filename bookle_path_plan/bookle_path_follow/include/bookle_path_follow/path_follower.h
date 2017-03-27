#ifndef PATH_FOLLOWER_H
#define PATH_FOLLOWER_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>

#include <vector>

namespace bookle {
	struct Point {
		int x, y, theta;
	};

	class PathFollow {
	public:
		PathFollow(ros::NodeHandle& nh, ros::NodeHandle& pnh);
		~PathFollow();

	private:
		void PlannedPathCallback(const nav_msgs::Path::ConstPtr& input_path);
		void CurrentPointCallback(const geometry_msgs::Point::ConstPtr& input_pose);
		void getNextPoint(Point& result);
		void Point2Pose(geometry_msgs::PoseStamped result);

	private:
		ros::Subscriber path_sub_;
		ros::Subscriber pose_sub_;

		// TODO: confirm interface with hardware
		ros::Publisher target_pose_pub_;
		// ros::Publisher current_pose_pub_;

		nav_msgs::Path nav_path;
		std::vector<Point> v_path;
		Point current_point;
		Point prev_point;
	};
}

#endif