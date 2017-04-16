#ifndef PATH_FOLLOWER_H
#define PATH_FOLLOWER_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>

#include <vector>
#include <tuple>
#include <algorithm>

namespace bookle {
	struct Point {
		int x, y, theta;
		bool operator==(const Point& p) {
			return (std::tie(this->x, this->y, this->theta) == std::tie(p.x, p.y, p.theta));
		}
	};

	class PathFollow {
	public:
		PathFollow(ros::NodeHandle& nh, ros::NodeHandle& pnh);
		~PathFollow();

	private:
		void PlannedPathCallback(const nav_msgs::Path::ConstPtr& input_path);
		void CurrentPointCallback(const geometry_msgs::Point::ConstPtr& input_pose);
		int getNextPoint(Point& input, Point& result);
		float getYawFloat(int yaw_i);

	private:
		ros::Subscriber path_sub_;
		ros::Subscriber pose_sub_;

		// TODO: confirm interface with hardware
		ros::Publisher target_point_pub_;
		// ros::Publisher current_pose_pub_;

		std::vector<Point> v_path;
		Point current_point;
		Point prev_target;
	};
}

#endif