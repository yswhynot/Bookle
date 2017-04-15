#include <bookle_path_follow/path_follower.h>

#include <algorithm>

namespace bookle {

	PathFollow::PathFollow(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
		// subscriptions
		path_sub_ = nh.subscribe("bookle/planned_path", 1, &PathFollow::PlannedPathCallback, this);
		pose_sub_ = nh.subscribe("bookle/current_pose/int", 1, &PathFollow::CurrentPointCallback, this);

		// publications
		target_point_pub_ = nh.advertise<geometry_msgs::Point>("bookle/target_pose", 1);
		// current_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("bookle/current_pose", 1);

	}

	PathFollow::~PathFollow() {
		path_sub_.shutdown();
		pose_sub_.shutdown();
	}

	void PathFollow::PlannedPathCallback(const nav_msgs::Path::ConstPtr& input_path) {
		// ROS_INFO("Path callback\n");
		nav_msgs::Path nav_path = *input_path;

		std::vector<Point> tmp_vec;
		for(int i = 0; i < nav_path.poses.size(); i++) {
			geometry_msgs::PoseStamped tmp_pos = nav_path.poses[i];
			Point tmp_point = {
				int(tmp_pos.pose.position.x),
				int(tmp_pos.pose.position.y),
				int(tmp_pos.pose.position.z)
			};
			tmp_vec.push_back(tmp_point);
		}
		v_path.clear();
		v_path = tmp_vec;
	}

	void PathFollow::CurrentPointCallback(const geometry_msgs::Point::ConstPtr& input_point) {
		// ROS_INFO("Current point callback\n");
		geometry_msgs::Point tmp_point = *input_point;

		current_point = (Point) {
			int(tmp_point.x), int(tmp_point.y), int(tmp_point.z)
		};

		Point next_p;
		if(getNextPoint(current_point, next_p) == -1)
			return;

		geometry_msgs::Point p;
		p.x = float(next_p.x);
		p.y = float(next_p.y);
		p.z = float(next_p.theta);
		ROS_INFO("Next point: %.2f, %.2f, %.2f\n", p.x, p.y, p.z);
		target_point_pub_.publish(p);
	}

	int PathFollow::getNextPoint(Point& input, Point& result) {
		if(v_path.empty())
			return -1;

		// find next point in the array
		std::vector<Point>::iterator it;
		it = std::find_if(v_path.begin(), v_path.end(), 
			[&input] (const Point& p) {
				return input == p;
			});
		if(it == v_path.begin()) {
			ROS_INFO("Reach target!\n");
			result = *it;
		} else if(it != v_path.end()) {
			ROS_INFO("Point found!\n");
			it --;
			result = *it;
		} else {
			ROS_INFO("Point: %d, %d, %d not found!\n", input.x, input.y, input.theta);
			result = v_path.back();
		}

		return 0;
	}

}