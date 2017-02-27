#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <vector>
#include <utility>
#include <limits>
#include <cmath>

#include <bookle_path_plan/graph_ros_interface.h>

namespace bookle {
	const int MAX_INT = 8888;

	enum Direction {BACK, RIGHT, FRONT, LEFT};

	struct Point {
		long unsigned int x, y, theta;
	};

	class PathPlan {
	public:
		PathPlan(ros::NodeHandle& nh, ros::NodeHandle& pnh);
		~PathPlan();

	private:
		void MapCallback(const nav_msgs::OccupancyGrid::ConstPtr& input_map);
		void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& input_goal);

	// Utils
	private:
		void UpdateStart(Point& input_p);
		void UpdateGoal(Point& input_p);
		long unsigned int getYawEnum(float yaw_f);
		float getYawFloat(int yaw_i);
		long unsigned int getPoseInt(float input_float);
		void getRPYFromQuaternion(float qx, float qy, float qz, float qw, float& r, float& p, float& y);
		void getQuaternionFromRPY(float r, float p, float y, float& qx, float& qy, float& qz, float& qw);

	private:
		ros::Subscriber goal_sub_;
		ros::Publisher path_pub_;
		tf::TransformListener tf_listener;

		nav_msgs::OccupancyGrid gmap;

		GraphHandler gh;
		Point goal;
		Point start;
	};
}

#endif