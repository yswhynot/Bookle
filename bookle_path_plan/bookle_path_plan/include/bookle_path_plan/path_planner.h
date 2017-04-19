#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include <tf/transform_datatypes.h>

#include <vector>
#include <utility>
#include <limits>
#include <cmath>

#include <bookle_path_plan/graph_ros_interface.h>

namespace bookle {
	const int MAX_INT = 8888;

	class PathPlan {
	public:
		PathPlan(ros::NodeHandle& nh, ros::NodeHandle& pnh);
		~PathPlan();

	private:
		void MapCallback(const nav_msgs::OccupancyGrid::ConstPtr& input_map);
		void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& input_goal);
		void CurrentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& input_pose);

	// Utils
	private:
		long unsigned int getYawEnum(float yaw_f);
		float getYawFloat(int yaw_i);
		long unsigned int getPoseInt(float input_float);
		void getRPYFromQuaternion(float qx, float qy, float qz, float qw, float& r, float& p, float& y);
		void getQuaternionFromRPY(float r, float p, float y, float& qx, float& qy, float& qz, float& qw);
		void PublishPath();

	private:
		ros::Subscriber goal_sub_;
		ros::Subscriber map_sub_;
		ros::Subscriber current_sub_;
		ros::Publisher path_pub_;
		ros::Publisher pose_int_pub_;

		nav_msgs::OccupancyGrid gmap;
		bool gmap_init;

		GraphHandler gh;
		Point goal;
		Point start;
	};
}

#endif