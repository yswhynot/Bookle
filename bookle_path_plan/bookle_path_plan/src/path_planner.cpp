#include <bookle_path_plan/path_planner.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>

#include <cmath>
#include <algorithm>

namespace bookle {

	PathPlan::PathPlan(ros::NodeHandle& nh, ros::NodeHandle& pnh) : goal((Point){0, 0, 0}), start((Point){0, 0, 0}), gmap_init(false) {
		// subscriptions
		goal_sub_ = nh.subscribe("/bookle/goal_pos", 1, &PathPlan::GoalCallback, this);
		map_sub_ = nh.subscribe("/map", 1, &PathPlan::MapCallback, this);
		current_sub_ = nh.subscribe("/bookle/current_pose", 1, &PathPlan::CurrentPoseCallback, this);

		// publications
		path_pub_ = nh.advertise<nav_msgs::Path>("bookle/planned_path", 1);
		pose_int_pub_ = nh.advertise<geometry_msgs::Point>("bookle/current_pose/int", 1);
	}

	PathPlan::~PathPlan() {
		goal_sub_.shutdown();
		map_sub_.shutdown();
	}

	void PathPlan::MapCallback(const nav_msgs::OccupancyGrid::ConstPtr& input_map) {
		gmap = *input_map;
		gmap_init = true;

		// Load map into the GridGraph
		ROS_INFO("New map received!");
		gh.UpdateGridGraph(gmap);

		PublishPath();
	}

	void PathPlan::CurrentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& input_pose) {
		geometry_msgs::PoseStamped tmp_pose = *input_pose;

		float r, p, y;
		getRPYFromQuaternion(
			tmp_pose.pose.orientation.x, 
			tmp_pose.pose.orientation.y,
			tmp_pose.pose.orientation.z,
			tmp_pose.pose.orientation.w,
			r, p, y);

		start = (Point) {
			getPoseInt((1 - tmp_pose.pose.position.x) * 100 / 2), 
			getPoseInt((tmp_pose.pose.position.y + 1) * 100 / 2), 
			getYawEnum(y)};
		gh.UpdateStart(start);


		geometry_msgs::Point tmp_point;
		tmp_point.x = start.x;
		tmp_point.y = start.y;
		tmp_point.z = start.theta;

		pose_int_pub_.publish(tmp_point);
	}

	void PathPlan::PublishPath() {
		// Check if everything init
		if((goal == Point{0, 0, 0}) || (start == Point{0, 0, 0}) || !gmap_init)
			return;

		// Load planned path
		nav_msgs::Path result_path;
		gh.LoadPlannedPath(result_path);
		path_pub_.publish(result_path);
	}

	void PathPlan::GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& input_goal) {
		geometry_msgs::PoseStamped tmp_pose = *input_goal;

		// Update goal
		float r, p, y;
		geometry_msgs::Quaternion q = tmp_pose.pose.orientation;
		getRPYFromQuaternion(q.x, q.y, q.z, q.w, r, p, y);
		// ROS_INFO("Goal callback - r: %f, p: %f, y: %f", r, p, y);

		goal = (Point) {getPoseInt(tmp_pose.pose.position.x), getPoseInt(tmp_pose.pose.position.y), getYawEnum(y)};
		// ROS_INFO("Goal callback - x: %lu, y: %lu, theta: %lu", goal.x, goal.y, goal.theta);

		// Update goal and start
		gh.UpdateGoal(goal);
		ROS_INFO("Start at: %lu, %lu, %lu", start.x, start.y, start.theta);

		PublishPath();
	}

	long unsigned int PathPlan::getYawEnum(float yaw_f) {
		if(yaw_f < -4) 
			return 888;

		long unsigned int tmp = std::lround((yaw_f + 3.141592) / 1.570796);
		if(tmp < 4)
			return tmp;

		return 999;
	}

	float PathPlan::getYawFloat(int yaw_i) {
		return (float)yaw_i * 1.570796 - 3.141592;
	}

	long unsigned int PathPlan::getPoseInt(float input_float) {
		// ROS_INFO("Pose float: %f", input_float);
		// When input is directly mapped to int 
		return std::lround(input_float);
	}

	void PathPlan::getRPYFromQuaternion(float qx, float qy, float qz, float qw, float& r, float& p, float& y) {
		tf::Quaternion q(qx, qy, qz, qw);
		tf::Matrix3x3 m(q);

		tfScalar rt, pt, yt;
		m.getRPY(rt, pt, yt);
		r = rt;
		p = pt;
		y = yt;
	}

	void PathPlan::getQuaternionFromRPY(float r, float p, float y, float& qx, float& qy, float& qz, float& qw) {
		tf::Matrix3x3 m;

		tfScalar rt = r, pt = p, yt = y;
		m.setRPY(rt, pt, yt);
		
		tf::Quaternion q;
		m.getRotation(q);
		qx = q.x();
		qy = q.y();
		qz = q.z();
		qw = q.w();
	}
}