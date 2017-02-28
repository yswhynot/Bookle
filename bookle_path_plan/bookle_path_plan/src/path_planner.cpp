#include <bookle_path_plan/path_planner.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>

#include <cmath>
#include <algorithm>

namespace bookle {

	PathPlan::PathPlan(ros::NodeHandle& nh, ros::NodeHandle& pnh) : goal((Point){0, 0, 0}), start((Point){0, 0, 0}) {
		// subscriptions
		goal_sub_ = nh.subscribe("/bookle/goal_pos", 1, &PathPlan::GoalCallback, this);
		map_sub_ = nh.subscribe("/map", 1, &PathPlan::MapCallback, this);

		// publications
		path_pub_ = nh.advertise<nav_msgs::Path>("bookle/planned_path", 1);
	}

	PathPlan::~PathPlan() {
		goal_sub_.shutdown();
		map_sub_.shutdown();
	}

	void PathPlan::MapCallback(const nav_msgs::OccupancyGrid::ConstPtr& input_map) {
		gmap = *input_map;

		// Load map into the GridGraph
		ROS_INFO("New map received!");
		gh.UpdateGridGraph(gmap);
	}

	void PathPlan::GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& input_goal) {
		geometry_msgs::PoseStamped tmp_pose = *input_goal;

		// Update goal
		float r, p, y;
		geometry_msgs::Quaternion q = tmp_pose.pose.orientation;
		getRPYFromQuaternion(q.x, q.y, q.z, q.w, r, p, y);
		ROS_INFO("Goal callback - r: %f, p: %f, y: %f", r, p, y);

		goal = (Point) {getPoseInt(tmp_pose.pose.position.x), getPoseInt(tmp_pose.pose.position.y), getYawEnum(y)};
		// ROS_INFO("Goal callback - x: %lu, y: %lu, theta: %lu", goal.x, goal.y, goal.theta);


		// Read TF msgs
		tf::StampedTransform transform;
		try{
			tf_listener.lookupTransform("/odom", "/base_footprint",  
				ros::Time(0), transform);
		}
		catch (tf::TransformException ex){
			ROS_ERROR("%s",ex.what());
		}

		// Set starting point from TF
		tf::Quaternion tf_q = transform.getRotation();
		getRPYFromQuaternion(tf_q.x(), tf_q.y(), tf_q.z(), tf_q.w(), r, p, y);
		start = (Point) {getPoseInt(transform.getOrigin().x() * 100 / 2), getPoseInt( (transform.getOrigin().y() + 1) * 100 / 2), getYawEnum(y)};
		ROS_INFO("Start tf - x: %lu, y: %lu, theta: %lu", start.x, start.y, start.theta);

		// Update goal and start
		gh.UpdateGoal(goal);
		gh.UpdateStart(start);
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