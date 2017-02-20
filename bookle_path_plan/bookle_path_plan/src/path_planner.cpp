#include <bookle_path_plan/path_planner.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>

#include <cmath>
#include <algorithm>

namespace bookle {

	PathPlan::PathPlan(ros::NodeHandle& nh, ros::NodeHandle& pnh) : yaw_int(-1), target_id(-1), last_planned_target(-1) {
		// subscriptions
		goal_sub_ = nh.subscribe("bookle/goal_pos", 1, &PathPlan::GoalCallback, this);

		// publications
		path_pub_ = nh.advertise<nav_msgs::Path>("bookle/planned_path", 1);
	}

	PathPlan::~PathPlan() {
		goal_sub_.shutdown();
	}

	void PathPlan::MapCallback(const nav_msgs::OccupancyGrid::ConstPtr& input_map) {
		gmap = *input_map;

		// Load map into the GridGraph
		gh.UpdateGridGraph(gmap);
	}

	void PathPlan::GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& input_goal) {
		geometry_msgs::PoseStamped tmp_pose = *input_goal;

		// Update goal
		float r, p, y;
		geometry_msgs::Quaternion q = tmp_pose.pose.orientation;
		getRPYFromQuaternion(q.x, q.y, q.z, q.w, r, p, y);
		goal = (Point) {getPoseInt(tmp_pose.pose.position.x), getPoseInt(tmp_pose.pose.position.y), getYawEnum(y)};

		// Read TF msgs
		tf::StampedTransform transform;
		try{
			tf_listener.lookupTransform("/map", "/base_footprint",  
				ros::Time(0), transform);
		}
		catch (tf::TransformException ex){
			ROS_ERROR("%s",ex.what());
		}

		// Set starting point from TF
		tf::Quaternion tf_q = transform.getRotation();
		getRPYFromQuaternion(tf_q.x, tf_q.y, tf_q.z, tf_q.w, r, p, y);
		start = (Point) {getPoseInt(transform.getOrigin().x()), getPoseInt(transform.getOrigin().y()), getYawEnum(y)};

	}

	void PathPlan::UpdateStart(Point& input_p) { start = input_p; }

	void PathPlan::UpdateGoal(Point& input_p) { goal = input_p; }

	int PathPlan::getYawEnum(float yaw_f) {
		if(yaw_f ==  -1) 
			return -1;

		int tmp = (int)round((yaw_f + 3.141592) / 1.570796);
		if(tmp > -1 && tmp < 4)
			return tmp;

		return -1;
	}

	float PathPlan::getYawFloat(int yaw_i) {
		return (float)yaw_i * 1.570796 - 3.141592;
	}

	int PathPlan::getPoseInt(float input_float) {
		// When input is directly mapped to int 
		return (int)input_float;
	}

	void PathPlan::getRPYFromQuaternion(float qx, float qy, float qz, float qw, float& r, float& p, float& y) {
		tf::Quaternion q(qx, qy, qz, qw);
		tf::Matrix3x3 m(q);
		m.getRPY(r, p, y);
	}

	void PathPlan::getQuaternionFromRPY(float r, float p, float y, float& qx, float& qy, float& qz, float& qw) {
		tf::Matrix3x3 m;
		m.setRPY(r, p, y);
		
		tf::Quaternion q;
		m.getRotation(q);
		qx = q.x();
		qy = q.y();
		qz = q.z();
		qw = q.w();
	}
}