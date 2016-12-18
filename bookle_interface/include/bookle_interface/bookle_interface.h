#ifndef BOOKLE_INTERFACE_H
#define BOOKLE_INTERFACE_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/GetModelState.h>

namespace bookle {

	class BookleInterface {
	public:
		BookleInterface(ros::NodeHandle& nh);
		~BookleInterface();
		void start();

	private:
		void StateCallback(const gazebo_msgs::ModelState::ConstPtr& input_state);

	private:
		ros::Publisher gmapping_pt_pub_;
		ros::Publisher gazebo_pt_pub_;
		// ros::Subscriber gazebo_state_sub_;

		tf::TransformListener tf_listener;
		tf::StampedTransform tf_transform;

		gazebo_msgs::GetModelState gms;
		ros::ServiceClient gms_c;

		geometry_msgs::Point est_point;
		geometry_msgs::Point gazebo_point;
	};
}

#endif