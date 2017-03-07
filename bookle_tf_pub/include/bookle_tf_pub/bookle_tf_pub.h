#ifndef TF_PUB_H
#define TF_PUB_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

namespace bookle {
	
	class TFPub {
	public:
		TFPub(ros::NodeHandle& nh, ros::NodeHandle&pnh);
		~TFPub();

	public:
		void PublishCurrentPose();

	private:
		ros::Publisher pose_pub_;
		tf::TransformListener tf_listener;
		tf::StampedTransform transform;
		geometry_msgs::TransformStamped trans_stamp;
		geometry_msgs::PoseStamped pose;
	};
}

#endif