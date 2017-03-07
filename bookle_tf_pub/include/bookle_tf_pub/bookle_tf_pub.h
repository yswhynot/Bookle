#ifndef TF_PUB_H
#define TF_PUB_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

namespace bookle {
	
	class TFPub {
	public:
		TFPub(ros::NodeHandle& nh, ros::NodeHandle&pnh);
		~TFPub();

	private:
		void PublishCurrentPose();

	private:
		ros::Publisher pose_pub_;
		tf::TransformListener tf_listener;
		tf::StampedTransform transform;
		geometry_msgs::PoseStamped pose;
	}
}

#endif