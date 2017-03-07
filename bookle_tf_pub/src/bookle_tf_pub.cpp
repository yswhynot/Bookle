#include <bookle_tf_pub/bookle_tf_pub.h>

namespace bookle {
	TFPub::TFPub(ros::NodeHandle& nh, ros::NodeHandle&pnh) {
		pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("bookle/current_pose", 1);

		
	}
}