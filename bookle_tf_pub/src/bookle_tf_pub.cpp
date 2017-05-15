#include <bookle_tf_pub/bookle_tf_pub.h>

namespace bookle {
	TFPub::TFPub(ros::NodeHandle& nh, ros::NodeHandle&pnh) {
		pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("bookle/current_pose", 1);
	}

	TFPub::~TFPub() {}

	void TFPub::PublishCurrentPose() {
		try{
			tf_listener.lookupTransform("/map", "/base_footprint",  
				ros::Time(0), transform);
		}
		catch (tf::TransformException ex){
			ROS_ERROR("%s", ex.what());
		}

		tf::transformStampedTFToMsg(transform, trans_stamp);
		pose.pose.position.x = trans_stamp.transform.translation.x;
		pose.pose.position.y = trans_stamp.transform.translation.y;
		pose.pose.position.z = trans_stamp.transform.translation.z;
		pose.pose.orientation = trans_stamp.transform.rotation;

		pose_pub_.publish(pose);
	}
}