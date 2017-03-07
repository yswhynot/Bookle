#include <bookle_tf_pub/bookle_tf_pub.h>
#include <ros/ros.h>

int main(int argc, char **argv) {
	ros::init(argc, argv, "bookle_tf_pub");
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");
	bookle::TFPub tf_pub(nh, pnh);
	
	ros::Rate loop_rate(10);
	while(ros::ok()) {
		tf_pub.PublishCurrentPose();
		ros::spinOnce();
		loop_rate.sleep();
	}
}