#include <bookle_path_follow/path_follower.h>
#include <ros/ros.h>

int main(int argc, char **argv) {
	ros::init(argc, argv, "path_follower");
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");
	bookle::PathFollow detector(nh, pnh);

	ros::spin();

	// while(ros::ok()) {
		// ros::spinOnce();
	// }

}