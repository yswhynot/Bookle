#include <ros/ros.h>
#include <zbar_bridge/zbar_bridge.h>

int main(int argc, char **argv) {
	ros::init(argc, argv, "zbar_bridge");
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");
	bookle::BarcodeReader reader(nh, pnh);

	ros::spin();
}