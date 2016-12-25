#include <path_plan/path_planner.h>
#include <ros/ros.h>

int main(int argc, char **argv) {
	ros::init(argc, argv, "path_planner");
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");
	path_plan::PathPlan detector(nh, pnh);
	ros::spin();
}