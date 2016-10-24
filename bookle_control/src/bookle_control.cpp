#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>

#include <gazebo/gazebo.hh>
// #include <gazebo/ModelState.hh>

#include "gazebo_msgs/SetModelState.h"
#include "gazebo_msgs/GetModelState.h" 
#include "gazebo_msgs/GetPhysicsProperties.h"
#include "gazebo_msgs/ModelState.h"

const float PI = 3.141593;

// struct Quaterniond {
// 	Quaterniond();
// 	double x, y, z, w;
// }

// Quaterniond toQuaternion(double pitch, double roll, double yaw)
// {
// 	Quaterniond q;
// 	double t0 = std::cos(yaw * 0.5f);
// 	double t1 = std::sin(yaw * 0.5f);
// 	double t2 = std::cos(roll * 0.5f);
// 	double t3 = std::sin(roll * 0.5f);
// 	double t4 = std::cos(pitch * 0.5f);
// 	double t5 = std::sin(pitch * 0.5f);

// 	q.w = t0 * t2 * t4 + t1 * t3 * t5;
// 	q.x = t0 * t3 * t4 - t1 * t2 * t5;
// 	q.y = t0 * t2 * t5 + t1 * t3 * t4;
// 	q.z = t1 * t2 * t4 - t0 * t3 * t5;
// 	return q;
// }

int main(int argc, char** argv)
{
	ros::init(argc, argv, "bookle_control");

	ros::NodeHandle* nh = new ros::NodeHandle();
	ros::Publisher state_pub = nh->advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1);
	ros::Publisher pose_pub = nh->advertise<geometry_msgs::PoseStamped>("/gazebo/ros_pose", 1);

	gazebo_msgs::ModelState ms;
	geometry_msgs::PoseStamped mpose;
	ms.model_name = "bookle";

	ms.pose.position.x = 0.75;
	ms.pose.position.y = -0.75;
	ms.pose.position.z = 0;

	ros::Rate loop_rate(10);
	
	double theta = 0;
	const float RADIUS = 1;

	mpose.pose.position.z = 0;

	unsigned int counter = 0;

	while (ros::ok()) {
		if(counter < 400) {
			ms.pose.position.y += 0.003;
			mpose.pose.position.y = ms.pose.position.y;
		}
		else
			break;

		counter++;

		// if(ms.pose.position.x < -3) {
		// 	theta += 0.0785;
			
		// 	ms.pose.position.x = -3 - sin(theta);
		// 	ms.pose.position.y = 1 - cos(theta);
		// 	mpose.pose.position.x = ms.pose.position.x;
		// 	mpose.pose.position.y = ms.pose.position.y;

		// 	ms.pose.orientation.z = -sin(theta / 2);	
		// 	ms.pose.orientation.w = cos(theta / 2);
		// } else {
		// 	ms.pose.position.x += -0.03;
		// }

		// if(ms.pose.position.y > 1)
		// 	break;

		state_pub.publish(ms);
		pose_pub.publish(mpose);
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
