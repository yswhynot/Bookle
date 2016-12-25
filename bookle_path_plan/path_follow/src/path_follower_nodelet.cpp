#include <path_follow/path_follower.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace path_follow {
	class PathFollowerNodelet : public nodelet::Nodelet {
	public:
		PathFollowerNodelet(){}

	private:
		void onInit() {
			detector_.reset(new PathFollow(getNodeHandle(), getPrivateNodeHandle()));
		}
		boost::shared_ptr<PathFollow> detector_;
	};
}

PLUGINLIB_DECLARE_CLASS(path_follow, PathFollowerNodelet, path_follow::PathFollowerNodelet, nodelet::Nodelet);