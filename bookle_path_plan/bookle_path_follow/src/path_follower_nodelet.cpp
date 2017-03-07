#include <bookle_path_follow/path_follower.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace bookle {
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

PLUGINLIB_DECLARE_CLASS(bookle_path_follow, PathFollowerNodelet, bookle::PathFollowerNodelet, nodelet::Nodelet);