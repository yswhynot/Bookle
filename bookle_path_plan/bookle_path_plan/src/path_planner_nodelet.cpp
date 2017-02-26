#include <bookle_path_plan/path_planner.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace bookle {
	class PathPlannerNodelet : public nodelet::Nodelet {
	public:
		PathPlannerNodelet(){}

	private:
		void onInit() {
			detector_.reset(new PathPlan(getNodeHandle(), getPrivateNodeHandle()));
		}
		boost::shared_ptr<PathPlan> detector_;
	};
}

PLUGINLIB_DECLARE_CLASS(bookle_path_plan, PathPlannerNodelet, bookle::PathPlannerNodelet, nodelet::Nodelet);