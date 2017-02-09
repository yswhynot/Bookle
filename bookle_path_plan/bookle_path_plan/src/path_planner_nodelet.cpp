#include <bookle_path_plan/path_planner.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace path_plan {
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

PLUGINLIB_DECLARE_CLASS(path_plan, PathPlannerNodelet, path_plan::PathPlannerNodelet, nodelet::Nodelet);