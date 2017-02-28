#include "bookle_path_plan/graph_ros_interface.h"

namespace bookle {

	bool GraphHandler::UpdateGridGraph(const nav_msgs::OccupancyGrid& input_map) {
		nav_msgs::OccupancyGrid map = input_map;
		width = map.info.width;
		height = map.info.height;
		resolution = map.info.resolution;
		// std::vector<int> data = map.data;

		bVertexSet barrier_set;

		for(long unsigned int x = 0; x < height; x++) {
			for(long unsigned int y = 0; y < width; y++) {
				if(map.data[height * x + y] > BARRIER_THRESHOLD) {
					// Update grid with all 4 dimentions
					for(long unsigned int z = 0; z < Z_LENGTH; z++)
						barrier_set.insert(bVertexDescriptor {{x, y, z}});
				}
			} // end for y
		} // end for x

		if(grid_graph.UpdateBarrier(barrier_set)) {
			ROS_INFO("Update success!");
			return true;
		}
		
		return false;
	}

	bool GraphHandler::LoadPlannedPath(nav_msgs::Path& result_path) {
		std::vector<BookleVertex> tmp_path;
		nav_msgs::Path tmp_nav_path;
		grid_graph.getPlannedPath(tmp_path);

		// Parse vertex set if not empty
		if(!tmp_path.empty()) {
			for(std::vector<BookleVertex>::iterator it = tmp_path.begin(); it != tmp_path.end(); it++) {
				geometry_msgs::PoseStamped g;
				g.pose.position.x = (float)it->x;
				g.pose.position.y = (float)it->y;
				g.pose.position.z = (float)it->z;
				tmp_nav_path.poses.push_back(g);
			}

			result_path = tmp_nav_path;
			return true;
		}

		return false;
	}

	void GraphHandler::UpdateGoal(Point& input_point) {
		grid_graph.UpdateGoal(input_point.x, input_point.y, input_point.theta);
	}

	void GraphHandler::UpdateStart(Point& input_point) {
		grid_graph.UpdateStart(input_point.x, input_point.y, input_point.theta);
	}
}