#include "bookle_path_plan/graph_ros_interface.h"

namespace bookle {
	GraphHandler::GraphHandler() : width(0), height(0), resolution(0.0) {}

	bool GraphHandler::UpdateGridGraph(nav_msgs::OccupancyGrid& input_map) {
		nav_msgs::OccupancyGrid map = *input_map;
		width = map.info.width;
		height = map.info.height;
		resolution = map.info.resolution;
		std::vector<int> data = map.data;

		bVertexSet barrier_set;

		for(int x = 0; x < height; x++) {
			for(int y = 0; y < width; y++) {
				if(data[height * x + y] > BARRIER_THRESHOLD) {
					// Update grid with all 4 dimentions
					for(int z = 0; z < Z_LENGTH; z++)
						barrier_set.insert(bVertexDescriptor {{x, y, z}});
				}
			} // end for y
		} // end for x

		if(grid_graph.UpdateBarrier(barrier_set))
			return true;
		
		return false;
	}

	bool GraphHandler::LoadPlannedPath(nav_msgs::Path& result_path) {
		std::vector<BookleVertex> tmp_path;
		nav_msgs::Path tmp_nav_path;
		grid_graph.getPlannedPath(tmp_path);

		// Parse vertex set if not emplty
		if(!tmp_path.empty()) {
			for(std::vector<BookleVertex>::iterator it = tmp_path.begin(); it = tmp_path.end(); it++) {
				geometry_msgs::PoseStamped g;
				g.pose.position.x = (float)it->x;
				g.pose.position.y = (float)it->y;
				g.pose.position.z = (float)it->z;
				tmp_nav_path.poses.pushback(g);
			}

			result_path = tmp_nav_path;
			return true;
		}

		return false;
	}

	void GraphHandler::UpdateGoal(int x, int y, int z) {
		grid_graph.UpdateGoal(bVertexDescriptor{{x, y, z}});
	}

	void GraphHandler::UpdateStart(int x, int y, int z) {
		grid_graph.UpdateStart(bVertexDescriptor{{x, y, z}});
	}
}