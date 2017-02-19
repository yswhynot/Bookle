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

		for(size_t h = 0; h < height; h++) {
			for(size_t w = 0; w < width; w++) {
				if(data[height * h + w] > BARRIER_THRESHOLD) {
					// TODO: insert barrier set and add filtered map in GridGraph

					bVertexDescriptor tmp_des()
					barrier_set.insert();
				}
			}
		}

		return true;
	}
}