#include "bookle_path_plan/graph_ros_interface.h"

namespace bookle {
	GraphHandler::GraphHandler() : width(0), height(0), resolution(0.0) {}

	bool GraphHandler::UpdateGridGraph(nav_msgs::OccupancyGrid& input_map) {
		nav_msgs::OccupancyGrid map = *input_map;
		width = map.info.width;
		height = map.info.height;
		resolution = map.info.resolution;

		bVertexSet barrier_set;

		for(size_t i = 0; i < height; i++) {
			for(size_t j = 0; j < width; j++) {
				if((height * i + width) > BARRIER_THRESHOLD) {
					// TODO: insert barrier set and add filtered map in GridGraph
					bVertexDescriptor tmp_des()
					barrier_set.insert();
				}
			}
		}

		return true;
	}
}