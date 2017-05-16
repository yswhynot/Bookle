#include "bookle_path_plan/graph_ros_interface.h"

namespace bookle {

	bool GraphHandler::UpdateGridGraph(const nav_msgs::OccupancyGrid& input_map) {
		nav_msgs::OccupancyGrid map = input_map;
		width = map.info.width;
		height = map.info.height;
		resolution = map.info.resolution;
		// std::vector<int> data = map.data;
		printf("resolution: %f\n", resolution);
		printf("origin: (%f, %f) (%f, %f, %f, %f)\n", map.info.origin.position.x, map.info.origin.position.y, map.info.origin.orientation.x, map.info.origin.orientation.y, map.info.origin.orientation.z, map.info.origin.orientation.w);

		bVertexSet barrier_set;

		std::vector<Point> point_vec;
		long unsigned int x_offset = (long unsigned int)(-map.info.origin.position.x / resolution);
		long unsigned int y_offset = (long unsigned int)(-map.info.origin.position.y / resolution);
		printf("offset: %lu, %lu\n", x_offset, y_offset);
		const float FILTER_THRESHOLD = 0.3;

		ROS_INFO("\nUpdate barrier of width: %d, height: %d \n", width, height);
		for(long unsigned int x = 0; x < width; x++) {
			for(long unsigned int y = 0; y < height; y++) {
				if(map.data[width * x + y] > BARRIER_THRESHOLD) {
					// Update grid with all 4 dimentions
					// printf("(%lu, %lu) ", x, y);

					point_vec.push_back(Point{x, y, 0});
				}
			} // end for y
		} // end for x

		// Filter for the physical shit
		for(std::vector<Point>::iterator it = point_vec.begin(); it != point_vec.end(); it++) {
			printf("(%lu, %lu) ", it->x, it->y);
			// if((it->x < 100) || (it->y < 100)) continue;
			for(long unsigned int z = 0; z < Z_LENGTH; z++) {
				// barrier_set.insert(bVertexDescriptor {{it->x, it->y, z}});
				// barrier_set.insert(bVertexDescriptor {{it->x - x_min, it->y - y_min, z}});
				// barrier_set.insert(bVertexDescriptor {{it->x - 100, it->y - 100, z}});
				barrier_set.insert(bVertexDescriptor {{(long unsigned int)(resolution * (float)(it->x - y_offset - y_offset) * 50), (long unsigned int)(resolution * (float)(it->y - x_offset + 60) * 50), z}});
			}
		}

		if(grid_graph.UpdateBarrier(barrier_set)) {
			return true;
		}
		
		return false;
	}

	bool GraphHandler::LoadPlannedPath(nav_msgs::Path& result_path) {
		grid_graph.AStarSearch();
		std::vector<BookleVertex> tmp_path;
		nav_msgs::Path tmp_nav_path;
		grid_graph.getPlannedPath(tmp_path);

		// Parse vertex set if not empty
		if(!tmp_path.empty()) {
			ROS_INFO("Planned Path:");
			for(std::vector<BookleVertex>::iterator it = tmp_path.begin(); it != tmp_path.end(); it++) {
				geometry_msgs::PoseStamped g;
				g.pose.position.x = (float)it->x;
				g.pose.position.y = (float)it->y;
				g.pose.position.z = (float)it->z;
				tmp_nav_path.poses.push_back(g);
				printf("(%.1f %.1f %.1f)\n", g.pose.position.x, g.pose.position.y, g.pose.position.z);
			}
			printf("\n\n-------------------------------\n\n");

			result_path = tmp_nav_path;
			return true;
		}

		return false;
	}

	void GraphHandler::UpdateGoal(Point& input_point) {
		grid_graph.UpdateGoal(input_point.x, input_point.y, input_point.theta);
	}

	void GraphHandler::UpdateStart(Point& input_point) {
		// ROS_INFO("Interface start");
		grid_graph.UpdateStart(input_point.x, input_point.y, input_point.theta);
	}
}