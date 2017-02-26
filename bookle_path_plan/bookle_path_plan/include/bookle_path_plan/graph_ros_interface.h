#ifndef GRAPH_ROS_INTERFACE_H
#define GRAPH_ROS_INTERFACE_H

#include "bookle_path_plan/boost_grid_graph.h"

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>

namespace bookle {
	const int BARRIER_THRESHOLD = 50;

	struct GraphHandler {
		GraphHandler();
		~GraphHandler();
		bool UpdateGridGraph(nav_msgs::OccupancyGrid& input_map);
		bool LoadPlannedPath(nav_msgs::Path& result_path);
		void UpdateGoal(int x, int y, int z);
		void UpdateStart(int x, int y, int z);

		GridGraph grid_graph;
		int width;
		int height;
		float resolution;

	};
}

#endif