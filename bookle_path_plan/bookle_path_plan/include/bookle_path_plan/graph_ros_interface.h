#ifndef GRAPH_ROS_INTERFACE_H
#define GRAPH_ROS_INTERFACE_H

#include "bookle_path_plan/boost_grid_graph.h"

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>

namespace bookle {
	struct GraphHandler {
		GraphHandler();
		~GraphHandler();
		UpdateGridGraph(nav_msgs::OccupancyGrid& input_map);

		GridGraph grid_graph;

	};
}

#endif