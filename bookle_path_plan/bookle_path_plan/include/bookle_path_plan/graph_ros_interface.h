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

	struct Point {
		long unsigned int x, y, theta;
		bool operator==(const Point& p) {
			return (std::tie(this->x, this->y, this->theta) == std::tie(p.x, p.y, p.theta));
		}
	};

	struct GraphHandler {
		GraphHandler() : width(0), height(0), resolution(0.0) {}
		bool UpdateGridGraph(const nav_msgs::OccupancyGrid& input_map);
		bool LoadPlannedPath(nav_msgs::Path& result_path);
		void UpdateGoal(Point& input_point);
		void UpdateStart(Point& input_point);

		GridGraph grid_graph;
		int width;
		int height;
		float resolution;

	};
}

#endif