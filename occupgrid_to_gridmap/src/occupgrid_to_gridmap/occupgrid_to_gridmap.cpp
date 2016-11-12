#include "occupgrid_to_gridmap/occupgrid_to_gridmap.h"

OcGridToGridMap::OccupancyGrid() {
	occupgrid_sub_ = node_.subscribe<nav_msgs::OccupancyGrid> ("/map", 100, &OcGridToGridMap::mapCallback, this);
	gridmap_pub_ = node_.advertise<grid_map_msgs::GridMap> ("/grid_map", 100, false);
}

void OcGridToGridMap::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& input) {
	grid_map::GridMap gm;
	if(GridMapRosConverter::fromOccupancyGrid(*input, "", gm))
		gridmap_pub_.publish(gm);
}