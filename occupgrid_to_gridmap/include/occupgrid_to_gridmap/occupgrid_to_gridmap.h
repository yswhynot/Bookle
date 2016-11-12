#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

class OcGridToGridMap {
public:
	OcGridToGridMap();
	~OcGridToGridMap();
	void mapCallback(nav_msgs::OccupancyGrid::ConstPtr input);
private:
	ros::NodeHandle node_;
	ros::Publisher gridmap_pub_;
	ros::Subscriber occupgrid_sub_;
};