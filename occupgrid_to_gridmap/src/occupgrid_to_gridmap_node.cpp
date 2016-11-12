#include <ros/ros.h>
#include <occupgrid_to_gridmap/occupgrid_to_gridmap.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "occupgrid_to_gridmap");

    OcGridToGridMap ocGridToGridMap;

    ros::spin();

    return 0;
}
