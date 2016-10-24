#include <ros/ros.h>
#include "laser_transform/laser_transform.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "laser_transform");

    My_Filter filter;

    ros::spin();

    return 0;
}


