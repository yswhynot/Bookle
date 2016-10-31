#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pc_transform");

    PCTransform filter;

    ros::spin();

    return 0;
}


