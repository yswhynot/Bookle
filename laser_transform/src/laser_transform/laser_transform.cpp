#include "laser_transform/laser_transform.h"

void My_Filter::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
    sensor_msgs::PointCloud2 cloud;
    projector_.transformLaserScanToPointCloud("velodyne", *scan, cloud, tfListener_);
    point_cloud_publisher_.publish(cloud);
}


