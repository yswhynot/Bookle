#include "laser_transform/laser_transform.h"

void My_Filter::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
	sensor_msgs::PointCloud2 cloud;
	pcl::PointCloud<pcl::pointxyz> pcl_cloud;
	sensor_msgs::PointCloud2 point_cloud_msg;
	pcl::PointCloud<pcl::pointxyz> trans_cloud;

	projector_.transformLaserScanToPointCloud("velodyne", *scan, cloud, tfListener_);

	pcl::fromROSMsg(cloud, pcl_cloud);
	pcl::transformPointCloud(pcl_cloud, trans_cloud, rotMatrixX * rotMatrixY * rotMatrixZ);
	pcl::toROSMsg(trans_cloud, point_cloud_msg);
	
	point_cloud_publisher_.publish(point_cloud_msg);
}


