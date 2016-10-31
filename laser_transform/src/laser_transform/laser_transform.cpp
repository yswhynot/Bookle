#include "laser_transform/laser_transform.h"

void My_Filter::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
	sensor_msgs::PointCloud2 cloud;
	pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
	pcl::PCLPointCloud2 pcl_pc2;
	
	pcl::PointCloud<pcl::PointXYZ> trans_cloud;
	pcl::PCLPointCloud2 trans_pcl_pc2;
	sensor_msgs::PointCloud2 trans_pc2;

	projector_.transformLaserScanToPointCloud("velodyne", *scan, cloud, tfListener_);
    pcl_conversions::toPCL(cloud, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, pcl_cloud);

	// pcl::fromROSMsg(cloud, pcl_cloud);
	pcl::transformPointCloud(pcl_cloud, trans_cloud, transform);
	// pcl::toROSMsg(trans_cloud, point_cloud_msg);
	
    pcl::toPCLPointCloud2(trans_cloud, trans_pcl_pc2);
    pcl_conversions::fromPCL(trans_pcl_pc2, trans_pc2);
	point_cloud_publisher_.publish(trans_pc2);
}

void My_Filter::pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& pc) {
	sensor_msgs::PointCloud2 input = *pc;
	sensor_msgs::PointCloud2 output;
	pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
	pcl::PCLPointCloud2 pcl_pc2;

	pcl_conversions::toPCL(input, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, pcl_cloud);
    pcl::toPCLPointCloud2(pcl_cloud, pcl_pc2);
    pcl_conversions::fromPCL(pcl_pc2, output);
	point_cloud_trans_pub_.publish(output);
}

