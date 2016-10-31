#include <math.h>
#include <iostream>

#include <ros/ros.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <tf/transform_listener.h>
#include "pcl/common/eigen.h"
#include "pcl/common/transforms.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

class PCTransform {
public:
    PCTransform();
    void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& pc2);
private:
    ros::NodeHandle node_;
    tf::TransformListener tfListener_;

    ros::Publisher point_cloud_publisher_;
    ros::Subscriber point_cloud_sub_;

    // Eigen::Matrix4f rotMatrixX;
    // Eigen::Matrix4f rotMatrixY;
    // Eigen::Matrix4f rotMatrixZ;

    // Eigen::Transform<double, 3, Eigen::Affine> transform;
    Eigen::Matrix4f transform;
};

PCTransform::PCTransform(){
    point_cloud_sub_ = node_.subscribe<sensor_msgs::PointCloud2> ("/velodyne_points", 100, &PCTransform::pointcloudCallback, this);
    point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud2> ("/velodyne_points_trans", 100, false);
    tfListener_.setExtrapolationLimit(ros::Duration(0.1));

    float rotx = 0.0;
    float roty = M_PI / 2;
    // double roty = M_PI;
    // double rotz = -M_PI/2.0;
    float rotz = 0.0;

    Eigen::Matrix4f rotMatrixX;
    Eigen::Matrix4f rotMatrixY;
    Eigen::Matrix4f rotMatrixZ;

    rotMatrixX <<
    1.0, 0.0, 0.0, 0.0,
    0.0, cos(rotx), -sin(rotx), 0.0,
    0.0, sin(rotx), cos(rotx), 0.0,
    0.0, 0.0, 0.0, 1.0;

    rotMatrixY <<
    cos(roty), 0.0, sin(roty), 0.0,
    0.0, 1.0, 0.0, 0.0,
    -sin(roty), 0.0, cos(roty), 0.0,
    0.0, 0.0, 0.0, 1.0;

    rotMatrixZ <<
    cos(rotz), -sin(rotz), 0.0, 0.0,
    sin(rotz), cos(rotz), 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 1.0;

    transform = (rotMatrixX * rotMatrixY * rotMatrixZ);
    // transform = Eigen::Matrix4f::Identity();
    std::cout << "Transform: \n" << transform << std::endl;
}
