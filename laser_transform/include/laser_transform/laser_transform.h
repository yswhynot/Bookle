#include <ros/ros.h>
#include <Eigen/Dense>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include "pcl/common/eigen.h"
#include "pcl/common/transforms.h"

const double M_PI = 3.1415926;

class My_Filter {
public:
    My_Filter();
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
private:
    ros::NodeHandle node_;
    laser_geometry::LaserProjection projector_;
    tf::TransformListener tfListener_;

    ros::Publisher point_cloud_publisher_;
    ros::Subscriber scan_sub_;

    Eigen::Matrix4f rotMatrixX;
    Eigen::Matrix4f rotMatrixY;
    Eigen::Matrix4f rotMatrixZ;
};

My_Filter::My_Filter(){
    scan_sub_ = node_.subscribe<sensor_msgs::LaserScan> ("/bookle/laser/scan", 100, &My_Filter::scanCallback, this);
    point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud2> ("/velodyne_points", 100, false);
    tfListener_.setExtrapolationLimit(ros::Duration(0.1));

    pcl::PointCloud<pcl::pointxyzrgb> transformed_cloud;
    double rotx = 0.0;
    double roty = M_PI/2.0;
    double rotz = -M_PI/2.0;

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
}