#ifndef ZBAR_BRIDGE_H
#define ZBAR_BRIDGE_H

#include <ros/ros.h>
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>

#include <string>
#include "zbar.h"

namespace bookle {
	class BarcodeReader {
	public:
		BarcodeReader(ros::NodeHandle& nh, ros::NodeHandle& pnh);
		~BarcodeReader();

	private:
		void ImageCallback(const sensor_msgs::ImageConstPtr &input);

	private:
		ros::Subscriber cam_sub_;
		ros::Publisher barcode_pub_;

		zbar::ImageScanner scanner;
	};
}

#endif