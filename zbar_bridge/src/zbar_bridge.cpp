#include <zbar_bridge/zbar_bridge.h>

namespace bookle {
	BarcodeReader::BarcodeReader(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
		scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);

		barcode_pub_ = nh.advertise<std_msgs::String>("bookle/barcode", 1);
		cam_sub_ = nh.subscribe("/camera/image", 10, &BarcodeReader::ImageCallback, this);
	}

	BarcodeReader::~BarcodeReader() {
		barcode_pub_.shutdown();
	}

	void BarcodeReader::ImageCallback(const sensor_msgs::ImageConstPtr &input) {
		cv_bridge::CvImageConstPtr cv_image;
		cv_image = cv_bridge::toCvShare(input, "mono16");


		zbar::Image zbar_image(cv_image->image.cols, cv_image->image.rows, "Y800", cv_image->image.data,
			cv_image->image.cols * cv_image->image.rows);
		scanner.scan(zbar_image);

		// iterate over all barcode readings from image
		for (zbar::Image::SymbolIterator symbol = zbar_image.symbol_begin();
			symbol != zbar_image.symbol_end(); ++symbol) {
			ROS_INFO("In symbol\n");
			std::string barcode = symbol->get_data();

			ROS_INFO("Barcode detected: %s\n", barcode.c_str());
			std_msgs::String barcode_string;
			barcode_string.data = barcode;
			barcode_pub_.publish(barcode_string);
		}
	}
}