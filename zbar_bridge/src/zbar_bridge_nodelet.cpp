#include <zbar_bridge/zbar_bridge.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace bookle {
	class BarcodeReaderNodelet : public nodelet::Nodelet {
	public:
		BarcodeReaderNodelet() {}

	private:
		void onInit() {
			reader_.reset(new BarcodeReader(
				getNodeHandle(),
				getPrivateNodeHandle()));
		}
		boost::shared_ptr<BarcodeReader> reader_;
	};
}

PLUGINLIB_DECLARE_CLASS(zbar_bridge, BarcodeReaderNodelet, bookle::BarcodeReaderNodelet, nodelet::Nodelet);