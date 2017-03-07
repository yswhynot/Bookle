#include <bookle_tf_pub/bookle_tf_pub.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace bookle {
	class TFPubNodelet : public nodelet::Nodelet {
	public:
		TFPubNodelet(){}

	private:
		void onInit() {
			detector_.reset(new TFPub(getNodeHandle(), getPrivateNodeHandle()));
		}
		boost::shared_ptr<TFPub> detector_;
	};
}

PLUGINLIB_DECLARE_CLASS(bookle_tf_pub, TFPubNodelet, bookle::TFPubNodelet, nodelet::Nodelet);