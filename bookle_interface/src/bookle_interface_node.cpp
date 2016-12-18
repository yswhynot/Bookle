#include "bookle_interface/bookle_interface.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "bookle_interface");

    ros::NodeHandle nh;
    bookle::BookleInterface bookleInterface(nh);

    ros::Rate rate(50);
    while (ros::ok()) {
    	bookleInterface.start();
    	ros::spinOnce();
    	rate.sleep();
    }

    // ros::spin();

    return 0;
}


