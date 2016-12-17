#include "bookle_interface/bookle_interface.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "bookle_interface");

    ros::NodeHandle nh;
    BookleInterface bookleInterface(nh);

    ros::spin();

    return 0;
}


