#include <memory>

#include <ros/ros.h>

#include "eskf_localization_wrapper.h"

int main (int argc, char** argv) {

    // Initialize ros.
    ros::init(argc, argv, "eskf_localization");
    ros::NodeHandle nh;

    // Initialize localizer.
    ESKFLocalizationWrapper localizer(nh);

    ros::spin();
    return 1;
}
