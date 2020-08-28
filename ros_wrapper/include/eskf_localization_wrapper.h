#include <ros/ros.h>
#include "eskf_localizer/eskf_localizer.h"

#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/MagneticField.h>
#include <nav_msgs/Path.h>

#include <tf/transform_broadcaster.h>

class ESKFLocalizationWrapper{

public:
	ESKFLocalizationWrapper(ros::NodeHandle nh);
	~ESKFLocalizationWrapper();

    void ImuCallback(const sensor_msgs::ImuConstPtr& imu_msg_ptr);
    void GpsPositionCallback(const sensor_msgs::NavSatFixConstPtr& gps_msg_ptr);
    void MagCallback(const geometry_msgs::Vector3StampedConstPtr& mag_msg_ptr);

private:

	ros::Subscriber imu_sub_;
	ros::Subscriber gps_sub_;
	ros::Subscriber mag_sub_;
	ros::Publisher fused_pose_pub_;
	ros::Publisher fused_path_pub_;
	ros::Publisher fused_fix_pub_;
	ros::Publisher fused_odom_pub_;

    nav_msgs::Path ros_path_;

	tf::TransformBroadcaster br_;

	std::unique_ptr<ESKF_Localization::ESKF_Localizer> eskf_localizer_;

	void addStateToPath(ESKF_Localization::State* state);
	void publishState();
};
