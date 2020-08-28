#include "eskf_localization_wrapper.h"
#include "eskf_localizer/base_type.h"

#include <Eigen/Geometry>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

ESKFLocalizationWrapper::ESKFLocalizationWrapper(ros::NodeHandle nh){

	//obtain constants from ros param
	double am_noise, wm_noise, ab_noise, wb_noise;
	double x,y,z;

	nh.param("am_noise", am_noise, 1e-2);
	nh.param("wm_noise", wm_noise, 1e-4);
	nh.param("ab_noise", ab_noise, 1e-6);
	nh.param("wb_noise", wb_noise, 1e-6);

	nh.param("I_p_Gps_x", x, 0.0);
	nh.param("I_p_Gps_y", y, 0.0);
	nh.param("I_p_Gps_z", z, 0.0);
	const Eigen::Vector3d I_p_Gps(x,y,z);

	//initialize eskf_localizer with these parameters
	eskf_localizer_ = std::make_unique<ESKF_Localization::ESKF_Localizer>(
			am_noise,wm_noise, ab_noise, wb_noise, I_p_Gps);

	imu_sub_ = nh.subscribe("/imu",10,&ESKFLocalizationWrapper::ImuCallback,this);
	gps_sub_ = nh.subscribe("/fix",10,&ESKFLocalizationWrapper::GpsPositionCallback,this);
	mag_sub_ = nh.subscribe("/mag",10,&ESKFLocalizationWrapper::MagCallback,this);

	fused_pose_pub_ = nh.advertise<geometry_msgs::Pose>("/fused_pose", 10);
	fused_path_pub_ = nh.advertise<nav_msgs::Path>("/fused_path", 10);
	fused_fix_pub_ = nh.advertise<sensor_msgs::NavSatFix>("/fused_fix",10);
	fused_odom_pub_ = nh.advertise<nav_msgs::Odometry>("/fused_odom",10);
}

ESKFLocalizationWrapper::~ESKFLocalizationWrapper(){

}

void ESKFLocalizationWrapper::ImuCallback(const sensor_msgs::ImuConstPtr& imu_msg_ptr){
	ESKF_Localization::ImuDataPtr imu_data_ptr = std::make_shared<ESKF_Localization::ImuData>();
	imu_data_ptr->timestamp = ros::Time::now().toSec();

	imu_data_ptr->accel << 	imu_msg_ptr->linear_acceleration.x,
							imu_msg_ptr->linear_acceleration.y,
							imu_msg_ptr->linear_acceleration.z;

	imu_data_ptr->gyro << 	imu_msg_ptr->angular_velocity.x,
							imu_msg_ptr->angular_velocity.y,
							imu_msg_ptr->angular_velocity.z;

	imu_data_ptr->quat = Eigen::Quaterniond(imu_msg_ptr->orientation.w,
							imu_msg_ptr->orientation.x,
							imu_msg_ptr->orientation.y,
							imu_msg_ptr->orientation.z);

	eskf_localizer_->processImuData(imu_data_ptr);

	publishState();
}

void ESKFLocalizationWrapper::GpsPositionCallback(const sensor_msgs::NavSatFixConstPtr& gps_msg_ptr){
	ESKF_Localization::GpsPositionDataPtr gps_data_ptr = std::make_shared<ESKF_Localization::GpsPositionData>();
	gps_data_ptr->timestamp = ros::Time::now().toSec();

	gps_data_ptr->lla << 	gps_msg_ptr->latitude,
							gps_msg_ptr->longitude,
							gps_msg_ptr->altitude;

	gps_data_ptr->cov = Eigen::Map<const Eigen::Matrix3d>(gps_msg_ptr->position_covariance.data());

	eskf_localizer_->processGpsData(gps_data_ptr);
	publishState();
}

void ESKFLocalizationWrapper::MagCallback(const sensor_msgs::MagneticFieldConstPtr& mag_msg_ptr){
	ESKF_Localization::MagDataPtr mag_data_ptr = std::make_shared<ESKF_Localization::MagData>();
	mag_data_ptr->timestamp = ros::Time::now().toSec();

	mag_data_ptr->mag << 	mag_msg_ptr->magnetic_field.x,
							mag_msg_ptr->magnetic_field.y,
							mag_msg_ptr->magnetic_field.z;

	eskf_localizer_->processMagData(mag_data_ptr);
	publishState();
}

void ESKFLocalizationWrapper::addStateToPath(ESKF_Localization::State* state){
    ros_path_.header.frame_id = "world";
    ros_path_.header.stamp = ros::Time::now();

    geometry_msgs::PoseStamped pose;
    pose.header = ros_path_.header;

    pose.pose.position.x = state->G_p_I[0];
    pose.pose.position.y = state->G_p_I[1];
    pose.pose.position.z = state->G_p_I[2];

    const Eigen::Quaterniond G_q_I(state->G_R_I);
    pose.pose.orientation.x = G_q_I.x();
    pose.pose.orientation.y = G_q_I.y();
    pose.pose.orientation.z = G_q_I.z();
    pose.pose.orientation.w = G_q_I.w();

    ros_path_.poses.push_back(pose);
}

void ESKFLocalizationWrapper::publishState(){
	//publish fused states
	fused_pose_pub_.publish(eskf_localizer_->getFusedPose());
	fused_odom_pub_.publish(eskf_localizer_->getFusedOdometry());
	fused_fix_pub_.publish(eskf_localizer_->getFusedFix());
	addStateToPath(eskf_localizer_->getState());
	fused_path_pub_.publish(ros_path_);

	//publish transform
	ESKF_Localization::State* state = eskf_localizer_->getState();
	tf::Transform tf;
	tf.setOrigin(tf::Vector3(state->G_p_I[0],state->G_p_I[1],state->G_p_I[2]));
	Eigen::Quaterniond quat(state->G_R_I);
	tf::Quaternion q(quat.x(),quat.y(),quat.z(),quat.w());//Caution! tf::Quaternion is in (x,y,z,w)
	tf.setRotation(q);
	br_.sendTransform(tf::StampedTransform(tf,ros::Time::now(),"world","XIMU"));
}

