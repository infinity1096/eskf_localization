#include "eskf_localizer/eskf_localizer.h"
#include "eskf_localizer/base_type.h"
#include "eskf_localizer/utils.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_msgs/Pose.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <memory>
#include <ros/console.h>

namespace ESKF_Localization{
	ESKF_Localizer::ESKF_Localizer(const double am_noise, const double wm_noise,
			const double ab_noise, const double wb_noise, Eigen::Vector3d I_p_Gps){
		state_.G_p_I = Eigen::Vector3d::Zero();
		state_.G_v_I = Eigen::Vector3d::Zero();
		state_.G_R_I = Eigen::Matrix3d::Identity();
		state_.ab = Eigen::Vector3d::Zero();
		state_.wb = Eigen::Vector3d::Zero();

		initializer_ = std::make_unique<Initializer>(20,20,20,&state_);
		Eigen::Vector3d g(0,0,-9.95);
		imu_processor_ = std::make_unique<ImuProcessor>(am_noise,wm_noise,ab_noise,wb_noise,g);
		gps_processor_ = std::make_unique<GpsProcessor>(I_p_Gps);
		mag_processor_ = std::make_unique<MagProcessor>(0.5*Eigen::Matrix3d::Identity());
		pt_processor_ = std::make_unique<PTProcessor>();
		last_t_ = ros::Time::now().toSec();
	}

	void ESKF_Localizer::processImuData(ImuDataPtr imu_data){
		if(!initializer_->is_initialized()){
			initializer_->Imu_initialize(imu_data);
			return;
		}

		double t = imu_data->timestamp;
		double dt = t - last_t_;
		last_t_ = t;
		if (dt >= 0.5 || dt <= 0){dt = 0.01;}
		//ROS_INFO("received quat from DMP = %f,%f,%f,%f", imu_data->quat.w(),imu_data->quat.x(),imu_data->quat.y(),imu_data->quat.z());
		imu_processor_->Imu_predict(imu_data,dt,&state_);
	}

	void ESKF_Localizer::processGpsData(GpsPositionDataPtr gps_data){
		if(!initializer_->is_initialized()){
			initializer_->Gps_initialize(gps_data);
			return;
		}

		gps_processor_->Gps_correct(gps_data,&state_);
	}

	void ESKF_Localizer::processMagData(MagDataPtr mag_data){
		if(!initializer_->is_initialized()){
			initializer_->Mag_initialize(mag_data);
			return;
		}

		mag_processor_->Mag_correct(mag_data,&state_);
	}

	void ESKF_Localizer::processPressureData(PressureDataPtr pressure_data){
		if(!initializer_->is_initialized()){
			initializer_->Pressure_initialize(pressure_data);
			return;
		}

		pt_processor_->recordPressure(pressure_data, &state_);
	}

	void ESKF_Localizer::processTempData(TempDataPtr temp_data){
		if (!initializer_->is_initialized()){
			initializer_->Temp_initialize(temp_data);
			return;
		}

		pt_processor_->recordTemperature(temp_data, &state_);
	}

	State* ESKF_Localizer::getState(){
		return &state_;
	}

    geometry_msgs::Pose ESKF_Localizer::getFusedPose(){
    	geometry_msgs::Pose fusedPose;
    	fusedPose.position.x = state_.G_p_I.x();
    	fusedPose.position.y = state_.G_p_I.y();
    	fusedPose.position.z = state_.G_p_I.z();

    	Eigen::Quaterniond q(state_.G_R_I);

    	fusedPose.orientation.w = q.w();
    	fusedPose.orientation.x = q.x();
    	fusedPose.orientation.y = q.y();
    	fusedPose.orientation.z = q.z();

    	return fusedPose;
    }

    sensor_msgs::NavSatFix ESKF_Localizer::getFusedFix(){
    	sensor_msgs::NavSatFix fusedFix;
    	Eigen::Vector3d fused_lla;
    	ConvertENUToLLA(state_.lla_origin, state_.G_p_I,&fused_lla);

    	fusedFix.header.frame_id = "world";
    	fusedFix.header.stamp = ros::Time::now();
    	fusedFix.latitude = fused_lla[0];
    	fusedFix.longitude = fused_lla[1];
    	fusedFix.altitude = fused_lla[2];
    	return fusedFix;
    }

    nav_msgs::Odometry ESKF_Localizer::getFusedOdometry(){
    	nav_msgs::Odometry odom;

    	odom.header.frame_id = "world";
    	odom.header.stamp = ros::Time::now();
    	odom.child_frame_id = "XIMU";

    	odom.pose.pose = getFusedPose();

    	Eigen::Vector3d I_v = state_.G_R_I.transpose() * state_.G_v_I;//global velocity to local velocity
    	//Eigen::Vector3d I_w = ; NO angular velocity estimation

    	//Note: twist message is specified in frame given by "child_frame_id", i.e. XIMU.
    	odom.twist.twist.linear.x = I_v[0];
    	odom.twist.twist.linear.y = I_v[1];
    	odom.twist.twist.linear.z = I_v[2];

    	return odom;
    }

}



