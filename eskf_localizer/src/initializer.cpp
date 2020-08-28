#include "eskf_localizer/initializer.h"
#include "eskf_localizer/base_type.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

//FIXME
#include <ros/console.h>

namespace ESKF_Localization{

Initializer::Initializer(int imu_initialize_goal, int gps_initialize_goal,int mag_initialize_goal, State* state){
	imu_initialize_goal_ = imu_initialize_goal;
	gps_initialize_goal_ = gps_initialize_goal;
	mag_initialize_goal_ = mag_initialize_goal;
	state_ = state;

	//initial value of P
	state->P.block<3,3>(0,0) = 10.0 * Eigen::Matrix3d::Identity();		//position cov
	state->P.block<3,3>(3,3) = 5.0 * Eigen::Matrix3d::Identity();		//velocity cov
	state->P.block<3,3>(6,6) = 0.04 * Eigen::Matrix3d::Identity();		//roll-pitch-yaw cov
	state->P.block<3,3>(9,9) = 0.0004 * Eigen::Matrix3d::Identity();	//acceleration bias cov
	state->P.block<3,3>(12,12) = 0.0004 * Eigen::Matrix3d::Identity();	//gyroscope bias cov
}

void Initializer::Imu_initialize(ESKF_Localization::ImuDataPtr imu_data){
	if(!got_first_gps_message_){
		return;
	}

	//if (gps_initialized_){return;}magnetic is slower than imu, constantly update buffer

	imu_buffer_.push_back(imu_data);
	if (imu_buffer_.size() > imu_initialize_goal_){
		imu_buffer_.pop_front();
		imu_initialized_ = true;
	}

}

void Initializer::Gps_initialize(ESKF_Localization::GpsPositionDataPtr gps_data){
	got_first_gps_message_ = true;

	if (gps_initialized_){return;}

	if (gps_buffer_.size() < gps_initialize_goal_){
		gps_buffer_.push_back(gps_data);
	}else{
		//compute lla_origin
		Eigen::Vector3d lla_average(0.,0.,0.);
		for (ESKF_Localization::GpsPositionDataPtr gps_d : gps_buffer_){
			lla_average += gps_d->lla;
		}
		lla_average /= gps_initialize_goal_;
		state_->lla_origin = lla_average;
		gps_initialized_ = true;
		ROS_INFO("[Initializer] Gps initialized with lla = %f,%f,%f",state_->lla_origin.x(),state_->lla_origin.y(),state_->lla_origin.z());
	}

}

void Initializer::Mag_initialize(ESKF_Localization::MagDataPtr mag_data){
	if (!got_first_gps_message_ || mag_initialized_){
		return;
	}
	if (mag_buffer_.size() < mag_initialize_goal_){
		mag_buffer_.push_back(mag_data);
	}
	if (imu_initialized_ && mag_buffer_.size() == mag_initialize_goal_){
		//average imu and mag samples
		Eigen::Vector3d am_init = Eigen::Vector3d::Zero();
		Eigen::Vector3d mm_init = Eigen::Vector3d::Zero();

		for (ESKF_Localization::ImuDataPtr imu_d : imu_buffer_){
			am_init += imu_d->accel;
		}
		am_init /= imu_buffer_.size();

		for (ESKF_Localization::MagDataPtr mag_d : mag_buffer_){
			mm_init += mag_d->mag;
		}
		mm_init /= mag_buffer_.size();

		//calculate initial orientation
		Eigen::Matrix3d I_R_G;
		Eigen::Vector3d x_ref, y_ref, z_ref;//global x,y,z axis reference in Imu frame

		z_ref = am_init.normalized();//z reference - negative gravity
		x_ref = mm_init.cross(z_ref).normalized();//mag(North + UP component) cross with z_ref(Up) to get East
		y_ref = z_ref.cross(x_ref).normalized();//z_ref(Up) cross with x_ref(East) to get North

		//fill in global-to-local rotation matrix, which is the inverse of state->G_R_I that we store orientation
		I_R_G.block<3,1>(0,0) = x_ref;
		I_R_G.block<3,1>(0,1) = y_ref;
		I_R_G.block<3,1>(0,2) = z_ref;

		//update state
		state_->G_R_I = I_R_G.transpose();


		//FIXME
		Eigen::Quaterniond q(state_->G_R_I);
		ROS_INFO("[Initializer] Orientation initialized with q: w:%f, x:%f, y:%f, z:%f",q.w(),q.x(),q.y(),q.z());

		//calculate m_ref
		state_->m_ref = state_->G_R_I * mm_init;

		//FIXME
		ROS_INFO("[Initializer] magnetic reference initialized with m_ref = %f,%f,%f",state_->m_ref[0],state_->m_ref[1],state_->m_ref[2]);

		mag_initialized_ = true;
	}

}

bool Initializer::is_initialized(){
	eskf_initialized_ = imu_initialized_ && gps_initialized_ && mag_initialized_;

	return eskf_initialized_;
}


}
