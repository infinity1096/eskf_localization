#include "eskf_localizer/imu_processor.h"
#include "eskf_localizer/base_type.h"
#include "eskf_localizer/utils.h"
#include "eskf_localizer/eskf.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ros/console.h>

namespace ESKF_Localization{

ImuProcessor::ImuProcessor(const double am_noise, const double wm_noise,
							const double ab_noise, const double wb_noise, const Eigen::Vector3d gravity){
	ImuProcessor::am_noise_ = am_noise;
	ImuProcessor::wm_noise_ = wm_noise;
	ImuProcessor::ab_noise_ = ab_noise;
	ImuProcessor::wb_noise_ = wb_noise;
	ImuProcessor::gravity_ = gravity;
}

void ImuProcessor::Imu_predict(const ImuDataPtr imu_data, double dt, State* state){

	ESKF_update_nominal(imu_data,dt,state,gravity_);

	Eigen::Matrix<double,15,15> Fx = Eigen::Matrix<double,15,15>::Identity();
	Fx.block<3,3>(0,3) = Eigen::Matrix3d::Identity() * dt;
	Fx.block<3,3>(3,6) = - state->G_R_I*hat(imu_data->accel - state->ab)*dt;
	Fx.block<3,3>(3,9) = - state->G_R_I*dt;

	Eigen::Vector3d delta_angle_axis = (imu_data->gyro - state->wb) * dt;
	if (delta_angle_axis.norm() >= 1e-12){
		Fx.block<3,3>(6,6) = Eigen::AngleAxisd(delta_angle_axis.norm(),delta_angle_axis.normalized()).toRotationMatrix().transpose();
	}else{
		Fx.block<3,3>(6,6) = Eigen::Matrix3d::Identity();

	}

	Fx.block<3,3>(6,12) = -Eigen::Matrix3d::Identity()*dt;

	Eigen::Matrix<double,15,12> Fi = Eigen::Matrix<double,15,12>::Zero();
	Fi.block<12,12>(3,0) = Eigen::Matrix<double,12,12>::Identity();

	Eigen::Matrix<double,12,12> Q = Eigen::Matrix<double,12,12>::Zero();
	Q.block<3,3>(0,0) = Eigen::Matrix3d::Identity() * am_noise_ * dt * dt;
	Q.block<3,3>(3,3) = Eigen::Matrix3d::Identity() * wm_noise_ * dt * dt;
	Q.block<3,3>(6,6) = Eigen::Matrix3d::Identity() * ab_noise_ * dt;
	Q.block<3,3>(9,9) = Eigen::Matrix3d::Identity() * wb_noise_ * dt;

	ESKF_predict(Fx, Fi * Q * Fi.transpose(), state);
}

}
