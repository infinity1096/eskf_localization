#include "eskf_localizer/gps_processor.h"
#include "eskf_localizer/base_type.h"
#include "eskf_localizer/utils.h"
#include "eskf_localizer/eskf.h"

#include <Eigen/Core>


namespace ESKF_Localization{

GpsProcessor::GpsProcessor(Eigen::Vector3d I_p_Gps){
	GpsProcessor::I_p_Gps_ = I_p_Gps;
}

void GpsProcessor::Gps_correct(const GpsPositionDataPtr GpsData, State* state){

	Eigen::Vector3d z;
	ConvertLLAToENU(state->lla_origin, GpsData->lla, &z);

	Eigen::Vector3d h_x = state->G_p_I + state->G_R_I * I_p_Gps_;

	Eigen::Matrix<double,3,15> H = Eigen::Matrix<double,3,15>::Zero();
	H.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
	H.block<3,3>(0,6) = -state->G_R_I * hat(I_p_Gps_);

	//Eigen::Matrix3d V = GpsData->cov;
	Eigen::Matrix3d V;
	V << 1.5,0,0,0,1.5,0,0,0,4.0;

	ESKF_correct(z,h_x,H,V,state);
}
}
