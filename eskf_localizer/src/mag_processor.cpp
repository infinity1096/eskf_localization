#include "eskf_localizer/mag_processor.h"
#include "eskf_localizer/base_type.h"
#include "eskf_localizer/utils.h"
#include "eskf_localizer/eskf.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

//FIXME Only for testing
#include <ros/console.h>


namespace ESKF_Localization{

MagProcessor::MagProcessor(Eigen::Matrix3d V){
	V_ = V;
}

void MagProcessor::Mag_correct(const MagDataPtr MagData, State* state){
	Eigen::Vector3d z = MagData->mag;
	Eigen::Vector3d h_x = state->G_R_I.transpose() * state->m_ref;
	Eigen::Quaterniond q(state->G_R_I);

	Eigen::Matrix<double,3,15> H = Eigen::Matrix<double,3,15>::Zero();
	H.block<3,3>(0,6) = hat(state->G_R_I.transpose() * state->m_ref);

	ESKF_correct(z,h_x,H,V_,state);
}
}
