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
	/*
	Eigen::Vector3d h_x = state->G_R_I.transpose() * state->m_ref;
	Eigen::Quaterniond q(state->G_R_I);

	Eigen::Matrix<double,3,15> H = Eigen::Matrix<double,3,15>::Zero();
	Eigen::Matrix<double,4,3> temp;
	temp << Eigen::Matrix<double,1,3>::Zero(), 0.5*Eigen::Matrix3d::Identity();
	H.block<3,3>(0,6) = diff_qT_a_q_diff_q(q,state->m_ref) * quat_l(q) * temp;

	ESKF_correct(z,h_x,H,V_,state);
	*/

	Eigen::Vector3d g_ref = state->G_R_I.transpose() * Eigen::Vector3d(0,0,-9.81);

	//calculate absolute orientation
	Eigen::Matrix3d I_R_G_mag, G_R_I_mag;
	Eigen::Vector3d x_ref, y_ref, z_ref;//global x,y,z axis reference in Imu frame

	z_ref = -g_ref.normalized();//z reference - negative gravity
	x_ref = z.cross(z_ref).normalized();//mag(North + UP component) cross with z_ref(Up) to get East
	y_ref = z_ref.cross(x_ref).normalized();//z_ref(Up) cross with x_ref(East) to get North

	//fill in global-to-local rotation matrix, which is the inverse of state->G_R_I that we store orientation
	I_R_G_mag.block<3,1>(0,0) = x_ref;
	I_R_G_mag.block<3,1>(0,1) = y_ref;
	I_R_G_mag.block<3,1>(0,2) = z_ref;

	G_R_I_mag = I_R_G_mag.transpose();

	//SLERP between state orientation and calculated absolute orientation
	Eigen::Quaterniond q0(state->G_R_I);
	Eigen::Quaterniond q1(G_R_I_mag);
	state->G_R_I = q0.slerp(0.01,q1).toRotationMatrix();


}
}
