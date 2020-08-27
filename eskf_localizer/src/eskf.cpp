#include "eskf_localizer/base_type.h"
#include "eskf_localizer/eskf.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace ESKF_Localization{

	void ESKF_update_nominal(ImuDataPtr imu_data,double dt, State* state, Eigen::Vector3d g){
		double dt2_2 = 0.5*dt*dt;
		state->G_p_I += dt * state->G_v_I + dt2_2 * (state->G_R_I * (imu_data->accel - state->ab) + g);
		state->G_v_I += dt * (state->G_R_I * (imu_data->accel - state->ab) + g);

		const Eigen::Vector3d d_theta = (imu_data->gyro - state->wb) * dt;
		if (d_theta.norm() >= 1e-12){
			Eigen::AngleAxisd d_rot(d_theta.norm(),d_theta.normalized());
			state->G_R_I *= d_rot.toRotationMatrix();
		}

		//state->ab = state->ab;
		//state->wb = state->wb;
	}

	void ESKF_predict(Eigen::MatrixXd Fx, Eigen::MatrixXd Q, State* state){
		state->P = (Fx * state->P * Fx.transpose() + Q).eval();
	}

	void ESKF_correct(Eigen::VectorXd z,Eigen::VectorXd h_x, Eigen::MatrixXd H, Eigen::MatrixXd V, State* state){
		Eigen::MatrixXd K = state->P * H.transpose() * (H * state->P * H.transpose() + V).inverse();
		Eigen::VectorXd del_x = K * (z - h_x);

		Eigen::MatrixXd I_KH = Eigen::Matrix<double, 15, 15>::Identity() - K * H;
		state->P = (I_KH * state->P * I_KH.transpose() + K * V * K.transpose()).eval();

		//inject error state
		state->G_p_I += del_x.segment<3>(0);
		state->G_v_I += del_x.segment<3>(3);
		const Eigen::Vector3d del_theta = del_x.segment<3>(6);
		if (del_theta.norm() >= 1e-12){
			state->G_R_I *= Eigen::AngleAxisd(del_theta.norm(),del_theta.normalized()).toRotationMatrix();
		}
		state->ab += del_x.segment<3>(9);
		state->wb += del_x.segment<3>(12);
	}
}



