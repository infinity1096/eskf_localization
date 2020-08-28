#include "eskf_localizer/base_type.h"
#include "eskf_localizer/eskf.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ros/console.h>

namespace ESKF_Localization{

	void ESKF_update_nominal(ImuDataPtr imu_data,double dt, State* state, Eigen::Vector3d g){
		double dt2_2 = 0.5*dt*dt;
		State old_state = *state;

		state->G_p_I = old_state.G_p_I + dt * old_state.G_v_I + dt2_2 * (old_state.G_R_I * (imu_data->accel - old_state.ab) + g);
		state->G_v_I = old_state.G_v_I + dt * (old_state.G_R_I * (imu_data->accel - old_state.ab) + g);

		if (imu_data->quat.norm() != 0 && imu_data->last_quat.norm() != 0){
			ROS_INFO("Updating orientation by quaternion");
			//quaternion valid, update orientation according to delta quaternion
			Eigen::Matrix3d d_rot(imu_data->last_quat.inverse() * imu_data->quat);

			state->G_R_I = old_state.G_R_I * d_rot;
			const Eigen::Vector3d d_theta = (- old_state.wb) * dt;
			if (d_theta.norm() >= 1e-12){
				Eigen::AngleAxisd d_rot2(d_theta.norm(),d_theta.normalized());
				state->G_R_I *= d_rot2.toRotationMatrix();
			}
		}else{
			const Eigen::Vector3d d_theta = (imu_data->gyro - old_state.wb) * dt;
			if (d_theta.norm() >= 1e-12){
				Eigen::AngleAxisd d_rot(d_theta.norm(),d_theta.normalized());
				state->G_R_I = old_state.G_R_I * d_rot.toRotationMatrix();
			}
		}

		if(imu_data->quat.norm() != 0){
			//current quaternion valid, haven't got last quaternion.
			imu_data->last_quat = imu_data->quat;
		}

		//state->ab = old_state.ab;
		//state->wb = old_state.wb;
	}

	void ESKF_predict(Eigen::MatrixXd Fx, Eigen::MatrixXd Q, State* state){
		State old_state = *state;
		state->P = (Fx * old_state.P * Fx.transpose() + Q).eval();
	}

	void ESKF_correct(Eigen::VectorXd z,Eigen::VectorXd h_x, Eigen::MatrixXd H, Eigen::MatrixXd V, State* state){
		State old_state = *state;

		Eigen::MatrixXd K = old_state.P * H.transpose() * (H * old_state.P * H.transpose() + V).inverse();
		Eigen::VectorXd del_x = K * (z - h_x);

		Eigen::MatrixXd I_KH = Eigen::Matrix<double, 15, 15>::Identity() - K * H;
		state->P = (I_KH * old_state.P * I_KH.transpose() + K * V * K.transpose()).eval();

		//inject error state
		state->G_p_I = old_state.G_p_I + del_x.segment<3>(0);
		state->G_v_I = old_state.G_v_I + del_x.segment<3>(3);
		const Eigen::Vector3d del_theta = del_x.segment<3>(6);
		if (del_theta.norm() >= 1e-12){
			state->G_R_I = old_state.G_R_I * Eigen::AngleAxisd(del_theta.norm(),del_theta.normalized()).toRotationMatrix();
		}
		state->ab = old_state.ab + del_x.segment<3>(9);
		state->wb = old_state.wb + del_x.segment<3>(12);
	}
}



