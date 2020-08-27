#pragma once

#include "eskf_localizer/base_type.h"

namespace ESKF_Localization{

void ESKF_update_nominal(ImuDataPtr imu_data,double dt, State* state, Eigen::Vector3d g);
void ESKF_predict(Eigen::MatrixXd Fx, Eigen::MatrixXd Q, State* state);
void ESKF_correct(Eigen::VectorXd z,Eigen::VectorXd h_x, Eigen::MatrixXd H, Eigen::MatrixXd V, State* state);

}
