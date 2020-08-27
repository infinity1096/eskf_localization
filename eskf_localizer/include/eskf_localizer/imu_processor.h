#pragma once

#include "eskf_localizer/base_type.h"

namespace ESKF_Localization{

class ImuProcessor{

public:
	ImuProcessor(const double am_noise, const double wm_noise,
				const double ab_noise, const double wb_noise, const Eigen::Vector3d gravity);

	void Imu_predict(const ImuDataPtr imu_data,double dt, State* state);

private:
	double am_noise_;	//accelerometer measurement noise
	double wm_noise_;	//gyroscope measurement noise
	double ab_noise_;	//accelerometer bias noise
	double wb_noise_;	//gyroscope bias noise
	Eigen::Vector3d gravity_;
};

}
