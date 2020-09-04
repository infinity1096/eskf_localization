#pragma once

#include "eskf_localizer/base_type.h"
#include "eskf_localizer/airpressure_filter.h"
#include <Eigen/Core>

namespace ESKF_Localization{

class GpsProcessor{

public:
	GpsProcessor(Eigen::Vector3d I_p_Gps);
	void Gps_correct(const GpsPositionDataPtr GpsData, State* state);
private:
	Eigen::Vector3d I_p_Gps_;		//position of GPS in IMU frame
	std::unique_ptr<AirpressureFilter> air_pressure_filter_;
};


}
