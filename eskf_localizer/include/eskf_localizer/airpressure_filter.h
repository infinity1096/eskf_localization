#pragma once

#include "eskf_localizer/base_type.h"
#include <Eigen/Core>

namespace ESKF_Localization{
class AirpressureFilter{
public:

	AirpressureFilter(double k);
	void correctHeight(Eigen::Vector3d* z, State* state);
	double calcP0(double air_pressure, double celsius, double GPS_height, State* state);

private:

	double calcPressureHeight(double air_pressure, double celsius, State* state);
	double k_;	//k_ <- [0,1], closer to 0 indicates more trust in air pressure sensor.
};
}
