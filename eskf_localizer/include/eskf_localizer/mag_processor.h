#pragma once

#include "eskf_localizer/base_type.h"
#include <Eigen/Core>

namespace ESKF_Localization{

class MagProcessor{

public:
	MagProcessor(Eigen::Matrix3d V);
	void Mag_correct(const MagDataPtr MagData, State* state);
private:
	Eigen::Matrix3d V_;				//covariance
};

}
