#pragma once

#include "eskf_localizer/base_type.h"
#include "eskf_localizer/initializer.h"
#include "eskf_localizer/imu_processor.h"
#include "eskf_localizer/gps_processor.h"
#include "eskf_localizer/mag_processor.h"
#include "eskf_localizer/pt_processor.h"

#include <Eigen/Core>

#include <geometry_msgs/Pose.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>

namespace ESKF_Localization{

class ESKF_Localizer{

public:
	ESKF_Localizer(const double am_noise, const double wm_noise,
			const double ab_noise, const double wb_noise, Eigen::Vector3d I_p_Gps);
	void processImuData(ImuDataPtr imu_data);
	void processGpsData(GpsPositionDataPtr gps_data);
	void processMagData(MagDataPtr mag_data);
	void processPressureData(PressureDataPtr pressure_data);
	void processTempData(TempDataPtr temp_data);

	State* getState();
	geometry_msgs::Pose getFusedPose();
	sensor_msgs::NavSatFix getFusedFix();
	nav_msgs::Odometry getFusedOdometry();

private:

	std::unique_ptr<Initializer> initializer_;
	std::unique_ptr<ImuProcessor> imu_processor_;
	std::unique_ptr<GpsProcessor> gps_processor_;
	std::unique_ptr<MagProcessor> mag_processor_;
	std::unique_ptr<PTProcessor> pt_processor_;

	State state_;

	double last_t_;
};
}
