#pragma once

#include "eskf_localizer/base_type.h"
#include <deque>

namespace ESKF_Localization{

class Initializer{

public:
	Initializer(int imu_initialize_goal, int gps_initialize_goal, int mag_initialize_goal, State* state);

	void Imu_initialize(ESKF_Localization::ImuDataPtr imu_data);
	void Gps_initialize(ESKF_Localization::GpsPositionDataPtr gps_data);
	void Mag_initialize(ESKF_Localization::MagDataPtr mag_data);
	void Pressure_initialize(ESKF_Localization::PressureDataPtr pressure_data);
	void Temp_initialize(ESKF_Localization::TempDataPtr temp_data);

	bool is_initialized();
private:

	int imu_initialize_goal_ = 10;
	int gps_initialize_goal_ = 10;
	int mag_initialize_goal_ = 10;
	int pressure_initialize_goal_ = 10;
	int temperature_initialize_goal_ = 10;

	bool got_first_gps_message_ = false;//usually GPS message comes after imu/mag messages.
							   //start collecting IMU/MAG data samples after the first message is received,
							   //which indicates consecutive GPS data begins.

	bool imu_initialized_ = false;
	bool gps_initialized_ = false;
	bool mag_initialized_ = false;
	bool pressure_initialized_ = false;
	bool temperature_initialized_ = false;
	bool eskf_initialized_ = false;

	std::deque<ImuDataPtr> imu_buffer_;
	std::deque<GpsPositionDataPtr> gps_buffer_;
	std::deque<MagDataPtr> mag_buffer_;
	std::deque<PressureDataPtr> pressure_buffer_;
	std::deque<TempDataPtr> temperature_buffer_;

	State* state_;
};

}
