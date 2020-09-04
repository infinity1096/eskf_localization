#pragma once

#include <memory>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace ESKF_Localization{

struct ImuData{
	double timestamp;			//Unit: Seconds

	Eigen::Vector3d accel;		//Unit: m/s^2
	Eigen::Vector3d gyro;		//Unit: rad/s

	Eigen::Quaterniond quat;	//Orientation estimation from IMU. For example, the DMP
								//feature of MPU6050/9250. Optional.
};
using ImuDataPtr = std::shared_ptr<ImuData>;

struct GpsPositionData{
	double timestamp;			//Unit: Seconds

	Eigen::Vector3d lla;		//Unit: Latitude,Longitude(degrees); Altitude(meters)
	Eigen::Matrix3d cov;		//Unit: Position covariance(m^2)
};
using GpsPositionDataPtr = std::shared_ptr<GpsPositionData>;

struct MagData{
	double timestamp;			//Unit: Seconds

	Eigen::Vector3d mag;		//Unit: Gauss
};
using MagDataPtr = std::shared_ptr<MagData>;

struct TempData{
	double timestamp;			//Unit: Seconds

	double celsius;				//Celsius temperature
};
using TempDataPtr = std::shared_ptr<TempData>;

struct PressureData{
	double timestamp;			//Unit: Seconds

	double pressure;			//Unit: mBar
};
using PressureDataPtr = std::shared_ptr<PressureData>;

struct State{
	double timestamp;

	//Nominal states
	Eigen::Vector3d G_p_I;			//Position of IMU frame(I) in global frame(G)
	Eigen::Vector3d G_v_I;			//Velocity of IMU frame(I) in global frame(G)
	Eigen::Matrix3d G_R_I;			//Rotation from IMU frame(I) to global frame(G)
	Eigen::Vector3d ab;				//Accelerometer bias
	Eigen::Vector3d wb;				//Gyroscope bias


	Eigen::Vector3d lla_origin;
	//Global frame is a ENU frame centered at lla_origin
	//lla = latitude, longitude, altitude

	Eigen::Vector3d m_ref;
	//megnetic field reference

	double p0;//sea-level pressure at lla_init with local air condition
	double last_pressure_height;//last altitude calculated from air pressure and temperature
	double height_filtered;//Complementary filter

	double pressure;
	double tempreature;//current temperature

	Eigen::Quaterniond last_quat;
	//last quaternion

	Eigen::Matrix<double,15,15> P;	//Covariance of the error state
};

}
