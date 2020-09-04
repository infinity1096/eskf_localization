#include <eskf_localizer/airpressure_filter.h>
#include <ros/console.h>
#include "math.h"
/**
 * This processor is called by GPS processor to correct height.
 */
namespace ESKF_Localization{
AirpressureFilter::AirpressureFilter(double k){
	k_ = k;
}
void AirpressureFilter::correctHeight(Eigen::Vector3d* z, State* state){
	//calculate height difference by air pressure
	double current_pressure_height = calcPressureHeight(state->pressure, state->tempreature, state);
	double delta_height = current_pressure_height - state->last_pressure_height;
	state->last_pressure_height = current_pressure_height;

	ROS_INFO("current_pressure_height = %f, delta_height = %f, last_height = %f",current_pressure_height,delta_height,state->last_pressure_height );

	//Complementary filter with GPS height
	state->height_filtered = k_ * (z->z() + state->lla_origin.z()) + (1 - k_) * (state->height_filtered + delta_height);
	z->segment<1>(2) = state->height_filtered * Eigen::Matrix<double,1,1>::Identity() - state->lla_origin.segment<1>(2);
	//Use filtered value as GPS observation
}

double AirpressureFilter::calcPressureHeight(double air_pressure, double celsius, State* state){
	ROS_INFO("pressure height calculated with p = %f , T = %f", air_pressure, celsius);
	return ((pow((state->p0 / air_pressure), 1/5.257) - 1.0) * (celsius + 273.15)) / 0.0065;
}
double AirpressureFilter::calcP0(double air_pressure, double celsius, double GPS_height, State* state){
	return pow(0.0065*GPS_height/(celsius + 273.15) + 1, 5.257) * air_pressure;
}
}
