#include <eskf_localizer/pt_processor.h>
namespace ESKF_Localization{

PTProcessor::PTProcessor(){

}

void PTProcessor::recordPressure(const PressureDataPtr pressure_data, State* state){
	state->pressure = pressure_data->pressure;
}

void PTProcessor::recordTemperature(const TempDataPtr temp_data, State* state){
	state->tempreature = temp_data->celsius;
}

}
