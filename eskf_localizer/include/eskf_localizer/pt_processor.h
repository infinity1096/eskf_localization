#pragma once

#include "eskf_localizer/base_type.h"

namespace ESKF_Localization{

class PTProcessor{

public:
	PTProcessor();
	void recordPressure(const PressureDataPtr pressure_data, State* state);
	void recordTemperature(const TempDataPtr temp_data, State* state);
private:

};
}
