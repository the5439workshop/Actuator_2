#include "Arduino.h"
#include <cmath>
#include <TemperatureSensor.h>

TemperatureSensor::TemperatureSensor(float beta, float t0_c, float r0, float r_fixed, float v_in):
    R0(r0), T0(t0_c+273.15), B(beta), R_fixed(r_fixed), V_IN(v_in){}
        
float TemperatureSensor::readTemp (float V_thermistor){
        float r_thermistor = R_fixed * (V_IN/V_thermistor - 1);
        float inv_T = (1.0 / T0) + (1.0 / B) * std::log(r_thermistor / R0);
        float T_c = (1.0 / inv_T) - 273.15;
        return T_c;
    }











