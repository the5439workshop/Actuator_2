#include "Arduino.h"
#include <cmath>

class TemperatureSensor{
    public:
        TemperatureSensor(float beta, float t0_c, float r0, float r_fixed, float v_in);
        float readTemp (float V_thermistor);

    private:
        float R0;
        float T0;
        float B;
        float R_fixed;
        float V_IN;
};










