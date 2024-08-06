#include "Arduino_LPS22HB.h"
#include <math.h>

const double PRESSURE_REF_ZERO_AGL = 100.826;

double getAltitude() {
    return 44330 * (1 - pow(BARO.readPressure() / PRESSURE_REF_ZERO_AGL, 1 / 5.255));
}