#include "Arduino_LPS22HB.h"
#include <math.h>

const double PRESSURE_REF_ZERO_AGL = 100.826;

double getAltitude(float pressureRef) {
    return 44330 * (1 - pow(BARO.readPressure() / pressureRef, 1 / 5.255f));
}