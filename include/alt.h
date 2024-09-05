#include "Arduino_LPS22HB.h"
#include <math.h>

const double PRESSURE_REF_ZERO_AGL = 100.826;

double calculateOffset(float pressureRef) {
    double avgPressure = 0;

    for (int i = 0; i < 100; i++) {
        avgPressure += BARO.readPressure();
    }

    return avgPressure / 100 - pressureRef;
}

double getAltitude(float pressureRef, float offset) {
    Serial.println(pressureRef);
    Serial.println(BARO.readPressure());
    return 44330 * (1 - pow((BARO.readPressure() - offset) / pressureRef, 1 / 5.255f));
}