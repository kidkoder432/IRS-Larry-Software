#include "Arduino_LPS22HB.h"
#include <math.h>

float calculateOffset(float pressureRef) {
    float avgPressure = 0;

    for (int i = 0; i < 100; i++) {
        avgPressure += BARO.readPressure();
    }

    return avgPressure / 100 - pressureRef;
}

float getAltitude(float pressureRef, float offset) {

    float TEMP = BARO.readTemperature() + 273.15f;
    return TEMP / 0.0065f * (1 - pow((BARO.readPressure() - offset) / pressureRef, 1 / 5.255f));
}

float getAltitude_compl(float pressureRef, float offset, SensorReadings readings, float alt, float vertvel, float dt) {

    float alt_baro = getAltitude(pressureRef, offset);

    float alt_acc = alt + vertvel * dt;

    return alt_baro * 0.02 + alt_acc * 0.98;

}