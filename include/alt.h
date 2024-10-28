#include "Arduino_LPS22HB.h"
#include <math.h>

double calculateOffset(double pressureRef) {
    double avgPressure = 0;

    for (int i = 0; i < 100; i++) {
        avgPressure += BARO.readPressure();
    }

    return avgPressure / 100 - pressureRef;
}

double getAltitude(double pressureRef, double offset) {

    double TEMP = BARO.readTemperature() + 273.15f;
    return TEMP / 0.0065f * (1 - pow((BARO.readPressure() - offset) / pressureRef, 1 / 5.255f));
}

double getAltitude_compl(double pressureRef, double offset, SensorReadings readings, double alt, double vertvel, double dt) {

    double alt_baro = getAltitude(pressureRef, offset);

    double alt_acc = alt + vertvel * dt;

    return alt_baro * 0.02 + alt_acc * 0.98;

}