#include <Arduino_LPS22HB.h>
#include <math.h>

float calculateOffset(float pressureRef) {
    float avgPressure = 0;

    for (int i = 0; i < 50; i++) {
        avgPressure += BARO.readPressure(KILOPASCAL);
        delay(20);
    }

    Serial.println(avgPressure / 50);
    return avgPressure / 50 - pressureRef;
}

float getAltitude(float pressureRef, float offset) {

    float TEMP = BARO.readTemperature() + 273.15f;
    return TEMP / 0.0065f * (1 - pow((BARO.readPressure() - offset) / pressureRef, 1 / 5.255f));
}
}