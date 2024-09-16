#include <Arduino.h>
#include <Serial.h>
#include <orientation.h>
#include <leds.h>


double yaw, pitch;
SensorReadings readings;
Vec2D dir;
Biases biases;

void setup() {
    Serial.begin(9600);
    delay(200);

    initIMU();

    pinMode(LED_BUILTIN, OUTPUT);
    showColor(COLOR_OFF);
    biases = calibrateSensors();
    Serial.print(biases.bx);
    Serial.print(" ");
    Serial.print(biases.by);
    Serial.print(" ");
    Serial.println(biases.bz);

    yaw = 0;
    pitch = 0;

    pinMode(3, INPUT_PULLUP);

}

long long lastMicros = micros();
void loop() {
    // --- Read Sensors --- //
    // IMU.readAcceleration(readings.ay, readings.ax, readings.az);
    readSensors(readings, biases);

    // IMU.readGyroscope(readings.gy, readings.gx, readings.gz);

    yaw -= (readings.gz) * DELTA_TIME;
    pitch += (readings.gy) * DELTA_TIME;

    Serial.print(yaw);
    Serial.print(" ");
    Serial.println(pitch);

    if (abs(yaw) >= 15) {
        showColor(COLOR_RED);
        if (abs(pitch) >= 15) {
            showColor(COLOR_PURPLE);
        }
    }

    else {
        showColor(COLOR_GREEN);
        if (abs(pitch) >= 15) {
            showColor(COLOR_LIGHTBLUE);
        }
    }


    while (micros() - lastMicros < 9990) {}
    DELTA_TIME = (micros() - lastMicros) / 1000000.;
    lastMicros = micros();
}