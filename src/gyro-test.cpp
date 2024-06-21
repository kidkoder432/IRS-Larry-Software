#include <Arduino.h>
#include <Serial.h>
#include <orientation.h>
#include <Arduino_BMI270_BMM150.h>
#include <leds.h>


double yaw, pitch;
SensorReadings readings;
Orientation dir;
Biases biases;

void setup() {
    Serial.begin(9600);
    delay(200);
    IMU.begin();




    pinMode(LED_BUILTIN, OUTPUT);
    biases = calibrateGyro();
    Serial.print(biases.bx);
    Serial.print(" ");
    Serial.print(biases.by);
    Serial.print(" ");
    Serial.println(biases.bz);

    float ax, ay, az;
    IMU.readAcceleration(ay, ax, az);
    Serial.println(atan2(az, -ay) * 180 / PI);
    yaw = 0;
    pitch = 0;

    pinMode(3, INPUT_PULLUP);

}

long long lastMicros = micros();
void loop() {
    // --- Read Sensors --- //
    IMU.readAcceleration(readings.ay, readings.ax, readings.az);
    IMU.readGyroscope(readings.gy, readings.gx, readings.gz);

    yaw -= (readings.gz - biases.bz) * DELTA_TIME;
    pitch += (readings.gx - biases.bx) * DELTA_TIME;

    // Serial.print(readings.gx - biases.bx);
    // Serial.print(" ");
    // Serial.print(readings.gy);
    // Serial.print(" ");
    // Serial.println(readings.gz - biases.bz);

    Serial.print(yaw);
    Serial.print(" ");
    Serial.println(pitch);

    if (digitalRead(3) == LOW) {
        delay(20);
        if (digitalRead(3) == LOW) {
            showColor(COLOR_YELLOW);
            yaw = pitch = 0;
        }
    }

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


    delay(10);
    DELTA_TIME = (micros() - lastMicros) / 1000000.;
    lastMicros = micros();
}