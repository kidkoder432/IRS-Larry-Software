#include <Arduino.h>
#include <Kalman.h>
#include <Serial.h>
#include <orientation.h>
#include <Arduino_BMI270_BMM150.h>
#include <leds.h>


Kalman kx, ky;
float yaw, pitch;
SensorReadings readings;
Vec2D dir;
GyroBiases biases;

void setup() {
    Serial.begin(9600);
    delay(200);
    IMU.begin();

    float ax, ay, az;
    IMU.readAcceleration(ay, ax, az);
    Serial.println(atan2(az, -ay) * 180 / PI);
    yaw = atan2(ax, -sign(ay) * sqrt(az * az + ay * ay)) * 180 / PI;
    pitch = atan2(az, -ay) * 180 / PI;

    kx.setAngle(yaw);
    ky.setAngle(pitch);

    pinMode(LED_BUILTIN, OUTPUT);
    biases = calibrateSensors();
    Serial.print(biases.bx);
    Serial.print(" ");
    Serial.print(biases.by);
    Serial.print(" ");
    Serial.println(biases.bz);


}

long long lastLoopTime = micros();
void loop() {
    // --- Read Sensors --- //
    IMU.readAcceleration(readings.ay, readings.ax, readings.az);
    IMU.readGyroscope(readings.gy, readings.gx, readings.gz);

    dir = get_angles_kalman(deltaTime, readings, kx, ky, biases);

    yaw = dir.x;
    pitch = dir.y;

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


    delay(10);
    deltaTime = (micros() - lastLoopTime) / 1000000.;
}