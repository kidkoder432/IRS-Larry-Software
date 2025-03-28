#include <Arduino.h>
#include <Serial.h>
#include <orientation.h>
#include <leds.h>
#include <tvc.h>

float roll, pitch, yaw;
SensorReadings readings;
Vec3D dir;
GyroBiases biases;
TVC tvc;

Quaternion attitude;


float ALPHA = 0.05;

void setup() {
    Serial.begin(115200);
    delay(200);
    initSensors();
    tvc.begin();

    pinMode(LED_BUILTIN, OUTPUT);

    digitalWrite(LED_BUILTIN, LOW);

    biases = calibrateSensors();

    // float ax, ay, az;
    // IMU.readAcceleration(ay, ax, az);
    // Serial.println(atan2(az, -ay) * 180 / PI);
    // yaw = atan2(ax, -sign(ay) * sqrt(az * az + ay * ay)) * 180 / PI;
    // pitch = atan2(az, -ay) * 180 / PI;
    attitude = Quaternion(1, 0, 0, 0);
}


long long lastLoopTime = micros();
void loop() {
    readSensors(readings, biases);

    // Serial.print(readings.gx - biases.bx);
    // Serial.print(" ");
    // Serial.print(readings.gy);
    // Serial.print(" ");
    // Serial.println(readings.gz - biases.bz);

    Vec3D dir = get_angles_quat(readings, attitude, deltaTime);

    tvc.update(dir, deltaTime);

    roll = dir.x;
    pitch = dir.y;
    yaw = dir.z;

    Serial.print("Roll: ");
    Serial.print(roll);
    Serial.print(" ");
    Serial.print("Pitch: ");
    Serial.print(pitch);
    Serial.print(" ");
    Serial.print("Yaw: ");
    Serial.println(yaw);

    if (yaw > 180) {
        yaw = yaw - 360;
    }
    else if (yaw < -180) {
        yaw = yaw + 360;
    }

    if (pitch > 180) {
        pitch = pitch - 360;
    }

    else if (pitch < -180) {
        pitch = pitch + 360;
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

    deltaTime = (micros() - lastLoopTime) / 1000000.;
    lastLoopTime = micros();
}