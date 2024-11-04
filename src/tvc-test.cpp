#include <Arduino.h>
#include <Serial.h>
#include <orientation.h>
#include <leds.h>
#include <tvc.h>

double roll, pitch, yaw;
SensorReadings readings;
Vec3D dir;
Biases biases;
TVC tvc;

Quaternion attitude;


double ALPHA = 0.05;

void setup() {
    Serial.begin(115200);
    delay(200);
    initIMU();
    tvc.begin();

    pinMode(LED_BUILTIN, OUTPUT);

    digitalWrite(LED_BUILTIN, LOW);

    biases = calibrateSensors();

    // double ax, ay, az;
    // IMU.readAcceleration(ay, ax, az);
    // Serial.println(atan2(az, -ay) * 180 / PI);
    // yaw = atan2(ax, -sign(ay) * sqrt(az * az + ay * ay)) * 180 / PI;
    // pitch = atan2(az, -ay) * 180 / PI;
    attitude = Quaternion(1, 0, 0, 0);
}


long long lastMicros = micros();
void loop() {
    readSensors(readings, biases);

    // Serial.print(readings.gx - biases.bx);
    // Serial.print(" ");
    // Serial.print(readings.gy);
    // Serial.print(" ");
    // Serial.println(readings.gz - biases.bz);

    Vec3D dir = get_angles_quat(readings, attitude, DELTA_TIME);

    tvc.update(dir, DELTA_TIME);

    roll = dir.x;
    pitch = dir.y;
    yaw = dir.z;


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

    DELTA_TIME = (micros() - lastMicros) / 1000000.;
    lastMicros = micros();
}