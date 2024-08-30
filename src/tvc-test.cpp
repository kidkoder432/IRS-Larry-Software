#include <Arduino.h>
#include <Serial.h>
#include <orientation.h>
#include <leds.h>
#include <tvc.h>

double yaw, pitch;
SensorReadings readings;
Vec2D dir;
Biases biases;
TVC tvc;


double ALPHA = 0.05;

void setup() {
    Serial.begin(9600);
    delay(200);
    IMU.begin();
    tvc.begin();

    pinMode(LED_BUILTIN, OUTPUT);

    digitalWrite(LED_BUILTIN, LOW);

    biases = calibrateGyro();

    float ax, ay, az;
    IMU.readAcceleration(ay, ax, az);
    Serial.println(atan2(az, -ay) * 180 / PI);
    yaw = atan2(ax, -sign(ay) * sqrt(az * az + ay * ay)) * 180 / PI;
    pitch = atan2(az, -ay) * 180 / PI;

}


long long lastMicros = micros();
void loop() {
    // --- Read Sensors --- //
    IMU.readAcceleration(readings.ay, readings.ax, readings.az);
    IMU.readGyroscope(readings.gy, readings.gx, readings.gz);

    // --- Read Sensors --- //
    IMU.readAcceleration(readings.ay, readings.ax, readings.az);
    IMU.readGyroscope(readings.gy, readings.gx, readings.gz);

    // Serial.print(readings.gx - biases.bx);
    // Serial.print(" ");
    // Serial.print(readings.gy);
    // Serial.print(" ");
    // Serial.println(readings.gz - biases.bz);

    Vec2D dir = get_angles_complementary(1 - ALPHA, DELTA_TIME, readings, yaw, pitch, biases);

    tvc.update(dir, DELTA_TIME);

    yaw = dir.x;
    pitch = dir.y;

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

    Serial.print("Yaw: ");
    Serial.print(yaw);
    Serial.print(" Pitch: ");
    Serial.print(pitch);
    Serial.print(" TVC Yaw: ");
    Serial.print(tvc.getAngle().x);
    Serial.print(" TVC Pitch: ");
    Serial.println(tvc.getAngle().y);


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