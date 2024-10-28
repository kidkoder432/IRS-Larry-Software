#include <Arduino.h>
#include <Serial.h>
#include <orientation.h>
#include <Arduino_BMI270_BMM150.h>
#include <leds.h>


double yaw, pitch;
SensorReadings readings;
Vec2D dir;
Biases biases;

double ALPHA = 0.05;

void setup() {
    Serial.begin(9600);
    delay(200);
    IMU.begin();

    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(LEDR, OUTPUT);
    pinMode(LEDG, OUTPUT);
    pinMode(LEDB, OUTPUT);
    biases = calibrateSensors();
    Serial.print(biases.bx);
    Serial.print(" ");
    Serial.print(biases.by);
    Serial.print(" ");
    Serial.println(biases.bz);

    double ax, ay, az;
    IMU.readAcceleration(ay, ax, az);
    yaw = atan2(ax, -sign(ay) * sqrt(az * az + ay * ay)) * 180 / PI;
    pitch = atan2(az, -ay) * 180 / PI;

}

long long lastMicros = micros();
void loop() {
    // --- Read Sensors --- //
    IMU.readAcceleration(readings.ay, readings.ax, readings.az);
    IMU.readGyroscope(readings.gy, readings.gx, readings.gz);

    // Serial.print(readings.gx - biases.bx);
    // Serial.print(" ");
    // Serial.print(readings.gy);
    // Serial.print(" ");
    // Serial.println(readings.gz - biases.bz);

    double mag3 = sqrtf(pow(readings.ax, 2) + pow(readings.ay, 2) + pow(readings.az, 2));

    if (mag3 < 0.95 || mag3 > 1.05) {
        digitalWrite(LED_BUILTIN, HIGH);
        ALPHA = 0;
    }

    else {
        digitalWrite(LED_BUILTIN, LOW);
        ALPHA = 0.1;
    }

    Vec2D dir = get_angles_complementary(1 - ALPHA, DELTA_TIME, readings, yaw, pitch, biases);
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
    DELTA_TIME = (micros() - lastMicros) / 1000000.;
    lastMicros = micros();
}