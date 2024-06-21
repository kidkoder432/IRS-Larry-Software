#include <Arduino.h>
#include <Serial.h>
#include <orientation.h>
#include <Arduino_BMI270_BMM150.h>
#include <leds.h>


double roll, pitch;
SensorReadings readings;
Orientation dir;
Biases biases;

double ALPHA = 0.05;

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
    roll = atan2(ax, -sign(ay) * sqrt(az * az + ay * ay)) * 180 / PI;
    pitch = atan2(az, -ay) * 180 / PI;
    pinMode(3, INPUT_PULLUP);

}

long long lastMicros = micros();
void loop() {
    // --- Read Sensors --- //
    IMU.readAcceleration(readings.ay, readings.ax, readings.az);
    IMU.readGyroscope(readings.gy, readings.gx, readings.gz);
    IMU.readMagneticField(readings.mx, readings.my, readings.mz);

    // Serial.print(readings.gx - biases.bx);
    // Serial.print(" ");
    // Serial.print(readings.gy);
    // Serial.print(" ");
    // Serial.println(readings.gz - biases.bz);

    Orientation dir = get_angles_complementary(1 - ALPHA, DELTA_TIME, readings, roll, pitch, biases);
    roll = dir.roll;
    pitch = dir.pitch;

    if (roll > 180) {
        roll = roll - 360;
    }
    else if (roll < -180) {
        roll = roll + 360;
    }

    if (pitch > 180) {
        pitch = pitch - 360;
    }

    else if (pitch < -180) {
        pitch = pitch + 360;
    }

    Serial.print(roll);
    Serial.print(" ");
    Serial.println(pitch);

    if (digitalRead(3) == LOW) {
        delay(20);
        if (digitalRead(3) == LOW) {
            showColor(COLOR_YELLOW);
            roll = pitch = 0;
        }
    }

    if (abs(roll) >= 15) {
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