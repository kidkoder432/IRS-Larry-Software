#include <Arduino.h>
#include <Serial.h>
#include <orientation.h>
#include <Arduino_BMI270_BMM150.h>
#include <leds.h>


double x_angle, y_angle;
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
    x_angle = atan2(ax, -ay) * 180 / PI;
    y_angle = atan2(az, -ay) * 180 / PI;


}

long long lastMicros = micros();
void loop() {
    // --- Read Sensors --- //
    IMU.readAcceleration(readings.ay, readings.ax, readings.az);
    IMU.readGyroscope(readings.gy, readings.gx, readings.gz);
    IMU.readMagneticField(readings.mx, readings.my, readings.mz);

    x_angle += (readings.gz - biases.bz) * DELTA_TIME;
    y_angle += (readings.gx - biases.bx + 0.002) * DELTA_TIME;

    // Serial.print(readings.gx - biases.bx);
    // Serial.print(" ");
    // Serial.print(readings.gy);
    // Serial.print(" ");
    // Serial.println(readings.gz - biases.bz);

    Serial.print(x_angle);
    Serial.print(" ");
    Serial.println(y_angle);

    if (abs(x_angle) >= 15) {
        showColor(COLOR_RED);
        if (abs(y_angle) >= 15) {
            showColor(COLOR_PURPLE);
        }
    }

    else {
        showColor(COLOR_GREEN);
        if (abs(y_angle) >= 15) {
            showColor(COLOR_LIGHTBLUE);
        }
    }


    delay(10);
    DELTA_TIME = (micros() - lastMicros) / 1000000.;
    lastMicros = micros();
}