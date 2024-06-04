#include <Arduino.h>
#include <Serial.h>
#include <orientation.h>
#include <Arduino_BMI270_BMM150.h>
#include <leds.h>


double x_angle, y_angle;
SensorReadings readings;
Orientation dir;
Biases biases;

double ALPHA = 0.01;

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
    x_angle = 0;
    y_angle = atan2(az, -ay) * 180 / PI;


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

    Orientation dir = get_angles_complementary(1-ALPHA, DELTA_TIME, readings, x_angle, y_angle, biases);
    x_angle = -dir.angle_x;
    y_angle = dir.angle_y;

    if (x_angle > 180) {
        x_angle = x_angle - 360;
    }
    else if (x_angle < -180) {
        x_angle = x_angle + 360;
    }

    if (y_angle > 180) {   
        y_angle = y_angle - 360;
    }

    else if (y_angle < -180) {
        y_angle = y_angle + 360;
    }

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