#include <Arduino.h>
#include <Kalman.h>
#include <Serial.h>
#include <orientation.h>
#include <Arduino_BMI270_BMM150.h>
#include <leds.h>


Kalman kx, ky;
double roll, pitch;
SensorReadings readings;
Orientation dir;
Biases biases;

void setup() {
    Serial.begin(9600);
    delay(200);
    IMU.begin();

    float ax, ay, az;
    IMU.readAcceleration(ay, ax, az);
    Serial.println(atan2(az, -ay) * 180 / PI);
    roll = atan2(ax, -ay) * 180 / PI;
    pitch = atan2(az, -ay) * 180 / PI;

    kx.setAngle(roll);
    ky.setAngle(pitch);

    pinMode(LED_BUILTIN, OUTPUT);
    biases = calibrateGyro();
    Serial.print(biases.bx);
    Serial.print(" ");
    Serial.print(biases.by);
    Serial.print(" ");
    Serial.println(biases.bz);


}

long long lastMicros = micros();
void loop() {
    // --- Read Sensors --- //
    IMU.readAcceleration(readings.ay, readings.ax, readings.az);
    IMU.readGyroscope(readings.gx, readings.gy, readings.gz);
    IMU.readMagneticField(readings.mx, readings.my, readings.mz);


    // --- Angle Calc --- //
    // dir = get_angles_kalman(DELTA_TIME, readings, kx, ky, biases);
    // dir = get_angles_complementary(DELTA_TIME, readings, x_angle, y_angle, biases);

    // x_angle = dir.angle_x;
    // y_angle = dir.angle_y;

    roll += (readings.gz - biases.bz) * DELTA_TIME;
    pitch += (readings.gx - biases.bx) * DELTA_TIME;


    Serial.print(roll);
    Serial.print(" ");
    Serial.println(pitch);

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
}