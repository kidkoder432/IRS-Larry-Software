#include <Arduino.h>
#include <Serial.h>
#include <orientation.h>
#include <Arduino_BMI270_BMM150.h>
#include <leds.h>
#include <Quaternion.h>


SensorReadings readings;
Orientation dir;
Biases biases;
Quaternion attitude;

double yaw, pitch;

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
    attitude = Quaternion();  // pointing forward and flat
    pinMode(3, INPUT_PULLUP);

}

long long lastMicros = micros();
void loop() {
    // --- Read Sensors --- //
    IMU.readAcceleration(readings.ay, readings.ax, readings.az);
    IMU.readGyroscope(readings.gx, readings.gy, readings.gz);

    // Serial.print(readings.gx - biases.bx);
    // Serial.print(" ");
    // Serial.print(readings.gy);
    // Serial.print(" ");
    // Serial.println(readings.gz - biases.bz);

    double wx = (readings.gx - biases.bx) * PI / 180;
    double wy = (readings.gy - biases.by) * PI / 180;
    double wz = (readings.gz - biases.bz) * PI / 180;

    float norm = sqrt(pow(wx, 2) + pow(wy, 2) + pow(wz, 2));


    Quaternion QM = Quaternion::from_axis_angle(norm * DELTA_TIME, wx / norm, wy / norm, wz / norm);

    attitude *= QM;

    pitch = asin(2.0f * (attitude.a * attitude.c - attitude.b * attitude.d));
    yaw = atan2(2.0f * (attitude.a * attitude.b + attitude.c * attitude.d),
        1 - 2.0f * (attitude.b * attitude.b + attitude.c * attitude.c));
    // if (abs(pitch) == PI/2) {
    //     yaw = 0;
    // }

    pitch *= 180 / PI;
    yaw *= 180 / PI;

    Serial.print(yaw);
    Serial.print(" ");
    Serial.print(pitch);
    Serial.println(" ");

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

    // Reset attitude if button is pressed (handle axis change with manual override)
    if (digitalRead(3) == LOW) {
        delay(20);
        if (digitalRead(3) == LOW) {
            showColor(COLOR_YELLOW);
            attitude = Quaternion();
        }
    }


    delay(10);
    DELTA_TIME = (micros() - lastMicros) / 1000000.;
    lastMicros = micros();
}