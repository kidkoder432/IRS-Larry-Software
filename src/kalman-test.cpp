#include <Arduino.h>
#include <Kalman.h>
#include <Serial.h>
#include <orientation.h>
#include <Arduino_BMI270_BMM150.h>

Kalman kx, ky;
double x_angle, y_angle;
SensorReadings readings;
Orientation dir;

void setup() {
    Serial.begin(9600);
    delay(200);
    IMU.begin();
    
    float ax, ay, az;
    IMU.readAcceleration(ax, ay, az);
    Serial.println(atan2(ax, az) * 180 / PI);
    x_angle = atan2(ay, ax) * 180 / PI - 90;
    y_angle = atan2(ax, az) * 180 / PI - 90;

    kx.setAngle(x_angle);
    ky.setAngle(y_angle);

    pinMode(LED_BUILTIN, OUTPUT);
    calibrateGyro();

    
}

long long lastMicros = micros();
void loop() {
    // --- Read Sensors --- //
    IMU.readAcceleration(readings.ax, readings.ay, readings.az);
    IMU.readGyroscope(readings.gx, readings.gy, readings.gz);
    IMU.readMagneticField(readings.mx, readings.my, readings.mz);

    
    // --- Angle Calc --- //
    dir = get_angles_kalman(DELTA_TIME, GYRO_BIAS_PER_SEC, readings, kx, ky);

    x_angle = dir.angle_x;
    y_angle = dir.angle_y;
    

    Serial.print(x_angle);
    Serial.print(" -x  y- ");
    Serial.println(y_angle);

    delay(10);
    DELTA_TIME = (micros() - lastMicros) / 1000000.;
}