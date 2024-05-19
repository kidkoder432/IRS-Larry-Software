// ORIENTATION CALCULATION

#define PI 3.14159265358979323846
#include <math.h>
#include <MadgwickAHRS.h>
#include <Arduino_BMI270_BMM150.h>


using namespace std;

// ========= Angles & Orientation ========= //
double TAU = 0.99;

double DELTA_TIME = 0.01; // Time step
double GYRO_BIAS_PER_SEC = 0.0615933787118;

// ========= Sensor Variables ========= //
struct SensorReadings {
    float ax, ay, az;
    float gx, gy, gz;
    float mx, my, mz;

};

struct Orientation {
    double angle_x, angle_y;

    Orientation() { angle_x = 0; angle_y = 0; }
    Orientation(double x, double y) : angle_x(x), angle_y(y) {}

};

void calibrateGyro() {
    Serial.println("Starting Gyro Calibration...");
    digitalWrite(LED_BUILTIN, HIGH);
    long long now = micros();
    double y_angle_c = 0;
    double dt = 0.005;
    SensorReadings r;
    long long lastM = micros();
    while (micros() - now < (long long)10000000) {

        IMU.readGyroscope(r.gx, r.gy, r.gz);
        Serial.print(r.gy);
        Serial.print(" ");
        Serial.println(y_angle_c);
        y_angle_c += dt * r.gy;
        dt = (micros() - lastM) / 1000000.0;
        lastM = micros();
    }

    Serial.println(y_angle_c);
    Serial.println(micros() - now);

    Serial.print("Calibration Complete. \n New Bias Estimate: ");

    GYRO_BIAS_PER_SEC = ((y_angle_c) / (double)(micros() - now)) * 1000000;
    Serial.println(GYRO_BIAS_PER_SEC);
    digitalWrite(LED_BUILTIN, LOW);
}

const double G = 9.80665;

// COMPLEMENTARY FILTERING (Written by hand)
Orientation get_angles_complementary(SensorReadings r, double lastX, double lastY) {

    double accel_angle_x = atan2(r.ay, r.az) * 180 / PI;
    double gyro_angle_x = lastX + (r.gx - GYRO_BIAS_PER_SEC) * DELTA_TIME;

    double angle_x = accel_angle_x * (1.0 - TAU) + gyro_angle_x * TAU;

    double accel_angle_y = atan2(r.ax, r.az) * 180 / PI;
    double gyro_angle_y = lastY + (r.gy - GYRO_BIAS_PER_SEC) * DELTA_TIME;

    double angle_y = accel_angle_y * (1.0 - TAU) + gyro_angle_y * TAU;

    return Orientation(angle_x, angle_y);
}

// MADGWICK FILTERING (Using library Madgwick)
Orientation get_angles_madgwick(double gb, SensorReadings r, Madgwick& f) {

    GYRO_BIAS_PER_SEC = gb;

    f.updateIMU(
        r.gy - GYRO_BIAS_PER_SEC, r.gx, r.gz - GYRO_BIAS_PER_SEC,
        r.ay, r.ax, r.az);

    double angle_x = f.getRoll();
    double angle_y = f.getYaw();

    return Orientation(angle_x, angle_y);
}
