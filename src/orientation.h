// ORIENTATION CALCULATION

#include <math.h>
#include <MadgwickAHRS.h>
#include <Arduino_BMI270_BMM150.h>
#include <Kalman.h>

#define PI 3.14159265358979323846

// ========= Angles & Orientation ========= //
double TAU = 1;

double DELTA_TIME = 0.01; // Time step
double GYRO_BIAS_X, GYRO_BIAS_Y, GYRO_BIAS_Z;

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

struct Biases {
    double bx, by, bz;
    Biases(){bx = by = bz = 0;};
    Biases(float x, float y, float z) : bx(x), by(y), bz(z) {}

};

Biases calibrateGyro() {

    Serial.println("Starting Gyro Calibration...");

    digitalWrite(LED_BUILTIN, HIGH);
    long long now = micros();
    double x_angle_c, y_angle_c, z_angle_c;
    x_angle_c = y_angle_c = z_angle_c = 0;
    double dt = 0.005;
    SensorReadings r;
    long long lastM = micros();
    while (micros() - now < 10000000LL) {

        IMU.readGyroscope(r.gy, r.gx, r.gz);
        x_angle_c += dt * r.gx;
        y_angle_c += dt * r.gy;
        z_angle_c += dt * r.gz;
        dt = (micros() - lastM) / 1000000.0;
        lastM = micros();
    }

    Serial.println(y_angle_c);
    Serial.println(micros() - now);

    Serial.print("Calibration Complete. \n New Bias Estimate: ");

    float bx = ((x_angle_c) / (double)(micros() - now)) * 1000000;
    float by = ((y_angle_c) / (double)(micros() - now)) * 1000000;
    float bz = ((z_angle_c) / (double)(micros() - now)) * 1000000;
    digitalWrite(LED_BUILTIN, LOW);

    return Biases(bx, by, bz);
}

const double G = 9.80665;

// COMPLEMENTARY FILTERING (Written by hand)
Orientation get_angles_complementary(double dt, SensorReadings r, double lastX, double lastY, Biases b) {

    double accel_angle_x = atan2(r.ay, -r.ax) * 180 / PI;
    double gyro_angle_x = lastX + (r.gz - b.bz) * dt;

    double angle_x = accel_angle_x * (1.0 - TAU) + gyro_angle_x * TAU;

    double accel_angle_y = atan2(r.az, -r.ax) * 180 / PI;
    double gyro_angle_y = lastY + (r.gx - b.bx) * dt;

    double angle_y = accel_angle_y * (1.0 - TAU) + gyro_angle_y * TAU;

    return Orientation(angle_x, angle_y);
}

// MADGWICK FILTERING (Using library Madgwick)
Orientation get_angles_madgwick(SensorReadings r, Madgwick& f, Biases b) {


    f.updateIMU(
        r.gy - b.by, r.gx - b.bx, r.gz - b.bz,
        r.ay, r.ax, r.az);

    double angle_x = f.getRoll();
    double angle_y = f.getYaw();

    return Orientation(angle_x, angle_y);
}

Orientation get_angles_kalman(double dt, SensorReadings r, Kalman& kx, Kalman& ky, Biases b) {
    double accel_angle_x = atan2(r.ay, -r.ax) * 180 / PI;
    double accel_angle_y = atan2(r.az, -r.ax) * 180 / PI;
    // Serial.print(r.ax);
    // Serial.print(" ");
    // Serial.print(r.ay);
    // Serial.print(" ");
    // Serial.println(r.az);


    double kal_x = kx.getAngle(accel_angle_x, r.gz - b.bz, dt);
    double kal_y = ky.getAngle(accel_angle_y, r.gx - b.bx, dt);

    return Orientation(kal_x, kal_y);

}