// ORIENTATION CALCULATION

#include <math.h>
#include <Arduino_BMI270_BMM150.h>
#include <Kalman.h>

// ========= Angles & Orientation ========= //

double DELTA_TIME = 0.01; // Time step

// ========= Sensor Variables ========= //
struct SensorReadings {
    float ax, ay, az;
    float gx, gy, gz;

};
 
struct Orientation {
    double yaw, pitch;

    Orientation() { yaw = 0; pitch = 0; }
    Orientation(double x, double y) : yaw(x), pitch(y) {}

};

int sign(double x) {
    if (x == 0) return 0;
    return x > 0 ? 1 : -1;
}

struct Biases {
    double bx, by, bz;
    Biases() { bx = by = bz = 0; };
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
    while (micros() - now < 3000000LL) {

        IMU.readGyroscope(r.gy, r.gx, r.gz);
        x_angle_c += dt * r.gx;
        y_angle_c += dt * r.gy;
        z_angle_c += dt * r.gz;
        dt = (micros() - lastM) / 1000000.0;
        lastM = micros();
    }

    Serial.println(y_angle_c);
    Serial.println(micros() - now);

    float bx = ((x_angle_c) / (double)(micros() - now)) * 1000000;
    float by = ((y_angle_c) / (double)(micros() - now)) * 1000000;
    float bz = ((z_angle_c) / (double)(micros() - now)) * 1000000;
    digitalWrite(LED_BUILTIN, LOW);

    return Biases(bx, by, bz);
}

Orientation local_to_global(SensorReadings r, Biases b, double yaw, double pitch) {
    double p = r.gx - b.bx * PI / 180;
    double q = r.gy - b.by * PI / 180;
    double r_ = r.gz - b.bz * PI / 180;

    double phidot = p + q * sin(yaw) * tan(pitch) + r_ * cos(yaw) * tan(pitch);
    double thetadot = q * cos(yaw) - r_ * sin(yaw);

    return Orientation(phidot * 180 / PI, thetadot * 180 / PI);

}

// COMPLEMENTARY FILTERING (Written by hand)
Orientation get_angles_complementary(double A, double dt, SensorReadings r, double yaw, double pitch, Biases b) {

    // Orientation w = local_to_global(r, b, yaw, pitch);
    Orientation w = Orientation(r.gz - b.bz, r.gx - b.bx);
    double accel_angle_x = atan2(r.ax, -sign(r.ay) * sqrt(r.az * r.az + r.ay * r.ay)) * 180 / PI;
    double gyro_angle_x = yaw - (w.yaw) * dt;

    double angle_x = accel_angle_x * (1.0 - A) + gyro_angle_x * A;

    double accel_angle_y = atan2(r.az, -r.ay) * 180 / PI;
    double gyro_angle_y = pitch + (w.pitch) * dt;

    double angle_y = accel_angle_y * (1.0 - A) + gyro_angle_y * A;

    return Orientation(angle_x, angle_y);
}


// KALMAN FILTERING
Orientation get_angles_kalman(double dt, SensorReadings r, Kalman& kx, Kalman& ky, Biases b) {
    double accel_angle_x = atan2(r.ax, -r.ay) * 180 / PI;
    double accel_angle_y = atan2(r.az, -r.ay) * 180 / PI;

    double kal_x = kx.getAngle(accel_angle_x, r.gz - b.bz, dt);
    double kal_y = ky.getAngle(accel_angle_y, r.gx - b.bx, dt);

    return Orientation(kal_x, kal_y);

}

