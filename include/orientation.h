// ORIENTATION CALCULATION

#ifndef ORIENTATION_H
#define ORIENTATION_H

#include <math.h>
#include <SparkFun_BMI270_Arduino_Library.h>
#include <Arduino_LPS22HB.h>
#include <config.h>


#include <Kalman.h>
#include <Quaternion.h>

// ========= Angles & Orientation ========= //

BMI270 imu;

// ========= Sensor Variables ========= //
struct SensorReadings {
    float ax, ay, az;
    float gx, gy, gz;

    SensorReadings() { ax = ay = az = 0; gx = gy = gz = 0; }

};

// 2D Vector

// For TVC:
// x = yaw servo, y = pitch servo
struct Vec2D {
    double x, y;

    Vec2D() { x = 0; y = 0; }
    Vec2D(double x, double y) : x(x), y(y) {}

};

// 3D Vector
// For Orientation:
// x = roll (cw = -), y = pitch (cw = +), z = yaw (cw = +)
// For Accelerometer:
// x = +up -down, y = +left -right, z = +forward -backward
struct Vec3D {
    double x, y, z;

    Vec3D() { x = 0; y = 0; z = 0; }
    Vec3D(double x, double y, double z) : x(x), y(y), z(z) {}
};

int sign(double x) {
    if (x == 0) return 0;
    return x > 0 ? 1 : -1;
}

struct Biases {
    double bx, by, bz;
    Biases() { bx = by = bz = 0; };
    Biases(double x, double y, double z) : bx(x), by(y), bz(z) {}

};

void wwinitSensors() {
    Wire1.begin();
    imu.beginI2C(0x68, Wire1);

    int err;

    // Configure accelerometer
    bmi2_sens_config accConfig;
    accConfig.type = BMI2_ACCEL;
    accConfig.cfg.acc.odr = BMI2_ACC_ODR_400HZ;
    accConfig.cfg.acc.range = BMI2_ACC_RANGE_16G;
    accConfig.cfg.acc.bwp = BMI2_ACC_OSR4_AVG1;
    accConfig.cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
    err = imu.setConfig(accConfig);
    Serial.println(err);

    // Configure gyroscope
    bmi2_sens_config gyroConfig;
    gyroConfig.type = BMI2_GYRO;
    gyroConfig.cfg.gyr.odr = BMI2_GYR_ODR_400HZ;
    gyroConfig.cfg.gyr.range = BMI2_GYR_RANGE_2000;
    gyroConfig.cfg.gyr.bwp = BMI2_GYR_OSR4_MODE;
    gyroConfig.cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
    gyroConfig.cfg.gyr.noise_perf = BMI2_PERF_OPT_MODE;
    err += imu.setConfig(gyroConfig);
    Serial.println(err);

    BARO.begin();
}

void readSensors(SensorReadings& r, Biases biases) {
    imu.getSensorData();

    r.ax = imu.data.accelX;
    r.ay = imu.data.accelY;
    r.az = imu.data.accelZ;

    r.gx = imu.data.gyroX - biases.bx;
    r.gy = imu.data.gyroY - biases.by;
    r.gz = imu.data.gyroZ - biases.bz;
}

Biases calibrateSensors(Config& config) {

    Serial.println("Starting Sensor Calibration...");

    // Accelerometer + gyroscope CRT calibration
    if (config["DO_CRT"] > 0) {
        Serial.println("Performing component retrimming...");
        imu.performComponentRetrim();
    }

    // Accelerometer FOC calibration
    if (config["DO_ACCEL_FOC"] > 0) {
        Serial.println("Performing acclerometer offset calibration...");
        imu.performAccelOffsetCalibration(BMI2_GRAVITY_POS_Y);
    }

    // Gyroscope FOC calibration
    if (config["DO_GYRO_FOC"] > 0) {
        Serial.println("Performing gyroscope offset calibration...");
        imu.performGyroOffsetCalibration();
    }

    Serial.println();
    Serial.println("Internal calibration complete!");
    Serial.println("Starting external gyroscope offset calibration...");

    long long now = micros();
    double x_angle_c, y_angle_c, z_angle_c;
    x_angle_c = y_angle_c = z_angle_c = 0;
    double dt = 0.005;
    SensorReadings r;
    long long lastM = micros();
    while (micros() - now < 3000000LL) {
        imu.getSensorData();

        r.gx = imu.data.gyroX;
        r.gy = imu.data.gyroY;
        r.gz = imu.data.gyroZ;
        x_angle_c += dt * r.gx;
        y_angle_c += dt * r.gy;
        z_angle_c += dt * r.gz;
        dt = (micros() - lastM) / 1000000.0;
        lastM = micros();
    }

    Serial.println(y_angle_c);
    Serial.println(micros() - now);

    double bx = ((x_angle_c) / (double)(micros() - now)) * 1000000;
    double by = ((y_angle_c) / (double)(micros() - now)) * 1000000;
    double bz = ((z_angle_c) / (double)(micros() - now)) * 1000000;

    return Biases(bx, by, bz);
}

// GYRO-BASED ANGLE CALCULATION
Vec3D get_angles(SensorReadings r, SensorReadings pr = SensorReadings(), Vec3D dir = Vec3D(0, 0, 0), double dt = 0.02) {
    double x = dir.x - ((r.gy + pr.gy) * dt / 2);
    double y = dir.y + ((r.gx + pr.gx) * dt / 2);
    double z = dir.z - ((r.gz + pr.gz) * dt / 2);

    x = x * 0.85 + dir.x * 0.15;
    y = y * 0.85 + dir.y * 0.15;
    z = z * 0.85 + dir.z * 0.15;

    return Vec3D(x, y, z);
}

// COMPLEMENTARY FILTERING
Vec2D get_angles_complementary(double A, double dt, SensorReadings r, double yaw, double pitch, Biases b) {

    // Orientation w = local_to_global(r, b, yaw, pitch);
    Vec2D w = Vec2D(r.gz - b.bz, r.gx - b.bx);

    double accel_angle_x = atan2(r.ax, -sign(r.ay) * sqrt(r.az * r.az + r.ay * r.ay)) * 180 / PI;
    double gyro_angle_x = yaw - (w.x) * dt;
    double angle_x = accel_angle_x * (1.0 - A) + gyro_angle_x * A;

    double accel_angle_y = atan2(r.az, -r.ay) * 180 / PI;
    double gyro_angle_y = pitch + (w.y) * dt;
    double angle_y = accel_angle_y * (1.0 - A) + gyro_angle_y * A;

    return Vec2D(angle_x, angle_y);
}


// KALMAN FILTERING
Vec2D get_angles_kalman(double dt, SensorReadings r, Kalman& kx, Kalman& ky, Biases b) {
    double accel_angle_x = atan2(r.ax, -sign(r.ay) * sqrt(r.az * r.az + r.ay * r.ay)) * 180 / PI;
    double accel_angle_y = atan2(r.az, -r.ay) * 180 / PI;

    double kal_x = kx.getAngle(accel_angle_x, r.gz - b.bz, dt);
    double kal_y = ky.getAngle(accel_angle_y, r.gx - b.bx, dt);

    return Vec2D(kal_x, kal_y);

}


// QUATERNION BASED ANGLE CALCULATION
// Returns roll, pitch, yaw
Vec3D get_angles_quat(SensorReadings readings, Quaternion& attitude, double deltaTime) {
    double wx = readings.gx * (PI / 180);
    double wy = readings.gy * (PI / 180);
    double wz = readings.gz * (PI / 180);

    double roll, pitch, yaw;

    // Update attitude quaternion
    double norm = sqrt(wx * wx + wy * wy + wz * wz);
    norm = copysignf(max(abs(norm), 1e-9), norm); // NO DIVIDE BY 0

    wx /= norm;
    wy /= norm;
    wz /= norm;

    Quaternion QM = Quaternion::from_axis_angle(deltaTime * norm, wx, wy, wz);
    attitude = attitude * QM;

    // --- Convert to Euler Angles --- //    
    // Switch axes (X pitch, Y roll, Z yaw --> Z pitch, X roll, Y yaw)
    double qw = attitude.a;
    double qz = attitude.b;
    double qx = attitude.c;
    double qy = attitude.d;

    // from https://www.euclideanspace.com/maths/standards/index.htm
    if (qx * qy + qz * qw >= 0.5) {  // North pole
        yaw = 2 * atan2(qx, qw);
        pitch = PI / 2;
        roll = 0;
    }
    else if (qx * qy + qz * qw <= -0.5) {  // South pole
        yaw = -2 * atan2(qx, qw);
        pitch = -PI / 2;
        roll = 0;
    }
    else {
        yaw = atan2(2 * qy * qw - 2 * qx * qz, 1 - 2 * qy * qy - 2 * qz * qz);
        pitch = asin(2 * qx * qy + 2 * qz * qw);
        roll = atan2(2 * qx * qw - 2 * qy * qz, 1 - 2 * qx * qx - 2 * qz * qz);
    }

    attitude = attitude.normalize();

    roll *= -180 / PI;
    pitch *= -180 / PI;
    yaw *= -180 / PI;

    return Vec3D(roll, pitch, yaw);
}

#endif