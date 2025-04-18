// ORIENTATION CALCULATION

#ifndef ORIENTATION_H
#define ORIENTATION_H

#include <math.h>
#include <config.h>
#include <sensors.h>
#include <Kalman.h>
#include <Quaternion.h>

double sign(double x) { return (x > 0) - (x < 0); }

// ========= Vectors ========= //

// For TVC:
// x = yaw servo, y = pitch servo
struct Vec2D {
    float x, y;

    Vec2D() { x = 0; y = 0; }
    Vec2D(float x, float y) : x(x), y(y) {}

};

// 3D Vector
// For Orientation:
// x = roll (cw = -), y = pitch (cw = +), z = yaw (cw = +)
// For Accelerometer:
// x = +up -down, y = +left -right, z = +forward -backward
struct Vec3D {
    float x, y, z;

    Vec3D() { x = 0; y = 0; z = 0; }
    Vec3D(float x, float y, float z) : x(x), y(y), z(z) {}
};

// ========= Angles & Orientation ========= //

// QUATERNION MATH
Quaternion slerp(Quaternion q1, Quaternion q2, float t) {
    float dot =
        q1.a * q2.a
        + q1.b * q2.b
        + q1.c * q2.c
        + q1.d * q2.d;

    // Ensure the shortest path
    if (dot < 0.0f) {
        q2 = -q2;
        dot = -dot;
    }

    // If quaternions are very close, use linear interpolation
    const float THRESHOLD = 0.9995f;
    if (dot > THRESHOLD) {
        Quaternion q = (q1 * (1.0f - t) + q2 * t);
        q.normalize();  // Linear interpolation (LERP)
        return q;
    }

    // Compute SLERP
    float theta_0 = acosf(dot);
    float theta = theta_0 * t;

    float sin_theta = sinf(theta);
    float sin_theta_0 = sinf(theta_0);

    float s1 = cosf(theta) - dot * sin_theta / sin_theta_0;
    float s2 = sin_theta / sin_theta_0;

    Quaternion q = (q1 * s1 + q2 * s2);
    q.normalize();

    return q;
}

float quaternion_to_roll(Quaternion attitude) {
    // --- Convert to Euler Angles --- //    
    // https://www.euclideanspace.com/maths/standards/index.htm#:~:text=Euler%20angles
    // Switch axes (X pitch, Y roll, Z yaw --> Z pitch, X roll, Y yaw)
    float qw = attitude.a;
    float qz = attitude.b;
    float qx = attitude.c;
    float qy = attitude.d;

    // from https://www.euclideanspace.com/maths/standards/index.htm
    if (qx * qy + qz * qw >= 0.5) {  // North pole
        return 0;
    }
    else if (qx * qy + qz * qw <= -0.5) {  // South pole
        return 0;
    }
    else {
        float roll = atan2(2 * qx * qw - 2 * qy * qz, 1 - 2 * qx * qx - 2 * qz * qz);
        return roll * -180 / PI;
    }
}

float quaternion_to_pitch(Quaternion attitude) {
    // --- Convert to Euler Angles --- //    
    // https://www.euclideanspace.com/maths/standards/index.htm#:~:text=Euler%20angles
    // Switch axes (X pitch, Y roll, Z yaw --> Z pitch, X roll, Y yaw)
    float qw = attitude.a;
    float qz = attitude.b;
    float qx = attitude.c;
    float qy = attitude.d;

    // from https://www.euclideanspace.com/maths/standards/index.htm
    if (qx * qy + qz * qw >= 0.5) {  // North pole
        return PI / 2;
    }
    else if (qx * qy + qz * qw <= -0.5) {  // South pole
        return -PI / 2;
    }
    else {
        float pitch = asin(2 * qx * qy + 2 * qz * qw);
        return pitch * 180 / PI;
    }
}

float quaternion_to_yaw(Quaternion attitude) {
    // --- Convert to Euler Angles --- //    
    // https://www.euclideanspace.com/maths/standards/index.htm#:~:text=Euler%20angles
    // Switch axes (X pitch, Y roll, Z yaw --> Z pitch, X roll, Y yaw)
    float qw = attitude.a;
    float qz = attitude.b;
    float qx = attitude.c;
    float qy = attitude.d;

    // from https://www.euclideanspace.com/maths/standards/index.htm
    if (qx * qy + qz * qw >= 0.5) {  // North pole
        return 2 * atan2(qx, qw);
    }
    else if (qx * qy + qz * qw <= -0.5) {  // South pole
        return -2 * atan2(qx, qw);
    }
    else {
        float yaw = atan2(2 * qy * qw - 2 * qx * qz, 1 - 2 * qy * qy - 2 * qz * qz);
        return yaw * 180 / PI;
    }
}

Vec3D quaternion_to_euler(Quaternion attitude) {
    float roll = quaternion_to_roll(attitude);
    float pitch = quaternion_to_pitch(attitude);
    float yaw = quaternion_to_yaw(attitude);
    return Vec3D(roll, pitch, yaw);
}

// GYRO-BASED ANGLE CALCULATION
Vec3D get_angles(SensorReadings r, SensorReadings pr = SensorReadings(), Vec3D dir = Vec3D(0, 0, 0), float dt = 0.02) {
    float x = dir.x - ((r.gy + pr.gy) * dt / 2);
    float y = dir.y + ((r.gx + pr.gx) * dt / 2);
    float z = dir.z - ((r.gz + pr.gz) * dt / 2);

    x = x * 0.85 + dir.x * 0.15;
    y = y * 0.85 + dir.y * 0.15;
    z = z * 0.85 + dir.z * 0.15;

    return Vec3D(x, y, z);
}

// COMPLEMENTARY FILTERING
Vec2D get_angles_complementary(float A, float dt, SensorReadings r, float yaw, float pitch) {

    // Orientation w = local_to_global(r, b, yaw, pitch);
    Vec2D w = Vec2D(r.gz, r.gx);


    float accel_angle_x = -atan2(r.ax, sqrt(r.az * r.az + r.ay * r.ay)) * 180 / PI;
    float gyro_angle_x = yaw - (w.x) * dt;
    float angle_x = accel_angle_x * (1.0 - A) + gyro_angle_x * A;

    float accel_angle_y = -atan2(r.az, r.ay) * 180 / PI;
    float gyro_angle_y = pitch + (w.y) * dt;
    float angle_y = accel_angle_y * (1.0 - A) + gyro_angle_y * A;


    return Vec2D(angle_x, angle_y);
}

// QUATERNION BASED ANGLE CALCULATION
Quaternion get_angles_quat(SensorReadings readings, Quaternion& attitude, float deltaTime) {

    // wx = pitch  (rotation around x-axis)
    // wy = roll   (rotation around y-axis) 
    // wz = yaw    (rotation around z-axis)
    float wx = readings.gx * (PI / 180);
    float wy = readings.gy * (PI / 180);
    float wz = readings.gz * (PI / 180);

    // Update attitude quaternion
    float norm = sqrt(wx * wx + wy * wy + wz * wz);
    norm = copysignf(max(abs(norm), 1e-9f), norm); // NO DIVIDE BY 0

    wx /= norm;
    wy /= norm;
    wz /= norm;

    Quaternion QM = Quaternion::from_axis_angle(deltaTime * norm, wx, wy, wz);
    attitude = attitude * QM;

    return attitude;
}

// COMPLEMENTARY FILTERING WITH QUATERNIONS
Quaternion get_angles_compl_quat(float A, float dt, SensorReadings r, Quaternion& attitude) {

    float accelYaw = atan2(r.ax, sqrt(r.az * r.az + r.ay * r.ay));
    float accelPitch = -atan2(r.az, sqrt(r.az * r.az + r.ay * r.ay));

    Quaternion gyro_q = get_angles_quat(r, attitude, dt);
    float roll = quaternion_to_roll(gyro_q) * (PI / 180);

    Quaternion accel_q = Quaternion::from_euler_rotation(accelYaw, accelPitch, -roll);

    attitude = slerp(gyro_q, accel_q, 1.0f - A);

    attitude.normalize();

    return attitude;
}   

// KALMAN FILTERING
Vec2D get_angles_kalman(float dt, SensorReadings r, Kalman& kx, Kalman& ky, GyroBiases b) {
    float accel_angle_x = atan2(r.ax, -sign(r.ay) * sqrt(r.az * r.az + r.ay * r.ay)) * 180 / PI;
    float accel_angle_y = atan2(r.az, -r.ay) * 180 / PI;

    float kal_x = kx.getAngle(accel_angle_x, r.gz - b.bz, dt);
    float kal_y = ky.getAngle(accel_angle_y, r.gx - b.bx, dt);

    return Vec2D(kal_x, kal_y);

}

#endif