// ORIENTATION CALCULATION

#ifndef ORIENTATION_H
#define ORIENTATION_H

#include <math.h>
#include <config.h>
#include <sensors.h>
#include <Kalman.h>
#include <Quaternion.h>

float sign(float x) { return (x > 0) - (x < 0); }
float clamp(float x, float min, float max) { return (x < min) ? min : (x > max) ? max : x; }

Quaternion from_euler_xyz(float roll, float pitch, float yaw) {
    // Half angles for convenience
    float half_roll = roll * 0.5f;
    float half_pitch = pitch * 0.5f;
    float half_yaw = yaw * 0.5f;

    // Compute cosines and sines of half angles
    float cos_roll = cosf(half_roll);
    float sin_roll = sinf(half_roll);

    float cos_pitch = cosf(half_pitch);
    float sin_pitch = sinf(half_pitch);

    float cos_yaw = cosf(half_yaw);
    float sin_yaw = sinf(half_yaw);

    Quaternion q;
    // According to XYZ intrinsic rotation (roll->pitch->yaw)
    q.a = cos_roll * cos_pitch * cos_yaw - sin_roll * sin_pitch * sin_yaw; // w
    q.b = sin_roll * cos_pitch * cos_yaw + cos_roll * sin_pitch * sin_yaw; // x
    q.c = cos_roll * sin_pitch * cos_yaw - sin_roll * cos_pitch * sin_yaw; // y
    q.d = cos_roll * cos_pitch * sin_yaw + sin_roll * sin_pitch * cos_yaw; // z

    return q;
}

float GIMBAL_LOCK_THRESHOLD = 0.99999f;

// ========= Vectors ========= //

// For TVC:
// x = yaw servo, y = pitch servo
struct Vec2D {
    float x, y;

    Vec2D() { x = 0; y = 0; }
    Vec2D(float x, float y) : x(x), y(y) {}
    Vec2D(const Vec2D& v) : x(v.x), y(v.y) {}

};

// 3D Vector
// For Orientation:
// x = roll, y = pitch, z = yaw
// For Accelerometer:
// x = +up -down, y = +left -right, z = +forward -backward
struct Vec3D {
    float x, y, z;

    Vec3D() { x = 0; y = 0; z = 0; }
    Vec3D(float x, float y, float z) : x(x), y(y), z(z) {}
    Vec3D(const Vec3D& v) : x(v.x), y(v.y), z(v.z) {}
};

// ========= Angles & Orientation ========= //

// QUATERNION MATH
Quaternion slerp(Quaternion q1, Quaternion q2, float t) {
    float dot = q1.a * q2.a + q1.b * q2.b + q1.c * q2.c + q1.d * q2.d;

    if (dot < 0.0f) {
        q2 = -q2; // Assuming Quaternion class has unary minus operator
        dot = -dot;
    }

    const float THRESHOLD = 0.9995f;
    if (dot > THRESHOLD) {
        // Linear interpolation (LERP) for very close quaternions
        // q = q1*(1-t) + q2*t
        Quaternion result_q;
        result_q.a = q1.a * (1.0f - t) + q2.a * t;
        result_q.b = q1.b * (1.0f - t) + q2.b * t;
        result_q.c = q1.c * (1.0f - t) + q2.c * t;
        result_q.d = q1.d * (1.0f - t) + q2.d * t;
        result_q.normalize();
        return result_q;
    }

    float theta_0 = acosf(dot);        // Angle between input quaternions
    float theta = theta_0 * t;         // Angle for interpolation

    float sin_theta = sinf(theta);
    float sin_theta_0 = sinf(theta_0);

    // Avoid division by zero if sin_theta_0 is extremely small (though dot > THRESHOLD should catch this)
    if (fabsf(sin_theta_0) < 1e-9f) {
        return q1; // Or q2, if t is closer to 1
    }

    float s1_coeff = cosf(theta) - dot * sin_theta / sin_theta_0; // More stable form: sinf((1.0f - t) * theta_0) / sin_theta_0;
    float s2_coeff = sin_theta / sin_theta_0;                    // More stable form: sinf(t * theta_0) / sin_theta_0;

    Quaternion result_q;
    result_q.a = q1.a * s1_coeff + q2.a * s2_coeff;
    result_q.b = q1.b * s1_coeff + q2.b * s2_coeff;
    result_q.c = q1.c * s1_coeff + q2.c * s2_coeff;
    result_q.d = q1.d * s1_coeff + q2.d * s2_coeff;
    result_q.normalize(); // SLERP should produce a unit quaternion if inputs are unit, but good practice.
    return result_q;
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

// RADIANS
Vec3D get_angles_accel(SensorReadings r) {
    float yaw = -atan2(r.ay, r.ax);
    float pitch = atan2(r.az, sqrt(r.ax * r.ax + r.ay * r.ay));

    return Vec3D(0, pitch, yaw);
}

// --- NEW Euler Angle Extraction Functions ---
// These functions return angles in RADIANS.

/**
 * @brief Calculates the Roll angle (rotation about the body X-axis) from a quaternion.
 * Uses the XYZ intrinsic Euler angle convention.
 * @param q The input quaternion.
 * @return Roll angle in radians.
 */
float get_roll_from_quaternion(Quaternion q) {
    float w = q.a, x = q.b, y = q.c, z = q.d;

    // roll = atan2(2(w*x - y*z), 1 - 2(x^2 + y^2))
    float sinr_cosp = 2.0f * (w * x - y * z);
    float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
    return atan2f(sinr_cosp, cosr_cosp);
}

/**
 * @brief Calculates the Pitch angle (rotation about the body Y-axis) from a quaternion.
 * Uses the XYZ intrinsic Euler angle convention.
 * @param q The input quaternion.
 * @return Pitch angle in radians.
 */
float get_pitch_from_quaternion(Quaternion q) {
    float w = q.a, x = q.b, y = q.c, z = q.d;

    // pitch = asin(clamp(2(w*y + z*x), -1, 1))
    float sinp = 2.0f * (w * y + z * x);
    sinp = clamp(sinp, -1.0f, 1.0f);
    return asinf(sinp);
}

/**
 * @brief Calculates the Yaw angle (rotation about the body Z-axis) from a quaternion.
 * Uses the XYZ intrinsic Euler angle convention.
 * @param q The input quaternion.
 * @return Yaw angle in radians.
 */
float get_yaw_from_quaternion(Quaternion q) {
    float w = q.a, x = q.b, y = q.c, z = q.d;

    // yaw = atan2(2(w*z - x*y), 1 - 2(y^2 + z^2))
    float siny_cosp = 2.0f * (w * z - x * y);
    float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
    return atan2f(siny_cosp, cosy_cosp);
}

/**
 * @brief Converts a quaternion to Euler angles (Roll, Pitch, Yaw) in degrees
 * using the XYZ intrinsic rotation sequence.
 * @param q The input quaternion.
 * @return Vec3D containing Roll (x), Pitch (y), Yaw (z) angles in degrees.
 */
Vec3D quaternion_to_euler(Quaternion q) {
    float roll_rad = get_roll_from_quaternion(q);
    float pitch_rad = get_pitch_from_quaternion(q);
    float yaw_rad = get_yaw_from_quaternion(q);

    float roll_deg = roll_rad * 180.0f / PI;
    float pitch_deg = pitch_rad * 180.0f / PI;
    float yaw_deg = yaw_rad * 180.0f / PI;

    return Vec3D(roll_deg, pitch_deg, yaw_deg);
}

// GYRO-BASED QUATERNION UPDATE
// This function updates 'attitude' (the current orientation) by the rotation measured by the gyroscope.
Quaternion get_angles_quat(SensorReadings readings, Quaternion& current_attitude, float deltaTime) {
    // User's Gyro Definitions:
    // readings.gx: angular rate around body X-axis (User's Roll Rate)
    // readings.gy: angular rate around body Y-axis (User's Pitch Rate)
    // readings.gz: angular rate around body Z-axis (User's Yaw Rate)

    float roll_rate_rad_s = readings.gx * (PI / 180.0f); // User's Roll Rate
    float pitch_rate_rad_s = readings.gy * (PI / 180.0f); // User's Pitch Rate
    float yaw_rate_rad_s = readings.gz * (PI / 180.0f); // User's Yaw Rate

    float angle_magnitude = sqrt(pitch_rate_rad_s * pitch_rate_rad_s + roll_rate_rad_s * roll_rate_rad_s + yaw_rate_rad_s * yaw_rate_rad_s);

    Quaternion delta_q_gyro; // Will be identity if no rotation

    if (angle_magnitude > 1e-9f) { // Check for non-zero rotation
        float half_angle = angle_magnitude * deltaTime / 2.0f;
        float sin_half_angle = sinf(half_angle);
        float cos_half_angle = cosf(half_angle);

        // Normalized axis of rotation (these are already body axes)
        float axis_x_norm = roll_rate_rad_s / angle_magnitude;
        float axis_y_norm = pitch_rate_rad_s / angle_magnitude;
        float axis_z_norm = yaw_rate_rad_s / angle_magnitude;

        delta_q_gyro.a = cos_half_angle;       // w
        delta_q_gyro.b = axis_x_norm * sin_half_angle; // x_body component
        delta_q_gyro.c = axis_y_norm * sin_half_angle; // y_body component
        delta_q_gyro.d = axis_z_norm * sin_half_angle; // z_body component
        // delta_q_gyro.normalize(); // from_axis_angle should ideally produce a normalized quat if axis is unit
    }
    else {
        // No significant rotation, delta_q is identity
        delta_q_gyro.a = 1.0f;
        delta_q_gyro.b = 0.0f;
        delta_q_gyro.c = 0.0f;
        delta_q_gyro.d = 0.0f;
    }

    // Update current_attitude by the delta rotation from the gyroscope
    // q_new = q_old * delta_q_body_rotation
    current_attitude = current_attitude * delta_q_gyro;
    current_attitude.normalize(); // Normalize after multiplication

    return current_attitude;
}


// COMPLEMENTARY FILTERING WITH QUATERNIONS (Revised)
// Fuses accelerometer and gyroscope data to update the attitude quaternion.
Quaternion get_angles_compl_quat(
    float gyro_weight,      // Weight for the gyroscope (e.g., 0.98 means 98% gyro, 2% accel)
    float dt,               // Time delta in seconds
    SensorReadings r,       // Current sensor readings (ax,ay,az, gx,gy,gz)
    Quaternion& current_attitude_estimate // Current best estimate of attitude (will be updated)
) {
    // 1. Gyro Update: Predict new orientation based on gyro
    // get_angles_quat updates 'current_attitude_estimate' by reference and returns it.
    Quaternion gyro_predicted_q = get_angles_quat(r, current_attitude_estimate, dt);
    // current_attitude_estimate is now the gyro-predicted orientation.

    // 2. Calculate Accelerometer-derived Angles
    // Your Coordinate System:
    //   X-axis: Up-Down (+ up, - down)
    //      Longitudinal, tail to nose, rocket initially pointing straight up 
    //      Gravity acts along -Z world, so accelerometer X-axis sees +g when upright.
    //   
    //   Y - axis: Left-Right (+right, -left)
    //
    //   Z - axis: Front-Back (+front, -back)
    //
    // Pitch:   Rotation around body Y-axis (tilt in YZ plane of rocket)
    // Yaw:     Rotation around body Z-axis (tilt in XY plane of rocket)

    float accel_user_pitch_rad;
    float accel_user_yaw_rad;

    // Safety check for r.ay to prevent instability when rocket is nearly horizontal.
    // Define g_approx based on your sensor scale or expected values.
    const float min_ax_for_accel_trust = 0.01; // Trust accel if ay is at least 0.2g

    if (fabsf(r.ax) < min_ax_for_accel_trust) {
        // Accelerometer data is unreliable for determining user's pitch and yaw.
        // For this step, use pitch/yaw from gyro prediction to avoid bad correction.
        // This means the SLERP will effectively do nothing for pitch/yaw correction.
        accel_user_pitch_rad = get_pitch_from_quaternion(gyro_predicted_q);
        accel_user_yaw_rad = get_yaw_from_quaternion(gyro_predicted_q);
    }
    else {
        Vec3D accel_angles = get_angles_accel(r);
        accel_user_pitch_rad = accel_angles.y;
        accel_user_yaw_rad = accel_angles.z;
    }

    // 3. Extract User's Roll from the gyro-predicted quaternion
    // Accelerometer cannot measure user's roll (rotation around body Y-axis) in this configuration.
    float user_roll_from_gyro_rad = get_roll_from_quaternion(gyro_predicted_q);

    // 4. Construct the "accelerometer target" quaternion (accel_derived_q)
    // This quaternion represents the orientation the accelerometer "sees" for pitch and yaw,
    // combined with the roll from the gyroscope.
    // Quaternion::from_euler_rotation expects standard roll (X), pitch (Y), yaw (Z) in RADIANS.
    // We need to map our user-defined angles to these standard slots:
    
    Quaternion accel_derived_q = from_euler_xyz(
        user_roll_from_gyro_rad, accel_user_pitch_rad, accel_user_yaw_rad
    );
    // accel_derived_q.normalize(); // from_euler_rotation should ideally produce a normalized quaternion

    // 5. Fuse gyro_predicted_q and accel_derived_q using SLERP
    // The interpolation factor 't' for slerp(q1, q2, t) goes from q1 (if t=0) to q2 (if t=1).
    // We want to move from gyro_predicted_q towards accel_derived_q by a small amount (accel_weight).
    float accel_weight = 1.0f - gyro_weight;
    current_attitude_estimate = slerp(gyro_predicted_q, accel_derived_q, accel_weight);

    current_attitude_estimate.normalize(); // Final normalization

    return current_attitude_estimate;
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