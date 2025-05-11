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
// x = pitch, y = roll, z = yaw
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

// float quaternion_to_roll(Quaternion attitude) {
//     // --- Convert to Euler Angles --- //    
//     // https://www.euclideanspace.com/maths/standards/index.htm#:~:text=Euler%20angles
//     // Switch axes (X pitch, Y roll, Z yaw --> Z pitch, X roll, Y yaw)
//     float qw = attitude.a;
//     float qz = attitude.b;
//     float qx = attitude.c;
//     float qy = attitude.d;

//     // from https://www.euclideanspace.com/maths/standards/index.htm
//     if (qx * qy + qz * qw >= 0.5) {  // North pole
//         return 0;
//     }
//     else if (qx * qy + qz * qw <= -0.5) {  // South pole
//         return 0;
//     }
//     else {
//         float roll = atan2(2 * qx * qw - 2 * qy * qz, 1 - 2 * qx * qx - 2 * qz * qz);
//         return roll * -180 / PI;
//     }
// }

// float quaternion_to_pitch(Quaternion attitude) {
//     // --- Convert to Euler Angles --- //    
//     // https://www.euclideanspace.com/maths/standards/index.htm#:~:text=Euler%20angles
//     // Switch axes (X pitch, Y roll, Z yaw --> Z pitch, X roll, Y yaw)
//     float qw = attitude.a;
//     float qz = attitude.b;
//     float qx = attitude.c;
//     float qy = attitude.d;

//     // from https://www.euclideanspace.com/maths/standards/index.htm
//     if (qx * qy + qz * qw >= 0.5) {  // North pole
//         return 90;
//     }
//     else if (qx * qy + qz * qw <= -0.5) {  // South pole
//         return -90;
//     }
//     else {
//         float pitch = asin(2 * qx * qy + 2 * qz * qw);
//         return pitch * 180 / PI;
//     }
// }

// float quaternion_to_yaw(Quaternion attitude) {
//     // --- Convert to Euler Angles --- //    
//     // https://www.euclideanspace.com/maths/standards/index.htm#:~:text=Euler%20angles
//     // Switch axes (X pitch, Y roll, Z yaw --> Z pitch, X roll, Y yaw)
//     float qw = attitude.a;
//     float qz = attitude.b;
//     float qx = attitude.c;
//     float qy = attitude.d;

//     // from https://www.euclideanspace.com/maths/standards/index.htm
//     if (qx * qy + qz * qw >= 0.5) {  // North pole
//         return 2 * atan2(qx, qw) * 180 / PI;
//     }
//     else if (qx * qy + qz * qw <= -0.5) {  // South pole
//         return -2 * atan2(qx, qw) * 180 / PI;
//     }
//     else {
//         float yaw = atan2(2 * qy * qw - 2 * qx * qz, 1 - 2 * qy * qy - 2 * qz * qz);
//         return yaw * 180 / PI;
//     }
// // }

// Vec3D quaternion_to_euler(Quaternion attitude) {
//     float roll = quaternion_to_roll(attitude);
//     float pitch = quaternion_to_pitch(attitude);
//     float yaw = quaternion_to_yaw(attitude);
//     return Vec3D(roll, pitch, yaw);
// }



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
// Quaternion get_angles_quat(SensorReadings readings, Quaternion& attitude, float deltaTime) {

//     // wx = pitch  (rotation around x-axis)
//     // wy = roll   (rotation around y-axis) 
//     // wz = yaw    (rotation around z-axis)
//     float wx = readings.gx * (PI / 180);
//     float wy = readings.gy * (PI / 180);
//     float wz = readings.gz * (PI / 180);

//     // Update attitude quaternion
//     float norm = sqrt(wx * wx + wy * wy + wz * wz);
//     norm = copysignf(max(abs(norm), 1e-9f), norm); // NO DIVIDE BY 0

//     wx /= norm;
//     wy /= norm;
//     wz /= norm;

//     Quaternion QM = Quaternion::from_axis_angle(deltaTime * norm, wx, wy, wz);
//     attitude = attitude * QM;

//     return attitude;
// }

// // COMPLEMENTARY FILTERING WITH QUATERNIONS
// Quaternion get_angles_compl_quat(float A, float dt, SensorReadings r, Quaternion& attitude) {

//     float accelYaw = atan2(r.ax, sqrt(r.az * r.az + r.ay * r.ay));
//     float accelPitch = -atan2(r.az, r.ay);

//     Quaternion gyro_q = get_angles_quat(r, attitude, dt);
//     float roll = quaternion_to_roll(gyro_q) * (PI / 180);

//     Quaternion accel_q = Quaternion::from_euler_rotation(accelYaw, accelPitch, roll);

//     attitude = slerp(gyro_q, accel_q, 1.0f - A);

//     attitude.normalize();

//     return attitude;
// }   

// --- NEW Euler Angle Extraction Functions (User's Definitions) ---
// These assume Quaternion stores: a=w, b=vector_x_body, c=vector_y_body, d=vector_z_body
// where vector_x_body corresponds to user's pitch axis (body X)
// vector_y_body corresponds to user's roll axis (body Y)
// vector_z_body corresponds to user's yaw axis (body Z)
// These functions return angles in RADIANS.

float get_user_pitch_from_quaternion(Quaternion q) { // User's Pitch: Rotation around Body X-axis
    // This is equivalent to standard roll from a ZYX Tait-Bryan sequence
    // where q.b is the x-component of the quaternion vector part.
    float sin_user_pitch_cosp_user_roll = 2.0f * (q.a * q.b + q.c * q.d);
    float cos_user_pitch_cosp_user_roll = 1.0f - 2.0f * (q.b * q.b + q.c * q.c);
    float user_pitch = atan2f(sin_user_pitch_cosp_user_roll, cos_user_pitch_cosp_user_roll);
    return user_pitch; // Radians
}

float get_user_roll_from_quaternion(Quaternion q) { // User's Roll: Rotation around Body Y-axis
    // This is equivalent to standard pitch from a ZYX Tait-Bryan sequence
    // where q.c is the y-component of the quaternion vector part.
    float sin_user_roll = 2.0f * (q.a * q.c - q.b * q.d);
    // Clamp sin_user_roll to [-1, 1] to avoid domain errors with asinf
    if (sin_user_roll > 1.0f) sin_user_roll = 1.0f;
    if (sin_user_roll < -1.0f) sin_user_roll = -1.0f;

    return asinf(sin_user_roll); // Radians
}

float get_user_yaw_from_quaternion(Quaternion q) { // User's Yaw: Rotation around Body Z-axis
    // This is equivalent to standard yaw from a ZYX Tait-Bryan sequence
    // where q.d is the z-component of the quaternion vector part.
    float sin_user_yaw_cosp_user_roll = 2.0f * (q.a * q.d + q.b * q.c);
    float cos_user_yaw_cosp_user_roll = 1.0f - 2.0f * (q.c * q.c + q.d * q.d);
    float user_yaw = atan2f(sin_user_yaw_cosp_user_roll, cos_user_yaw_cosp_user_roll);
    return user_yaw; // Radians
}

Vec3D quaternion_to_user_euler(Quaternion attitude) {
    // Returns user-defined pitch, roll, yaw in the Vec3D
    // Assuming Vec3D x=user_pitch, y=user_roll, z=user_yaw based on your gyro comments
    // The original Vec3D comment said: x=roll, y=pitch, z=yaw. Clarify which Vec3D field is which user angle.
    // For now, I'll assume Vec3D.x = User Pitch, Vec3D.y = User Roll, Vec3D.z = User Yaw to match gyro inputs.
    float user_pitch_rad = get_user_pitch_from_quaternion(attitude);
    float user_roll_rad = get_user_roll_from_quaternion(attitude);
    float user_yaw_rad = get_user_yaw_from_quaternion(attitude);
    // Convert to degrees if Vec3D is expected to store degrees
    return Vec3D(user_pitch_rad * 180.0f / PI,
        user_roll_rad * 180.0f / PI,
        user_yaw_rad * 180.0f / PI);
}


// --- Deprecate or Remove Old Euler Functions ---
// The following functions (quaternion_to_roll, quaternion_to_pitch, quaternion_to_yaw, quaternion_to_euler)
// use a confusing remapping. It's better to use the new `get_user_...` functions.
// If you keep them, ensure they are clearly documented for their specific behavior.
/*
float quaternion_to_roll(Quaternion attitude) { ... } // OLD - uses remapping
float quaternion_to_pitch(Quaternion attitude) { ... } // OLD - uses remapping
float quaternion_to_yaw(Quaternion attitude) { ... } // OLD - uses remapping
Vec3D quaternion_to_euler(Quaternion attitude) { ... } // OLD
*/

// GYRO-BASED QUATERNION UPDATE
// (Formerly get_angles_quat)
// This function updates 'attitude' (the current orientation) by the rotation measured by the gyroscope.
Quaternion get_angles_quat(SensorReadings readings, Quaternion& current_attitude, float deltaTime) {
    // User's Gyro Definitions:
    // readings.gx: angular rate around body X-axis (User's Pitch Rate)
    // readings.gy: angular rate around body Y-axis (User's Roll Rate)
    // readings.gz: angular rate around body Z-axis (User's Yaw Rate)

    // IMPORTANT: Confirm units of readings.g{x,y,z}.
    // If ALREADY in radians/second, REMOVE the *(PI/180) conversion.
    // If in degrees/second, the conversion is correct.
    float pitch_rate_rad_s = readings.gx * (PI / 180.0f); // User's Pitch Rate
    float roll_rate_rad_s = readings.gy * (PI / 180.0f); // User's Roll Rate
    float yaw_rate_rad_s = readings.gz * (PI / 180.0f); // User's Yaw Rate

    // Calculate rotation vector components for this time step
    float rot_vec_x = pitch_rate_rad_s * deltaTime; // Rotation around body X
    float rot_vec_y = roll_rate_rad_s * deltaTime; // Rotation around body Y
    float rot_vec_z = yaw_rate_rad_s * deltaTime; // Rotation around body Z

    float angle_magnitude = sqrtf(rot_vec_x * rot_vec_x + rot_vec_y * rot_vec_y + rot_vec_z * rot_vec_z);

    Quaternion delta_q_gyro; // Will be identity if no rotation

    if (angle_magnitude > 1e-9f) { // Check for non-zero rotation
        float half_angle = angle_magnitude / 2.0f;
        float sin_half_angle = sinf(half_angle);
        float cos_half_angle = cosf(half_angle);

        // Normalized axis of rotation (these are already body axes)
        float axis_x_norm = rot_vec_x / angle_magnitude;
        float axis_y_norm = rot_vec_y / angle_magnitude;
        float axis_z_norm = rot_vec_z / angle_magnitude;

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

    // 2. Calculate Accelerometer-derived User Angles (User's Pitch and User's Yaw)
    // Your Coordinate System:
    //   X-axis: Left-Right
    //   Y-axis: Up-Down (longitudinal, tail to nose, rocket initially pointing straight up)
    //           Gravity acts along -Y world, so accelerometer Y-axis sees +g when upright.
    //   Z-axis: Front-Back
    // User's Pitch: Rotation around body X-axis (tilt in YZ plane of rocket)
    // User's Yaw:   Rotation around body Z-axis (tilt in XY plane of rocket)

    float accel_user_pitch_rad;
    float accel_user_yaw_rad;

    // Safety check for r.ay to prevent instability when rocket is nearly horizontal.
    // Define g_approx based on your sensor scale or expected values.
    const float g_approx = 9.81f; // Approximate gravity
    const float min_ay_for_accel_trust = 0.01; // Trust accel if ay is at least 0.2g

    if (fabsf(r.ay) < min_ay_for_accel_trust) {
        // Accelerometer data is unreliable for determining user's pitch and yaw.
        // For this step, use pitch/yaw from gyro prediction to avoid bad correction.
        // This means the SLERP will effectively do nothing for pitch/yaw correction.
        accel_user_pitch_rad = get_user_pitch_from_quaternion(gyro_predicted_q);
        accel_user_yaw_rad = get_user_yaw_from_quaternion(gyro_predicted_q);
    }
    else {
        // Calculate user's pitch from accelerometer (rotation around body X-axis)
        accel_user_pitch_rad = -atan2(r.az, sqrt(r.ax * r.ax + r.ay * r.ay));

        // Calculate user's yaw from accelerometer (rotation around body Z-axis)
        accel_user_yaw_rad = atan2(r.ax, sqrt(r.az * r.az + r.ay * r.ay));
    }

    // 3. Extract User's Roll from the gyro-predicted quaternion
    // Accelerometer cannot measure user's roll (rotation around body Y-axis) in this configuration.
    float user_roll_from_gyro_rad = get_user_roll_from_quaternion(gyro_predicted_q);

    // 4. Construct the "accelerometer target" quaternion (accel_derived_q)
    // This quaternion represents the orientation the accelerometer "sees" for pitch and yaw,
    // combined with the roll from the gyroscope.
    // Quaternion::from_euler_rotation expects standard roll (X), pitch (Y), yaw (Z) in RADIANS.
    // We need to map our user-defined angles to these standard slots:
    //   Standard Roll (X rot) input = User's Pitch (body X rot)
    //   Standard Pitch (Y rot) input = User's Roll (body Y rot)
    //   Standard Yaw (Z rot) input = User's Yaw (body Z rot)
    Quaternion accel_derived_q = Quaternion::from_euler_rotation(
        accel_user_pitch_rad,        // Argument for standard roll (X-axis)
        user_roll_from_gyro_rad,     // Argument for standard pitch (Y-axis)
        accel_user_yaw_rad           // Argument for standard yaw (Z-axis)
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