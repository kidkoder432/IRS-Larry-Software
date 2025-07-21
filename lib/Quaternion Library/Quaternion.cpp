#include "Quaternion.h"

#ifdef ARDUINO
#include "Arduino.h"
#else
#include <math.h>
#endif

Quaternion::Quaternion(float w_val, float x_val, float y_val, float z_val) {
    a = w_val;
    b = x_val;
    c = y_val;
    d = z_val;
}

// http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/arithmetic/index.htm
// 800B
Quaternion& Quaternion::operator*=(const Quaternion& q) {
    Quaternion ret;
    ret.a = a * q.a - b * q.b - c * q.c - d * q.d;
    ret.b = b * q.a + a * q.b + c * q.d - d * q.c;
    ret.c = a * q.c - b * q.d + c * q.a + d * q.b;
    ret.d = a * q.d + b * q.c - c * q.b + d * q.a;
    return (*this = ret);
}

Quaternion& Quaternion::operator+=(const Quaternion& q) {
    a += q.a;
    b += q.b;
    c += q.c;
    d += q.d;
    return *this;
}

Quaternion& Quaternion::operator*=(float scale) {
    a *= scale;
    b *= scale;
    c *= scale;
    d *= scale;
    return *this;
}

float Quaternion::norm() const {
    float norm2 = a * a + b * b + c * c + d * d;
    return sqrtf(norm2);
}

// 400B
Quaternion& Quaternion::normalize() {
    float n = norm();
    a /= n;
    b /= n;
    c /= n;
    d /= n;
    return *this;
}

const Quaternion Quaternion::from_axis_angle(float x, float y, float z) {
    float angle = sqrtf(x * x + y * y + z * z);
    float s = sinf(angle / 2.0f);
    float w = cosf(angle / 2.0f);
    Quaternion ret;
    ret.a = w;
    ret.b = x / angle * s;
    ret.c = y / angle * s;
    ret.d = z / angle * s;
    return ret;
}

const Quaternion Quaternion::from_axis_angle_approx(float x, float y, float z) {
    float angle = sqrtf(x * x + y * y + z * z);
    float w = 1.0f - (angle * angle / 8.0f);
    Quaternion ret;
    ret.a = w;
    ret.b = x / 2.0f;
    ret.c = y / 2.0f;
    ret.d = z / 2.0f;
    return ret;
}

/**
 * @brief Creates a quaternion from Euler angles using ZYX intrinsic rotation sequence.
 *
 * The input angles are rotations about the body axes in the order:
 * yaw (Z-axis), pitch (Y-axis), then roll (X-axis).
 *
 * @param roll Rotation about X-axis in radians.
 * @param pitch Rotation about Y-axis in radians.
 * @param yaw Rotation about Z-axis in radians.
 * @return Quaternion representing the combined rotation.
 */
const Quaternion Quaternion::from_euler_rotation(float roll, float pitch, float yaw) {
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
    // According to ZYX intrinsic rotation (yaw->pitch->roll),
    // quaternion is computed as q = q_yaw * q_pitch * q_roll
    q.a = cos_yaw * cos_pitch * cos_roll + sin_yaw * sin_pitch * sin_roll;
    q.b = cos_yaw * cos_pitch * sin_roll - sin_yaw * sin_pitch * cos_roll;
    q.c = cos_yaw * sin_pitch * cos_roll + sin_yaw * cos_pitch * sin_roll;
    q.d = sin_yaw * cos_pitch * cos_roll - cos_yaw * sin_pitch * sin_roll;

    return q;
}


const Quaternion Quaternion::from_euler_rotation_approx(float x, float y, float z) {
    // approximage cos(theta) as 1 - theta^2 / 2
    float c1 = 1.0f - (y * y / 8.0f);
    float c2 = 1.0f - (z * z / 8.0f);
    float c3 = 1.0f - (x * x / 8.0f);

    // appromixate sin(theta) as theta
    float s1 = y / 2.0f;
    float s2 = z / 2.0f;
    float s3 = x / 2.0f;
    Quaternion ret;
    ret.a = c1 * c2 * c3 + s1 * s2 * s3;
    ret.b = c1 * c2 * s3 - s1 * s2 * c3;
    ret.c = s1 * c2 * c3 + c1 * s2 * s3;
    ret.d = c1 * s2 * c3 - s1 * c2 * s3;
    return ret;
}

const Quaternion Quaternion::conj() const {
    Quaternion ret(*this);
    ret.b *= -1.0f;
    ret.c *= -1.0f;
    ret.d *= -1.0f;
    return ret;
}

// This method takes two vectors and computes the rotation vector between them.
// Both the left and right hand sides must be pure vectors (a == 0)
// Both the left and right hand sides must normalized already.
// This computes the rotation that will tranform this to q.
// 500B
const Quaternion Quaternion::rotation_between_vectors(const Quaternion& q) const {
    // http://www.euclideanspace.com/maths/algebra/vectors/angleBetween/
    // We want to compute the below values.
    // w = 1 + v1•v2
    // x = (v1 x v2).x 
    // y = (v1 x v2).y
    // z = (v1 x v2).z

    // Instead of writing the below code direclty, we reduce code size by
    // just using multiplication to implement it.
    //Quaternion ret;
    //ret.a = 1 + b * q.b + c * q.c + d * q.d;
    //ret.b = c * q.d - d * q.c;
    //ret.c = d * q.b - b * q.d;
    //ret.d = b * q.c - c * q.b;
    //ret.normalize();
    //return ret;

    // From wikipedia https://en.wikipedia.org/wiki/Quaternion#Quaternions_and_the_geometry_of_R3
    // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    // The cross product p x q is just the vector part of multiplying p * q
    // the scalar part after a multiply is -p•q 
    Quaternion ret = (*this) * q;
    ret.a = 1.0f - ret.a;
    ret.normalize();
    return ret;
}

float Quaternion::dot_product(const Quaternion& q) const {
    return a * q.a + b * q.b + c * q.c + d * q.d;
}

// This will roate the input vector by this normalized rotation quaternion.
const Quaternion Quaternion::rotate(const Quaternion& q) const {
    return (*this) * q * conj();
}

// This modifies this normalized rotation quaternion and makes it 
// rotate between 0-1 as much as it would normally rotate.
// The math here is pretty sloppy but should work for 
// most cases.
Quaternion& Quaternion::fractional(float f) {
    a = 1.0f - f + f * a;
    b *= f;
    c *= f;
    d *= f;
    return normalize();
}

