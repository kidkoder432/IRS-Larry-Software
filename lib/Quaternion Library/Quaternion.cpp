#include "Quaternion.h"

#ifdef ARDUINO
#include "Arduino.h"
#else
#include <math.h>
#endif

// http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/arithmetic/index.htm
// 800B
Quaternion& Quaternion::operator*=(const Quaternion& q) {
    Quaternion ret;
    ret.a = a * q.a - b * q.b - c * q.c - d * q.d;
    ret.b = a * q.b + b * q.a + c * q.d - d * q.c;
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

Quaternion& Quaternion::operator*=(double scale) {
    a *= scale;
    b *= scale;
    c *= scale;
    d *= scale;
    return *this;
}

double Quaternion::norm() const {
    double norm2 = a * a + b * b + c * c + d * d;
    return sqrt(norm2);
}

// 400B
Quaternion& Quaternion::normalize() {
    double n = norm();
    a /= n;
    b /= n;
    c /= n;
    d /= n;
    return *this;
}

// This method takes an euler rotation in rad and converts it to an equivilent 
// Quaternion rotation.
// 800B
const Quaternion Quaternion::from_euler_rotation(double yaw, double pitch, double roll) {
    double cy = cos(yaw / 2);
    double cp = cos(pitch / 2);
    double cr = cos(roll / 2);

    double sy = sin(yaw / 2);
    double sp = sin(pitch / 2);
    double sr = sin(roll / 2);
    Quaternion ret;
    ret.a = cr * cp * cy + sr * sp * sy;
    ret.b = sr * cp * cy - cr * sp * sy;
    ret.c = cr * sp * cy + sr * cp * sy;
    ret.d = cr * cp * sy - sr * sp * cy;
    return ret;
}

const Quaternion Quaternion::from_euler_rotation_approx(double yaw, double pitch, double roll) {
    // approximage cos(theta/2) as 1 - theta^2 / 8 (1 - theta^2 / 2 * 1/2^2)
    double cy = 1 - ((yaw * yaw) / 8);
    double cp = 1 - ((pitch * pitch) / 8);
    double cr = 1 - ((roll * roll) / 8);

    // appromixate sin(theta/2) as theta/2
    double sy = yaw / 2;
    double sp = pitch / 2;
    double sr = roll / 2;
    Quaternion ret;
    ret.a = cr * cp * cy + sr * sp * sy;
    ret.b = sr * cp * cy - cr * sp * sy;
    ret.c = cr * sp * cy + sr * cp * sy;
    ret.d = cr * cp * sy - sr * sp * cy;
    return ret;
}

const Quaternion Quaternion::from_axis_angle(double angle, double x, double y, double z) {
    double sa = sin(angle / 2);

    Quaternion ret;
    ret.a = cos(angle / 2);
    ret.b = x * sa;
    ret.c = y * sa;
    ret.d = z * sa;

    return ret;
}

const Quaternion Quaternion::conj() const {
    Quaternion ret(*this);
    ret.b *= -1;
    ret.c *= -1;
    ret.d *= -1;
    return ret;
}

