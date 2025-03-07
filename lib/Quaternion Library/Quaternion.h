#ifndef QUATERNION_H
#define QUATERNION_H

class Quaternion {
public:
    float a;
    float b;
    float c;
    float d;

    Quaternion() { a = 1; b = c = d = 0; }

    // This is a vector that can be rotated in Quaternion space.
    Quaternion(float x, float y, float z) { a = 0; b = x; c = y; d = z; }

    // ADDED: This is a quaternion
    Quaternion(float x, float y, float z, float g) { a = x; b = y; c = z; d = g; }

    // This returns a Quaternion that rotates in each given axis in radians.
    // We use standard right hand rule for rotations and coordinates.
    static const Quaternion from_euler_rotation(float yaw, float pitch, float roll);

    // This is like from_euler_rotation but for small angles (less than 45 deg (PI/4))
    static const Quaternion from_euler_rotation_approx(float yaw, float pitch, float roll);

    // These should hopefully give better results (less jank)
    static const Quaternion from_axis_angle(float angle, float x, float y, float z);

    Quaternion& operator=(const Quaternion& rhs) {
        a = rhs.a;
        b = rhs.b;
        c = rhs.c;
        d = rhs.d;
        return *this;
    }

    // http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/arithmetic/index.htm
    Quaternion& operator*=(const Quaternion& q);
    const Quaternion operator* (const Quaternion& q) const { return Quaternion(*this) *= q; }
    Quaternion& operator+=(const Quaternion& q);
    const Quaternion operator+(const Quaternion& q) const { return Quaternion(*this) += q; }
    Quaternion& operator*=(float scale);
    const Quaternion operator*(float scale) const { return Quaternion(*this) *= scale; }
    float norm() const;
    Quaternion& normalize();
    const Quaternion conj() const;
    const Quaternion operator-() const { return Quaternion(-a, -b, -c, -d); }
};

#endif
