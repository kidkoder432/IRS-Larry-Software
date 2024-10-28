#ifndef QUATERNION_H
#define QUATERNION_H

class Quaternion {
public:
    double a;
    double b;
    double c;
    double d;

    Quaternion() { a = 1; b = c = d = 0; }

    // This is a vector that can be rotated in Quaternion space.
    Quaternion(double x, double y, double z) { a = 0; b = x; c = y; d = z; }

    // ADDED: This is a quaternion
    Quaternion(double x, double y, double z, double g) { a = x; b = y; c = z; d = g; }

    // This returns a Quaternion that rotates in each given axis in radians.
    // We use standard right hand rule for rotations and coordinates.
    static const Quaternion from_euler_rotation(double yaw, double pitch, double roll);

    // This is like from_euler_rotation but for small angles (less than 45 deg (PI/4))
    static const Quaternion from_euler_rotation_approx(double yaw, double pitch, double roll);

    // These should hopefully give better results (less jank)
    static const Quaternion from_axis_angle(double angle, double x, double y, double z);

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
    Quaternion& operator*=(double scale);
    const Quaternion operator*(double scale) const { return Quaternion(*this) *= scale; }
    double norm() const;
    Quaternion& normalize();
    const Quaternion conj() const;
};

#endif
