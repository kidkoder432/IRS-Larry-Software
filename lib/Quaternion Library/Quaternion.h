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

    // Full constructor for all components
    Quaternion(float w_val, float x_val, float y_val, float z_val);

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
    static const Quaternion from_euler_rotation(float roll, float pitch, float yaw);

    // This is like from_euler_rotation but for small angles (less than 45 deg (PI/4))
    static const Quaternion from_euler_rotation_approx(float x, float y, float z);

    static const Quaternion from_axis_angle(float x, float y, float z);
    static const Quaternion from_axis_angle_approx(float x, float y, float z);

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
    const Quaternion operator-() const { return Quaternion(-a, -b, -c, -d);}
    float norm() const;
    Quaternion& normalize();
    const Quaternion conj() const;
    // This method takes two vectors and computes the rotation vector between them.
    // Both the left and right hand sides must be pure vectors (a == 0)
    // Both the left and right hand sides must normalized already.
    // This computes the rotation that will tranform this to q.
    const Quaternion rotation_between_vectors(const Quaternion& v) const;
    float dot_product(const Quaternion& q) const;

    // This method takes one vector and rotates it using this Quaternion.
    // The input must be a pure vector (a == 0)
    const Quaternion rotate(const Quaternion& q) const;
    Quaternion& fractional(float f);
};

#endif
