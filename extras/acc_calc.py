import math

# Coords
# x-axis = left-right, y-axis = up-down, z-axis = front-back
# roll = y-axis, pitch = x-axis, yaw = z-axis


def euler_to_quaternion(roll, pitch, yaw):
    """
    Converts a Euler angle to a quaternion.

    Args:
        roll, pitch, yaw: Euler angles in radians.

    Returns:
        x, y, z, w: Quaternion components.
    """

    roll, pitch = pitch, roll
    cr = math.cos(roll / 2)
    sr = math.sin(roll / 2)
    cp = math.cos(pitch / 2)
    sp = math.sin(pitch / 2)
    cy = math.cos(yaw / 2)
    sy = math.sin(yaw / 2)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return w, x, y, z


V = [0, 0, math.sqrt(2) / 2, math.sqrt(2) / 2]
Q = euler_to_quaternion(0, -math.pi / 4, 0)
Qconj = [Q[0], -Q[1], -Q[2], -Q[3]]


def quaternion_multiply(Q1, Q2):
    """
    Multiply two quaternions.

    Args:
        Q1, Q2: Quaternions as (x, y, z, w).

    Returns:
        Q: Quaternion as (x, y, z, w).
    """

    w1, x1, y1, z1 = Q1
    w2, x2, y2, z2 = Q2

    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2

    return w, x, y, z


Vp = quaternion_multiply(quaternion_multiply(Q, V), Qconj)

print(Vp)
