from re import X
from vpython import *
import serial
import math

from pid import PIDController

# Adapted from https://toptechboy.com/9-axis-imu-lesson-20-vpython-visualization-of-roll-pitch-and-yaw/

# Initialize VPython scene
scene = canvas(
    title="Rocket Model",
    width=600,
    height=400,
    center=vec(0, 0, 0),
    background=vec(0.2, 0.2, 0.2),
)

scene.forward = vector(0, 1, -1)
scene.up = vector(1, 0, 0)

scene.width = 600
scene.height = 600

xarrow = arrow(length=2, shaftwidth=0.1, color=color.red, axis=vector(1, 0, 0))
yarrow = arrow(length=2, shaftwidth=0.1, color=color.green, axis=vector(0, 1, 0))
# zarrow = arrow(length=2, shaftwidth=0.1, color=color.blue, axis=vector(0, 0, 1))

frontArrow = arrow(length=1, shaftwidth=0.1, color=color.purple, axis=vector(1, 0, 0))
upArrow = arrow(length=1, shaftwidth=0.1, color=color.magenta, axis=vector(0, 1, 0))

tube = cylinder(length=1, radius=0.1, color=color.cyan)
nose = cone(
    length=0.4,
    radius=0.1,
    color=color.red,
    pos=vector(1, 0, 0),
)
tvc = arrow(length=1, shaftwidth=0.1, color=color.yellow, axis=vector(-1, 0, 0))

rocket = compound([tube, nose])

# bBoard = box(
#     length=6,
#     width=2,
#     height=0.2,
#     opacity=0.8,
#     pos=vector(
#         0,
#         0,
#         0,
#     ),
# )
# bn = box(
#     length=1, width=0.75, height=0.1, pos=vector(-0.5, 0.1 + 0.05, 0), color=color.blue
# )
# nano = box(
#     length=1.75, width=0.6, height=0.1, pos=vector(-2, 0.1 + 0.05, 0), color=color.green
# )
# rocket = compound([bBoard, bn, nano])

# Initialize serial port
# ser = serial.Serial("COM9", 115200)  # Replace '/dev/ttyUSB0' with your serial port

roll, pitch, yaw = 0, 0, 0


def quaternion_to_euler(qw, qx, qy, qz):
    """
    Converts a quaternion to Euler angles (heading, attitude, bank).

    Args:
        qw, qx, qy, qz: Quaternion components.

    Returns:
        heading, attitude, bank: Euler angles in radians.
    """

    qw /= sqrt(qw**2 + qx**2 + qy**2 + qz**2)
    qx /= sqrt(qw**2 + qx**2 + qy**2 + qz**2)
    qy /= sqrt(qw**2 + qx**2 + qy**2 + qz**2)
    qz /= sqrt(qw**2 + qx**2 + qy**2 + qz**2)

    if qx * qy + qz * qw >= 0.5:  # North pole
        yaw = 2 * math.atan2(qx, qw)
        pitch = math.pi / 2
        roll = 0
        print("north pole")
    elif qx * qy + qz * qw <= -0.5:  # South pole
        yaw = -2 * math.atan2(qx, qw)
        pitch = -math.pi / 2
        roll = 0
        print("south pole")
    else:
        yaw = math.atan2(2 * qy * qw - 2 * qx * qz, 1 - 2 * qy**2 - 2 * qz**2)
        pitch = math.asin(2 * qx * qy + 2 * qz * qw)
        roll = math.atan2(2 * qx * qw - 2 * qy * qz, 1 - 2 * qx**2 - 2 * qz**2)

    return yaw, pitch, roll


pidx = PIDController(1, 0.4, 0.1, 0)
pidy = PIDController(1, 0.4, 0.1, 0)


t = 0

# Main lop to continuously read serial data and update rocket's orientation
while True:

    line = "e e e"
    # Read roll, pitch, and yaw values from serial port
    try:
        # line = ser.readline().decode("utf-8").strip()
        pass
    except serial.SerialException:
        line = None
        print("Serial communication error")
        input("Press Enter to retry...")
        ser = serial.Serial(
            "COM9", 115200
        )  # Replace '/dev/ttyUSB0' with your serial port
        continue

    if line and len(line.split(" ")) == 3:

        # Uncomment to convert quaternions on Python side
        # q0, q1, q2, q3 = map(double, line.split()[:4])
        # yaw, pitch, roll = quaternion_to_euler(q0, q1, q2, q3)

        # Uncomment to use Arduino-reported angles
        # roll, pitch, yaw = map(float, line.split())

        roll = (t * 20) % 360
        pitch = 20
        yaw = 0

        rocket.pos = vector(0, 0, 0)

        tvcx = pidx.compute(yaw, 0.01)
        tvcy = pidy.compute(pitch, 0.01)

        # pitch += 90
        yaw *= -pi / 180
        pitch *= -pi / 180
        roll *= -pi / 180

        # print(
        #     round(roll * 180 / pi, 2),
        #     round(pitch * 180 / pi, 2),
        #     round(yaw * 180 / pi, 2),
        # )

        k = vector(cos(yaw) * cos(pitch), sin(pitch), sin(yaw) * cos(pitch))
        y = vector(0, 1, 0)
        s = cross(k, y)
        v = cross(s, k)
        vrot = v * cos(roll) + cross(k, v) * sin(roll)

        xrot = vrot
        yrot = cross(k, vrot)

        print("r", tvcx, tvcy)

        tvcx_a = cos(roll) * tvcx + sin(roll) * tvcy
        tvcy_a = cos(roll) * tvcy - sin(roll) * tvcx

        tvcx, tvcy = tvcx_a, tvcy_a

        print("a", roll * 180 / pi, tvcx, tvcy)

        tvc.axis = -k
        tvc.up = vrot
        tvc.rotate(angle=tvcx * pi / 180, axis=xrot)
        tvc.rotate(angle=tvcy * pi / 180, axis=yrot)
    else:
        print("Invalid line, skipping...")

    

    rate(120)  # Update scene at 60 frames per second
    t += 0.01

    frontArrow.axis = k
    upArrow.axis = vrot
    rocket.axis = k
    rocket.up = vrot

