from vpython import *
import serial
import math

# Adapted from https://toptechboy.com/9-axis-imu-lesson-20-vpython-visualization-of-roll-pitch-and-yaw/

# Initialize VPython scene
scene = canvas(
    title="Rocket Model",
    width=600,
    height=400,
    center=vec(0, 0, 0),
    background=vec(0.2, 0.2, 0.2),
)

scene.forward = vector(-1, -1, -1)

scene.width = 600
scene.height = 600

xarrow = arrow(length=1, shaftwidth=0.2, color=color.red, axis=vector(1, 0, 0))
yarrow = arrow(length=1, shaftwidth=0.2, color=color.green, axis=vector(0, 1, 0))
zarrow = arrow(length=1, shaftwidth=0.2, color=color.blue, axis=vector(0, 0, 1))

frontArrow = arrow(length=1, shaftwidth=0.1, color=color.purple, axis=vector(1, 0, 0))
upArrow = arrow(length=1, shaftwidth=0.1, color=color.magenta, axis=vector(0, 1, 0))
sideArrow = arrow(length=1, shaftwidth=0.1, color=color.orange, axis=vector(0, 0, 1))

bBoard = box(
    length=6,
    width=2,
    height=0.2,
    opacity=0.8,
    pos=vector(
        0,
        0,
        0,
    ),
)
bn = box(
    length=1, width=0.75, height=0.1, pos=vector(-0.5, 0.1 + 0.05, 0), color=color.blue
)
nano = box(
    length=1.75, width=0.6, height=0.1, pos=vector(-2, 0.1 + 0.05, 0), color=color.green
)
rocket = compound([bBoard, bn, nano])

# Initialize serial port
ser = serial.Serial("COM6", 115200)  # Replace '/dev/ttyUSB0' with your serial port

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

t = 0

# Main lop to continuously read serial data and update rocket's orientation
while True:
    # Read roll, pitch, and yaw values from serial port
    try:
        line = ser.readline().decode("utf-8").strip()
    except serial.SerialException:
        line = None
        print("Serial communication error")
        input("Press Enter to retry...")
        ser = serial.Serial("COM6", 115200)  # Replace '/dev/ttyUSB0' with your serial port
        continue

    if line and len(line.split(" ")) == 3:
        
        # Uncomment to convert quaternions on Python side
        # q0, q1, q2, q3 = map(double, line.split()[:4])
        # yaw, pitch, roll = quaternion_to_euler(q0, q1, q2, q3)

        # Uncomment to use Arduino-reported angles
        yaw, pitch, roll = map(double, line.split())
        yaw *= -pi / 180
        pitch *= -pi / 180
        # pitch += pi / 2
        roll *= -pi / 180
        
        print(round(roll * 180 / pi, 2), round(pitch * 180 / pi, 2), round(yaw * 180 / pi, 2))
    else:
        print("Invalid line, skipping...")

    k = vector(cos(yaw) * cos(pitch), sin(pitch), sin(yaw) * cos(pitch))
    y = vector(0, 1, 0)
    s = cross(k, y)
    v = cross(s, k)
    vrot = v * cos(roll) + cross(k, v) * sin(roll)

    # rate(60)  # Update scene at 60 frames per second
    t += 0.01

    frontArrow.axis = k
    sideArrow.axis = cross(k, vrot)
    upArrow.axis = vrot
    rocket.axis = k
    rocket.up = vrot
    sideArrow.length = 2
    frontArrow.length = 4
    upArrow.length = 1
