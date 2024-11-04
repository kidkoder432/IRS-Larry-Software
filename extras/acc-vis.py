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

accX = arrow(length=2, shaftwidth=0.2, color=color.red, axis=vector(1, 0, 0))
accY = arrow(length=2, shaftwidth=0.2, color=color.green, axis=vector(0, 1, 0))
accZ = arrow(length=2, shaftwidth=0.2, color=color.blue, axis=vector(0, 0, 1))

scene.forward = vector(-1, -1, -1)

scene.width = 600
scene.height = 600

accSensor = arrow(length=2, shaftwidth=0.2, color=color.yellow, axis=vector(0, 0, 1))

# Initialize serial port
ser = serial.Serial("COM6", 115200)  # Replace '/dev/ttyUSB0' with your serial port

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
        ser = serial.Serial(
            "COM6", 115200
        )  # Replace '/dev/ttyUSB0' with your serial port
        continue

    # print(line)
    if line and line.startswith("RAW"):

        # Uncomment to convert quaternions on Python side
        # q0, q1, q2, q3 = map(float, line.split()[:4])
        # yaw, pitch, roll = quaternion_to_euler(q0, q1, q2, q3)

        # Uncomment to use Arduino-reported angles
        asx, asy, asz, awx, awy, awz = map(float, line.split()[1:])

        print("ACC Sensor (AS):", asx, asy, asz)
        print("ACC World (AW):", awx, awy, awz)

        accSensor.axis = vector(asx, asy, asz)
        accSensor.length = 2 * sqrt(asx**2 + asy**2 + asz**2)
        
        accX.length = 2 * awx
        accY.length = 2 * awy
        accZ.length = 2 * awz
    

