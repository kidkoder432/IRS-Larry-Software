import numpy as np
import quaternion

from sympy.algebras import Quaternion
from sympy import simplify, sin, cos, latex
from sympy import symbols

y = symbols("y")
x = symbols("x")
theta = symbols("theta")

tvcQ = Quaternion(0, 0, y, x)
rollQ = Quaternion(cos(theta / 2), sin(theta / 2), 0, 0)
print(latex(simplify((rollQ * tvcQ) * rollQ.conjugate())))


while True:
    rollAngle = float(input("Roll Angle: "))
    yawAngle = float(input("TVC Yaw Angle: "))
    pitchAngle = float(input("TVC Pitch Angle: "))

    roll = rollAngle * np.pi / 180
    pitch = pitchAngle * np.pi / 180
    yaw = yawAngle * np.pi / 180

    tvcQuat = np.quaternion(0, 0, pitch, yaw)
    rollQuat = np.quaternion(np.cos(roll / 2), np.sin(roll / 2), 0, 0)

    correctedQuat = (rollQuat * tvcQuat) * rollQuat.conj()

    print("Quaternion Calculation")
    print(f"Adjusted Yaw: {correctedQuat.z * 180 / np.pi:.3f}")
    print(f"Adjusted Pitch: {correctedQuat.y * 180 / np.pi:.3f}")

    x = yaw
    y = pitch
    r = np.sqrt(x * x + y * y)
    theta = np.arctan2(y, x)

    theta -= roll

    adjusted_yaw = r * np.cos(theta)
    adjusted_pitch = r * np.sin(theta)

    print("Polar Calculation")
    print(f"Adjusted Yaw: {adjusted_yaw * 180 / np.pi:.3f}")
    print(f"Adjusted Pitch: {adjusted_pitch * 180 / np.pi:.3f}")
