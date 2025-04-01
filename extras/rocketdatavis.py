"""
Visualization of rocket orientation and TVC using VPython.

Load data from a CSV file, create a scene with a rocket and 4 fins, and animate the rocket's orientation and TVC based on the data.

The data file is expected to have the following columns:
    Time, Dt, Ax, Ay, Az, Gx, Gy, Gz, Roll, Pitch, Yaw, TvcX, TvcY, State, Alt, Vel, Px, Ix, Dx, Py, Iy, Dy

The visualization includes:
    - A rocket body with 4 fins
    - A movable nozzle for TVC
    - A table displaying key values (time, roll, pitch, yaw, TVC X, TVC Y, and state)
    - Thin lines showing the main 3 axes
"""

import csv
from vpython import *

# Load the data
data = []
with open("./data0328.csv") as file:
    reader = csv.reader(file)
    next(reader)  # Skip the header
    for row in reader:
        if len(row) < 22:  # Ensure row has all necessary columns
            continue
        data.append([float(x) for x in row])

# Create the scene
scene = canvas(
    title="Rocket Orientation and TVC Visualization",
    width=800,
    height=600,
    center=vector(0, 2, 0),
    forward=vector(0, -1, -2),
)

# Rocket body (fixed position)
rocketc = cylinder(
    pos=vector(0, 0, 0), axis=vector(1, 0, 0), radius=0.2, color=color.red
)

# Create fins at four symmetrical positions
fin_offset = -0.25  # Distance of the fins from the rocket center
fin_size = vector(0.3, 0.05, 0.05)  # (thickness, height, width)

# Define initial fin positions relative to the rocket
fin_positions = [
    vector(0.5, 0, fin_offset),
    vector(0.5, 0, -fin_offset),
    vector(0.5, fin_offset, 0),
    vector(0.5, -fin_offset, 0),
]

# Create fin objects
fins = [
    box(pos=rocketc.pos + pos, size=fin_size, color=color.red) for pos in fin_positions
]

# Engine nozzle (moves for TVC)
nozzle = cone(
    pos=rocketc.pos + vector(-0.6, 0, 0),
    axis=vector(-0.3, 0, 0),
    radius=0.15,
    color=color.orange,
)

# Create rocket with fins as a single compound object
rocket = compound(fins + [rocketc])
# Update the rocket's position to point upwards
rocket.axis = vector(0, 1, 0)

# UI Elements
scene.append_to_caption("\n")

i = 0
k = 1


# Button to control animation
def toggle_play(b):
    global i, k
    if b.text == "Play":
        b.text = "Pause"
        k = 1
    else:
        b.text = "Play"
        k = 0


button_play = button(text="Play", bind=toggle_play)


# Function to handle slider change
def on_slider_change():
    global i
    i = int(slider_t.value)


# Slider (full width)
slider_t = slider(
    min=0, max=len(data) - 1, value=0, length=scene.width, bind=on_slider_change
)
scene.append_to_caption("\nUse the slider to scrub through time\n")

# Table for displaying key values
metrics_text = wtext(
    text="Time: 0  |  Roll: 0°  |  Pitch: 0°  |  Yaw: 0°  |  TVC X: 0°  |  TVC Y: 0°  |  State: 0"
)

# Add thin lines showing the main 3 axes
x_axis = arrow(
    pos=vector(0, 0, 0), axis=vector(1, 0, 0), color=color.green, shaftwidth=0.04
)
y_axis = arrow(
    pos=vector(0, 0, 0), axis=vector(0, 1, 0), color=color.blue, shaftwidth=0.04
)
z_axis = arrow(
    pos=vector(0, 0, 0), axis=vector(0, 0, 1), color=color.red, shaftwidth=0.04
)


# Function to update rocket orientation and TVC
def update_rocket(index):
    if 0 <= index < len(data):
        time, _, _, _, _, _, _, _, roll, pitch, yaw, tvc_x, tvc_y, state, _, _, *_ = (
            data[index]
        )

        # Convert angles to radians
        roll_rad, pitch_rad, yaw_rad = radians(roll), radians(pitch), radians(yaw)
        tvc_x_rad, tvc_y_rad = radians(tvc_x - 100), radians(
            tvc_y - 97
        )  # Adjusting to center

        # Reset rocket to initial orientation
        rocket.axis = vector(0, 1, 0)
        rocket.up = vector(0, 0, 1)  # Ensures correct rotation order
        rocket.pos = vector(0, 0, 0)

        # Apply rotations in correct order:
        rocket.rotate(angle=roll_rad, axis=rocket.axis)  # Roll around main axis
        rocket.rotate(angle=pitch_rad, axis=vector(1, 0, 0))  # Tilt forward/back
        rocket.rotate(angle=yaw_rad, axis=vector(0, 0, 1))  # Tilt left/right

        # Apply independent rotation to nozzle (TVC deflection)
        # We update the nozzle's axis based on TVC while keeping it in the rocket's compound
        nozzle.pos = (
            rocket.pos + rocket.axis * -0.6
        )  # Keep nozzle position relative to the rocket

        nozzle.axis = rocket.axis * -0.4
        nozzle.rotate(angle=tvc_x_rad, axis=vector(0, 0, 1))
        nozzle.rotate(angle=tvc_y_rad, axis=vector(1, 0, 0))

        # Update table text
        metrics_text.text = (
            f"Time: {time}  |  Roll: {roll:.2f}°  |  Pitch: {pitch:.2f}°  |  Yaw: {yaw:.2f}°  |  "
            f"TVC X: {tvc_x:.2f}°  |  TVC Y: {tvc_y:.2f}°  |  State: {state}"
        )


while True:
    rate(100)
    slider_t.value = i
    update_rocket(i)
    i = (i + k) % len(data)
