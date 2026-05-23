import struct

# <  : Little-endian
# L  : timestamp (uint32)
# 9f : 3 accel, 3 gyro, 3 orientation (float32)
# 12h: x_out, y_out, state, alt, v_vel, px, ix, dx, py, iy, dy, dt (int16 signed)
fmt = "<L9f12h"

SIZEOF_STRUCT = struct.calcsize(fmt)
SIZEOF_HEADER = 86


def strify(val):
    if isinstance(val, float):
        return str(round(val, 4))
    return str(val)


with open("data.bin", "rb") as f_bin:
    raw_payload = f_bin.read()

    # Strip header
    data = raw_payload[SIZEOF_HEADER:]

    print(f"Struct Size: {SIZEOF_STRUCT} bytes")
    print(f"Records found: {len(data) // SIZEOF_STRUCT}")

    with open("data.csv", "w") as f_csv:
        # Write CSV Header (Adjust names if they don't match your Vec3D order)
        cols = [
            "time",
            "ax",
            "ay",
            "az",
            "gx",
            "gy",
            "gz",
            "ox",
            "oy",
            "oz",
            "x_out",
            "y_out",
            "state",
            "alt",
            "vert_vel",
            "px",
            "ix",
            "dx",
            "py",
            "iy",
            "dy",
            "dt",
        ]
        f_csv.write(",".join(cols) + "\n")

        # Step through the binary data
        for i in range(0, (len(data) // SIZEOF_STRUCT) * SIZEOF_STRUCT, SIZEOF_STRUCT):
            chunk = data[i : i + SIZEOF_STRUCT]

            try:
                # Unpack into a list to allow modification
                dp = list(struct.unpack(fmt, chunk))

                # --- REVERSE THE SCALING ---
                # dp[0..9] are already floats (Time, IMU, Orientation)

                # Outputs & Altimetry (Indices 10, 11, 13, 14)
                dp[10] /= 100.0  # x_out
                dp[11] /= 100.0  # y_out
                # dp[12] is 'state', no scaling needed
                dp[13] /= 100.0  # alt
                dp[14] /= 100.0  # vert_vel

                # PID Values (Indices 15 through 20)
                for idx in range(15, 21):
                    dp[idx] /= 100.0

                # Delta Time (Index 21) - ms to seconds
                dp[21] /= 1000.0

                f_csv.write(",".join(map(strify, dp)) + "\n")

            except struct.error:
                continue

print("Conversion complete: data.csv")
