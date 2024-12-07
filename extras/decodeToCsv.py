# Timestamp,Delta Time,Ax,Ay,Az,Gx,Gy,Gz,Yaw,Pitch,Roll,TvcX,TvcY,Alt,State,Vel,Px,Ix,Dx,Py,Iy,Dy

from re import S
import struct

fmt = "<l13fi7f"

SIZEOF_STRUCT = struct.calcsize(fmt)

def strify(f):
    return str(round(f, 4))

with open("data.bin", "rb") as f:
    data = f.read()
    print((len(data) - 97))

    header = data[:97]
    data = data[97:]

    with open("data.csv", "w") as f:
        f.write(header.decode("utf-8"))
        print(repr(header.decode("utf-8")))

        data = [data[i:i+88] for i in range(0, len(data), 88)]

        i = 0
        while i < len(data):
            if len(data[i]) != 88:
                print(f"Invalid line {i}, skipping")
                i += 1
                continue

            else:
                line = data[i]
                i += 1

            print(i / len(data) * 100, end="\r")
            try:
                dp = struct.unpack(fmt, line)
            except struct.error as e:
                print(f"Invalid line {i}. {e}")
                continue

            f.write(",".join(map(strify, dp)) + "\n")
