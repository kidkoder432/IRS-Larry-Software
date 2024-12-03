# Timestamp,Delta Time,Ax,Ay,Az,Gx,Gy,Gz,Yaw,Pitch,Roll,TvcX,TvcY,Alt,State,Vel,Px,Ix,Dx,Py,Iy,Dy

import struct

with open("data.bin", "rb") as f:
    data = f.read()
    print((len(data) - 97))

    header = data[:97]
    startIdx = 97

    with open("data.csv", "w") as f:
        f.write(header.decode("utf-8"))
        print(repr(header.decode("utf-8")))
        while startIdx < len(data):
            line = data[startIdx : startIdx + 90]
            print(repr(line[88:].decode("utf-8")))
            output = struct.unpack("<l13fi7f", line[:88])

            print(output)
            startIdx += 90

            f.write(",".join([str(i) for i in output]) + "\n")
