import struct

fmt = "<l12f3h2x8f"

SIZEOF_STRUCT = struct.calcsize(fmt)

print(SIZEOF_STRUCT)

# Time,Dt,Ax,Ay,Az,Gx,Gy,Gz,Roll,Pitch,Yaw,TvcX,TvcY,ActX,ActY,State,Alt,Vel,Px,Ix,Dx,Py,Iy,Dy
SIZEOF_HEADER = 94

def strify(f):
    return str(round(f, 4))

with open("data.bin", "rb") as f:
    data = f.read()
    print((len(data) - SIZEOF_HEADER))

    header = data[:SIZEOF_HEADER]
    data = data[SIZEOF_HEADER:]

    with open("data.csv", "w") as f:
        f.write(header.decode("utf-8"))
        print(repr(header.decode("utf-8")))

        data = [data[i:i+SIZEOF_STRUCT] for i in range(0, len(data), SIZEOF_STRUCT)]

        i = 0
        while i < len(data):
            if len(data[i]) != SIZEOF_STRUCT:
                print(f"Invalid line {i}, skipping", len(data[i]))
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
