import serial
import time
import threading
from rplidar import RPLidar as rp

# =========================
# ROVER SERIAL
# =========================
ser = serial.Serial("/dev/serial0", 115200, timeout=1)
time.sleep(2)

def send(x, z):
    try:
        cmd = f'{{"T":13,"X":{x},"Z":{z}}}\n'
        ser.write(cmd.encode())
    except:
        pass


# =========================
# LIDAR
# =========================
LIDAR_PORT = "/dev/ttyUSB0"
lidar = rp(LIDAR_PORT, baudrate=115200)


# =========================
# SHARED DATA
# =========================
front = 999
left = 999
right = 999


# =========================
# LIDAR THREAD
# =========================
def lidar_loop():
    global front, left, right

    try:
        for scan in lidar.iter_scans():

            f = []
            l = []
            r = []

            for _, angle, dist in scan:

                d = dist / 1000.0

                if 330 <= angle or angle <= 30:
                    f.append(d)
                elif 30 <= angle <= 90:
                    l.append(d)
                elif 270 <= angle <= 330:
                    r.append(d)

            front = min(f) if f else 999
            left = min(l) if l else 999
            right = min(r) if r else 999

    except Exception as e:
        print("LIDAR ERROR:", e)


# =========================
# CONTROL THREAD
# =========================
def control_loop():

    while True:

        print(f"F:{front:.2f} L:{left:.2f} R:{right:.2f}")

        if front < 0.5:
            if left > right:
                send(0, 0.5)
            else:
                send(0, -0.5)
        else:
            send(0.25, 0)

        time.sleep(0.1)


# =========================
# MAIN
# =========================
try:
    t1 = threading.Thread(target=lidar_loop)
    t1.daemon = True
    t1.start()

    control_loop()

except KeyboardInterrupt:
    send(0, 0)
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()
    ser.close()