import serial
import time
import json

ROVER_PORT = "/dev/ttyUSB1"
ROVER_BAUD = 115200


def send_cmd(ser, cmd):
    msg = json.dumps(cmd) + "\n"
    ser.write(msg.encode("utf-8"))
    print("Sent:", msg.strip())


def main():
    ser = serial.Serial(ROVER_PORT, ROVER_BAUD, timeout=1)
    time.sleep(2)

    try:
        send_cmd(ser, {"T": 1, "L": 0.0, "R": 0.0})
        time.sleep(1)

        send_cmd(ser, {"T": 1, "L": 0.15, "R": 0.15})
        time.sleep(2)

        send_cmd(ser, {"T": 1, "L": 0.0, "R": 0.0})
        time.sleep(1)

        send_cmd(ser, {"T": 1, "L": -0.12, "R": 0.12})
        time.sleep(1.5)

        send_cmd(ser, {"T": 1, "L": 0.0, "R": 0.0})
        time.sleep(1)

        send_cmd(ser, {"T": 1, "L": 0.12, "R": -0.12})
        time.sleep(1.5)

        send_cmd(ser, {"T": 1, "L": 0.0, "R": 0.0})

    finally:
        ser.close()


if __name__ == "__main__":
    main()
