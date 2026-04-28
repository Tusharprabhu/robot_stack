import cv2
import time
import json
import os
import threading
import serial
from flask import Flask, Response, jsonify, render_template
from rplidar import RPLidar

# =========================
# CONFIG
# =========================

CAMERA_PREFERRED_DEVICES = [
    "/dev/video0",
    "/dev/video1",
    "/dev/video2",
]

FRAME_W = 640
FRAME_H = 480
FRAME_FPS = 30

LIDAR_PORT = "/dev/ttyUSB0"

ROVER_PORT = "/dev/ttyUSB1"
ROVER_BAUD = 115200

STOP_DIST_M = 0.80
CAUTION_DIST_M = 1.50

FORWARD_SPEED = 0.15
TURN_SPEED = 0.14
TURN_TIME = 0.45

FRONT_SECTOR = [(330, 360), (0, 30)]
LEFT_SECTOR = [(30, 90)]
RIGHT_SECTOR = [(270, 330)]

USE_ROVER = True

# =========================
# GLOBALS
# =========================

app = Flask(__name__)

frame_lock = threading.Lock()
latest_jpeg = None
latest_frame = None

rover_ser = None
camera_cap = None

state = {
    "status": "BOOTING",
    "decision": "NONE",
    "camera_ok": False,
    "lidar_ok": False,
    "rover_ok": False,
    "camera_source": None,
    "nearest_front_m": None,
    "nearest_left_m": None,
    "nearest_right_m": None,
}

# =========================
# HELPERS
# =========================

def try_open_camera(source):
    cap = cv2.VideoCapture(source, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap.release()
        return None

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
    cap.set(cv2.CAP_PROP_FPS, FRAME_FPS)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    time.sleep(1.0)

    for _ in range(5):
        cap.read()

    ok, frame = cap.read()
    if not ok or frame is None:
        cap.release()
        return None

    return cap


def open_best_camera():
    for dev in CAMERA_PREFERRED_DEVICES:
        if os.path.exists(dev):
            cap = try_open_camera(dev)
            if cap is not None:
                return cap, dev

    for idx in range(6):
        cap = try_open_camera(idx)
        if cap is not None:
            return cap, str(idx)

    return None, None


def angle_in_ranges(angle, ranges):
    for a1, a2 in ranges:
        if a1 <= a2:
            if a1 <= angle <= a2:
                return True
        else:
            if angle >= a1 or angle <= a2:
                return True
    return False


def min_distance_for_sector(scan, ranges):
    vals = []
    for quality, angle, distance_mm in scan:
        if distance_mm <= 0:
            continue
        if angle_in_ranges(angle, ranges):
            vals.append(distance_mm / 1000.0)
    return min(vals) if vals else None


def send_rover(L, R):
    global rover_ser
    if not USE_ROVER or rover_ser is None:
        return
    cmd = {"T": 1, "L": round(L, 3), "R": round(R, 3)}
    msg = json.dumps(cmd) + "\n"
    rover_ser.write(msg.encode("utf-8"))


def rover_stop():
    send_rover(0.0, 0.0)


def rover_forward():
    send_rover(FORWARD_SPEED, FORWARD_SPEED)


def rover_turn_left():
    send_rover(-TURN_SPEED, TURN_SPEED)


def rover_turn_right():
    send_rover(TURN_SPEED, -TURN_SPEED)


def init_rover():
    global rover_ser
    if not USE_ROVER:
        state["rover_ok"] = False
        return

    rover_ser = serial.Serial(ROVER_PORT, ROVER_BAUD, timeout=1)
    time.sleep(2)
    rover_stop()
    state["rover_ok"] = True


# =========================
# THREADS
# =========================

def camera_loop():
    global latest_jpeg, latest_frame, camera_cap

    cap, source = open_best_camera()

    if cap is None:
        state["camera_ok"] = False
        state["camera_source"] = None
        state["status"] = "CAMERA ERROR"
        return

    camera_cap = cap
    state["camera_ok"] = True
    state["camera_source"] = str(source)

    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            time.sleep(0.05)
            continue

        decision = state["decision"]
        front = state["nearest_front_m"]
        left = state["nearest_left_m"]
        right = state["nearest_right_m"]
        cam_src = state["camera_source"]

        if decision == "STOP":
            color = (0, 0, 255)
        elif "TURN" in decision:
            color = (0, 165, 255)
        elif decision == "CAUTION":
            color = (0, 255, 255)
        else:
            color = (0, 255, 0)

        cv2.putText(frame, f"Decision: {decision}", (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

        cv2.putText(frame, f"Front: {front} m", (20, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(frame, f"Left : {left} m", (20, 100),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(frame, f"Right: {right} m", (20, 130),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(frame, f"Camera: {cam_src}", (20, 160),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        h, w = frame.shape[:2]
        x1, x2 = int(w * 0.35), int(w * 0.65)
        y1, y2 = int(h * 0.25), int(h * 0.95)
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        cv2.putText(frame, "FORWARD ZONE", (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        ok, jpeg = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
        if ok:
            with frame_lock:
                latest_frame = frame
                latest_jpeg = jpeg.tobytes()

        time.sleep(0.02)


def lidar_loop():
    lidar = RPLidar(LIDAR_PORT)

    try:
        print("LIDAR info:", lidar.get_info())
        print("LIDAR health:", lidar.get_health())
        state["lidar_ok"] = True

        for scan in lidar.iter_scans():
            front = min_distance_for_sector(scan, FRONT_SECTOR)
            left = min_distance_for_sector(scan, LEFT_SECTOR)
            right = min_distance_for_sector(scan, RIGHT_SECTOR)

            state["nearest_front_m"] = None if front is None else round(front, 2)
            state["nearest_left_m"] = None if left is None else round(left, 2)
            state["nearest_right_m"] = None if right is None else round(right, 2)

            if front is not None and front < STOP_DIST_M:
                if USE_ROVER:
                    rover_stop()
                    time.sleep(0.15)

                if left is None and right is None:
                    decision = "STOP"
                elif right is None:
                    decision = "TURN LEFT"
                    if USE_ROVER:
                        rover_turn_left()
                        time.sleep(TURN_TIME)
                        rover_stop()
                elif left is None:
                    decision = "TURN RIGHT"
                    if USE_ROVER:
                        rover_turn_right()
                        time.sleep(TURN_TIME)
                        rover_stop()
                elif left > right:
                    decision = "TURN LEFT"
                    if USE_ROVER:
                        rover_turn_left()
                        time.sleep(TURN_TIME)
                        rover_stop()
                else:
                    decision = "TURN RIGHT"
                    if USE_ROVER:
                        rover_turn_right()
                        time.sleep(TURN_TIME)
                        rover_stop()

            elif front is not None and front < CAUTION_DIST_M:
                decision = "CAUTION"
                if USE_ROVER:
                    rover_forward()

            else:
                decision = "CLEAR"
                if USE_ROVER:
                    rover_forward()

            state["decision"] = decision
            state["status"] = decision

    except Exception as e:
        state["lidar_ok"] = False
        state["status"] = f"LIDAR ERROR: {e}"
        print("LIDAR error:", e)

    finally:
        try:
            rover_stop()
        except Exception:
            pass
        try:
            lidar.stop()
            lidar.stop_motor()
            lidar.disconnect()
        except Exception:
            pass


# =========================
# FLASK
# =========================

@app.route("/")
def index():
    return render_template("index.html")


@app.route("/video_feed")
def video_feed():
    def gen():
        global latest_jpeg
        while True:
            with frame_lock:
                frame = latest_jpeg
            if frame is not None:
                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
                )
            time.sleep(0.03)

    return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/api/status")
def api_status():
    return jsonify(state)


if __name__ == "__main__":
    if USE_ROVER:
        init_rover()

    threading.Thread(target=camera_loop, daemon=True).start()
    threading.Thread(target=lidar_loop, daemon=True).start()

    app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)
