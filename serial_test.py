
import serial
import time
import threading
from rplidar import RPLidar

# ==========================================
# CONFIG
# ==========================================
ROVER_PORT = "/dev/serial0"
ROVER_BAUD = 115200

LIDAR_PORT = "/dev/ttyUSB0"

# Try most common Slamtec baud rates
# A1 often uses 115200
# A2/A3/S1 often use 256000
LIDAR_BAUD_CANDIDATES = [256000, 115200]

FRONT_STOP_DIST = 0.60
SIDE_CLEAR_DIST = 0.80
DANGER_DIST = 0.30

FORWARD_SPEED = 0.28
TURN_SPEED = 0.55
REVERSE_SPEED = -0.18

CONTROL_DT = 0.08
RECONNECT_DELAY = 2.0

# ==========================================
# SHARED STATE
# ==========================================
front = 999.0
left = 999.0
right = 999.0

running = True
lidar_ok = False

lock = threading.Lock()
lidar = None
ser = None


# ==========================================
# ROVER SERIAL
# ==========================================
def open_rover():
    global ser
    ser = serial.Serial(ROVER_PORT, ROVER_BAUD, timeout=1)
    time.sleep(2)
    print(f"[INFO] Rover connected on {ROVER_PORT}")


def send(x, z):
    global ser
    if ser is None:
        return
    try:
        cmd = f'{{"T":13,"X":{x},"Z":{z}}}\n'
        ser.write(cmd.encode())
    except Exception as e:
        print("[SEND ERROR]:", e)


# ==========================================
# HELPERS
# ==========================================
def safe_min(values, default=999.0):
    if not values:
        return default
    filtered = [v for v in values if 0.05 < v < 8.0]
    return min(filtered) if filtered else default


def cleanup_lidar():
    global lidar
    if lidar is not None:
        try:
            lidar.stop()
        except:
            pass
        try:
            lidar.stop_motor()
        except:
            pass
        try:
            lidar.disconnect()
        except:
            pass
        lidar = None


def flush_serial_port(port_obj):
    """
    Try to clear old/corrupted bytes before starting scans.
    """
    try:
        port_obj.stop_motor()
    except:
        pass

    try:
        time.sleep(0.2)
        port_obj._serial_port.reset_input_buffer()
        port_obj._serial_port.reset_output_buffer()
        time.sleep(0.2)
    except:
        pass


def try_connect_lidar():
    global lidar

    for baud in LIDAR_BAUD_CANDIDATES:
        try:
            print(f"[INFO] Trying LiDAR on {LIDAR_PORT} @ {baud}")
            lidar = RPLidar(LIDAR_PORT, baudrate=baud, timeout=2)

            time.sleep(1.0)

            # Flush stale bytes if possible
            flush_serial_port(lidar)

            try:
                info = lidar.get_info()
                health = lidar.get_health()
                print("[INFO] LiDAR info:", info)
                print("[INFO] LiDAR health:", health)
            except Exception as e:
                print("[WARN] Info/health read failed:", e)
                cleanup_lidar()
                continue

            print(f"[INFO] LiDAR connected successfully @ {baud}")
            return True

        except Exception as e:
            print(f"[WARN] Failed at baud {baud}: {e}")
            cleanup_lidar()

    return False


# ==========================================
# LIDAR PROCESSING
# ==========================================
def process_scan(scan):
    global front, left, right

    f = []
    l = []
    r = []

    for _, angle, dist in scan:
        d = dist / 1000.0  # mm -> m

        if angle >= 330 or angle <= 30:
            f.append(d)
        elif 30 < angle <= 100:
            l.append(d)
        elif 260 <= angle < 330:
            r.append(d)

    fmin = safe_min(f)
    lmin = safe_min(l)
    rmin = safe_min(r)

    with lock:
        front = fmin
        left = lmin
        right = rmin


def lidar_loop():
    global running, lidar_ok, front, left, right

    while running:
        try:
            lidar_ok = False
            cleanup_lidar()

            if not try_connect_lidar():
                print("[ERROR] Could not connect to LiDAR. Retrying...")
                with lock:
                    front, left, right = 999.0, 999.0, 999.0
                time.sleep(RECONNECT_DELAY)
                continue

            lidar_ok = True

            # Smaller buffer helps reduce backlog during glitches
            for scan in lidar.iter_scans(max_buf_meas=300):
                if not running:
                    break
                process_scan(scan)

        except Exception as e:
            msg = str(e)
            print("[LIDAR ERROR]:", msg)

            # These are stream-corruption errors from the driver layer
            if "Descriptor length mismatch" in msg or "Wrong body size" in msg:
                print("[INFO] Serial stream corrupted or wrong baud. Reconnecting...")

            lidar_ok = False
            with lock:
                front, left, right = 999.0, 999.0, 999.0

            cleanup_lidar()
            time.sleep(RECONNECT_DELAY)


# ==========================================
# MOTION DECISION
# ==========================================
def choose_motion(f, l, r, lidar_status):
    if not lidar_status:
        return 0.0, 0.0, "LIDAR LOST - STOP"

    if f < DANGER_DIST:
        if l < SIDE_CLEAR_DIST and r < SIDE_CLEAR_DIST:
            if l > r:
                return REVERSE_SPEED, TURN_SPEED, "EMERGENCY REVERSE LEFT"
            else:
                return REVERSE_SPEED, -TURN_SPEED, "EMERGENCY REVERSE RIGHT"

        if l > r:
            return 0.0, TURN_SPEED, "TURN LEFT (DANGER)"
        else:
            return 0.0, -TURN_SPEED, "TURN RIGHT (DANGER)"

    if f < FRONT_STOP_DIST:
        if l > r and l > SIDE_CLEAR_DIST:
            return 0.0, TURN_SPEED, "TURN LEFT"
        elif r > l and r > SIDE_CLEAR_DIST:
            return 0.0, -TURN_SPEED, "TURN RIGHT"
        else:
            if l >= r:
                return 0.0, TURN_SPEED * 0.85, "SEARCH LEFT"
            else:
                return 0.0, -TURN_SPEED * 0.85, "SEARCH RIGHT"

    return FORWARD_SPEED, 0.0, "FORWARD"


# ==========================================
# CONTROL LOOP
# ==========================================
def control_loop():
    global running, lidar_ok

    while running:
        with lock:
            f = front
            l = left
            r = right

        x, z, state = choose_motion(f, l, r, lidar_ok)
        print(f"F:{f:.2f}  L:{l:.2f}  R:{r:.2f}  |  {state}")
        send(x, z)
        time.sleep(CONTROL_DT)


# ==========================================
# CLEANUP
# ==========================================
def cleanup():
    global ser

    print("[INFO] Cleaning up...")
    try:
        send(0, 0)
    except:
        pass

    cleanup_lidar()

    if ser is not None:
        try:
            ser.close()
        except:
            pass


# ==========================================
# MAIN
# ==========================================
def main():
    global running

    try:
        open_rover()

        t1 = threading.Thread(target=lidar_loop, daemon=True)
        t1.start()

        time.sleep(1.0)
        control_loop()

    except KeyboardInterrupt:
        print("\n[INFO] Keyboard interrupt received.")
        running = False

    except Exception as e:
        print("[FATAL ERROR]:", e)
        running = False

    finally:
        cleanup()


if __name__ == "__main__":
    main()
