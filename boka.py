"""
rover.py — UGV02 obstacle avoidance with RPLidar
Raspberry Pi 5  →  ESP32 (UART JSON)  +  RPLidar (USB)
"""

import json
import serial
import threading
import time
from flask import Flask, jsonify, render_template_string
from rplidar import RPLidar

# ── Config ────────────────────────────────────────────────────────────────────

ROVER_PORT  = "/dev/serial0"
LIDAR_PORT  = "/dev/ttyUSB0"
BAUD_ROVER  = 115200
BAUD_LIDAR  = [256000, 115200]      # tried in order; A1=115200, A2/A3=256000

# Distance thresholds (metres)
DANGER  = 0.30      # front this close → emergency reverse
CAUTION = 0.60      # front this close → prepare to turn
SIDE_OK = 0.80      # a side counts as "clear" only if reading > this

# Speeds  (range: –1.0 … +1.0)
SPD_FWD  =  0.28
SPD_TURN =  0.55
SPD_REV  = -0.18

LOOP_HZ    = 12         # control decisions per second
RETRY_WAIT = 2.0        # seconds before LiDAR reconnect attempt

# ── Shared state ──────────────────────────────────────────────────────────────

_state   = {"f": 9.9, "l": 9.9, "r": 9.9, "b": 9.9, "ok": False}
_lock    = threading.Lock()
_running = threading.Event()
_running.set()

_scan_points: list[dict[str, float]] = []
_scan_seq = 0
_scan_ts = 0.0

app = Flask(__name__)

WEB_HOST = "0.0.0.0"
WEB_PORT = 8080
MAX_VIS_RANGE_M = 8.0

HTML_PAGE = """
<!doctype html>
<html lang="en">
<head>
    <meta charset="utf-8" />
    <meta name="viewport" content="width=device-width,initial-scale=1" />
    <title>RPLidar Text Display</title>
    <style>
        body {
            margin: 0;
            padding: 24px;
            background: #0a0e14;
            color: #d6e6f8;
            font-family: "Courier New", monospace;
            font-size: 16px;
        }
        .container {
            max-width: 800px;
            margin: 0 auto;
        }
        h1 {
            font-size: 32px;
            margin: 0 0 20px 0;
            color: #58e6b0;
        }
        .status {
            background: #0d1624;
            border: 1px solid #23405f;
            border-radius: 8px;
            padding: 16px;
            margin-bottom: 20px;
        }
        .status-line {
            margin: 8px 0;
            line-height: 1.8;
        }
        .label {
            color: #90a6bf;
            font-weight: bold;
            display: inline-block;
            width: 120px;
        }
        .value {
            color: #58e6b0;
            font-weight: bold;
        }
        .offline { color: #ff6b6b; }
        .near { color: #ffd166; }
        .ok { color: #72f1bc; }
        .grid {
            background: #0d1624;
            border: 1px solid #23405f;
            border-radius: 8px;
            padding: 16px;
            font-size: 12px;
        }
        .grid-header {
            color: #90a6bf;
            margin-bottom: 8px;
            text-align: center;
        }
        .grid-row {
            display: grid;
            grid-template-columns: repeat(12, 1fr);
            gap: 2px;
            margin-bottom: 2px;
        }
        .grid-cell {
            background: #11263d;
            border: 1px solid #244765;
            padding: 4px;
            text-align: center;
            border-radius: 2px;
        }
        .grid-cell.obstacle { background: #ff6b6b; color: black; }
        .grid-cell.near { background: #ffd166; color: black; }
        .grid-cell.ok { background: #58e6b0; color: black; }
    </style>
</head>
<body>
    <div class="container">
        <h1>RPLidar Text Display</h1>
        <div class="status">
            <div class="status-line">
                <span class="label">LiDAR:</span>
                <span id="lidarStatus" class="value">--</span>
            </div>
            <div class="status-line">
                <span class="label">Front:</span>
                <span id="frontVal" class="value">-- m</span>
            </div>
            <div class="status-line">
                <span class="label">Back:</span>
                <span id="backVal" class="value">-- m</span>
            </div>
            <div class="status-line">
                <span class="label">Left:</span>
                <span id="leftVal" class="value">-- m</span>
            </div>
            <div class="status-line">
                <span class="label">Right:</span>
                <span id="rightVal" class="value">-- m</span>
            </div>
            <div class="status-line">
                <span class="label">Scan:</span>
                <span id="scanSeq" class="value">--</span>
            </div>
            <div class="status-line">
                <span class="label">Age:</span>
                <span id="scanAge" class="value">-- ms</span>
            </div>
        </div>
        <div class="grid">
            <div class="grid-header">360° Sweep (12 sectors, 0°=front)</div>
            <div id="gridContainer"></div>
        </div>
    </div>
    <script>
        function fmt(v) {
            if (typeof v !== "number") return "--";
            return v.toFixed(2);
        }

        function formatGrid(points) {
            const sectors = {};
            for (let i = 0; i < 360; i += 30) {
                sectors[i] = [];
            }
            
            for (const p of points) {
                const angle = p.a;
                const sector = Math.round(angle / 30) * 30 % 360;
                sectors[sector].push(p.d);
            }

            let html = '<div class="grid-row">';
            for (let i = 0; i < 12; i++) {
                const deg = i * 30;
                const dists = sectors[deg];
                const minDist = dists.length > 0 ? Math.min(...dists) : 9.9;
                
                let cellClass = 'grid-cell';
                let cellText = minDist < 9.0 ? minDist.toFixed(2) : '∞';
                
                if (minDist < 0.6) cellClass += ' obstacle';
                else if (minDist < 1.2) cellClass += ' near';
                else if (minDist < 9.9) cellClass += ' ok';
                
                const label = deg === 0 ? 'F' : deg === 180 ? 'B' : deg === 90 ? 'L' : deg === 270 ? 'R' : '';
                html += `<div class="${cellClass}" title="${deg}°: ${cellText}m">${label || deg}</div>`;
            }
            html += '</div>';
            return html;
        }

        async function refresh() {
            try {
                const res = await fetch('/api/lidar');
                const data = await res.json();

                const status = document.getElementById('lidarStatus');
                if (data.ok) {
                    status.textContent = 'online';
                    status.className = 'value ok';
                } else {
                    status.textContent = 'offline';
                    status.className = 'value offline';
                }

                document.getElementById('frontVal').textContent = fmt(data.f) + ' m';
                document.getElementById('backVal').textContent = fmt(data.b) + ' m';
                document.getElementById('leftVal').textContent = fmt(data.l) + ' m';
                document.getElementById('rightVal').textContent = fmt(data.r) + ' m';
                document.getElementById('scanSeq').textContent = data.scan_seq || '--';
                
                const ageMs = Math.max(0, Math.round((Date.now() / 1000 - (data.scan_ts || 0)) * 1000));
                document.getElementById('scanAge').textContent = ageMs + ' ms';
                
                document.getElementById('gridContainer').innerHTML = formatGrid(data.points || []);
            } catch (e) {
                console.error(e);
            }
        }

        setInterval(refresh, 120);
        refresh();
    </script>
</body>
</html>
"""

# ── Utilities ─────────────────────────────────────────────────────────────────

def nearest(readings: list[float]) -> float:
    """Return the minimum valid distance from a list, or 9.9 if none."""
    valid = [d for d in readings if 0.05 < d < 8.0]
    return min(valid) if valid else 9.9


def send(ser: serial.Serial, x: float, z: float) -> None:
    """Send a move command to the ESP32 as a JSON line."""
    try:
        cmd = json.dumps({"T": 13, "X": round(x, 2), "Z": round(z, 2)})
        ser.write(cmd.encode() + b"\n")
    except Exception as e:
        print(f"[SEND ERR] {e}")


@app.get("/")
def index():
    return render_template_string(HTML_PAGE)


@app.get("/api/lidar")
def api_lidar():
    with _lock:
        return jsonify({
            "ok": _state["ok"],
            "f": _state["f"],
            "b": _state["b"],
            "l": _state["l"],
            "r": _state["r"],
            "max_range_m": MAX_VIS_RANGE_M,
            "points": _scan_points,
            "scan_seq": _scan_seq,
            "scan_ts": _scan_ts,
        })


def web_thread() -> None:
    print(f"[WEB] http://127.0.0.1:{WEB_PORT}")
    app.run(host=WEB_HOST, port=WEB_PORT, debug=False, use_reloader=False, threaded=True)

# ── Decision logic ────────────────────────────────────────────────────────────

def decide(f: float, l: float, r: float, ok: bool) -> tuple[float, float, str]:
    """
    Return (x_speed, z_rotation, label) for the current sensor snapshot.

    Priority order:
      1. LiDAR offline            → full stop
      2. Front < DANGER           → reverse + steer toward open side
      3. Front < CAUTION          → spin toward open side (or slow search)
      4. All clear                → drive forward
    """
    if not ok:
        return 0.0, 0.0, "stop — no lidar"

    if f < DANGER:
        # Too close: back away and turn toward whichever side has more room
        spin = SPD_TURN if l >= r else -SPD_TURN
        side = "left" if l >= r else "right"
        return SPD_REV, spin, f"emergency — reverse + {side}"

    if f < CAUTION:
        if l > SIDE_OK and l >= r:
            return 0.0, SPD_TURN,        "turn left"
        if r > SIDE_OK and r > l:
            return 0.0, -SPD_TURN,       "turn right"
        # Both sides tight: slow search spin toward the slightly more open side
        spin = SPD_TURN * 0.65 if l >= r else -SPD_TURN * 0.65
        return 0.0, spin,                "searching..."

    return SPD_FWD, 0.0, "forward"

# ── LiDAR thread ─────────────────────────────────────────────────────────────

def lidar_thread() -> None:
    """
    Background thread: connect to LiDAR, stream scans, update shared state.
    Auto-reconnects on any error.
    """
    global _scan_seq, _scan_ts
    while _running.is_set():
        lidar = None
        try:
            # Try each baud rate until get_info() succeeds
            for baud in BAUD_LIDAR:
                try:
                    lidar = RPLidar(LIDAR_PORT, baudrate=baud, timeout=2)
                    lidar.get_info()
                    print(f"[LIDAR] connected @ {baud} baud")
                    break
                except Exception:
                    try: lidar.disconnect()
                    except: pass
                    lidar = None

            if lidar is None:
                raise RuntimeError("could not connect on any baud rate")

            with _lock:
                _state["ok"] = True

            for scan in lidar.iter_scans(max_buf_meas=300):
                if not _running.is_set():
                    break

                f, b, l, r = [], [], [], []
                points = []
                for _, angle, dist in scan:
                    d = dist / 1000.0                   # mm → metres
                    if   angle >= 330 or angle <= 30:   f.append(d)   # front
                    elif angle >= 150 and angle <= 210: b.append(d)   # back
                    elif 30  < angle <= 100:             l.append(d)   # left
                    elif 260 <= angle <  330:            r.append(d)   # right

                    if 0.05 < d < MAX_VIS_RANGE_M:
                        points.append({"a": float(angle), "d": float(d)})

                with _lock:
                    _state.update(f=nearest(f), b=nearest(b), l=nearest(l), r=nearest(r))
                    _scan_points[:] = points
                    _scan_seq += 1
                    _scan_ts = time.time()

        except Exception as e:
            print(f"[LIDAR] {e} — retrying in {RETRY_WAIT}s")

        finally:
            if lidar:
                try: lidar.stop(); lidar.stop_motor(); lidar.disconnect()
                except: pass
            with _lock:
                _state.update(ok=False, f=9.9, l=9.9, r=9.9)
                _scan_points.clear()

        _running.wait(RETRY_WAIT)       # interruptible sleep — exits fast on shutdown

# ── Control loop ──────────────────────────────────────────────────────────────

def control_loop(ser: serial.Serial) -> None:
    """
    Main loop: read shared state, decide, send command, sleep to hit LOOP_HZ.
    Uses time.monotonic() so drift doesn't accumulate.
    """
    dt = 1.0 / LOOP_HZ
    while _running.is_set():
        t0 = time.monotonic()

        with _lock:
            f, b, l, r, ok = _state["f"], _state["b"], _state["l"], _state["r"], _state["ok"]

        x, z, label = decide(f, l, r, ok)
        send(ser, x, z)
        print(f"  F:{f:.2f}  B:{b:.2f}  L:{l:.2f}  R:{r:.2f}   →   {label}")

        time.sleep(max(0.0, dt - (time.monotonic() - t0)))

# ── Entry point ───────────────────────────────────────────────────────────────

def main() -> None:
    ser = serial.Serial(ROVER_PORT, BAUD_ROVER, timeout=1)
    time.sleep(1)
    print(f"[ROVER] serial ready on {ROVER_PORT}")

    tw = threading.Thread(target=web_thread, daemon=True)
    tw.start()

    t = threading.Thread(target=lidar_thread, daemon=True)
    t.start()

    try:
        control_loop(ser)
    except KeyboardInterrupt:
        print("\n[INFO] ctrl-c — shutting down")
    finally:
        _running.clear()        # signals lidar_thread to exit
        send(ser, 0, 0)         # hard stop
        time.sleep(0.2)
        ser.close()
        print("[INFO] done")


if __name__ == "__main__":
    main()