"""UGV02 obstacle avoidance with RPLidar."""

from __future__ import annotations

import json
import threading
import time

import serial
from flask import Flask, jsonify, render_template_string
from rplidar import RPLidar

# ── Config ────────────────────────────────────────────────────────────────────

ROVER_PORT = "/dev/serial0"
LIDAR_PORT = "/dev/ttyUSB0"
BAUD_ROVER = 115200
BAUD_LIDAR = [256000, 115200]

# Distance thresholds (metres)
DANGER = 0.30
CAUTION = 0.60
SLOW_RANGE = 2.00
SIDE_OK = 0.80

# Speeds  (range: –1.0 … +1.0)
SPD_FWD = 0.28
SPD_SLOW = 0.14
SPD_TURN = 0.55
SPD_REV = -0.18

LOOP_HZ = 12
RETRY_WAIT = 0.5  # Reduced from 2.0s to recover instantly from data checksum errors

# ── Shared state ──────────────────────────────────────────────────────────────

_state = {"f": 9.9, "l": 9.9, "r": 9.9, "b": 9.9, "ok": False, "cmd": "Initializing", "info": "Waiting for LIDAR"}
_lock = threading.Lock()
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
    <title>Rover Dashboard Live</title>
    <style>
        :root { --bg: #0f172a; --panel: #1e293b; --text: #94a3b8; --accent: #38bdf8; --danger: #f87171; --ok: #4ade80; --warn: #facc15; }
        body { margin: 0; padding: 20px; background: var(--bg); color: var(--text); font-family: "Segoe UI", system-ui, sans-serif; }
        .grid-layout { display: grid; grid-template-columns: 300px 1fr; gap: 20px; max-width: 1200px; margin: 0 auto; }
        @media (max-width: 800px) { .grid-layout { grid-template-columns: 1fr; } }
        .panel { background: var(--panel); border-radius: 12px; padding: 20px; box-shadow: 0 4px 6px -1px rgb(0 0 0 / 0.1); }
        h1, h2, h3 { color: white; margin-top: 0; font-weight: 500; }
        .stat-box { margin-bottom: 20px; }
        .stat-label { font-size: 0.85em; text-transform: uppercase; letter-spacing: 1px; color: var(--text); margin-bottom: 4px; }
        .stat-value { font-size: 1.6em; font-weight: bold; color: var(--accent); }
        .stat-sm { font-size: 1.1em; color: var(--text); font-weight: normal; }
        
        /* Command Display */
        .cmd-display {
            font-size: 2em; text-align: center; margin-bottom: 20px; padding: 15px; 
            background: rgba(0, 0, 0, 0.3); border-radius: 8px; border: 2px solid var(--ok); 
            text-transform: uppercase; letter-spacing: 2px; font-weight: bold; color: var(--ok);
            transition: all 0.2s;
        }
        
        /* Radar Canvas */
        .radar-container {
            position: relative; width: 100%; max-width: 600px; margin: 0 auto;
            aspect-ratio: 1; border-radius: 50%; border: 2px solid #334155;
            box-shadow: 0 0 30px rgba(56, 189, 248, 0.1); background: #0b1121;
            overflow: hidden;
        }
        canvas { width: 100%; height: 100%; display: block; }
        
        .status-indicator { display: inline-block; width: 12px; height: 12px; border-radius: 50%; margin-right: 8px; }
        .online { background: var(--ok); box-shadow: 0 0 8px var(--ok); }
        .offline { background: var(--danger); box-shadow: 0 0 8px var(--danger); }
    </style>
</head>
<body>
    <div class="grid-layout">
        <!-- Sidebar -->
        <div class="sidebar">
            <div class="panel" style="margin-bottom: 20px;">
                <h2>System Status</h2>
                <div style="margin-bottom: 20px; display: flex; align-items: center; color: white; font-size: 1.1em;">
                    <span id="connDot" class="status-indicator offline"></span>
                    <span id="connText">LIDAR Offline</span>
                </div>
                <div class="stat-box">
                    <div class="stat-label">LiDAR Device Info</div>
                    <div class="stat-value stat-sm" id="lidarInfo">Waiting...</div>
                </div>
                <div class="stat-box">
                    <div class="stat-label">Latency / Buffer</div>
                    <div class="stat-value stat-sm"><span id="scanAge">--</span> ms (<span id="scanSeq">--</span>)</div>
                </div>
            </div>
            
            <div class="panel">
                <h2>Distance Sensors</h2>
                <div class="stat-box"><div class="stat-label">Front (0°)</div><div class="stat-value" id="valF">-- m</div></div>
                <div class="stat-box"><div class="stat-label">Back (180°)</div><div class="stat-value" id="valB">-- m</div></div>
                <div class="stat-box"><div class="stat-label">Left (90°)</div><div class="stat-value" id="valL">-- m</div></div>
                <div class="stat-box"><div class="stat-label">Right (270°)</div><div class="stat-value" id="valR">-- m</div></div>
            </div>
        </div>
        
        <!-- Main Content -->
        <div class="main-content panel">
            <h2>Live Navigator</h2>
            <div class="cmd-display" id="cmdDisplay">Waiting for Instruction</div>
            
            <div class="radar-container">
                <canvas id="lidarCanvas" width="800" height="800"></canvas>
            </div>
        </div>
    </div>

    <script>
        const canvas = document.getElementById('lidarCanvas');
        const ctx = canvas.getContext('2d');
        const center = canvas.width / 2;
        const maxRange = 4.0; // Draw up to 4 meters visually
        const scale = center / maxRange;

        function drawRadar(points) {
            ctx.clearRect(0, 0, canvas.width, canvas.height);
            
            // Draw grid rings
            ctx.lineWidth = 1.5;
            for (let r = 1; r <= 4; r++) {
                ctx.beginPath();
                ctx.strokeStyle = r === 4 ? '#334155' : '#1e293b'; 
                ctx.arc(center, center, r * scale, 0, 2 * Math.PI);
                ctx.stroke();
                
                // Ring Labels
                ctx.fillStyle = '#64748b';
                ctx.font = '16px sans-serif';
                ctx.fillText(r + 'm', center + 6, center - (r * scale) + 20);
            }
            
            // Draw axes / crosshairs
            ctx.strokeStyle = '#2d3b54';
            ctx.beginPath();
            ctx.moveTo(center, 0); ctx.lineTo(center, canvas.height);
            ctx.moveTo(0, center); ctx.lineTo(canvas.width, center);
            ctx.stroke();

            // Draw Points
            if (points && points.length) {
                for (const p of points) {
                    if (p.d > maxRange) continue;
                    
                    // Convert rover angle to canvas angle. 
                    // Rover: 0 is Front. Canvas: 0 is Right, -90 is Top (Front).
                    const canvasAngleRad = (p.a - 90) * Math.PI / 180;
                    
                    const x = center + (p.d * scale) * Math.cos(canvasAngleRad);
                    const y = center + (p.d * scale) * Math.sin(canvasAngleRad);
                    
                    ctx.beginPath();
                    if (p.d < 0.4) ctx.fillStyle = '#f87171'; // Danger
                    else if (p.d < 1.0) ctx.fillStyle = '#facc15'; // Near
                    else ctx.fillStyle = '#38bdf8'; // Safe
                    ctx.arc(x, y, 4, 0, 2 * Math.PI);
                    ctx.fill();
                }
            }

            // Draw Rover Avatar in Center
            ctx.fillStyle = '#4ade80';
            ctx.beginPath();
            // Arrow pointing UP
            ctx.moveTo(center, center - 24);
            ctx.lineTo(center - 16, center + 20);
            ctx.lineTo(center, center + 8);
            ctx.lineTo(center + 16, center + 20);
            ctx.fill();
        }

        async function refresh() {
            try {
                const res = await fetch('/api/lidar');
                const data = await res.json();
                
                // Status markers
                const dot = document.getElementById('connDot');
                const txt = document.getElementById('connText');
                if (data.ok) {
                    dot.className = 'status-indicator online';
                    txt.textContent = 'LIDAR Online Data Live';
                } else {
                    dot.className = 'status-indicator offline';
                    txt.textContent = 'LIDAR Offline / Searching...';
                }

                // Info 
                document.getElementById('lidarInfo').textContent = data.info || '--';
                document.getElementById('scanSeq').textContent = data.scan_seq;
                const ageMs = Math.max(0, Math.round((Date.now() / 1000 - (data.scan_ts || 0)) * 1000));
                document.getElementById('scanAge').textContent = ageMs;

                // Format sensors
                const fmt = v => (typeof v === 'number' && v < 9.0) ? v.toFixed(2) + ' m' : 'Clear (∞)';
                document.getElementById('valF').textContent = fmt(data.f);
                document.getElementById('valB').textContent = fmt(data.b);
                document.getElementById('valL').textContent = fmt(data.l);
                document.getElementById('valR').textContent = fmt(data.r);
                
                // Command Instruction Style
                const cmdEl = document.getElementById('cmdDisplay');
                cmdEl.textContent = data.cmd.toUpperCase();
                if (data.cmd.includes('emergency') || data.cmd.includes('stop')) {
                    cmdEl.style.color = 'var(--danger)'; cmdEl.style.borderColor = 'var(--danger)';
                } else if (data.cmd.includes('slow') || data.cmd.includes('search') || data.cmd.includes('turn')) {
                    cmdEl.style.color = 'var(--warn)'; cmdEl.style.borderColor = 'var(--warn)';
                } else {
                    cmdEl.style.color = 'var(--ok)'; cmdEl.style.borderColor = 'var(--ok)';
                }

                // Protect against frozen / static data
                if (ageMs > 1000 || !data.ok) {
                    drawRadar([]); // Clear map completely if data is older than 1 second
                    if (data.ok) {
                        dot.className = 'status-indicator offline';
                        txt.textContent = 'DATA STALE - Waiting for new scan...';
                    }
                } else {
                    drawRadar(data.points);
                }
            } catch (e) { console.error(e); }
        }

        setInterval(refresh, 60); // Faster refresh for visual snappiness
        drawRadar([]);
    </script>
</body>
</html>
"""

# ── Utilities ─────────────────────────────────────────────────────────────────

def nearest(readings: list[float]) -> float | None:
    """Return the minimum valid distance from a list, or None if none."""
    valid = [d for d in readings if 0.05 < d < 8.0]
    return min(valid) if valid else None


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
            "cmd": _state.get("cmd", "--"),
            "info": _state.get("info", "--"),
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
    if not ok:
        return 0.0, 0.0, "stop — no lidar"

    if f < DANGER:
        spin = SPD_TURN if l >= r else -SPD_TURN
        side = "left" if l >= r else "right"
        return SPD_REV, spin, f"emergency — reverse + {side}"

    if f < CAUTION:
        if l > SIDE_OK and l >= r:
            return 0.0, SPD_TURN, "turn left"
        if r > SIDE_OK and r > l:
            return 0.0, -SPD_TURN, "turn right"
        spin = SPD_TURN * 0.65 if l >= r else -SPD_TURN * 0.65
        return 0.0, spin, "searching..."

    if f < SLOW_RANGE:
        spin = SPD_TURN * 0.25 if l >= r else -SPD_TURN * 0.25
        return SPD_SLOW, spin, "slow approach"

    return SPD_FWD, 0.0, "forward"


# ── LiDAR thread ─────────────────────────────────────────────────────────────

def lidar_thread() -> None:
    global _scan_seq, _scan_ts
    while _running.is_set():
        lidar = None
        try:
            for baud in BAUD_LIDAR:
                try:
                    lidar = RPLidar(LIDAR_PORT, baudrate=baud, timeout=2)
                    info = lidar.get_info()
                    with _lock:
                        _state["info"] = f"Model {info.get('model', '?')}, Firmware {info.get('firmware', '?')}"
                    print(f"[LIDAR] connected @ {baud} baud")
                    break
                except Exception:
                    try:
                        if lidar:
                            lidar.disconnect()
                    except Exception:
                        pass
                    lidar = None

            if lidar is None:
                raise RuntimeError("could not connect on any baud rate")

            with _lock:
                _state["ok"] = True

            lidar.clean_input()
            
            # Track rolling minimums for each sector
            min_f, min_b, min_l, min_r = 9.9, 9.9, 9.9, 9.9
            points = []
            
            # Record initial time so first new_scan trigger doesn't have an empty timestamp
            start_time = time.time()

            # Increased max_buf_meas to 3000 to prevent buffer exhaustion on fast scanning (common on RPi)
            for new_scan, quality, angle, dist in lidar.iter_measurements(max_buf_meas=3000):
                if not _running.is_set():
                    break

                if new_scan:
                    # Filter out partial/empty sweeps when LiDAR is first booting up
                    if time.time() - start_time > 0.05 and points:
                        # A new 360 sweep started, update the global state with our fastest findings
                        with _lock:
                            _state["f"] = min_f if min_f < 8.0 else 9.9
                            _state["b"] = min_b if min_b < 8.0 else 9.9
                            _state["l"] = min_l if min_l < 8.0 else 9.9
                            _state["r"] = min_r if min_r < 8.0 else 9.9
                            _state["ok"] = True
                            _scan_points[:] = points
                            _scan_seq += 1
                            _scan_ts = time.time()
                    
                    # Reset accumulators for the next sweep
                    min_f, min_b, min_l, min_r = 9.9, 9.9, 9.9, 9.9
                    points = []
                    start_time = time.time()

                d = dist / 1000.0
                if 0.05 < d < MAX_VIS_RANGE_M:
                    points.append({"a": float(angle), "d": float(d)})
                    
                    # Instantly update running minimums
                    if angle >= 330 or angle <= 30:
                        min_f = min(min_f, d)
                    elif 150 <= angle <= 210:
                        min_b = min(min_b, d)
                    elif 30 < angle <= 100:
                        min_l = min(min_l, d)
                    elif 260 <= angle < 330:
                        min_r = min(min_r, d)

        except Exception as e:
            print(f"[LIDAR] {e} — retrying in {RETRY_WAIT}s")

        finally:
            if lidar:
                try:
                    lidar.stop()
                    lidar.stop_motor()
                    lidar.disconnect()
                except Exception:
                    pass
            with _lock:
                _state["ok"] = False
                _scan_points.clear()

        _running.wait(RETRY_WAIT)


# ── Control loop ──────────────────────────────────────────────────────────────

def control_loop(ser: serial.Serial) -> None:
    dt = 1.0 / LOOP_HZ
    while _running.is_set():
        t0 = time.monotonic()

        with _lock:
            f = _state["f"]
            b = _state["b"]
            l = _state["l"]
            r = _state["r"]
            ok = _state["ok"]

        x, z, label = decide(f, l, r, ok)
        with _lock:
            _state["cmd"] = label
        send(ser, x, z)
        print(f"  F:{f:.2f}  B:{b:.2f}  L:{l:.2f}  R:{r:.2f}   →   {label}")

        time.sleep(max(0.0, dt - (time.monotonic() - t0)))


# ── Entry point ───────────────────────────────────────────────────────────────

def main() -> None:
    ser = serial.Serial(ROVER_PORT, BAUD_ROVER, timeout=1)
    time.sleep(1)
    print(f"[ROVER] serial ready on {ROVER_PORT}")

    threading.Thread(target=web_thread, daemon=True).start()
    threading.Thread(target=lidar_thread, daemon=True).start()

    try:
        control_loop(ser)
    except Exception as e:
        print(f"\n[INFO] shutdown — {e}")
    finally:
        _running.clear()
        try:
            send(ser, 0, 0)
            time.sleep(0.2)
            ser.close()
        except:
            pass
        print("[INFO] done")


if __name__ == "__main__":
    main()