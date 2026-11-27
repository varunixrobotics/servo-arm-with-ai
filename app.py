"""
=============================================================================
S.A.M. Mission Control - Lightweight Web Interface
=============================================================================
- 480p @ 2 FPS MJPEG stream (no lag)
- Camera always-on; toggle detection overlays via terminal (press T)
- Gemini API key entry in UI
- Robot arm photo as background
- 5-axis joint sliders + gripper toggle
- Colored cube detection cards + 4-dot gripper status
- Voice trigger "give it to me"
=============================================================================
"""

import os
import sys
import time
import math
import json
import re
import logging
import threading
import signal

import cv2
import numpy as np
import urllib.request
import urllib.error
from flask import Flask, render_template, Response, request, jsonify
from flask_cors import CORS

# ── Graceful import of optional hardware libs ──────────────────────────────
try:
    import mediapipe as mp
    HAS_MP = True
except ImportError:
    HAS_MP = False

try:
    import serial
    import serial.tools.list_ports
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False

try:
    import speech_recognition as sr
    HAS_SPEECH = True
except ImportError:
    HAS_SPEECH = False

# ── Logging ────────────────────────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] %(levelname)s  %(message)s",
    datefmt="%H:%M:%S"
)
log = logging.getLogger("SAM")
logging.getLogger("werkzeug").setLevel(logging.WARNING)

# ── Flask ──────────────────────────────────────────────────────────────────
app = Flask(__name__, static_folder="static", template_folder="templates")
app.config['SEND_FILE_MAX_AGE_DEFAULT'] = 0
app.config['TEMPLATES_AUTO_RELOAD'] = True
CORS(app)

# =============================================================================
# CONFIGURATION
# =============================================================================
class Cfg:
    CAM_INDEX   = 0
    CAM_W       = 640     # 480p-ish
    CAM_H       = 480
    STREAM_FPS  = 10      # 10 FPS stream
    JPEG_Q      = 55      # Lower = faster / less bandwidth

    SERIAL_PORT = "AUTO"
    BAUD        = 115200

    # HSV – white gripper dots
    W_LO = np.array([0,   0,  195], np.uint8)
    W_HI = np.array([180, 60, 255], np.uint8)
    DOT_AREA_MIN = 10
    DOT_AREA_MAX = 600
    DOT_CLUSTER  = 150

    # HSV – colored cubes
    CUBE_HSV = {
        "RED":   [(np.array([0,110,70],np.uint8),  np.array([10,255,255],np.uint8)),
                  (np.array([170,110,70],np.uint8), np.array([180,255,255],np.uint8))],
        "GREEN": [(np.array([35,80,60],np.uint8),   np.array([85,255,255],np.uint8))],
        "BLUE":  [(np.array([95,110,60],np.uint8),  np.array([130,255,255],np.uint8))],
        "BLACK": [(np.array([0,0,0],np.uint8),      np.array([180,255,45],np.uint8))],
    }
    CUBE_MIN = 300
    CUBE_MAX = 22000

    HOME   = [90, 90, 90, 90, 90, 90]  # J1 Base, J2 Shoulder, J3 Elbow, J4 Wrist Pitch, J5 Wrist Roll, J6 Gripper (Max 145°)
    G_OPEN = 145
    G_SHUT = 0

# =============================================================================
# SERIAL MANAGER  (mock-safe)
# =============================================================================
class SerialMgr:
    def __init__(self):
        self.conn   = None
        self.mock   = True
        self.angles = list(Cfg.HOME)
        self.lock   = threading.Lock()
        self.port   = "NONE"
        self._connect()

    def _auto_port(self):
        if not HAS_SERIAL: return None
        com_list = list(serial.tools.list_ports.comports())
        if not com_list: return None

        # 1. Primary match: Known Arduino / USB serial hardware
        for p in com_list:
            d = p.description.lower()
            if "bluetooth" in d or "bth" in d:
                continue
            if any(k in d for k in ("arduino", "ch340", "cp210", "ftdi", "usb serial", "usb-serial", "cdc", "uno", "mega", "nano")):
                return p.device

        # 2. Secondary match: Any genuine non-bluetooth COM port
        for p in com_list:
            d = p.description.lower()
            if "bluetooth" not in d and "bth" not in d:
                return p.device

        # 3. Do not auto-connect to Bluetooth link for Arduino Uno
        return None

    def _connect(self, port="AUTO"):
        if not HAS_SERIAL:
            log.warning("[SERIAL] pyserial not installed – mock mode.")
            return False, "pyserial not installed"
        target = port if port != "AUTO" else self._auto_port()
        if not target or target == "NONE":
            log.info("[SERIAL] No Arduino detected – running in simulation/mock mode.")
            self.mock = True
            self.port = "NONE"
            return False, "No Arduino detected (Mock Mode)"
        try:
            if self.conn:
                try: self.conn.close()
                except: pass
            self.conn = serial.Serial(target, Cfg.BAUD, timeout=1.0)
            time.sleep(2.0)  # Allow Arduino DTR reset / bootloader to initialize
            self.mock = False
            self.port = target
            log.info(f"[SERIAL] Connected successfully → {target}")
            add_log("SERIAL", f"Connected to {target}")
            return True, f"Connected to {target}"
        except Exception as e:
            err = str(e)
            log.warning(f"[SERIAL] Connection to {target} failed: {err} – mock mode active.")
            self.mock = True
            self.port = "NONE"
            add_log("SERIAL", f"Failed {target}: {err[:60]}")
            return False, err

    def _send_raw(self, pkt: str):
        if self.mock or not self.conn: return
        try:
            self.conn.write(pkt.encode())
        except Exception as e:
            log.error(f"[SERIAL] write error: {e}")

    def send(self, angles):
        with self.lock:
            # Auto-reconnect if Arduino was plugged in after startup
            if self.mock:
                target = self._auto_port()
                if target:
                    self._connect(target)

            # Limits: J1 Base (0-180), J2 Shoulder (0-180), J3 Elbow (0-180), J4 Wrist Pitch (0-180), J5 Wrist Roll (0-160), J6 Gripper (0-145)
            limits = [(0, 180), (0, 180), (0, 180), (0, 180), (0, 160), (0, 145)]
            if len(angles) == 5:
                # Graceful fallback: insert default 90 for wrist roll
                angles = [angles[0], angles[1], angles[2], angles[3], 90, angles[4]]
            a = [int(np.clip(v, *lim)) for v, lim in zip(angles[:6], limits)]
            if hasattr(self, "angles") and self.angles == a:
                return  # Prevent serial flooding if angles haven't changed
            self.angles = a
            self._send_raw(f"<{','.join(map(str, a))}>\n")

    def home(self):
        with self.lock:
            self.angles = list(Cfg.HOME)
            self._send_raw("HOME\n")

    def stop(self):
        with self.lock:
            self._send_raw("STOP\n")

    def relax(self):
        with self.lock:
            self._send_raw("RELAX\n")

    def unjam(self):
        with self.lock:
            self._send_raw("UNJAM\n")

    def hold(self):
        with self.lock:
            self._send_raw("HOLD\n")

    def test(self):
        with self.lock:
            self._send_raw("TEST\n")

    def gripper(self, open_=True):
        a = list(self.angles)
        if len(a) >= 6:
            a[5] = Cfg.G_OPEN if open_ else Cfg.G_SHUT
        elif len(a) == 5:
            a[4] = Cfg.G_OPEN if open_ else Cfg.G_SHUT
        self.send(a)

    def reconnect(self, port="AUTO"):
        with self.lock:
            if self.conn:
                try: self.conn.close()
                except: pass
            self.conn = None
            self.mock = True
            return self._connect(port)

    def close(self):
        if self.conn:
            try: self.conn.close()
            except: pass

# =============================================================================
# GLOBAL STATE & LOGGING
# =============================================================================
# Vision state (shared between camera thread & flask)
state = {
    "detection_on": True,    # toggle via terminal T key
    "gripper": None,          # dict or None
    "cubes": {c: None for c in Cfg.CUBE_HSV},
    "hand": None,
    "fps": 0.0,
    "logs": [],
    "voice_last": "",
    "voice_active": False,
}
state_lock = threading.Lock()

def add_log(src: str, msg: str):
    with state_lock:
        state["logs"].append({"t": time.strftime("%H:%M:%S"), "s": src, "m": msg})
        if len(state["logs"]) > 50:
            state["logs"].pop(0)

serial_mgr   = SerialMgr()
gemini_key   = os.environ.get("GEMINI_API_KEY", "")
gemini_ready = False

latest_jpeg: bytes = b""
jpeg_lock   = threading.Lock()

delivery_event = threading.Event()
voice_running  = threading.Event()

# ── terminal detection toggle ───────────────────────────────────────────────
def terminal_listener():
    """Press T <Enter> in terminal to toggle detection overlays."""
    log.info("[TERMINAL] Type  T  then Enter  to toggle AI detection on/off.")
    log.info("[TERMINAL] Type  Q  then Enter  to quit.")
    while True:
        try:
            cmd = input().strip().upper()
            if cmd == "T":
                with state_lock:
                    state["detection_on"] = not state["detection_on"]
                    status = "ON" if state["detection_on"] else "OFF"
                log.info(f"[TERMINAL] Detection overlay: {status}")
                add_log("SYSTEM", f"Detection overlay toggled {status}")
            elif cmd == "Q":
                log.info("[TERMINAL] Shutting down...")
                serial_mgr.close()
                os.kill(os.getpid(), signal.SIGTERM)
        except (EOFError, KeyboardInterrupt):
            break

threading.Thread(target=terminal_listener, daemon=True, name="TerminalListener").start()

# =============================================================================
# VISION HELPERS
# =============================================================================
def _detect_dots(hsv):
    mask = cv2.inRange(hsv, Cfg.W_LO, Cfg.W_HI)
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    pts = []
    for c in cnts:
        a = cv2.contourArea(c)
        if not (Cfg.DOT_AREA_MIN <= a <= Cfg.DOT_AREA_MAX): continue
        p = cv2.arcLength(c, True)
        if p == 0: continue
        if 4*math.pi*(a/(p*p)) < 0.3: continue
        m = cv2.moments(c)
        if m["m00"] == 0: continue
        pts.append((int(m["m10"]/m["m00"]), int(m["m01"]/m["m00"])))
    if len(pts) < 4: return None
    # pick tightest 4-cluster
    from itertools import combinations
    best, best_s = None, 1e9
    for q in combinations(pts, 4):
        arr = np.array(q); c = arr.mean(0)
        s = max(np.linalg.norm(arr - c, axis=1))
        if s < Cfg.DOT_CLUSTER and s < best_s:
            best, best_s = list(q), s
    if not best: return None
    arr = np.array(best, np.float32)
    cx, cy = int(arr[:,0].mean()), int(arr[:,1].mean())
    _, ev = cv2.PCACompute(arr, mean=None)
    ang = math.degrees(math.atan2(ev[0,1], ev[0,0]))
    return {"center": (cx, cy), "dots": best, "angle": ang}

def _detect_cubes(hsv):
    k = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
    out = {}
    for name, ranges in Cfg.CUBE_HSV.items():
        m = np.zeros(hsv.shape[:2], np.uint8)
        for lo, hi in ranges:
            m |= cv2.inRange(hsv, lo, hi)
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, k)
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, k, iterations=2)
        cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best, ba = None, 0
        for c in cnts:
            a = cv2.contourArea(c)
            if not (Cfg.CUBE_MIN <= a <= Cfg.CUBE_MAX): continue
            x,y,w,h = cv2.boundingRect(c)
            if not (0.4 <= w/h <= 2.5): continue
            if a > ba:
                ba = a; best = (x+w//2, y+h//2, w, h)
        out[name] = best
    return out

def _detect_hand(frame_bgr):
    if not HAS_MP: return None
    if not hasattr(_detect_hand, "det"):
        _detect_hand.det = mp.solutions.hands.Hands(
            max_num_hands=1, min_detection_confidence=0.55,
            min_tracking_confidence=0.45
        )
        _detect_hand.draw = mp.solutions.drawing_utils
        _detect_hand.connections = mp.solutions.hands.HAND_CONNECTIONS
    res = _detect_hand.det.process(cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB))
    if not res.multi_hand_landmarks: return None
    lm = res.multi_hand_landmarks[0]
    h, w = frame_bgr.shape[:2]
    hx = int((lm.landmark[9].x + lm.landmark[0].x)/2 * w)
    hy = int((lm.landmark[9].y + lm.landmark[0].y)/2 * h)
    return {"center": (hx, hy), "lm": lm, "res": res}

# =============================================================================
# CAMERA WORKER  (always-on, 2 FPS encode)
# =============================================================================
def camera_worker():
    global latest_jpeg
    cap = cv2.VideoCapture(Cfg.CAM_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  Cfg.CAM_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, Cfg.CAM_H)

    synthetic = not cap.isOpened()
    if synthetic:
        log.warning("[CAM] No webcam – using synthetic test pattern.")
        add_log("CAM", "No webcam detected – synthetic feed active.")
    else:
        log.info(f"[CAM] Opened webcam {Cfg.CAM_INDEX} @ 640×480")
        add_log("CAM", f"Webcam online @ 640×480, streaming at {Cfg.STREAM_FPS} FPS")

    # Timing
    interval   = 1.0 / Cfg.STREAM_FPS   # 0.5 s between encoded frames
    fc, t0     = 0, time.time()
    phase      = 0.0

    # MediaPipe draw util (lazy)
    mp_draw  = mp.solutions.drawing_utils      if HAS_MP else None
    mp_conns = mp.solutions.hands.HAND_CONNECTIONS if HAS_MP else None

    while True:
        loop_start = time.time()

        # ── grab frame ─────────────────────────────────────────────────────
        if synthetic:
            phase += 0.05
            frame = np.full((Cfg.CAM_H, Cfg.CAM_W, 3), (22, 26, 35), np.uint8)
            # grid
            for x in range(0, Cfg.CAM_W, 60):
                cv2.line(frame, (x,0), (x,Cfg.CAM_H), (35,42,55), 1)
            for y in range(0, Cfg.CAM_H, 60):
                cv2.line(frame, (0,y), (Cfg.CAM_W,y), (35,42,55), 1)
            # fake cubes
            cv2.rectangle(frame, (40,320),  (110,390), (0,0,220),  -1)   # red
            cv2.rectangle(frame, (200,340), (270,410), (0,210,0),  -1)   # green
            cv2.rectangle(frame, (360,310), (430,380), (220,130,0),-1)   # blue
            cv2.rectangle(frame, (510,330), (580,400), (30,30,30), -1)   # black
            # fake gripper dots
            gx = int(320 + 100*math.sin(phase*0.4))
            gy = int(180 + 50*math.cos(phase*0.4))
            for ox, oy in [(-14,-14),(14,-14),(14,14),(-14,14)]:
                cv2.circle(frame, (gx+ox, gy+oy), 7, (255,255,255), -1)
        else:
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.05); continue
            frame = cv2.resize(frame, (Cfg.CAM_W, Cfg.CAM_H))

        # ── CV detections ──────────────────────────────────────────────────
        det_on = state["detection_on"]

        if det_on:
            hsv     = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            gripper = _detect_dots(hsv)
            cubes   = _detect_cubes(hsv)
            hand    = _detect_hand(frame)
        else:
            gripper = cubes = hand = None
            cubes   = {c: None for c in Cfg.CUBE_HSV}

        with state_lock:
            state["gripper"] = gripper
            state["cubes"]   = cubes
            state["hand"]    = hand

        # ── draw overlays ──────────────────────────────────────────────────
        display = frame.copy()

        if det_on:
            C_BGR = {"RED":(0,0,255),"GREEN":(0,230,0),"BLUE":(255,130,0),"BLACK":(60,60,60)}
            active_cube = None  # (resolved in state; we just draw all for now)

            for name, cube in cubes.items():
                if cube is None: continue
                cx,cy,cw,ch = cube
                col = C_BGR[name]
                cv2.rectangle(display, (cx-cw//2, cy-ch//2),(cx+cw//2, cy+ch//2), col, 2)
                cv2.putText(display, name, (cx-cw//2, max(14,cy-ch//2-4)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 2)

            if gripper:
                gx,gy = gripper["center"]
                for d in gripper["dots"]:
                    cv2.circle(display, d, 5, (0,255,255), -1)
                cv2.drawMarker(display,(gx,gy),(0,255,255),cv2.MARKER_CROSS,20,2)

            if hand and mp_draw:
                mp_draw.draw_landmarks(display, hand["lm"], mp_conns)
                hx,hy = hand["center"]
                cv2.circle(display,(hx,hy),28,(0,255,0),2)

        # Corner tags
        tag = "DETECTION ON" if det_on else "DETECTION OFF"
        tcol = (0,255,0) if det_on else (0,0,255)
        cv2.putText(display, tag, (8,18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, tcol, 1, cv2.LINE_AA)
        cv2.putText(display, f"{state['fps']:.1f} FPS | {Cfg.CAM_W}x{Cfg.CAM_H}",
                    (8, Cfg.CAM_H-8), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180,180,180), 1)

        # ── encode JPEG @ 2 FPS rate ────────────────────────────────────────
        elapsed = time.time() - loop_start
        sleep_t = interval - elapsed
        if sleep_t > 0:
            time.sleep(sleep_t)

        ret2, buf = cv2.imencode(".jpg", display,
                                 [cv2.IMWRITE_JPEG_QUALITY, Cfg.JPEG_Q])
        if ret2:
            with jpeg_lock:
                latest_jpeg = buf.tobytes()

        # FPS counter (updates every 10 frames)
        fc += 1
        if fc >= 10:
            dt = time.time() - t0
            with state_lock:
                state["fps"] = round(fc/dt, 1)
            fc, t0 = 0, time.time()

cam_thread = threading.Thread(target=camera_worker, daemon=True, name="CamWorker")
cam_thread.start()

# =============================================================================
# MJPEG STREAM
# =============================================================================
def _gen_stream():
    while True:
        with jpeg_lock:
            frame = latest_jpeg
        if frame:
            yield (b"--f\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")
        time.sleep(1.0 / Cfg.STREAM_FPS)   # 10 FPS push


# =============================================================================
# FLASK ROUTES
# =============================================================================
@app.route("/")
def index():
    return render_template("index.html")

@app.route("/video")
def video():
    return Response(_gen_stream(),
                    mimetype="multipart/x-mixed-replace; boundary=f")

@app.route("/api/status")
def api_status():
    with state_lock:
        g = state["gripper"]
        cubes_out = {}
        for name, c in state["cubes"].items():
            if c:
                cubes_out[name] = {"ok": True, "x": c[0], "y": c[1]}
            else:
                cubes_out[name] = {"ok": False}
        data = {
            "fps":       state["fps"],
            "det_on":    state["detection_on"],
            "gripper":   {"ok": bool(g), "cx": g["center"][0] if g else 0,
                          "cy": g["center"][1] if g else 0,
                          "ang": round(g["angle"],1) if g else 0} if g else {"ok": False},
            "cubes":     cubes_out,
            "hand":      {"ok": bool(state["hand"]),
                          "x": state["hand"]["center"][0] if state["hand"] else 0,
                          "y": state["hand"]["center"][1] if state["hand"] else 0},
            "angles":    serial_mgr.angles,
            "serial":    {"ok": not serial_mgr.mock, "port": serial_mgr.port},
            "gemini":    {"ok": gemini_ready},
            "voice":     {"ok": state["voice_active"],
                          "last": state["voice_last"]},
            "logs":      state["logs"][-20:],
        }
    return jsonify(data)

@app.route("/api/angles", methods=["POST"])
def api_angles():
    d = request.json or {}
    a = d.get("angles", [])
    if len(a) == 6:
        serial_mgr.send(a)
        return jsonify({"ok": True, "angles": serial_mgr.angles})
    elif len(a) == 5:
        # Backward compatibility for 5-axis packets
        roll_angle = serial_mgr.angles[4] if len(serial_mgr.angles) >= 6 else 90
        serial_mgr.send([a[0], a[1], a[2], a[3], roll_angle, a[4]])
        return jsonify({"ok": True, "angles": serial_mgr.angles})
    return jsonify({"ok": False, "err": "need 6 angles"}), 400

@app.route("/api/home", methods=["POST"])
def api_home():
    serial_mgr.home()
    add_log("ARM", "Homed.")
    return jsonify({"ok": True, "angles": serial_mgr.angles})

@app.route("/api/gripper", methods=["POST"])
def api_gripper():
    d = request.json or {}
    open_ = bool(d.get("open", True))
    serial_mgr.gripper(open_)
    target = Cfg.G_OPEN if open_ else Cfg.G_SHUT
    add_log("ARM", f"Gripper {'opened' if open_ else 'closed'} ({target}°).")
    return jsonify({"ok": True, "open": open_, "angle": target, "angles": serial_mgr.angles})

@app.route("/api/deliver", methods=["POST"])
def api_deliver():
    delivery_event.set()
    add_log("USER", "Delivery triggered.")
    return jsonify({"ok": True})

@app.route("/api/speed", methods=["POST"])
def api_speed():
    d = request.json or {}
    mode = d.get("mode", "INSTANT")
    serial_mgr._send_raw(f"SPEED:{mode}\n")
    add_log("ARM", f"Speed profile updated: {mode}")
    return jsonify({"ok": True, "mode": mode})

@app.route("/api/mode360", methods=["POST"])
def api_mode360():
    d = request.json or {}
    enabled = bool(d.get("enabled", True))
    cmd = "MODE:360:ON\n" if enabled else "MODE:360:OFF\n"
    serial_mgr._send_raw(cmd)
    add_log("ARM", f"MG90S 360° Alignment: {'ACTIVE (1:1)' if enabled else 'OFF (Standard 180°)'}")
    return jsonify({"ok": True, "enabled": enabled})

@app.route("/api/unjam", methods=["POST"])
def api_unjam():
    serial_mgr.unjam()
    add_log("ARM", "EMERGENCY UNJAM: Wrist & Gripper detached.")
    return jsonify({"ok": True})

@app.route("/api/detection", methods=["POST"])
def api_detection():
    with state_lock:
        state["detection_on"] = not state["detection_on"]
        on = state["detection_on"]
    add_log("SYSTEM", f"Detection {'ON' if on else 'OFF'}")
    return jsonify({"ok": True, "det_on": on})

@app.route("/api/gemini", methods=["POST"])
def api_gemini():
    global gemini_key, gemini_ready
    d = request.json or {}
    key = d.get("key", "").strip()
    
    # Auto-sanitize common paste artifacts
    key = key.strip('"\'`')
    if "AIza" in key:
        key = key[key.find("AIza"):].strip().strip('"\'`')
    elif "=" in key:
        key = key.split("=")[-1].strip().strip('"\'`')

    if not key:
        return jsonify({"ok": False, "err": "No key provided."}), 400

    # 1. First try validating via official Google Gemini REST API (built-in urllib, zero dependencies)
    url = f"https://generativelanguage.googleapis.com/v1beta/models?key={key}"
    req = urllib.request.Request(url, headers={"User-Agent": "SAM-Robotic-Arm/1.0"})
    
    verified = False
    fail_reason = ""

    try:
        with urllib.request.urlopen(req, timeout=8) as resp:
            if resp.status == 200:
                verified = True
    except urllib.error.HTTPError as e:
        try:
            body = json.loads(e.read().decode())
            msg = body.get("error", {}).get("message", "")
            if "API key not valid" in msg or e.code == 400:
                fail_reason = "Invalid API key — please check the key from aistudio.google.com."
            elif e.code == 403:
                fail_reason = "API key lacks permission or Gemini API is not enabled in your project."
            elif e.code == 429:
                fail_reason = "Gemini API quota or rate limit exceeded on this key."
            else:
                fail_reason = f"Google API error ({e.code}): {msg[:100]}"
        except Exception:
            fail_reason = f"Google API returned error code {e.code}."
    except urllib.error.URLError:
        # If server has no internet or DNS issue, still save key so it works when connected
        verified = True
        add_log("GEMINI", "Key saved (network unreachable during check).")
    except Exception as e:
        fail_reason = str(e)[:120]

    # 2. If REST check succeeded or didn't fail with invalid key, check SDK if installed
    if not verified and not fail_reason:
        try:
            import google.generativeai as gai
            gai.configure(api_key=key)
            model = gai.GenerativeModel("gemini-1.5-flash")
            model.generate_content("hi", generation_config={"max_output_tokens": 1})
            verified = True
        except ImportError:
            pass
        except Exception as e:
            fail_reason = str(e)[:120]

    if verified:
        os.environ["GEMINI_API_KEY"] = key
        gemini_key   = key
        gemini_ready = True
        add_log("GEMINI", "API key verified & online ✓")
        return jsonify({"ok": True, "msg": "Gemini API verified."})
    else:
        add_log("GEMINI", f"Key rejected: {fail_reason}")
        return jsonify({"ok": False, "err": fail_reason or "Verification failed."}), 400

PRESET_POSES = {
    "HOME":  [90, 90, 90, 90, 90, 90],
    "SLEEP": [90, 142, 180, 90, 0, 0],
}

@app.route("/api/pose", methods=["POST"])
def api_pose():
    d = request.json or {}
    name = d.get("name", "HOME").upper()
    angles = PRESET_POSES.get(name)
    if not angles:
        return jsonify({"ok": False, "err": f"Unknown pose '{name}'"}), 400
    serial_mgr.send(angles)
    add_log("ARM", f"Pose applied: {name} -> {angles}")
    return jsonify({"ok": True, "name": name, "angles": angles})

@app.route("/api/gemini/ask", methods=["POST"])
def api_gemini_ask():
    global gemini_key, gemini_ready
    d = request.json or {}
    prompt = d.get("prompt", "").strip()
    if not prompt:
        return jsonify({"ok": False, "err": "Empty prompt"}), 400
    if not gemini_key:
        return jsonify({"ok": False, "err": "Gemini API key is not configured"}), 400

    current_angles = list(serial_mgr.angles) if hasattr(serial_mgr, "angles") and len(serial_mgr.angles) == 6 else [90, 90, 90, 90, 90, 90]

    system_instruction = (
        "You are the intelligent robotic kinematics controller for S.A.M. (6-Axis Robotic Arm).\n"
        "Your task is to interpret the user's natural language command and calculate the target 6 joint angles [J1, J2, J3, J4, J5, J6] to move the arm.\n\n"
        f"CURRENT ARM STATE [J1..J6]: {current_angles}\n"
        "JOINT MAPPING & PHYSICAL SAFE RANGES:\n"
        "- J1 (Base Pan): 0° (full right) to 180° (full left), 90° center\n"
        "- J2 (Shoulder Pitch): 0° (backwards) to 180° (forward down), 90° upright\n"
        "- J3 (Elbow Pitch): 0° (folded) to 180° (straight out), 90° right angle\n"
        "- J4 (Wrist Pitch): 0° (tilted up) to 180° (tilted down), 90° level\n"
        "- J5 (Wrist Roll): 0° to 160°, 90° neutral\n"
        "- J6 (Gripper Claw): 0° (closed shut) to 145° (full open), 90° mid open\n\n"
        "STANDARD PRESET POSES:\n"
        "- HOME: [90, 90, 90, 90, 90, 90]\n"
        "- SLEEP: [90, 142, 180, 90, 0, 0]\n\n"
        "FIXED JOINT PRESET ANGLES (When commanding individual joints, use these exact fixed values):\n"
        "- J1 (Base Pan): 0°, 90°, 180° (Lowest: 0°, Middle: 90°, Highest: 180°)\n"
        "- J2 (Shoulder): 45°, 90°, 135° (Lowest: 45°, Middle: 90°, Highest: 135°)\n"
        "- J3 (Elbow): 45°, 90°, 135° (Lowest: 45°, Middle: 90°, Highest: 135°)\n"
        "- J4 (Wrist Pitch): 30°, 90°, 150° (Lowest: 30°, Middle: 90°, Highest: 150°)\n"
        "- J5 (Wrist Roll): 30°, 90°, 160° (Lowest: 30°, Middle: 90°, Highest: 160°)\n"
        "- J6 (Gripper): ONLY Open (145°) or Close (0°)\n\n"
        "INSTRUCTIONS:\n"
        "Respond ONLY with a JSON object in this exact schema (no markdown formatting, no backticks):\n"
        "{\n"
        '  "action": "move" | "pose" | "gripper" | "status",\n'
        '  "angles": [J1, J2, J3, J4, J5, J6],\n'
        '  "description": "Short action summary (under 8 words)"\n'
        "}\n"
        "If the user asks to move in any direction (left, right, up, down, forward, back, grab, drop), calculate appropriate angles and set 'angles'.\n"
        "Always stay within physical limits [J1..J4: 0..180, J5: 0..160, J6: 0..145].\n"
        "Note: The AI must never talk or include spoken text. Description is only displayed visually on screen."
    )

    url = f"https://generativelanguage.googleapis.com/v1beta/models/gemini-1.5-flash:generateContent?key={gemini_key}"
    payload = {
        "contents": [{
            "parts": [{
                "text": system_instruction + "\nUser command: " + prompt
            }]
        }],
        "generationConfig": {
            "maxOutputTokens": 120,
            "temperature": 0.2
        }
    }
    req = urllib.request.Request(
        url,
        data=json.dumps(payload).encode("utf-8"),
        headers={"Content-Type": "application/json", "User-Agent": "SAM-Robotic-Arm/1.0"}
    )
    try:
        with urllib.request.urlopen(req, timeout=10) as resp:
            data = json.loads(resp.read().decode())
            candidates = data.get("candidates", [])
            if candidates:
                raw_text = candidates[0].get("content", {}).get("parts", [{}])[0].get("text", "").strip()
                clean_text = raw_text
                if clean_text.startswith("```"):
                    clean_text = re.sub(r"^```(?:json)?\s*", "", clean_text)
                    clean_text = re.sub(r"\s*```$", "", clean_text)
                clean_text = clean_text.strip()

                action_data = {}
                try:
                    action_data = json.loads(clean_text)
                except Exception:
                    m = re.search(r"\{[\s\S]*\}", clean_text)
                    if m:
                        try:
                            action_data = json.loads(m.group(0))
                        except Exception:
                            pass

                action = action_data.get("action", "move")
                desc = action_data.get("description", raw_text[:50])
                target_angles = action_data.get("angles")

                if isinstance(target_angles, list) and len(target_angles) == 6:
                    limits = [(0, 180), (0, 180), (0, 180), (0, 180), (0, 160), (0, 145)]
                    clamped = [int(np.clip(float(v), *lim)) for v, lim in zip(target_angles, limits)]
                    serial_mgr.send(clamped)
                    add_log("GEMINI", f"AI Move: {desc} -> {clamped}")
                    matched_pose = None
                    for pname, pangles in PRESET_POSES.items():
                        if all(abs(c - p) <= 2 for c, p in zip(clamped, pangles)):
                            matched_pose = pname
                            break
                    return jsonify({"ok": True, "action": action, "angles": clamped, "pose": matched_pose, "description": desc, "reply": desc})
                else:
                    return jsonify({"ok": True, "action": "status", "angles": current_angles, "description": desc or raw_text[:60], "reply": desc})
            return jsonify({"ok": False, "err": "No response generated."})
    except Exception as e:
        return jsonify({"ok": False, "err": str(e)[:120]})

@app.route("/api/serial", methods=["POST"])
def api_serial():
    d = request.json or {}
    port = d.get("port", "AUTO")
    ok, msg = serial_mgr.reconnect(port)
    return jsonify({
        "ok": not serial_mgr.mock,
        "port": serial_mgr.port,
        "mock": serial_mgr.mock,
        "msg": msg
    })

@app.route("/api/ports")
def api_ports():
    if not HAS_SERIAL:
        return jsonify({"ports": [], "current": "MOCK", "connected": False})
    
    # Auto-reconnect if Arduino was plugged in while in simulation/mock mode
    if serial_mgr.mock:
        detected = serial_mgr._auto_port()
        if detected:
            serial_mgr._connect(detected)

    ports = []
    for p in serial.tools.list_ports.comports():
        d_lower = p.description.lower()
        is_bt = "bluetooth" in d_lower or "bth" in d_lower
        is_ard = any(k in d_lower for k in ("arduino","ch340","cp210","ftdi","usb serial","usb-serial","cdc","uno","mega","nano"))
        ports.append({
            "d": p.device,
            "n": p.description,
            "is_arduino": is_ard,
            "is_bt": is_bt
        })
    return jsonify({
        "ports": ports,
        "current": serial_mgr.port,
        "connected": not serial_mgr.mock
    })

@app.route("/api/arm/start", methods=["POST"])
def api_arm_start():
    serial_mgr.home()
    add_log("ARM", "Arm started & homed.")
    return jsonify({"ok": True, "status": "RUNNING"})

@app.route("/api/arm/stop", methods=["POST"])
def api_arm_stop():
    serial_mgr.stop()
    add_log("ARM", "STOP command dispatched → Arm halted immediately.")
    return jsonify({"ok": True, "status": "STOPPED"})

@app.route("/api/arm/relax", methods=["POST"])
def api_arm_relax():
    serial_mgr.relax()
    add_log("ARM", "Servos relaxed (anti-jitter silent mode).")
    return jsonify({"ok": True, "status": "RELAXED"})

@app.route("/api/arm/hold", methods=["POST"])
def api_arm_hold():
    serial_mgr.hold()
    add_log("ARM", "Servos energized (holding torque active).")
    return jsonify({"ok": True, "status": "HOLDING"})

@app.route("/api/arm/test", methods=["POST"])
def api_arm_test():
    serial_mgr.test()
    add_log("DIAG", "Hardware diagnostic sweep dispatched (Pins 8 to 13).")
    return jsonify({"ok": True, "status": "TESTING"})

@app.route("/api/shutdown", methods=["POST"])
def api_shutdown():
    add_log("SYSTEM", "Program shutdown requested via Web UI.")
    def _kill():
        time.sleep(0.6)
        log.info("[SYSTEM] Shutting down S.A.M. Server...")
        os._exit(0)
    threading.Thread(target=_kill, daemon=True).start()
    return jsonify({"ok": True, "msg": "S.A.M. Server is shutting down."})


# =============================================================================
# VOICE THREAD  (background, optional)
# =============================================================================
def voice_worker():
    if not HAS_SPEECH:
        log.warning("[VOICE] speech_recognition not installed.")
        return
    rec = sr.Recognizer()
    rec.dynamic_energy_threshold = True
    with state_lock:
        state["voice_active"] = True
    add_log("VOICE", "Mic listener started – say 'give it to me'")
    try:
        with sr.Microphone() as src:
            rec.adjust_for_ambient_noise(src, duration=1)
            while True:
                try:
                    audio = rec.listen(src, timeout=4, phrase_time_limit=4)
                    text  = rec.recognize_google(audio).lower()
                    with state_lock:
                        state["voice_last"] = text
                    if "give it to me" in text:
                        delivery_event.set()
                        add_log("VOICE", "Trigger heard: 'give it to me'")
                except (sr.WaitTimeoutError, sr.UnknownValueError):
                    pass
                except Exception as e:
                    log.debug(f"[VOICE] {e}")
    except Exception as e:
        log.error(f"[VOICE] Mic error: {e}")
        with state_lock:
            state["voice_active"] = False

threading.Thread(target=voice_worker, daemon=True, name="VoiceWorker").start()

# =============================================================================
# MAIN
# =============================================================================
if __name__ == "__main__":
    print()
    print("=" * 55)
    print("  S.A.M.  Mission Control  -  Lightweight Edition")
    print("  http://localhost:5000")
    print()
    print("  TERMINAL COMMANDS (type then press Enter):")
    print("    T  -> Toggle AI detection overlays ON / OFF")
    print("    Q  -> Quit")
    print("=" * 55)
    print()
    app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)
