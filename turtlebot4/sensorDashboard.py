#!/usr/bin/env python3
"""Flask sensor dashboard for the TurtleBot 4.

Run this ON the Raspberry Pi of the TurtleBot 4 (ROS 2 sourced).
It starts a ROS 2 node on ROS_DOMAIN_ID=4, subscribes to the robot's
sensor topics and serves a web page that visualises them live:

    * Bumpers / cliff / wheel-drop   -> /hazard_detection
    * Proximity IR sensors           -> /ir_intensity   (TB4 has IR, not sonar)
    * LIDAR                          -> /scan           (drawn on a canvas)
    * OAK-D camera (depthai)         -> /oakd/rgb/image_raw/compressed (MJPEG)
    * OAK-D depth / 3D (depthai)     -> /oakd/stereo/image_raw (colorised depth MJPEG)
    * Battery / IMU / dock           -> /battery_state, /imu, /dock_status

Usage:
    source /opt/ros/humble/setup.bash
    python3 sensorDashboard.py
    # then browse to http://<pi-ip>:5000
"""

import os

# Force the correct ROS domain before rclpy reads the environment.
os.environ["ROS_DOMAIN_ID"] = "4"

import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from rclpy.action import ActionClient

from sensor_msgs.msg import LaserScan, BatteryState, Imu, CompressedImage, Image
from geometry_msgs.msg import Twist

# numpy + OpenCV are only needed to colourise the OAK-D depth image. If they
# are missing we simply skip the depth view instead of crashing the dashboard.
try:
    import numpy as np
    import cv2
    HAVE_CV = True
except ImportError:  # pragma: no cover
    HAVE_CV = False

# irobot_create_msgs is optional: if it is not installed we simply skip
# those subscriptions (and dock/undock) instead of crashing the dashboard.
try:
    from irobot_create_msgs.msg import (
        HazardDetectionVector,
        IrIntensityVector,
        DockStatus,
    )
    from irobot_create_msgs.action import Dock, Undock
    HAVE_CREATE_MSGS = True
except ImportError:  # pragma: no cover
    HAVE_CREATE_MSGS = False

from flask import Flask, Response, jsonify, render_template_string, request


# --------------------------------------------------------------------------
# ROS 2 node that collects the latest reading of every sensor.
# --------------------------------------------------------------------------
class SensorHub(Node):
    # Human-readable names for the hazard vector types.
    HAZARD_TYPES = {
        0: "BACKUP_LIMIT",
        1: "BUMP",
        2: "CLIFF",
        3: "WHEEL_DROP",
        4: "OBJECT_PROXIMITY",
    }

    # Teleop speeds and how long one button press keeps driving.
    LINEAR_SPEED = 0.15   # m/s
    ANGULAR_SPEED = 0.8   # rad/s
    DRIVE_DURATION = 0.6  # seconds per press

    def __init__(self):
        super().__init__("sensor_dashboard")
        self._lock = threading.Lock()

        # Latest values, protected by _lock.
        self.hazards = []           # list of {"type": str, "frame": str}
        self.ir = {}                # {sensor_frame: value}
        self.scan = None            # {"ranges": [...], "angle_min":..,"angle_increment":..,"range_max":..}
        self.battery = None         # {"percentage": float, "voltage": float}
        self.imu = None             # {"roll":..,"pitch":..,"yaw":..}
        self.docked = None          # bool
        self.jpeg = None            # latest RGB camera frame as JPEG bytes
        self.depth_jpeg = None      # latest colourised depth frame as JPEG bytes
        self.depth_center_m = None  # distance (m) at the centre of the frame

        # Teleop: cmd_vel publisher + a deadman timer so the robot stops
        # automatically shortly after each button press.
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._cmd_twist = Twist()
        self._drive_until = 0.0  # ROS time (seconds) the current command expires
        self.create_timer(0.1, self._drive_tick)

        sensor_qos = qos_profile_sensor_data

        self.create_subscription(LaserScan, "/scan", self._on_scan, sensor_qos)
        self.create_subscription(
            BatteryState, "/battery_state", self._on_battery, sensor_qos
        )
        self.create_subscription(Imu, "/imu", self._on_imu, sensor_qos)
        self.create_subscription(
            CompressedImage,
            "/oakd/rgb/image_raw/compressed",
            self._on_image,
            sensor_qos,
        )
        if HAVE_CV:
            self.create_subscription(
                Image, "/oakd/stereo/image_raw", self._on_depth, sensor_qos
            )
        else:
            self.get_logger().warn(
                "numpy/cv2 not found: OAK-D depth (3D) view disabled."
            )

        if HAVE_CREATE_MSGS:
            self.create_subscription(
                HazardDetectionVector, "/hazard_detection", self._on_hazard, sensor_qos
            )
            self.create_subscription(
                IrIntensityVector, "/ir_intensity", self._on_ir, sensor_qos
            )
            self.create_subscription(
                DockStatus, "/dock_status", self._on_dock, sensor_qos
            )
            self._dock_client = ActionClient(self, Dock, "/dock")
            self._undock_client = ActionClient(self, Undock, "/undock")
        else:
            self._dock_client = None
            self._undock_client = None
            self.get_logger().warn(
                "irobot_create_msgs not found: bumpers/IR/dock/undock disabled."
            )

    # ---- callbacks -------------------------------------------------------
    def _on_hazard(self, msg):
        with self._lock:
            self.hazards = [
                {
                    "type": self.HAZARD_TYPES.get(d.type, str(d.type)),
                    "frame": d.header.frame_id,
                }
                for d in msg.detections
            ]

    def _on_ir(self, msg):
        with self._lock:
            self.ir = {r.header.frame_id: int(r.value) for r in msg.readings}

    def _on_scan(self, msg):
        # Replace inf/nan with None so JSON stays valid.
        ranges = [
            (r if (r == r and r != float("inf")) else None) for r in msg.ranges
        ]
        with self._lock:
            self.scan = {
                "ranges": ranges,
                "angle_min": msg.angle_min,
                "angle_increment": msg.angle_increment,
                "range_max": msg.range_max,
            }

    def _on_battery(self, msg):
        with self._lock:
            self.battery = {
                "percentage": round(msg.percentage * 100.0, 1),
                "voltage": round(msg.voltage, 2),
            }

    def _on_imu(self, msg):
        q = msg.orientation
        # Quaternion -> roll/pitch/yaw (radians).
        sinr = 2.0 * (q.w * q.x + q.y * q.z)
        cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr, cosr)
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy)
        with self._lock:
            self.imu = {
                "roll": round(math.degrees(roll), 1),
                "pitch": round(math.degrees(pitch), 1),
                "yaw": round(math.degrees(yaw), 1),
            }

    def _on_dock(self, msg):
        with self._lock:
            self.docked = bool(msg.is_docked)

    def _on_image(self, msg):
        # CompressedImage.data is already JPEG for the standard transport.
        with self._lock:
            self.jpeg = bytes(msg.data)

    # Depth beyond this (metres) is clipped so the colour map keeps its range
    # useful for the couple of metres the OAK-D Lite actually resolves indoors.
    DEPTH_MAX_M = 4.0

    def _on_depth(self, msg):
        """Colourise the OAK-D stereo depth image (16UC1, millimetres)."""
        if not HAVE_CV:
            return
        h, w = msg.height, msg.width
        if h == 0 or w == 0:
            return
        # 16-bit, honour byte order; step may be padded so slice to width.
        dt = np.dtype(">u2") if msg.is_bigendian else np.dtype("<u2")
        try:
            depth = np.frombuffer(bytes(msg.data), dtype=dt)
            depth = depth.reshape(h, msg.step // 2)[:, :w].astype(np.float32)
        except ValueError:
            return
        depth_m = depth / 1000.0  # mm -> m
        invalid = depth == 0      # depthai uses 0 for "no reading"

        # Median distance over a small central patch -> robust centre reading.
        cy, cx = h // 2, w // 2
        patch = depth_m[max(0, cy - 5):cy + 5, max(0, cx - 5):cx + 5]
        valid_patch = patch[patch > 0]
        center_m = round(float(np.median(valid_patch)), 2) if valid_patch.size else None

        # Normalise 0..DEPTH_MAX_M -> 0..255 and apply a JET colour map.
        norm = np.clip(depth_m / self.DEPTH_MAX_M, 0.0, 1.0)
        # Invert so near = warm/red, far = cool/blue.
        img8 = ((1.0 - norm) * 255.0).astype(np.uint8)
        color = cv2.applyColorMap(img8, cv2.COLORMAP_JET)
        color[invalid] = (0, 0, 0)  # no reading -> black

        ok, jpg = cv2.imencode(".jpg", color)
        with self._lock:
            self.depth_center_m = center_m
            if ok:
                self.depth_jpeg = bytes(jpg)

    # ---- snapshots for the web layer ------------------------------------
    def snapshot(self):
        with self._lock:
            return {
                "hazards": list(self.hazards),
                "ir": dict(self.ir),
                "battery": self.battery,
                "imu": self.imu,
                "docked": self.docked,
                "have_create_msgs": HAVE_CREATE_MSGS,
                "have_camera": self.jpeg is not None,
                "have_depth": self.depth_jpeg is not None,
                "depth_center_m": self.depth_center_m,
            }

    def scan_snapshot(self):
        with self._lock:
            return self.scan

    def latest_jpeg(self):
        with self._lock:
            return self.jpeg

    def latest_depth_jpeg(self):
        with self._lock:
            return self.depth_jpeg

    # ---- teleop / actions ------------------------------------------------
    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _drive_tick(self):
        """Deadman: publish the commanded twist, or stop when it expires."""
        if self._now() < self._drive_until:
            self.cmd_pub.publish(self._cmd_twist)
        else:
            self.cmd_pub.publish(Twist())  # zero -> stop

    def drive(self, linear, angular):
        """Drive for DRIVE_DURATION seconds (re-pressing extends it)."""
        t = Twist()
        t.linear.x = float(linear)
        t.angular.z = float(angular)
        self._cmd_twist = t
        self._drive_until = self._now() + self.DRIVE_DURATION

    def command(self, action):
        """Handle a button press. Returns (ok, message)."""
        moves = {
            "forward": (self.LINEAR_SPEED, 0.0),
            "backward": (-self.LINEAR_SPEED, 0.0),
            "left": (0.0, self.ANGULAR_SPEED),
            "right": (0.0, -self.ANGULAR_SPEED),
            "stop": (0.0, 0.0),
        }
        if action in moves:
            self.drive(*moves[action])
            return True, action
        if action == "dock":
            if self._dock_client is None:
                return False, "dock action not available"
            self._dock_client.send_goal_async(Dock.Goal())
            return True, "dock"
        if action == "undock":
            if self._undock_client is None:
                return False, "undock action not available"
            self._undock_client.send_goal_async(Undock.Goal())
            return True, "undock"
        return False, f"unknown action: {action}"


# --------------------------------------------------------------------------
# ROS spinning in a background thread so Flask stays responsive.
# --------------------------------------------------------------------------
hub = None


def ros_thread():
    global hub
    rclpy.init()
    hub = SensorHub()
    rclpy.spin(hub)


# --------------------------------------------------------------------------
# Flask app
# --------------------------------------------------------------------------
app = Flask(__name__)

INDEX_HTML = """
<!doctype html><html><head><meta charset="utf-8">
<title>TurtleBot 4 sensors</title>
<style>
  body{font-family:sans-serif;margin:1.5rem;background:#0f1720;color:#e6edf3;}
  h1{margin:0 0 1rem;font-size:1.4rem;}
  .grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(280px,1fr));gap:1rem;}
  .card{background:#161f2b;border:1px solid #263445;border-radius:12px;padding:1rem;}
  .card h2{margin:0 0 .6rem;font-size:1rem;color:#7ee787;}
  .kv{display:flex;justify-content:space-between;padding:.15rem 0;border-bottom:1px solid #22303f;}
  .kv:last-child{border-bottom:none;}
  .pill{display:inline-block;padding:.15rem .5rem;border-radius:.6rem;font-size:.8rem;margin:.15rem;}
  .ok{background:#12351d;border:1px solid #2ea043;color:#7ee787;}
  .alert{background:#3d1418;border:1px solid #f85149;color:#ff9a9a;}
  .muted{color:#8b98a5;}
  canvas,img{width:100%;border-radius:8px;background:#000;}
  .bar{height:14px;border-radius:7px;background:#22303f;overflow:hidden;}
  .bar>span{display:block;height:100%;background:#2ea043;}
  .irrow{display:flex;align-items:center;gap:.5rem;margin:.2rem 0;font-size:.85rem;}
  .irrow .bar{flex:1;}
  .pad{display:grid;grid-template-columns:repeat(3,1fr);gap:.5rem;max-width:260px;}
  .pad button{padding:.7rem;font-size:1.1rem;border:1px solid #263445;border-radius:10px;
    background:#22303f;color:#e6edf3;cursor:pointer;}
  .pad button:hover{background:#2d3f52;}
  .pad button:active{background:#2ea043;}
  .dockrow{display:flex;gap:.5rem;margin-bottom:.6rem;}
  .dockrow button{flex:1;padding:.6rem;border:1px solid #263445;border-radius:10px;
    background:#1b2b3a;color:#e6edf3;cursor:pointer;}
  .dockrow button:hover{background:#243848;}
</style></head><body>
<h1>🐢 TurtleBot 4 &mdash; live sensors <span class="muted" id="conn"></span></h1>
<div class="grid">

  <div class="card">
    <h2>Besturing</h2>
    <div class="dockrow">
      <button onclick="send('dock')">⚓ Dock</button>
      <button onclick="send('undock')">⏏ Undock</button>
    </div>
    <div class="pad">
      <span></span>
      <button onclick="send('forward')">▲</button>
      <span></span>
      <button onclick="send('left')">◀</button>
      <button onclick="send('stop')">■</button>
      <button onclick="send('right')">▶</button>
      <span></span>
      <button onclick="send('backward')">▼</button>
      <span></span>
    </div>
    <div class="muted" id="cmdmsg" style="margin-top:.5rem;font-size:.8rem;"></div>
  </div>

  <div class="card">
    <h2>Bumpers / Hazards</h2>
    <div id="hazards"><span class="muted">wachten op data…</span></div>
  </div>

  <div class="card">
    <h2>Proximity IR</h2>
    <div id="ir"><span class="muted">wachten op data…</span></div>
  </div>

  <div class="card">
    <h2>Battery / IMU / Dock</h2>
    <div id="status"><span class="muted">wachten op data…</span></div>
  </div>

  <div class="card">
    <h2>LIDAR (/scan)</h2>
    <canvas id="lidar" width="320" height="320"></canvas>
  </div>

  <div class="card">
    <h2>OAK-D camera</h2>
    <img id="cam" src="{{ url_for('camera') }}" alt="camera stream"
         onerror="this.replaceWith(Object.assign(document.createElement('div'),{className:'muted',textContent:'geen camerabeeld'}))">
  </div>

  <div class="card">
    <h2>OAK-D diepte (3D)</h2>
    <img id="depth" src="{{ url_for('depth') }}" alt="depth stream"
         onerror="this.replaceWith(Object.assign(document.createElement('div'),{className:'muted',textContent:'geen dieptebeeld'}))">
    <div class="kv" style="margin-top:.5rem">
      <span>Afstand (midden)</span><span id="depthmid" class="muted">…</span>
    </div>
    <div class="muted" style="font-size:.75rem;margin-top:.3rem">
      rood = dichtbij · blauw = ver · zwart = geen meting
    </div>
  </div>

</div>

<script>
function pill(text, alert){return `<span class="pill ${alert?'alert':'ok'}">${text}</span>`;}

async function send(action){
  const el = document.getElementById('cmdmsg');
  try{
    const j = await (await fetch("/cmd/" + action, {method:"POST"})).json();
    el.textContent = (j.ok ? "→ " : "⚠ ") + j.msg;
  }catch(e){ el.textContent = "⚠ commando mislukt"; }
}

// Pijltjestoetsen als extra besturing.
document.addEventListener('keydown', (e) => {
  const map = {ArrowUp:'forward', ArrowDown:'backward', ArrowLeft:'left', ArrowRight:'right', ' ':'stop'};
  if(map[e.key]){ e.preventDefault(); send(map[e.key]); }
});

async function refresh(){
  try{
    const j = await (await fetch("{{ url_for('data') }}")).json();
    document.getElementById('conn').textContent = "• verbonden";

    // Hazards / bumpers
    const hz = document.getElementById('hazards');
    if(!j.have_create_msgs){
      hz.innerHTML = '<span class="muted">irobot_create_msgs niet geïnstalleerd</span>';
    } else if(j.hazards.length === 0){
      hz.innerHTML = pill("clear", false);
    } else {
      hz.innerHTML = j.hazards.map(h => pill(h.type + " (" + h.frame + ")", true)).join("");
    }

    // IR proximity
    const ir = document.getElementById('ir');
    const keys = Object.keys(j.ir).sort();
    if(keys.length === 0){
      ir.innerHTML = '<span class="muted">geen IR-data</span>';
    } else {
      ir.innerHTML = keys.map(k => {
        const v = j.ir[k];
        const pct = Math.min(100, v / 40);   // IR intensity ~0..4000
        const label = k.replace('ir_intensity_','');
        return `<div class="irrow"><span style="width:70px">${label}</span>
                <div class="bar"><span style="width:${pct}%"></span></div>
                <span style="width:48px;text-align:right">${v}</span></div>`;
      }).join("");
    }

    // Battery / IMU / dock
    const st = document.getElementById('status');
    let html = "";
    if(j.battery){
      html += `<div class="kv"><span>Batterij</span><span>${j.battery.percentage}% (${j.battery.voltage} V)</span></div>`;
      html += `<div class="bar" style="margin:.3rem 0"><span style="width:${j.battery.percentage}%"></span></div>`;
    }
    if(j.imu){
      html += `<div class="kv"><span>Roll / Pitch / Yaw</span><span>${j.imu.roll}° / ${j.imu.pitch}° / ${j.imu.yaw}°</span></div>`;
    }
    if(j.docked !== null){
      html += `<div class="kv"><span>Dock</span><span>${j.docked ? "gedockt" : "los"}</span></div>`;
    }
    st.innerHTML = html || '<span class="muted">geen data</span>';

    // OAK-D depth centre distance
    const dm = document.getElementById('depthmid');
    if(dm){
      if(!j.have_depth){
        dm.textContent = "geen dieptedata";
      } else if(j.depth_center_m === null || j.depth_center_m === undefined){
        dm.textContent = "—";
      } else {
        dm.textContent = j.depth_center_m.toFixed(2) + " m";
      }
    }
  }catch(e){
    document.getElementById('conn').textContent = "• geen verbinding";
  }
}

// LIDAR polar plot
async function drawLidar(){
  try{
    const s = await (await fetch("{{ url_for('lidar') }}")).json();
    const c = document.getElementById('lidar'), ctx = c.getContext('2d');
    const W = c.width, H = c.height, cx = W/2, cy = H/2;
    ctx.clearRect(0,0,W,H);
    // grid
    ctx.strokeStyle = "#22303f"; ctx.fillStyle = "#7ee787";
    for(let r=1;r<=3;r++){ ctx.beginPath(); ctx.arc(cx,cy,(r/3)*(W/2-6),0,2*Math.PI); ctx.stroke(); }
    if(!s || !s.ranges){ return; }
    const scale = (W/2 - 6) / (s.range_max || 3.0);
    for(let i=0;i<s.ranges.length;i++){
      const r = s.ranges[i];
      if(r === null) continue;
      const a = s.angle_min + i * s.angle_increment;
      // robot x forward -> up on screen
      const x = cx + Math.sin(a) * r * scale;
      const y = cy - Math.cos(a) * r * scale;
      ctx.fillRect(x-1, y-1, 2, 2);
    }
    // center (robot)
    ctx.fillStyle = "#f0883e"; ctx.beginPath(); ctx.arc(cx,cy,3,0,2*Math.PI); ctx.fill();
  }catch(e){}
}

refresh(); setInterval(refresh, 500);
drawLidar(); setInterval(drawLidar, 300);
</script>
</body></html>
"""


@app.route("/")
def index():
    return render_template_string(INDEX_HTML)


@app.route("/data")
def data():
    if hub is None:
        return jsonify({"have_create_msgs": False, "hazards": [], "ir": {}}), 503
    return jsonify(hub.snapshot())


@app.route("/lidar.json")
def lidar():
    if hub is None:
        return jsonify(None), 503
    return jsonify(hub.scan_snapshot())


@app.route("/cmd/<action>", methods=["POST"])
def cmd(action):
    if hub is None:
        return jsonify({"ok": False, "msg": "ROS not ready"}), 503
    ok, msg = hub.command(action)
    return jsonify({"ok": ok, "msg": msg}), (200 if ok else 400)


def mjpeg_generator(getter):
    """Yield frames from `getter` (a callable returning JPEG bytes) as MJPEG."""
    import time

    boundary = b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
    while True:
        frame = getter() if hub else None
        if frame:
            yield boundary + frame + b"\r\n"
        time.sleep(0.05)  # ~20 fps cap


@app.route("/camera")
def camera():
    return Response(
        mjpeg_generator(lambda: hub.latest_jpeg() if hub else None),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


@app.route("/depth")
def depth():
    return Response(
        mjpeg_generator(lambda: hub.latest_depth_jpeg() if hub else None),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


if __name__ == "__main__":
    threading.Thread(target=ros_thread, daemon=True).start()
    # threaded=True so the MJPEG stream doesn't block the JSON endpoints.
    app.run(host="0.0.0.0", port=5000, threaded=True, debug=False)
