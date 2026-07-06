#!/usr/bin/env python3
"""Flask sensor dashboard for the TurtleBot 4.

Run this ON the Raspberry Pi of the TurtleBot 4 (ROS 2 sourced).
It starts a ROS 2 node on ROS_DOMAIN_ID=4, subscribes to the robot's
sensor topics and serves a web page that visualises them live:

    * Bumpers / cliff / wheel-drop   -> /hazard_detection
    * Proximity IR sensors           -> /ir_intensity   (TB4 has IR, not sonar)
    * LIDAR                          -> /scan           (drawn on a canvas)
    * OAK-D camera (depthai)         -> /oakd/rgb/image_raw/compressed (MJPEG)
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

from sensor_msgs.msg import LaserScan, BatteryState, Imu, CompressedImage

# irobot_create_msgs is optional: if it is not installed we simply skip
# those subscriptions instead of crashing the whole dashboard.
try:
    from irobot_create_msgs.msg import (
        HazardDetectionVector,
        IrIntensityVector,
        DockStatus,
    )
    HAVE_CREATE_MSGS = True
except ImportError:  # pragma: no cover
    HAVE_CREATE_MSGS = False

from flask import Flask, Response, jsonify, render_template_string


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
        self.jpeg = None            # latest camera frame as JPEG bytes

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
        else:
            self.get_logger().warn(
                "irobot_create_msgs not found: bumpers/IR/dock disabled."
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
            }

    def scan_snapshot(self):
        with self._lock:
            return self.scan

    def latest_jpeg(self):
        with self._lock:
            return self.jpeg


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
</style></head><body>
<h1>🐢 TurtleBot 4 &mdash; live sensors <span class="muted" id="conn"></span></h1>
<div class="grid">

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

</div>

<script>
function pill(text, alert){return `<span class="pill ${alert?'alert':'ok'}">${text}</span>`;}

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


def mjpeg_generator():
    """Yield the latest camera frame as an MJPEG stream."""
    import time

    boundary = b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
    while True:
        frame = hub.latest_jpeg() if hub else None
        if frame:
            yield boundary + frame + b"\r\n"
        time.sleep(0.05)  # ~20 fps cap


@app.route("/camera")
def camera():
    return Response(
        mjpeg_generator(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


if __name__ == "__main__":
    threading.Thread(target=ros_thread, daemon=True).start()
    # threaded=True so the MJPEG stream doesn't block the JSON endpoints.
    app.run(host="0.0.0.0", port=5000, threaded=True, debug=False)
