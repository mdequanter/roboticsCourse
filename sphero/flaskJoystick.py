# app.py
import time, math, threading, json, os
from typing import Optional
from flask import Flask, request, redirect, url_for, render_template_string, jsonify
import pygame
from spherov2 import scanner
from spherov2.types import Color
from spherov2.sphero_edu import SpheroEduAPI
from spherov2.commands.power import Power

SETTINGS_FILE = "last_settings.json"

def load_settings():
    if os.path.exists(SETTINGS_FILE):
        with open(SETTINGS_FILE, "r") as f:
            return json.load(f)
    return {"toy_name": "", "joystick_id": 0, "player_number": 1}

def save_settings(data):
    with open(SETTINGS_FILE, "w") as f:
        json.dump(data, f)

buttons = {'1':0,'2':1,'3':2,'4':3,'L1':4,'L2':6,'R1':5,'R2':7,'SELECT':8,'START':9}

class SpheroController:
    def __init__(self, joystick, color: Color, ball_number: int):
        self.toy=None; self.speed=50; self.heading=0; self.base_heading=0
        self.calibration_mode=False; self.joystick=joystick
        self.color=color; self.number=int(ball_number)
        self.gameOn=False; self.hillCounter=0
        self._stop_evt=threading.Event(); self._thread=None; self._api_ctx=None

        # --- Battery state ---
        self._last_batt_check = 0.0
        self.battery_voltage: Optional[float] = None
        self.battery_state: str = "unknown"  # green/yellow/orange/red/critical/unknown

    def discover_toy(self,toy_name:str)->bool:
        try:
            self.toy=scanner.find_toy(toy_name=toy_name)
            print(f"Sphero '{toy_name}' gevonden."); return True
        except Exception as e:
            print(f"Error discovering toy: {e}"); return False

    def connect_toy(self):
        if self.toy:
            try: return SpheroEduAPI(self.toy)
            except Exception as e: print(f"Error connecting: {e}")
        return None

    def move(self,api,heading,speed):
        api.set_heading(heading%360); api.set_speed(speed)

    def display_number(self,api):
        try: api.set_matrix_character(str(self.number),self.color)
        except Exception: api.set_main_led(self.color)

    # ---------- Battery logic (from second script, adapted for Flask/threading) ----------
    def _update_battery_led(self, api, voltage: float):
        """
        Zet front LED volgens de spanning en update state string.
        """
        # Drempels zoals in je voorbeeldscript
        # >4.1V = groen; 3.9-4.1 = geel; <3.9 = oranje; <3.7 = rood; <3.5 = critical stop
        try:
            if voltage > 4.1:
                api.set_front_led(Color(0, 255, 0)); self.battery_state = "green"
            elif 3.9 < voltage <= 4.1:
                api.set_front_led(Color(255, 255, 0)); self.battery_state = "yellow"
            elif 3.7 < voltage <= 3.9:
                api.set_front_led(Color(255, 100, 0)); self.battery_state = "orange"
            elif 3.5 < voltage <= 3.7:
                api.set_front_led(Color(255, 0, 0)); self.battery_state = "red"
            else:
                # <= 3.5V -> critical
                api.set_front_led(Color(255, 0, 0)); self.battery_state = "critical"
        except Exception as e:
            print(f"LED update error: {e}")

    def _check_battery(self, api):
        """
        Vraagt batterijspanning op en past LED/drempels toe. Roept stop aan bij critical.
        """
        try:
            voltage = Power.get_battery_voltage(self.toy)
            self.battery_voltage = float(voltage) if voltage is not None else None
            if self.battery_voltage is not None:
                print(f"Battery {self.number}: {self.battery_voltage:.2f} V")
                self._update_battery_led(api, self.battery_voltage)
                # Veilig stoppen bij kritieke spanning
                if self.battery_voltage <= 3.5:
                    print("Batterij kritiek (<3.5V). Controller wordt gestopt.")
                    self.stop()
            else:
                self.battery_state = "unknown"
        except Exception as e:
            print(f"Battery read error: {e}")

    def _loop(self):
        api=self.connect_toy()
        if api is None: return
        self._api_ctx=api
        try:
            with api:
                # Toon speler-nummer op matrix
                self.display_number(api)
                # Initiele battery check meteen bij start
                self._check_battery(api)
                self._last_batt_check = time.time()

                while not self._stop_evt.is_set():
                    pygame.event.pump()
                    X=self.joystick.get_axis(0); Y=self.joystick.get_axis(1)

                    # Snelheid presets + nummerkleur opnieuw tonen
                    if self.joystick.get_button(buttons['1']):
                        self.speed, self.color=(100,Color(255,200,0)); self.display_number(api)
                    if self.joystick.get_button(buttons['2']):
                        self.speed, self.color=(150,Color(255,100,0)); self.display_number(api)
                    if self.joystick.get_button(buttons['3']):
                        self.speed, self.color=(200,Color(255,50,0)); self.display_number(api)
                    if self.joystick.get_button(buttons['4']):
                        self.speed, self.color=(240,Color(255,0,0)); self.display_number(api)

                    # Besturing
                    if Y<-0.7: self.move(api,self.base_heading,self.speed)
                    elif Y>0.7: self.move(api,self.base_heading+180,self.speed)
                    elif X>0.7: self.move(api,self.base_heading+22,speed)
                    elif X<-0.7: self.move(api,self.base_heading-22,speed)
                    else: api.set_speed(0)

                    # Heading bijhouden
                    try: self.base_heading=api.get_heading()
                    except Exception: pass

                    # Elke 30s batterij-status updaten
                    now = time.time()
                    if now - self._last_batt_check >= 30.0:
                        self._check_battery(api)
                        self._last_batt_check = now

                    time.sleep(0.01)
        finally:
            try:
                if self._api_ctx: self._api_ctx.set_speed(0)
            except Exception: pass
            self._api_ctx=None

    def start(self):
        if self._thread and self._thread.is_alive(): return
        self._stop_evt.clear()
        self._thread=threading.Thread(target=self._loop,daemon=True)
        self._thread.start()

    def stop(self):
        self._stop_evt.set()
        try:
            if self._api_ctx: self._api_ctx.set_speed(0)
        except Exception: pass
        if self._thread and self._thread.is_alive(): self._thread.join(timeout=3)

    @property
    def running(self): return bool(self._thread and self._thread.is_alive())

# ---------- Flask -------------
app=Flask(__name__)
controller:Optional[SpheroController]=None
joystick_obj=None

def init_pygame_and_joystick(jid:int):
    global joystick_obj
    pygame.init(); pygame.joystick.init()
    if pygame.joystick.get_count()==0: raise RuntimeError("Geen joystick gevonden.")
    joystick_obj=pygame.joystick.Joystick(jid); joystick_obj.init(); return joystick_obj

INDEX_HTML="""
<!doctype html><html><head><meta charset="utf-8">
<title>Sphero {{toy_name}} - {{player_number}}</title>
<style>
body{font-family:sans-serif;margin:2rem;}form{display:grid;gap:.75rem;max-width:520px;}
.status{background:#f6f6f6;padding:.6rem;border-radius:8px;margin-bottom:1rem;line-height:1.4}
.status .batt{font-weight:bold}
.badge{display:inline-block;padding:.1rem .4rem;border-radius:.5rem;font-size:.85rem}
.badge.green{background:#e7f8e7;border:1px solid #6ac46a}
.badge.yellow{background:#fff9da;border:1px solid #e2c000}
.badge.orange{background:#ffe9d9;border:1px solid #ff8a3d}
.badge.red{background:#ffd9d9;border:1px solid #ff4d4f}
.badge.critical{background:#ffd9d9;border:1px solid #ff0000}
.badge.unknown{background:#eee;border:1px solid #bbb}
button{padding:.5rem .7rem;margin-right:.5rem;}
</style></head><body>
<h1>Sphero Controller</h1>
<div class="status" id="status">Status laden…</div>
<form method="post" action="{{ url_for('start') }}">
  <label>Toy name <input name="toy_name" value="{{toy_name}}" required></label>
  <label>Joystick ID 
    <select name="joystick_id">
      <option value="0" {% if joystick_id==0 %}selected{% endif %}>0</option>
      <option value="1" {% if joystick_id==1 %}selected{% endif %}>1</option>
    </select>
  </label>
  <label>Speler #
    <select name="player_number">
      {% for n in range(1,6) %}
      <option value="{{n}}" {% if player_number==n %}selected{% endif %}>{{n}}</option>
      {% endfor %}
    </select>
  </label>
  <div>
    <button type="submit">Start</button>
    <button formaction="{{ url_for('stop') }}" formmethod="post">Stop</button>
  </div>
</form>
<script>
async function refresh(){
  try{
    let r=await fetch("{{ url_for('status') }}");
    let j=await r.json();
    const batt = (j.battery_voltage!=null) ? j.battery_voltage.toFixed(2)+" V" : "—";
    const badge = `<span class="badge ${j.battery_state||'unknown'}">${j.battery_state||'unknown'}</span>`;
    document.getElementById('status').innerHTML =
      `running: ${j.running} <br>
       toy: ${j.toy_name||'—'} <br>
       speler: ${j.player_number||'—'} <br>
       <span class="batt">batterij:</span> ${batt} ${badge}`;
  }catch(e){
    document.getElementById('status').textContent='Status niet beschikbaar';
  }}
refresh();setInterval(refresh,2000);
</script></body></html>
"""

@app.route("/",methods=["GET"])
def index():
    s=load_settings()
    return render_template_string(INDEX_HTML,**s)

@app.route("/status")
def status():
    global controller
    return jsonify({
        "running":bool(controller and controller.running),
        "toy_name":getattr(controller.toy,"name",None) if controller else None,
        "player_number":controller.number if controller else None,
        # batterij info naar de UI
        "battery_voltage":controller.battery_voltage if controller else None,
        "battery_state":controller.battery_state if controller else "unknown",
    })

@app.route("/start",methods=["POST"])
def start():
    global controller,joystick_obj
    toy_name=request.form["toy_name"].strip()
    jid=int(request.form["joystick_id"]); pn=int(request.form["player_number"])
    save_settings({"toy_name":toy_name,"joystick_id":jid,"player_number":pn})
    try:
        if joystick_obj is None: init_pygame_and_joystick(jid)
    except Exception as e: return f"Joystick fout: {e}",400
    if controller and controller.running: controller.stop()
    controller=SpheroController(joystick_obj,Color(255,0,0),pn)
    if not controller.discover_toy(toy_name): return "Sphero niet gevonden.",404
    controller.start(); return redirect(url_for('index'))

@app.route("/stop",methods=["POST"])
def stop():
    global controller
    if controller: controller.stop()
    return redirect(url_for('index'))

if __name__=="__main__":
    try: app.run(host="0.0.0.0",port=5000,debug=True)
    finally:
        if controller: controller.stop()
        pygame.quit()
