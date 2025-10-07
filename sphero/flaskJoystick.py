# app.py
import time, math, threading, json, os
from typing import Optional
from flask import Flask, request, redirect, url_for, render_template_string, jsonify
import pygame
from spherov2 import scanner
from spherov2.types import Color
from spherov2.sphero_edu import SpheroEduAPI
from spherov2.commands.power import Power  # we’ll call what’s available at runtime

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

        # batterijstatus
        self.battery_voltage_v: Optional[float] = None
        self.battery_percent: Optional[int] = None
        self._last_batt_poll = 0.0

        # debugging
        self._last_batt_error: Optional[str] = None

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

    # ---------- batterij helpers ----------
    def _normalize_voltage(self, v: float) -> float:
        """
        Converteer naar Volt indien lib andere schaal gebruikt.
        Heuristiek:
          - > 20   : waarschijnlijk millivolt -> /1000
          - 5..20  : waarschijnlijk centivolt -> /100
          - anders : al in Volt
        """
        try:
            fv = float(v)
        except Exception:
            return None
        if fv > 20.0:
            return round(fv / 1000.0, 2)
        if 5.0 < fv <= 20.0:
            return round(fv / 100.0, 2)
        return round(fv, 2)

    def _poll_battery(self, api):
        """
        Probeer meerdere paden zodat dit werkt met verschillende spherov2 versies / robots.
        """
        toy = getattr(api, "toy", None) or getattr(api, "_toy", None) or self.toy
        self._last_batt_error = None

        # 1) Probeer SpheroEduAPI convenience methods (als ze bestaan)
        try:
            if hasattr(api, "get_battery_percentage"):
                pct = api.get_battery_percentage()
                self.battery_percent = int(pct) if pct is not None else None
        except Exception as e:
            self.battery_percent = None
            self._last_batt_error = f"SpheroEduAPI.get_battery_percentage: {e}"

        try:
            if hasattr(api, "get_battery_voltage"):
                vv = api.get_battery_voltage()
                nv = self._normalize_voltage(vv)
                self.battery_voltage_v = nv
                return
        except Exception as e:
            self._last_batt_error = f"SpheroEduAPI.get_battery_voltage: {e}"

        # 2) Force refresh als ondersteund
        try:
            if toy and hasattr(Power, "force_battery_refresh"):
                Power.force_battery_refresh(toy)
        except Exception as e:
            # niet kritisch
            self._last_batt_error = f"force_battery_refresh: {e}"

        # 3) Power.* varianten
        # 3a) percentage
        if self.battery_percent is None:
            try:
                if toy and hasattr(Power, "get_battery_percentage"):
                    pct = Power.get_battery_percentage(toy)
                    self.battery_percent = int(pct) if pct is not None else None
            except Exception as e:
                self.battery_percent = None
                self._last_batt_error = f"Power.get_battery_percentage: {e}"

        # 3b) voltage in volt (sommige versies)
        try:
            if toy and hasattr(Power, "get_battery_voltage_in_volts"):
                vv = Power.get_battery_voltage_in_volts(toy)
                self.battery_voltage_v = self._normalize_voltage(vv)
                return
        except Exception as e:
            self._last_batt_error = f"Power.get_battery_voltage_in_volts: {e}"

        # 3c) generieke voltage (schaal onbekend)
        try:
            if toy and hasattr(Power, "get_battery_voltage"):
                vv = Power.get_battery_voltage(toy)
                self.battery_voltage_v = self._normalize_voltage(vv)
                return
        except Exception as e:
            self._last_batt_error = f"Power.get_battery_voltage: {e}"
            self.battery_voltage_v = None

    # ---------- hoofdloop ----------
    def _loop(self):
        api=self.connect_toy()
        if api is None: return
        self._api_ctx=api
        try:
            with api:
                self.display_number(api)
                while not self._stop_evt.is_set():
                    pygame.event.pump()
                    X=self.joystick.get_axis(0); Y=self.joystick.get_axis(1)
                    if self.joystick.get_button(buttons['1']): self.speed, self.color=(50,Color(255,200,0)); self.display_number(api)
                    if self.joystick.get_button(buttons['2']): self.speed, self.color=(70,Color(255,100,0)); self.display_number(api)
                    if self.joystick.get_button(buttons['3']): self.speed, self.color=(100,Color(255,50,0)); self.display_number(api)
                    if self.joystick.get_button(buttons['4']): self.speed, self.color=(200,Color(255,0,0)); self.display_number(api)
                    if Y<-0.7: self.move(api,self.base_heading,self.speed)
                    elif Y>0.7: self.move(api,self.base_heading+180,self.speed)
                    elif X>0.7: self.move(api,self.base_heading+22,0)
                    elif X<-0.7: self.move(api,self.base_heading-22,0)
                    else: api.set_speed(0)

                    try: self.base_heading=api.get_heading()
                    except Exception: pass

                    # batterijstatus 1x/s verversen
                    now = time.time()
                    if now - self._last_batt_poll >= 1.0:
                        self._last_batt_poll = now
                        try:
                            self._poll_battery(api)
                        except Exception as e:
                            self._last_batt_error = f"_poll_battery wrapper: {e}"
                            self.battery_voltage_v = None
                            self.battery_percent = None

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
<title>Sphero Controller</title>
<style>
body{font-family:sans-serif;margin:2rem;}form{display:grid;gap:.75rem;max-width:520px;}
.status{background:#f6f6f6;padding:.6rem;border-radius:8px;margin-bottom:1rem;white-space:pre-wrap;}
button{padding:.5rem .7rem;margin-right:.5rem;}
small{opacity:.7}
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
<small>Tip: als batterij “—” blijft, controleer Bluetooth-verbinding en library-versie.</small>
<script>
async function refresh(){
  try{
    let r=await fetch("{{ url_for('status') }}");
    let j=await r.json();
    const battV = (j.battery_voltage_v!==null && j.battery_voltage_v!==undefined) ? `${Number(j.battery_voltage_v).toFixed(2)} V` : '—';
    const battPct = (j.battery_percent!==null && j.battery_percent!==undefined) ? ` (${j.battery_percent}%)` : '';
    const dbg = j.last_batt_error ? `\\n(debug: ${j.last_batt_error})` : '';
    document.getElementById('status').textContent =
`running: ${j.running}
toy: ${j.toy_name||'—'}
speler: ${j.player_number||'—'}
batterij: ${battV}${battPct}${dbg}`;
  }catch(e){
    document.getElementById('status').textContent='Status niet beschikbaar';
  }
}
refresh();setInterval(refresh,1500);
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
        "battery_voltage_v": controller.battery_voltage_v if controller else None,
        "battery_percent": controller.battery_percent if controller else None,
        "last_batt_error": controller._last_batt_error if controller else None
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
