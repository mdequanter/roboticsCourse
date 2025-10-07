# app.py
import time
import math
import threading
from typing import Optional

from flask import Flask, request, redirect, url_for, render_template_string, jsonify

import pygame
from spherov2 import scanner
from spherov2.types import Color
from spherov2.sphero_edu import SpheroEduAPI
from spherov2.commands.power import Power

# -----------------------------
# Sphero / Joystick configuratie
# -----------------------------

buttons = {
    '1': 0, '2': 1, '3': 2, '4': 3,
    'L1': 4, 'L2': 6, 'R1': 5, 'R2': 7,
    'SELECT': 8, 'START': 9
}

class SpheroController:
    def __init__(self, joystick, color: Color, ball_number: int):
        self.toy = None
        self.speed = 50
        self.heading = 0
        self.base_heading = 0
        self.calibration_mode = False
        self.joystick = joystick
        self.last_command_time = time.time()
        self.heading_reset_interval = 1
        self.last_heading_reset_time = time.time()
        self.threshold_accel_mag = 0.05
        self.collision_occurred = False
        self.color = color
        self.previous_button = 1
        self.number = int(ball_number)
        self.gameStartTime = time.time()
        self.gameOn = False
        self.boosterCounter = 0
        self.calibrated = False
        self.hillCounter = 0

        # Thread/stop-state
        self._stop_evt = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._api_ctx = None

    # ---------- Discovery / Connect ----------
    def discover_nearest_toy(self) -> Optional[str]:
        try:
            toys = scanner.find_toys()
            if not toys:
                print("Geen Sphero's gevonden.")
                return None
            self.toy = toys[0]
            print(f"Dichtstbijzijnde Sphero toy '{self.toy.name}' ontdekt.")
            return self.toy.name
        except Exception as e:
            print(f"Error no toys nearby: {e}")
            return None

    def discover_toy(self, toy_name: str) -> bool:
        try:
            self.toy = scanner.find_toy(toy_name=toy_name)
            print(f"Sphero toy '{toy_name}' discovered.")
            return True
        except Exception as e:
            print(f"Error discovering toy: {e}")
            return False

    def connect_toy(self):
        if self.toy is not None:
            try:
                return SpheroEduAPI(self.toy)
            except Exception as e:
                print(f"Error connecting to toy: {e}")
        else:
            print("No toy discovered. Please run discover_toy() first.")
        return None

    # ---------- Besturing ----------
    def move(self, api, heading, speed):
        api.set_heading(heading % 360)
        api.set_speed(speed)

    def toggle_calibration_mode(self, api, Y):
        if not self.calibration_mode:
            self.enter_calibration_mode(api, Y)
        else:
            self.exit_calibration_mode(api)

    def enter_calibration_mode(self, api, X):
        api.set_speed(0)
        self.gameStartTime = time.time()
        self.calibration_mode = True
        self.gameOn = False
        api.set_front_led(Color(255, 0, 0))

        self.base_heading = api.get_heading()
        if X < -0.7:
            new_heading = self.base_heading - 5
        elif X > 0.7:
            new_heading = self.base_heading + 5
        else:
            new_heading = self.base_heading
        api.set_heading(new_heading)

    def exit_calibration_mode(self, api):
        self.calibrated = True
        self.calibration_mode = False
        self.gameOn = True
        self.boosterCounter = 0
        self.gameStartTime = time.time()
        api.set_front_led(Color(0, 255, 0))

    LED_PATTERNS = {1: '1', 2: '2', 3: '3', 4: '4', 5: '5'}

    def set_number(self, number: int):
        self.number = int(number)

    def display_number(self, api):
        number_char = self.LED_PATTERNS.get(self.number)
        if number_char:
            try:
                api.set_matrix_character(number_char, self.color)
            except Exception as e:
                # fallback voor modellen zonder matrix
                print(f"Matrix niet beschikbaar ({e}); fallback naar main LED.")
                api.set_main_led(self.color)
        else:
            print(f"Error in matrix '{self.number}'")

    def print_battery_level(self, api):
        try:
            battery_voltage = Power.get_battery_voltage(self.toy)
            print(f"Battery status of {self.number}: {battery_voltage} V ")
            if (battery_voltage > 4.1):
                api.set_front_led(Color(r=0, g=255, b=0))
            if 3.9 < battery_voltage <= 4.1:
                api.set_front_led(Color(r=255, g=255, b=0))
            if battery_voltage <= 3.9:
                api.set_front_led(Color(r=255, g=100, b=0))
            if battery_voltage < 3.7:
                api.set_front_led(Color(r=255, g=0, b=0))
            if battery_voltage < 3.5:
                print("Battery te laag — stop.")
                self._stop_evt.set()
        except Exception as e:
            print(f"Kon batterijstatus niet lezen: {e}")

    # ---------- Loop / Thread ----------
    def _loop(self):
        api = self.connect_toy()
        if api is None:
            return

        self._api_ctx = api
        try:
            with api:
                last_battery_print_time = time.time()
                self.set_number(self.number)
                self.display_number(api)
                self.enter_calibration_mode(api, 0)
                self.exit_calibration_mode(api)

                while not self._stop_evt.is_set():
                    pygame.event.pump()

                    if not self.gameOn:
                        self.gameStartTime = time.time()

                    now = time.time()
                    if now - last_battery_print_time >= 30:
                        self.print_battery_level(api)
                        last_battery_print_time = now

                    if self.gameOn:
                        try:
                            acceleration_data = api.get_acceleration()
                        except Exception:
                            acceleration_data = None

                        if acceleration_data is not None:
                            x_acc = acceleration_data.get('x', 0.0)
                            z_acc = acceleration_data.get('z', 0.0)
                            angle = math.degrees(math.atan2(x_acc, z_acc))
                            if abs(angle) >= 30:
                                self.hillCounter += 1
                                if self.hillCounter > 10:
                                    seconds = (now - self.gameStartTime)
                                    print(f"Player {self.number} going wild ({seconds:.1f}s)")
                            else:
                                self.hillCounter = 0

                    # Joystick axes
                    X = self.joystick.get_axis(0)
                    Y = self.joystick.get_axis(1)

                    # Snelheidspresets + kleur
                    if self.joystick.get_button(buttons['1']) == 1:
                        self.speed = 50
                        self.color = Color(r=255, g=200, b=0)
                        self.display_number(api)
                    if self.joystick.get_button(buttons['2']) == 1:
                        self.speed = 70
                        self.color = Color(r=255, g=100, b=0)
                        self.display_number(api)
                    if self.joystick.get_button(buttons['3']) == 1:
                        self.speed = 100
                        self.color = Color(r=255, g=50, b=0)
                        self.display_number(api)
                    if self.joystick.get_button(buttons['4']) == 1:
                        self.speed = 200
                        self.color = Color(r=255, g=0, b=0)
                        self.display_number(api)

                    # Besturing
                    if Y < -0.7:
                        self.move(api, self.base_heading, self.speed)
                    elif Y > 0.7:
                        self.move(api, self.base_heading + 180, self.speed)
                    elif X > 0.7:
                        self.move(api, self.base_heading + 22, 0)
                    elif X < -0.7:
                        self.move(api, self.base_heading - 22, 0)
                    else:
                        api.set_speed(0)

                    try:
                        self.base_heading = api.get_heading()
                    except Exception:
                        pass

                    time.sleep(0.01)
        finally:
            # altijd stilleggen
            try:
                if self._api_ctx:
                    try:
                        self._api_ctx.set_speed(0)
                    except Exception:
                        pass
            finally:
                self._api_ctx = None

    def start(self):
        if self._thread and self._thread.is_alive():
            return
        self._stop_evt.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self, join_timeout: float = 3.0):
        # signaleer stop
        self._stop_evt.set()
        # noodstop + netjes sluiten
        try:
            if self._api_ctx:
                try:
                    self._api_ctx.set_speed(0)
                except Exception:
                    pass
        except Exception:
            pass
        # wacht kort op thread-einde
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=join_timeout)

    @property
    def running(self) -> bool:
        return bool(self._thread and self._thread.is_alive())

# -----------------------------
# Flask app
# -----------------------------

app = Flask(__name__)

controller: Optional[SpheroController] = None
joystick_obj = None

def init_pygame_and_joystick(joystick_id: int):
    """Initialiseer pygame en kies joystick."""
    global joystick_obj
    pygame.init()
    pygame.joystick.init()
    num = pygame.joystick.get_count()
    if num == 0:
        raise RuntimeError("Geen joysticks gevonden (check verbinding).")
    if joystick_id < 0 or joystick_id >= num:
        raise RuntimeError(f"Joystick {joystick_id} bestaat niet (gevonden: 0..{num-1}).")
    joystick_obj = pygame.joystick.Joystick(joystick_id)
    joystick_obj.init()
    return joystick_obj

INDEX_HTML = """
<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <title>Sphero Controller</title>
  <style>
    body { font-family: system-ui, sans-serif; margin: 2rem; }
    form { display: grid; gap: 0.75rem; max-width: 420px; }
    label { font-weight: 600; }
    input, select, button { padding: .5rem .6rem; font-size: 1rem; }
    .row { display: grid; grid-template-columns: 1fr 1fr; gap: .75rem; }
    .status { margin: 1rem 0; padding: .75rem; background: #f6f6f6; border-radius: 8px; }
    .actions { display: flex; gap: .5rem; flex-wrap: wrap; }
    code { background:#eee; padding:.2rem .4rem; border-radius:4px;}
  </style>
</head>
<body>
  <h1>Sphero Controller</h1>

  <div class="status" id="status">Status laden…</div>

  <form method="post" action="{{ url_for('start') }}">
    <label for="toy_name">Toy name</label>
    <input id="toy_name" name="toy_name" placeholder="bv. SB-9DD8" value="{{ request.form.get('toy_name', '') }}" required>

    <div class="row">
      <div>
        <label for="joystick_id">Joystick ID</label>
        <select id="joystick_id" name="joystick_id">
          <option value="0">0</option>
          <option value="1">1</option>
        </select>
      </div>
      <div>
        <label for="player_number">Speler # (1–5)</label>
        <select id="player_number" name="player_number">
          <option>1</option><option>2</option><option>3</option><option>4</option><option>5</option>
        </select>
      </div>
    </div>

    <div class="actions">
      <button type="submit">Start</button>
      <button formaction="{{ url_for('discover_nearest') }}" formmethod="post" type="submit">Zoek dichtstbijzijnde</button>
      <button formaction="{{ url_for('stop') }}" formmethod="post" type="submit">Stop</button>
    </div>
  </form>

  <p>Tip: bekende mappings:
     <code>SB-9DD8 → 1</code>, <code>SB-2BBE → 2</code>, <code>SB-27A5 → 3</code>, <code>SB-81E0 → 4</code>, <code>SB-7740 → 5</code>
  </p>

  <script>
    async function refreshStatus() {
      try {
        const r = await fetch("{{ url_for('status') }}");
        const j = await r.json();
        const s = document.getElementById('status');
        s.textContent =
          `running: ${j.running}, toy: ${j.toy_name || '—'}, joystick: ${j.joystick_id ?? '—'}, speler: ${j.player_number ?? '—'}`;
      } catch (e) {
        document.getElementById('status').textContent = 'Kon status niet laden.';
      }
    }
    refreshStatus();
    setInterval(refreshStatus, 2000);
  </script>
</body>
</html>
"""

@app.route("/", methods=["GET"])
def index():
    return render_template_string(INDEX_HTML)

@app.route("/status", methods=["GET"])
def status():
    global controller, joystick_obj
    running = bool(controller and controller.running)
    return jsonify({
        "running": running,
        "toy_name": getattr(controller.toy, "name", None) if controller else None,
        "joystick_id": joystick_obj.get_id() if joystick_obj else None,
        "player_number": controller.number if controller else None
    })

@app.route("/discover-nearest", methods=["POST"])
def discover_nearest():
    global controller, joystick_obj
    jid = int(request.form.get("joystick_id", "0"))
    pn = int(request.form.get("player_number", "1"))

    try:
        if joystick_obj is None:
            init_pygame_and_joystick(jid)
    except Exception as e:
        return f"Joystick-initialisatie faalde: {e}", 400

    # (Re)maak controller
    controller = SpheroController(joystick_obj, Color(255, 0, 0), pn)
    name = controller.discover_nearest_toy()
    if not name:
        return "Geen Sphero gevonden.", 404
    return redirect(url_for('index'))

@app.route("/start", methods=["POST"])
def start():
    global controller, joystick_obj
    toy_name = request.form.get("toy_name", "").strip()
    jid = int(request.form.get("joystick_id", "0"))
    pn = int(request.form.get("player_number", "1"))

    if not (1 <= pn <= 5):
        return "Spelernummer moet tussen 1 en 5 liggen.", 400

    try:
        if joystick_obj is None:
            init_pygame_and_joystick(jid)
    except Exception as e:
        return f"Joystick-initialisatie faalde: {e}", 400

    # Stop eventuele vorige controller veilig
    if controller and controller.running:
        controller.stop()

    controller = SpheroController(joystick_obj, Color(255, 0, 0), pn)

    if not toy_name:
        return "Geen toy_name opgegeven.", 400

    ok = controller.discover_toy(toy_name)
    if not ok:
        return f"Kon toy '{toy_name}' niet vinden.", 404

    controller.start()
    return redirect(url_for('index'))

@app.route("/stop", methods=["POST"])
def stop():
    global controller
    if controller:
        controller.stop()
    return redirect(url_for('index'))

# Netjes afsluiten bij proces-stop
def _shutdown():
    global controller
    try:
        if controller:
            controller.stop()
            time.sleep(0.2)
    except Exception:
        pass
    try:
        pygame.quit()
    except Exception:
        pass

if __name__ == "__main__":
    try:
        # Luister op alle IP's
        app.run(host="0.0.0.0", port=5000, debug=True)
    finally:
        _shutdown()
