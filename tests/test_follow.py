"""
tests/test_follow.py - runs follow.py (the real entry point) with a fake rover link, a fake external camera and a fake
ESP32 sending lidar over real UDP. Checks planning on/off, evasion from the lidar and from the camera, the speed cap
option, and a clean shutdown.

    python tests/test_follow.py          (or run everything with: python run_tests.py)
"""
import json, os, socket, sys, tempfile, threading, time, types
import numpy as np

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, ROOT)
os.chdir(tempfile.mkdtemp(prefix="follow_test_"))
import evade, lidar_rx, follow

results = []
def check(name, ok, detail=""):
    results.append((name, bool(ok), detail)); print(("PASS " if ok else "FAIL ") + name + (f"  -- {detail}" if detail else ""))

ARENA = {"arena": [[0, 0], [1.2, 0], [1.2, 1.1], [0, 1.1]], "nogo": [[0.525, 0.475], [0.675, 0.475], [0.675, 0.625], [0.525, 0.625]]}

CONNECT_DELAY = 0.0
KEYS = []                      # scripted key presses [(seconds, key)] and the status lines seen
STATUS = []
follow.AUTO_CONNECT_S = 0.0
follow.OPEN_BROWSER = False


LAST_EVENTS = []


class FakeTr:
    fov_deg = 70.0
    opp_hsv = {"A": [((100, 120, 60), (130, 255, 255))], "B": [((40, 100, 60), (80, 255, 255))], "C": [((20, 120, 100), (35, 255, 255))]}
    colour_calls, fov_calls = [], []
    def set_opponent_color(self, n, h, s_, v): self.colour_calls.append((n, int(h)))
    def set_fov(self, f): self.fov_calls.append(f); self.fov_deg = f
    def lock_floor(self, cap): return True


SIDE_TASK = None


def run_follow(argv, states_fn=None, esp32_fn=None, secs=2.5, arena=True, drive=False, window=True):
    """One run of follow.main with everything fake. Returns what happened."""
    events, chase_yaws, drawn = [], [], []
    LAST_EVENTS[:] = [events]
    if arena: json.dump(ARENA, open("arena.json", "w"))
    elif os.path.exists("arena.json"): os.remove("arena.json")

    class FakeRC:
        def __init__(s):
            s.writeAttribute = lambda op, data, timeout=None: True
            s.doMotorOperation = lambda op, data, timeout=None: None
        def connectBLE(s, i): time.sleep(CONNECT_DELAY); events.append(("connect", i))
        def startRover(s): events.append(("start",))
        def stopRover(s): events.append(("stop",))
        def end(s): events.append(("end",))
    follow.RoverController = lambda: FakeRC()
    follow.MicromelonMotors.turn_degrees = lambda self, d, sp: (events.append(("evade-turn", round(d, 1), sp)), time.sleep(0.2))
    follow.read_ir = lambda: (25.0, 25.0)
    follow.Motors = types.SimpleNamespace(write=lambda *a: events.append(("write",) + a))
    follow.my_chase.chase = (lambda yaw: (chase_yaws.append(yaw), follow.Motors.write(20, 20))) if drive else (lambda yaw: chase_yaws.append(yaw))
    follow.stop.clear(); follow.yaw_to_leader = follow.yaw_to_target = None
    evade.set_max_speed(30)

    def vision_main(camera, flip, on_yaw=None, on_state=None, on_draw=None, on_key=None, status=None, **kw):
        events.append(("window",))
        t0 = time.time()
        todo = sorted(KEYS)
        while time.time() - t0 < secs:
            t = time.time() - t0
            while todo and todo[0][0] <= t:
                if on_key: on_key(todo.pop(0)[1])
            if status: STATUS.append(status())
            st = states_fn(t) if states_fn else {"follower": {"x": 0.3, "y": 0.55, "yaw": 0.0, "age": 0}, "A": {"x": 0.9, "y": 0.55, "age": 0}}
            if on_yaw: on_yaw(0.0)
            if on_state: on_state(st)
            if kw.get('on_tick'): kw['on_tick'](FakeTr(), None)
            if kw.get('on_frame'):
                import numpy as _np
                img = _np.zeros((480, 640, 3), _np.uint8); img[:, :] = (30, 60, 200)
                kw['on_frame'](img, img.copy(), FakeTr(), st)
            if on_draw: drawn.append(1)
            time.sleep(1 / 30)
    fake_tracker = types.ModuleType("tracker"); fake_tracker.main = vision_main
    sys.modules["tracker"] = fake_tracker
    if esp32_fn:
        port = 4400 + int(time.time()) % 400
        argv = argv + ["--lidar-port", str(port)]
        def esp32():
            tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); t0 = time.time(); seq = 0
            while time.time() - t0 < secs - 0.1:
                tx.sendto(lidar_rx.make_packet(seq, int((time.time() - t0) * 1000), esp32_fn(time.time() - t0).ravel()), ("127.0.0.1", port))
                seq += 1; time.sleep(1 / 30)
        threading.Thread(target=esp32, daemon=True).start()
    if SIDE_TASK is not None: threading.Thread(target=SIDE_TASK, daemon=True).start()
    follow.main(argv + (['--window'] if window else []))
    return events, chase_yaws, drawn

# 1. planning ON: the target is across the no-go square
ev1, ch, drawn = run_follow(["--camera", "0", "--no-lidar"])
vals = [v for v in ch if v is not None]
check("planning on: chase() gets the yaw to the next waypoint round the no-go square, not straight at the target",
      vals and abs(vals[-1]) > 20 and abs(follow.yaw_to_target) < 1.0, f"waypoint yaw {vals[-1]:.0f}, straight yaw {follow.yaw_to_target:.0f}" if vals else "no yaw")
check("planning on: the live view is asked to draw the plan", len(drawn) > 20)

# 2. --no-plan
ev2, ch, drawn = run_follow(["--camera", "0", "--no-lidar", "--no-plan"])
vals = [v for v in ch if v is not None]
check("--no-plan: chase() gets the straight yaw", vals and abs(vals[-1]) < 1.0 and not drawn)

# 3. no arena.json: falls back to straight yaw and says so
ev3, ch, _ = run_follow(["--camera", "0", "--no-lidar"], arena=False)
vals = [v for v in ch if v is not None]
check("no arena.json: falls back to the straight yaw", vals and abs(vals[-1]) < 1.0)

# 4. lidar chaser behind-left over real UDP -> an evasion turn to the left, never chasing mid-turn, clean shutdown
def esp32_chaser(t):
    n = evade.GRID                                           # whatever resolution the detector is set to (4x4 or 8x8)
    f = np.full((n, n), np.nan)
    if t > 0.8:
        d = 1000 - 300 * (t - 0.8)
        if d > 100: f[n // 4:n - n // 4, 0:max(1, n // 4)] = d      # the middle rows, the leftmost column(s): behind-left
    return f
ev4, ch, _ = run_follow(["--camera", "0"], states_fn=lambda t: {"follower": {"x": 0.3, "y": 0.2, "yaw": 0.0, "age": 0}}, esp32_fn=esp32_chaser, secs=3.5)
turns = [e for e in ev4 if e[0] == "evade-turn"]
check("lidar over UDP: a chaser behind-left makes the rover turn LEFT to face it", turns and 120 < turns[0][1] <= 180, f"{turns[0][1:]}" if turns else "no evasion")
check("shutdown: motors set to 0, rover stopped and disconnected, in that order",
      [e[0] for e in ev4 if e[0] in ("stop", "end")] == ["stop", "end"] and ("write", 0, 0) in ev4)


# 4b. the same chaser, but the lidar arrives over the ESP-NOW bridge (USB serial) with --lidar-serial
class FakeBridgePort:
    """What the bridge ESP32 would send down USB: one framed lidar packet every 1/30 s, from the same chaser as the UDP test."""
    def __init__(self): self.t0 = time.time(); self.sent = 0; self.seq = 0
    def read(self, n):
        due = int((time.time() - self.t0) * 30)
        if self.sent >= due:
            time.sleep(0.01); return b""
        out = b""
        while self.sent < due:
            tt = self.sent / 30.0
            out += lidar_rx.frame_bytes(lidar_rx.make_packet(self.seq, int(tt * 1000), esp32_chaser(tt).ravel())); self.sent += 1; self.seq += 1
        return out
    def close(self): pass
_orig_serial_rx = lidar_rx.SerialLidarReceiver
lidar_rx.SerialLidarReceiver = lambda port, cb, baud=921600, on_jpeg=None: _orig_serial_rx(port, cb, baud=baud, opener=lambda p, b: FakeBridgePort(), on_jpeg=on_jpeg)
ev4b, ch, _ = run_follow(["--camera", "0", "--lidar-serial", "FAKE"], states_fn=lambda t: {"follower": {"x": 0.3, "y": 0.2, "yaw": 0.0, "age": 0}}, secs=3.5)
lidar_rx.SerialLidarReceiver = _orig_serial_rx
turns = [e for e in ev4b if e[0] == "evade-turn"]
check("lidar over the ESP-NOW bridge (--lidar-serial): a chaser behind-left makes the rover turn LEFT to face it",
      turns and 120 < turns[0][1] <= 180, f"{turns[0][1:]}" if turns else "no evasion")

# 5. external camera only: side attack
def side_attack(t):
    y = max(0.22, 1.0 - 0.3 * max(0.0, t - 0.6))
    return {"follower": {"x": 0.0, "y": 0.0, "yaw": 0.0, "age": 0}, "B": {"x": 0.0, "y": y, "age": 0}}
ev5, ch, _ = run_follow(["--camera", "0", "--no-lidar", "--no-plan"], states_fn=side_attack, secs=3.0)
left_spins = [e for e in ev5 if e[0] == "write" and len(e) == 3 and e[1] < -3 and e[2] > 3]     # left wheel back, right forward
right_spins = [e for e in ev5 if e[0] == "write" and len(e) == 3 and e[1] > 3 and e[2] < -3]
check("external camera only: an attack from the left side makes the rover turn LEFT to face it (continuous wheel commands)",
      len(left_spins) > 3 and not right_spins, f"{len(left_spins)} left-spin commands, {len(right_spins)} right-spin commands")

# 6. speed cap option
run_follow(["--camera", "0", "--no-lidar", "--max-speed", "18"], secs=0.5)
check("--max-speed 18 sets the global cap", evade.MAX_SPEED_CM_S == 18.0)
evade.set_max_speed(30)

# 7. --floor-sensors: calibrates on the bare floor, then polls the sensors
floor_calls = []
floor_mode = ["floor"]
def fake_floor():
    floor_calls.append(1)
    return [[0, 200, 200, 200, 200]] * 3 if floor_mode[0] == "floor" else [[0, 40, 40, 40, 50]] * 3
follow.read_floor_colours = fake_floor
def drive_then_tape(t):
    if t > 1.2: floor_mode[0] = "tape"
    return {"follower": {"x": 0.3, "y": 0.3, "yaw": 0.0, "age": 0}, "A": {"x": 0.9, "y": 0.3, "age": 0}}
ev7, ch, _ = run_follow(["--camera", "0", "--no-lidar", "--no-plan", "--floor-sensors"], states_fn=drive_then_tape, secs=3.0, drive=True)
check("--floor-sensors: a run with tape under the sensors completes and shuts down cleanly (the guard itself is covered in test_units)",
      [e[0] for e in ev7 if e[0] in ("stop", "end")] == ["stop", "end"])
check("--floor-sensors: the sensors are read (calibration + polling)", len(floor_calls) >= 12, f"{len(floor_calls)} reads")
floor_mode[0] = "floor"

CONNECT_DELAY = 0.8                                          # a slow Bluetooth connection
ev_slow, _, _ = run_follow(["--camera", "0"], secs=2.5)
CONNECT_DELAY = 0.0
names = [e[0] for e in ev_slow]
check("the window opens BEFORE the (slow) rover connection finishes", "window" in names and "connect" in names and names.index("window") < names.index("connect"), str(names[:4]))
check("... and the rover is still connected and started afterwards", "start" in names)

follow.AUTO_CONNECT_S = 30.0                                  # would wait 30 s: only the GUI can start the connection
KEYS[:] = [(0.3, ord("r")), (0.4, ord("4")), (0.5, ord("2")), (0.6, 13)]
STATUS.clear()
ev_id, _, _ = run_follow(["--camera", "0"], secs=2.0)
follow.AUTO_CONNECT_S = 0.0; KEYS.clear()
check("rover ID typed in the window (R, 4, 2, Enter) connects to rover 42", ("connect", 42) in ev_id, str([e for e in ev_id if e[0] == "connect"]))
check("the window shows the ID prompt while typing", any("ROVER ID: 42_" in x for x in STATUS), str(STATUS[:3]))


# ---- the browser dashboard
import re
import urllib.request, urllib.error
DPORT = 8900 + int(time.time()) % 90
dres = {}

def http(path, data=None, token=None):
    req = urllib.request.Request(f"http://127.0.0.1:{DPORT}{path}", data=json.dumps(data).encode() if data is not None else None,
                                 headers={"X-Token": token} if token else {})
    try:
        with urllib.request.urlopen(req, timeout=3) as r:
            return r.status, r.read()
    except urllib.error.HTTPError as e:
        return e.code, e.read()

def side():
    try:
        for _ in range(60):                                   # wait for the server
            try: code, page = http("/"); break
            except Exception: time.sleep(0.1)
        tok = re.search(rb'TOKEN = "(\w+)"', page).group(1).decode()
        dres["page"] = (code, b"Rover control" in page, tok != "__TOKEN__")
        dres["no_token"] = http("/api/cmd", {"cmd": "start"})[0]
        dres["bad_token"] = http("/api/cmd", {"cmd": "start"}, "nope")[0]
        for _ in range(40):                                   # until the (fake) rover is connected
            st = json.loads(http("/api/state")[1])
            if "rover" not in st: dres["state_error"] = st; break
            if st["rover"]["state"] == "connected": break
            time.sleep(0.1)
        dres["ready"] = (st["rover"]["state"], st["run"]["state"])
        n_before = len([e for e in dres_events() if e[0] == "write"])
        time.sleep(0.4)
        dres["idle_writes"] = len([e for e in dres_events() if e[0] == "write"]) - n_before          # nothing drives before START
        http("/api/cmd", {"cmd": "start"}, tok); time.sleep(0.5)
        st = json.loads(http("/api/state")[1]); dres["running"] = st["run"]["state"]
        dres["state_bits"] = (st["follower"] is not None, "A" in st["opps"], st["plan"] is not None and st["plan"]["pref"], st["arena"] is not None, len(st["slots"]))
        http("/api/cmd", {"cmd": "target", "name": "B"}, tok); time.sleep(0.3)
        dres["target"] = json.loads(http("/api/state")[1])["plan"]["pref"]
        http("/api/cmd", {"cmd": "pause"}, tok); time.sleep(0.3)
        dres["paused"] = json.loads(http("/api/state")[1])["run"]["state"]
        http("/api/cmd", {"cmd": "pause"}, tok); time.sleep(0.2)
        http("/api/cmd", {"cmd": "speed", "value": 18}, tok); http("/api/cmd", {"cmd": "fov", "value": 80}, tok)
        http("/api/cmd", {"cmd": "colour", "slot": "B", "cam": "external", "x": 0.5, "y": 0.5}, tok)
        http("/api/cmd", {"cmd": "safety", "camstop": False}, tok)
        time.sleep(0.4)
        st = json.loads(http("/api/state")[1]); dres["settings"] = (st["settings"]["speed"], st["settings"]["camstop"], FakeTr.fov_calls[-1:] , FakeTr.colour_calls[-1:])
        code, jpg = http("/frame.jpg?cam=external"); dres["jpg"] = (code, jpg[:2] == b"\xff\xd8")
        http("/api/cmd", {"cmd": "mode", "mode": "manual"}, tok); http("/api/cmd", {"cmd": "drive", "dir": "fwd"}, tok); time.sleep(0.4)
        dres["manual_writes"] = [e for e in dres_events() if e[0] == "write" and e[1:] == (15, 15)][:1]
        http("/api/cmd", {"cmd": "drive", "dir": "stop"}, tok)
        http("/api/cmd", {"cmd": "experiment", "id": "backoff", "value": True}, tok); time.sleep(0.2)
        xs = {e["id"]: e for e in json.loads(http("/api/state")[1])["experiments"]}
        dres["exp_on"] = (xs["backoff"]["value"], follow.evade.BACKOFF_SPEED)
        http("/api/cmd", {"cmd": "experiments_reset"}, tok); time.sleep(0.2)
        dres["exp_reset"] = follow.evade.BACKOFF_SPEED
        http("/api/cmd", {"cmd": "stop"}, tok); time.sleep(0.3)
        dres["stopped"] = json.loads(http("/api/state")[1])["run"]["state"]
        dres["log_tags"] = {l["tag"] for l in json.loads(http("/api/state")[1])["log"]}
    except Exception as e:
        dres["error"] = repr(e)

def dres_events(): return list(LAST_EVENTS[0]) if LAST_EVENTS else []


SIDE_TASK = side
follow.OPEN_BROWSER = False
run_follow(["--camera", "0", "--port", str(DPORT)], secs=7.0, drive=True, window=False)
SIDE_TASK = None
check("dashboard: the page is served with a token filled in", dres.get("page") == (200, True, True), str(dres.get("page") or dres.get("error")))
check("dashboard: a command without the token, or with a wrong one, is refused (403)", dres.get("no_token") == 403 and dres.get("bad_token") == 403, str((dres.get("no_token"), dres.get("bad_token"))))
check("dashboard: the rover connects but stands still until START", dres.get("ready") == ("connected", "ready") and dres.get("idle_writes") == 0, str((dres.get("ready"), dres.get("idle_writes"), dres.get("error"), str(dres.get("state_error"))[:300])))
check("dashboard: START runs the chase; the state has follower, opponents, plan, arena and slots", dres.get("running") == "running" and dres.get("state_bits") == (True, True, "nearest", True, 3), str((dres.get("running"), dres.get("state_bits"))))
check("dashboard: target button, PAUSE and RESUME", dres.get("target") == "B" and dres.get("paused") == "paused", str((dres.get("target"), dres.get("paused"))))
check("dashboard: speed, field of view, colour click and the camera fail-safe switch are applied",
      dres.get("settings") == (18.0, False, [80.0], [("B", 112)]) or (dres.get("settings") and dres["settings"][0] == 18.0 and dres["settings"][1] is False and dres["settings"][2] == [80.0] and dres["settings"][3]), str(dres.get("settings")))
check("dashboard: the camera picture is a JPEG", dres.get("jpg") == (200, True), str(dres.get("jpg")))
check("dashboard: manual drive writes the wheel speeds, STOP stops", bool(dres.get("manual_writes")) and dres.get("stopped") == "stopped", str((dres.get("manual_writes"), dres.get("stopped"))))
check("dashboard: an experiment toggle changes the setting and Reset puts it back", dres.get("exp_on") == (True, 28) and dres.get("exp_reset") == 0, str((dres.get("exp_on"), dres.get("exp_reset"))))
check("dashboard: events reach the log", {"ROVER", "INFO"} <= (dres.get("log_tags") or set()), str(dres.get("log_tags")))

print("\nSUMMARY: %d/%d passed" % (sum(1 for r in results if r[1]), len(results)))
for r in results:
    if not r[1]: print("   FAILED:", r[0], r[2])
sys.exit(0 if all(r[1] for r in results) else 1)
