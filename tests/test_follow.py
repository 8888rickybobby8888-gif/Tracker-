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

def run_follow(argv, states_fn=None, esp32_fn=None, secs=2.5, arena=True):
    """One run of follow.main with everything fake. Returns what happened."""
    events, chase_yaws, drawn = [], [], []
    if arena: json.dump(ARENA, open("arena.json", "w"))
    elif os.path.exists("arena.json"): os.remove("arena.json")

    class FakeRC:
        def __init__(s):
            s.writeAttribute = lambda op, data, timeout=None: True
            s.doMotorOperation = lambda op, data, timeout=None: None
        def connectBLE(s, i): events.append(("connect", i))
        def startRover(s): events.append(("start",))
        def stopRover(s): events.append(("stop",))
        def end(s): events.append(("end",))
    follow.RoverController = lambda: FakeRC()
    follow.MicromelonMotors.turn_degrees = lambda self, d, sp: (events.append(("evade-turn", round(d, 1), sp)), time.sleep(0.2))
    follow.read_ir = lambda: (25.0, 25.0)
    follow.Motors = types.SimpleNamespace(write=lambda *a: events.append(("write",) + a))
    follow.my_chase.chase = lambda yaw: chase_yaws.append(yaw)
    follow.stop.clear(); follow.yaw_to_leader = follow.yaw_to_target = None
    evade.set_max_speed(30)

    def vision_main(camera, flip, on_yaw=None, on_state=None, on_draw=None):
        t0 = time.time()
        while time.time() - t0 < secs:
            t = time.time() - t0
            st = states_fn(t) if states_fn else {"follower": {"x": 0.3, "y": 0.55, "yaw": 0.0, "age": 0}, "A": {"x": 0.9, "y": 0.55, "age": 0}}
            if on_yaw: on_yaw(0.0)
            if on_state: on_state(st)
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
    follow.main(argv)
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
    f = np.full((4, 4), np.nan)
    if t > 0.8:
        d = 1000 - 300 * (t - 0.8)
        if d > 100: f[1, 0] = d; f[2, 0] = d
    return f
ev4, ch, _ = run_follow(["--camera", "0"], states_fn=lambda t: {"follower": {"x": 0.3, "y": 0.2, "yaw": 0.0, "age": 0}}, esp32_fn=esp32_chaser, secs=3.5)
turns = [e for e in ev4 if e[0] == "evade-turn"]
check("lidar over UDP: a chaser behind-left makes the rover turn LEFT to face it", turns and 120 < turns[0][1] <= 180, f"{turns[0][1:]}" if turns else "no evasion")
check("shutdown: motors set to 0, rover stopped and disconnected, in that order",
      [e[0] for e in ev4 if e[0] in ("stop", "end")] == ["stop", "end"] and ("write", 0, 0) in ev4)

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

print("\nSUMMARY: %d/%d passed" % (sum(1 for r in results if r[1]), len(results)))
for r in results:
    if not r[1]: print("   FAILED:", r[0], r[2])
sys.exit(0 if all(r[1] for r in results) else 1)
