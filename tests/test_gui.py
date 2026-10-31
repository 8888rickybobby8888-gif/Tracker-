"""
tests/test_gui.py - runs the screen-based tools and scripts on the simulator's rendered camera frames, with the
windows, keys and mouse scripted: tracker.py live view (1/2/3, click a balloon, M, [ ], F), mark_arena.py (clicking the
arena and no-go corners while the camera drifts), onboard.py, and learn_mask.py over real UDP.

    python tests/test_gui.py          (or run everything with: python run_tests.py)
"""
import sys, os, math, json, time, threading, subprocess, socket, tempfile
import numpy as np, cv2
ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, ROOT)
os.chdir(tempfile.mkdtemp(prefix="gui_test_"))
import sim_world as sw, tracker as tk, mark_arena as ma, onboard as ob, lidar_rx as lx, evade as ev

results = []
def check(name, ok, detail=""):
    results.append((name, bool(ok), detail)); print(("PASS " if ok else "FAIL ") + name + ("  -- " + detail if detail else ""))

# ---------------- a simulated external camera + scripted GUI ----------------
rig = sw.CameraRig(70.0, drift=True)
f = sw.Agent("follower", 0.45, 0.30, 0.0, gains=(1.0, 0.985))
opps = [sw.Opponent("A", 1.25, -0.30, 90.0, "static", speed=0.0, path=None, seed=0),
        sw.Opponent("B", 0.45, -0.40, 0.0, "static", speed=0.0, path=None, seed=1),
        sw.Opponent("C", 1.30, 0.45, 180.0, "static", speed=0.0, path=None, seed=2)]
world = sw.World(f, opps); rend = sw.Renderer(seed=1); snap0 = world.snapshot_at(0.0)

class FakeCam:
    def __init__(self): self.t = 0.0
    def read(self):
        self.t += 1/15
        return True, rend.render(snap0, rig.cam(self.t))
    def release(self): pass
cam = FakeCam()
tk.open_camera = lambda camera=None, flip=None: (cam, sw.SIZE)

gui = {"i": 0, "cb": None, "shown": 0, "script": {}, "bad_image": 0}
cv2.namedWindow = lambda *a, **k: None
cv2.setMouseCallback = lambda name, cb: gui.__setitem__("cb", cb)
def imshow(name, img):
    gui["shown"] += 1
    if not (hasattr(img, "shape") and img.ndim in (2, 3)): gui["bad_image"] += 1
cv2.imshow = imshow
cv2.destroyWindow = lambda *a: None
cv2.destroyAllWindows = lambda: None
def waitKey(ms=0):
    gui["i"] += 1
    act = gui["script"].get(gui["i"], 255)
    if callable(act): act(); return 255
    return act
cv2.waitKey = waitKey
def balloon_xy(name):
    x, y, yaw, _ = snap0.agents[name]
    return x - sw.BALLOON_BACK * math.cos(yaw), y - sw.BALLOON_BACK * math.sin(yaw)
def balloon_px(name, t):
    bx, by = balloon_xy(name)
    return rig.cam(t).project([(bx, by, sw.BALLOON_Z)])[0]
def click(px): 
    gui["cb"](cv2.EVENT_LBUTTONDOWN, int(round(px[0])), int(round(px[1])), 0, None)

# ---------------- 1. tracker.main: the live view, keys, mouse, callbacks ----------------
states, yaws, draws = [], [], []
gui.update(i=0, shown=0, script={
    3: ord("2"), 4: lambda: click(balloon_px("B", cam.t)), 6: ord("m"), 8: ord("]"), 12: ord("["), 15: ord("m"),
    18: ord("f"), 24: ord("1"), 25: lambda: click(balloon_px("A", cam.t)), 60: 27})
try:
    tk.main(0, None, on_yaw=lambda y: yaws.append(y), on_state=lambda st: states.append(st), on_draw=lambda fr, tr: draws.append(1))
    check("tracker.main runs the live view (keys 1/2, click, M mask, [ ] FOV, F re-lock) without error", True, f"{gui['shown']} frames drawn")
except Exception as e:
    import traceback; traceback.print_exc(); check("tracker.main runs the live view", False, repr(e))
check("tracker.main: callbacks fire every frame", len(states) > 40 and len(yaws) == len(states) and len(draws) == len(states), f"{len(states)} states")
last = [s for s in states if "follower" in s and all(k in s for k in "ABC")]
check("tracker.main: follower and all three opponents tracked", len(last) > 25, f"{len(last)}/{len(states)} frames")
if last:
    s = last[-1]; ftrue = (0.45, 0.30)
    check("tracker.main: follower position within 4 cm", math.hypot(s["follower"]["x"]-ftrue[0], s["follower"]["y"]-ftrue[1]) < 0.04,
          "%.1f mm" % (1000*math.hypot(s["follower"]["x"]-ftrue[0], s["follower"]["y"]-ftrue[1])))
    errs = {o.name: math.hypot(s[o.name]["x"] - balloon_xy(o.name)[0], s[o.name]["y"] - balloon_xy(o.name)[1]) for o in opps}
    check("tracker.main: opponent positions within 4 cm", max(errs.values()) < 0.04, " ".join(f"{k} {v*1000:.0f}mm" for k, v in errs.items()))
check("tracker.main: no blank/invalid frames shown", gui["bad_image"] == 0)

# ---------------- 1b. tracker.main: the pause key (P / space) and the status line ----------------
pk = {"paused": False, "keys": [], "msgs": []}
def _on_key(k):
    pk["keys"].append(k)
    if k in (ord("p"), ord(" ")): pk["paused"] = not pk["paused"]
def _status():
    msg = "PAUSED (P or space = resume)" if pk["paused"] else ""
    pk["msgs"].append(msg)
    return msg
gui.update(i=0, shown=0, script={5: ord("p"), 12: ord(" "), 14: ord("p"), 20: 27})
tk.main(0, None, on_key=_on_key, status=_status)
check("pause key: P and space reach the on_key callback, other frames send nothing", pk["keys"] == [ord("p"), ord(" "), ord("p")], str(pk["keys"]))
check("pause key: the status line is asked for on every frame and shows PAUSED only between the presses",
      len(pk["msgs"]) >= 18 and pk["msgs"][2] == "" and pk["msgs"][7].startswith("PAUSED") and pk["msgs"][13] == "" and pk["msgs"][16].startswith("PAUSED"),
      f"{len(pk['msgs'])} frames: " + ",".join("P" if m else "-" for m in pk["msgs"][:20]))

def _bad_key(k): raise RuntimeError("key handler broke")
def _bad_status(): raise RuntimeError("status broke")
gui.update(i=0, shown=0, script={5: ord("p"), 12: 27})
tk.main(0, None, on_key=_bad_key, status=_bad_status)
check("tracker.main: a key handler or status callback that raises does not stop the live view", gui["shown"] >= 10, f"{gui['shown']} frames shown")

# ---------------- 2. mark_arena.main: click the arena + no-go corners while the camera drifts ----------------
cam.t = 0.0
saved = {}
script = {}
def make_click(px_fn): return lambda: click(px_fn(cam.t))
pts = list(sw.ARENA) + list(sw.NOGO)
for k, (x, y) in enumerate(pts):
    script[8 + 3*k] = make_click(lambda t, x=x, y=y: rig.cam(t).project([(x, y, 0.0)])[0])
script[8 + 3*8 + 2] = 13                                   # Enter
gui.update(i=0, shown=0, script=script)
if os.path.exists("arena.json"): os.remove("arena.json")
try:
    ma.main(0, None)
    ok = os.path.exists("arena.json")
    check("mark_arena.main runs and saves arena.json", ok)
    if ok:
        d = json.load(open("arena.json"))
        ea = max(min(math.dist(p, q) for q in sw.ARENA) for p in d["arena"]); en = max(min(math.dist(p, q) for q in sw.NOGO) for p in d["nogo"])
        check("mark_arena: arena corners within 2 cm of the truth (camera drifting while clicking)", ea < 0.02, "%.1f mm" % (ea*1000))
        check("mark_arena: no-go corners within 2 cm of the truth", en < 0.02, "%.1f mm" % (en*1000))
        P = pl.Planner(pl.load_arena("arena.json")) if "pl" in globals() else None
except Exception as e:
    import traceback; traceback.print_exc(); check("mark_arena.main", False, repr(e))
import plan as pl
try:
    A = pl.load_arena("arena.json"); P = pl.Planner(A)
    st = {"follower": {"x": 0.45, "y": 0.3, "yaw": 0.0}, "A": {"x": 1.25, "y": -0.3}}
    pr = P.update(st, 0.0)
    check("planner accepts the saved arena.json", pr is not None and pr.yaw_to_waypoint is not None)
except Exception as e:
    check("planner accepts the saved arena.json", False, repr(e))

# ---------------- 3. onboard.main (ESP32 camera mode) ----------------
W, H, FOV = 640, 480, 65.0
fpx = (W/2)/math.tan(math.radians(FOV/2))
class FakeLF:
    def __init__(self, src): self.t = 0.0; self.n = 0
    def read(self):
        self.n += 1; self.t += 1/20
        th = 20.0 if self.n < 40 else -10.0                      # balloon 20 deg left, then 10 deg right
        img = np.full((H, W, 3), 90, np.uint8)
        cx = W/2 - fpx*math.tan(math.radians(th)); cv2.circle(img, (int(round(cx)), 240), 14, (255, 60, 0), -1)
        return img, self.t
    def release(self): pass
ob.LatestFrame = FakeLF
got = []
gui.update(i=0, shown=0, script={3: lambda: click((W/2 - fpx*math.tan(math.radians(20.0)), 240)), 6: ord("m"), 9: ord("["), 12: ord("]"), 15: ord("m"), 70: 27})
try:
    ob.main("fake", on_yaw=lambda y: got.append(y))
    check("onboard.main runs (click colour, mask, FOV keys) without error", True, f"{gui['shown']} frames")
    a = [g for g in got[10:35] if g is not None]; b = [g for g in got[50:] if g is not None]
    check("onboard.main: yaw to the balloon is ~+20 then ~-10 degrees (FOV nudged by keys)", a and b and abs(np.mean(a)-20) < 4 and abs(np.mean(b)+10) < 3,
          "first %.1f, then %.1f" % (np.mean(a) if a else float('nan'), np.mean(b) if b else float('nan')))
except Exception as e:
    import traceback; traceback.print_exc(); check("onboard.main", False, repr(e))

# ---------------- 4. learn_mask.py over real UDP ----------------
if os.path.exists("lidar_mask.json"): os.remove("lidar_mask.json")
proc = subprocess.Popen([sys.executable, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "learn_mask.py")], cwd=os.getcwd(), stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
time.sleep(1.0)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); t0 = time.time(); seq = 0
while time.time() - t0 < 4.6:
    n = ev.GRID; own = [(n - 1) * n + n // 2 - 1, (n - 1) * n + n // 2]            # own balloon: two zones of the bottom row
    fr = np.full(n * n, np.nan); fr[own[0]] = 90; fr[own[1]] = 95
    sock.sendto(lx.make_packet(seq, int((time.time()-t0)*1000), fr), ("127.0.0.1", 4210)); seq += 1; time.sleep(1/30)
out = proc.communicate(timeout=20)[0]
mask_ok = os.path.exists("lidar_mask.json") and np.array(json.load(open("lidar_mask.json"))).flatten().nonzero()[0].tolist() == sorted([(ev.GRID - 1) * ev.GRID + ev.GRID // 2 - 1, (ev.GRID - 1) * ev.GRID + ev.GRID // 2])
check("learn_mask.py learns the masked zones from live UDP packets", mask_ok, out.strip().splitlines()[-2] if out.strip() else "")

print("\nSUMMARY: %d/%d passed" % (sum(1 for r in results if r[1]), len(results)))
for r in results:
    if not r[1]: print("   FAILED:", r[0], r[2])
sys.exit(0 if all(r[1] for r in results) else 1)
