"""
tests/test_scan.py - the automatic arena scan (scan_arena.py) on the simulator's rendered camera frames: accuracy with a
drifting camera, robots inside the arena, gaps in the tape, an arena cut off by the image edge, a too-dark floor, and
the live tool (ENTER to save) with scripted keys.

    python tests/test_scan.py          (or run everything with: python run_tests.py)
"""
import json, math, os, sys, tempfile, time, types
import numpy as np, cv2

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, ROOT)
os.chdir(tempfile.mkdtemp(prefix="scan_test_"))
import sim_world as sw, tracker as tk, scan_arena as sa

results = []
def check(name, ok, detail=""):
    results.append((name, bool(ok), detail)); print(("PASS " if ok else "FAIL ") + name + (f"  -- {detail}" if detail else ""))

rig = sw.CameraRig(70.0, drift=True)
rend = sw.Renderer(seed=1)
world = sw.World(sw.Agent("follower", 0.45, 0.30, 0.0, gains=(1.0, 0.985)),
                 [sw.Opponent("A", 1.25, -0.30, 90.0, "static", speed=0.0, path=None, seed=0),
                  sw.Opponent("B", 0.45, -0.40, 0.0, "static", speed=0.0, path=None, seed=1)])
snap_full = world.snapshot_at(0.0)
snap_empty = types.SimpleNamespace(agents={}, t=0.0)

def lighten(img, t=0.0):
    """The simulator's floor outside the arena is almost black; make it a lighter grey like a real room floor,
    leaving the tape and everything inside the arena exactly as rendered."""
    poly = np.round(rig.cam(t).project([(x, y, 0.0) for x, y in sw.ARENA])).astype(np.int32)
    inside = np.zeros(img.shape[:2], np.uint8); cv2.fillPoly(inside, [poly], 255)
    inside = cv2.dilate(inside, np.ones((13, 13), np.uint8))
    out = img.copy(); out[(img.max(axis=2) < 70) & (inside == 0)] = (110, 110, 110)
    return out

def frame(snap, t): return lighten(rend.render(snap, rig.cam(t)), t)

def tracker_at(snap, t):
    tr = tk.Tracker(size=sw.SIZE, fov_deg=70.0)
    class Cap:
        def read(s): return True, frame(snap, t)
    assert tr.lock_floor(Cap(), n=10)
    return tr

def err(corners, truth): return max(min(math.dist(p, q) for q in truth) for p in corners)

# ---------------- accuracy, camera in different places ----------------
# The scan can only be as exact as the tracker's own estimate of the floor frame (the floor marker's pose), which is off by
# 1-4 cm at the far corners. So check the scan against the arena AS THE TRACKER SEES IT (truth pushed through the same
# pose estimate), and separately against the true arena with a looser limit.
worst_scan = worst_abs = worst_ng = worst_ng_abs = 0.0; found = 0
for t in (0.0, 2.5, 5.0, 7.5, 10.0):
    tr = tracker_at(snap_empty, t); cam = rig.cam(t)
    seen = lambda poly: [tr._pixel_to_floor(cam.project([(x, y, 0.0)])[0], 0.0) for x, y in poly]
    r = sa.scan_frame(frame(snap_empty, t), tr)
    worst_scan = max(worst_scan, err(r["arena"], seen(sw.ARENA))); worst_abs = max(worst_abs, err(r["arena"], sw.ARENA))
    if r["nogo"]:
        found += 1; worst_ng = max(worst_ng, err(r["nogo"], seen(sw.NOGO))); worst_ng_abs = max(worst_ng_abs, err(r["nogo"], sw.NOGO))
check("scan: arena corners within 15 mm of the arena as the tracker sees it (5 camera positions)", worst_scan < 0.015, f"worst {worst_scan*1000:.1f} mm")
check("scan: ... and within 6 cm of the true arena (the rest is the floor-marker pose error)", worst_abs < 0.06, f"worst {worst_abs*1000:.1f} mm")
check("scan: no-go zone found every time, within 12 mm as seen and 4 cm true", found == 5 and worst_ng < 0.012 and worst_ng_abs < 0.04,
      f"found {found}/5, {worst_ng*1000:.1f} mm as seen, {worst_ng_abs*1000:.1f} mm true")

# ---------------- robots inside the arena ----------------
tr = tracker_at(snap_full, 0.0); img = frame(snap_full, 0.0); st = tr.update(img, now=time.time())
robots = [(v["x"], v["y"]) for v in st.values()]
r = sa.scan_frame(img, tr, exclude=robots)
check("scan: robots the tracker can see are ignored, no-go still found", r["nogo"] and err(r["nogo"], sw.NOGO) < 0.02 and not r["debug"]["warnings"])
r = sa.scan_frame(img, tr, exclude=[])
check("scan: if you forget to clear the arena it still picks the squarest object (the no-go) and warns you",
      r["nogo"] and err(r["nogo"], sw.NOGO) < 0.02 and r["debug"]["warnings"], (r["debug"]["warnings"] or [""])[0][:70])

# ---------------- failures give a plain reason ----------------
tr = tracker_at(snap_empty, 0.0); img = frame(snap_empty, 0.0); cam0 = rig.cam(0.0)
def raises(im):
    try: sa.scan_frame(im, tr); return None
    except sa.ScanError as e: return str(e)
gap = img.copy(); cv2.circle(gap, tuple(cam0.project([(0.8, 0.565, 0.0)])[0].astype(int)), 32, (110, 110, 110), -1)
check("scan: a gap in the red tape is reported", (raises(gap) or "").startswith("found tape but no closed ring"))
gap2 = img.copy(); cv2.circle(gap2, tuple(cam0.project([(1.4, 0.0, 0.0)])[0].astype(int)), 32, (110, 110, 110), -1)
check("scan: a gap in the black tape is reported", (raises(gap2) or "").startswith("found tape but no closed ring"))
xs = cam0.project([(x, y, 0.0) for x, y in sw.ARENA])[:, 0]
crop = img.copy(); crop[:, int(xs.max() - 70):] = (120, 120, 120)
check("scan: an arena cut off by the edge of the picture is reported", raises(crop) is not None, raises(crop) or "")
check("scan: a picture with no tape is reported", (raises(np.full_like(img, (110, 110, 110))) or "").startswith("no red or black tape"))
r = sa.scan_frame(rend.render(snap_empty, cam0), tr)
check("scan: a floor as dark as the black tape triggers a warning", any("too dark" in w for w in r["debug"]["warnings"]))

# ---------------- averaging ----------------
tr = tracker_at(snap_empty, 0.0); res = []
for i in range(15):
    t = i / 5.0; im = frame(snap_empty, t); tr.update(im, now=time.time()); res.append(sa.scan_frame(im, tr))
arena, nogo, spread = sa.combine(res)
check("scan: 15 scans while the camera drifts average to within 4 cm and read as steady", err(arena, sw.ARENA) < 0.04 and nogo and spread <= sa.STEADY_MM,
      f"arena err {err(arena, sw.ARENA)*1000:.1f} mm, spread {spread:.1f} mm (limit {sa.STEADY_MM})")

# ---------------- the live tool: ENTER before steady does nothing, ENTER when steady saves arena.json ----------------
class FakeCam:
    def __init__(self): self.t = 0.0
    def read(self):
        self.t += 0.1; return True, frame(snap_empty, self.t)
    def release(self): pass
cam = FakeCam()
tk.open_camera = lambda camera=None, flip=None: (cam, sw.SIZE)
fake_clock = {"t": 1000.0}
def fake_time(): fake_clock["t"] += 0.1; return fake_clock["t"]
sa.time = types.SimpleNamespace(time=fake_time)
gui = {"i": 0, "script": {5: 13, 150: 13}}
cv2.namedWindow = lambda *a, **k: None
cv2.imshow = lambda *a: None
cv2.destroyWindow = lambda *a: None
cv2.destroyAllWindows = lambda: None
def waitKey(ms=0):
    gui["i"] += 1
    return gui["script"].get(gui["i"], 255 if gui["i"] < 400 else 27)
cv2.waitKey = waitKey
if os.path.exists("arena.json"): os.remove("arena.json")
saved_early = {}
orig_save = sa.save
def watch_save(*a, **k):
    saved_early["i"] = gui["i"]; return orig_save(*a, **k)
sa.save = watch_save
try:
    sa.main(0, None)
    check("scan tool: ENTER at frame 5 (not steady yet) does not save, ENTER later does", saved_early.get("i", 0) >= 100, f"saved at frame {saved_early.get('i')}")
    d = json.load(open("arena.json"))
    check("scan tool: the saved arena.json matches the truth (arena < 5 cm, no-go < 4 cm)", err(d["arena"], sw.ARENA) < 0.05 and d["nogo"] and err(d["nogo"], sw.NOGO) < 0.04,
          f"arena {err(d['arena'], sw.ARENA)*1000:.0f} mm, no-go {err(d['nogo'], sw.NOGO)*1000:.0f} mm")
    import plan as pl
    P = pl.Planner(pl.load_arena("arena.json"))
    p = P.update({"follower": {"x": 0.45, "y": 0.3, "yaw": 0.0}, "A": {"x": 1.25, "y": -0.3}}, 0.0)
    check("scan tool: the planner can use the saved file", p is not None and p.yaw_to_waypoint is not None)
except Exception as e:
    import traceback; traceback.print_exc(); check("scan tool runs", False, repr(e))

print("\nSUMMARY: %d/%d passed" % (sum(1 for r in results if r[1]), len(results)))
for r in results:
    if not r[1]: print("   FAILED:", r[0], r[2])
sys.exit(0 if all(r[1] for r in results) else 1)
