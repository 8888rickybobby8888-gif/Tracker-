"""
tests/test_units.py - assert-based checks of the pure logic and the single-purpose tools. No camera, rover or ESP32.

    python tests/test_units.py          (or run everything with: python run_tests.py)
"""
import itertools
import math
import os
import socket
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import evade as ev
import lidar_rx as lx
import onboard as ob
import plan as pl

results = []


def check(name, ok, detail=""):
    results.append((name, bool(ok), detail))
    print(("PASS " if ok else "FAIL ") + name + (f"  -- {detail}" if detail else ""))


# ------------------------------------------------------------------------------------------------ planner
ARENA = pl.Arena([(0, 0), (1.2, 0), (1.2, 1.1), (0, 1.1)], [(0.525, 0.475), (0.675, 0.475), (0.675, 0.625), (0.525, 0.625)])
P = pl.Planner(ARENA)
r = pl.ROBOT_RADIUS_M + pl.NOGO_MARGIN_M
sq = ARENA.nogo
check("planner: grown no-go zone is the square grown by radius + margin",
      all(abs(abs(c[0] - 0.6) - (0.075 + r)) < 1e-9 and abs(abs(c[1] - 0.55) - (0.075 + r)) < 1e-9 for c in P.block))
check("planner: line through the square crosses it", pl.crosses(sq, (0.3, 0.55), (0.9, 0.55)))
check("planner: line along an edge or clear of it does not", not pl.crosses(sq, (0.525, 0.3), (0.525, 0.9)) and not pl.crosses(sq, (0.2, 0.2), (0.2, 0.9)))


def length(path):
    return sum(float(np.linalg.norm(np.array(path[i + 1]) - np.array(path[i]))) for i in range(len(path) - 1))


def best_corner_route(planner, s, g):
    C = [tuple(c) for c in planner.block]
    best = float("inf")
    for k in (1, 2):
        for seq in itertools.permutations(C, k):
            pts = [s] + list(seq) + [g]
            if all(not pl.crosses(planner.block, pts[i], pts[i + 1]) for i in range(len(pts) - 1)):
                best = min(best, sum(math.dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1)))
    return best


worst = 0.0
for s, g in [((0.3, 0.35), (0.9, 0.8)), ((0.2, 0.9), (1.0, 0.25)), ((0.35, 0.55), (0.9, 0.55)), ((0.3, 0.2), (0.9, 0.9))]:
    Pn = pl.Planner(ARENA)
    path, blocked = Pn.route(s, g)
    ok_valid = all(not pl.crosses(Pn.block, path[i], path[i + 1]) for i in range(len(path) - 1))
    worst = max(worst, abs(length(path) - best_corner_route(Pn, s, g)))
    check(f"planner: route {s}->{g} stays out of the grown zone", ok_valid)
check("planner: routes are as short as the best brute-force route", worst < 1e-6, f"worst gap {worst:.2e} m")
Ph = pl.Planner(ARENA)
firsts = []
for i in range(60):
    path, _ = Ph.route((0.3 + 0.005 * math.sin(i * 0.7), 0.30 + 0.002 * i), (0.9, 0.8))
    if len(path) > 2:
        firsts.append(tuple(np.round(path[1], 2)))
check("planner: hysteresis keeps a committed side (at most 1 switch while sliding across the tie)",
      sum(1 for a, b in zip(firsts, firsts[1:]) if a != b) <= 1)
path, _ = P.route((0.6, 0.55), (1.0, 0.9))
check("planner: starting INSIDE the grown zone, the first waypoint steps out", len(path) >= 3 and not pl.inside(P.block, path[1]))
path, _ = P.route((0.3, 0.3), (1.3, 1.2))
check("planner: a goal outside the arena is pulled inside", pl.inside(P.free, path[-1], 1e-6))
path, _ = P.route((0.3, 0.3), (0.6, 0.55))
check("planner: a goal inside the no-go zone is pushed out", not pl.inside(P.block, path[-1]))
plan = pl.Planner(ARENA).update({"follower": {"x": 0.2, "y": 0.3, "yaw": 0.0}, "A": {"x": 0.2, "y": 0.9}}, 0.0)
check("planner: opponent directly LEFT gives yaw +90 (left is positive)", abs(plan.yaw_to_target - 90) < 1e-6)
plan = pl.Planner(ARENA).update({"follower": {"x": 0.3, "y": 0.55, "yaw": 0.0}, "A": {"x": 0.9, "y": 0.55}}, 0.0)
check("planner: opponent across the no-go square gives a detour (blocked, waypoint yaw != target yaw)",
      plan.blocked and abs(plan.yaw_to_waypoint - plan.yaw_to_target) > 20)
t0 = time.time(); Pn = pl.Planner(ARENA)
for i in range(1000):
    Pn.update({"follower": {"x": 0.2 + 0.0003 * i, "y": 0.3, "yaw": 10.0}, "A": {"x": 1.0 - 0.0002 * i, "y": 0.8}, "B": {"x": 0.9, "y": 0.2}}, i / 30.0)
check("planner: fast enough for every frame (< 5 ms per update)", (time.time() - t0) / 1000 < 0.005, "%.2f ms" % ((time.time() - t0)))

# ------------------------------------------------------------------------------------------------ lidar chaser detector
N, HZ = 4, 30
nan = float("nan")


def lframe(bg=nan, objs=()):
    f = np.full((N, N), bg, float)
    for (rr, cc), d in objs:
        f[rr, cc] = d
    return f


def run_det(gen, secs, own_fwd=0.0, rotating=False, det=None):
    det = det or ev.ChaserDetector(N)
    first = None
    for i in range(int(secs * HZ)):
        t = i / HZ
        st = det.update(gen(t), t, own_fwd(t) if callable(own_fwd) else own_fwd, rotating, wall=t)
        if st.active and first is None:
            first = (t, st)
    return first, det


first, _ = run_det(lambda t: lframe(bg=600), 3)
check("lidar: a wall behind a still rover is not a chaser", first is None)
first, _ = run_det(lambda t: lframe(objs=[((1, 0), 1000 - 250 * t), ((2, 0), 1000 - 250 * t)]), 4)
check("lidar: chaser closing at 250 mm/s is caught on the correct side (left column -> +offset)", first and first[1].offset_deg > 10, f"offset {first[1].offset_deg:+.1f}" if first else "missed")
check("lidar: ... early enough to turn round (distance when caught >= pin reach + 250 mm/s x spin time)",
      first and (1000 - 250 * first[0]) > ev.POP_REACH_MM + 250 * ev.turn_time_s(170), f"caught at {1000 - 250 * first[0]:.0f} mm" if first else "")
first, _ = run_det(lambda t: lframe(objs=[((1, 1), 400 + 200 * t)]), 2, own_fwd=200.0)
check("lidar: rover driving away from a still object is not a chaser", first is None)
first, _ = run_det(lambda t: lframe(objs=[((1, 2), 1100 - 200 * t)]), 4, own_fwd=200.0)
check("lidar: chaser faster than the rover's own retreat is still caught", first is not None)
first, _ = run_det(lambda t: lframe(bg=700 + 200 * math.sin(6 * t)), 3, rotating=True)
check("lidar: scenery sweeping past a spinning rover is not a chaser", first is None)
bal = lambda t: lframe(objs=[((2, 1), 90)])
det2 = ev.ChaserDetector(N)
mask = det2.learn_mask([bal(0)] * 30)
check("lidar: learn_mask finds your own balloon", np.argwhere(mask).tolist() == [[2, 1]])
first, _ = run_det(bal, 2, det=det2)
check("lidar: the masked balloon no longer triggers", first is None)
first, _ = run_det(lambda t: lframe(objs=[((2, 1), 90), ((1, 3), 1000 - 250 * t)]), 4, det=det2)
check("lidar: a real chaser is still caught next to the masked balloon (right column -> -offset)", first and first[1].offset_deg < -10)

# ------------------------------------------------------------------------------------------------ external-camera threats
rng = np.random.default_rng(1)


def sim_ext(follower_fn, opp_fns, secs, noise=0.005, hz=30):
    w = ev.ExternalWatch()
    first = None
    for i in range(int(secs * hz)):
        t = i / hz
        fx, fy, fyaw = follower_fn(t)
        f = {"x": fx + rng.normal(0, noise), "y": fy + rng.normal(0, noise), "yaw": fyaw + rng.normal(0, 1.0)}
        o = {k: (fn(t)[0] + rng.normal(0, noise), fn(t)[1] + rng.normal(0, noise)) for k, fn in opp_fns.items()}
        st = w.update(f, o, t, wall=t)
        if st.active and first is None:
            first = (t, st)
    return first


still = lambda t: (0.0, 0.0, 0.0)


def approach(start, speed, stop_at=0.2):
    d = math.hypot(*start); tmax = (d - stop_at) / speed
    return lambda t: (start[0] - start[0] / d * speed * min(t, tmax), start[1] - start[1] / d * speed * min(t, tmax))


first = sim_ext(still, {"A": approach((-1.0, 0.5), 0.3)}, 5)
check("external: opponent coming from behind-LEFT: turn is about +153 (left)", first and 140 < first[1].turn_deg < 165, f"{first[1].turn_deg:+.0f}" if first else "missed")
first = sim_ext(still, {"B": approach((-1.0, -0.5), 0.3)}, 5)
check("external: opponent coming from behind-RIGHT: turn is about -153", first and -165 < first[1].turn_deg < -140)
first = sim_ext(still, {"A": approach((0.0, 1.0), 0.3)}, 5)
check("external: an attack from the SIDE (invisible to the rear lidar) is caught: turn about +90", first and 75 < first[1].turn_deg < 105)
first = sim_ext(still, {"A": approach((1.0, 0.0), 0.3)}, 5)
ring_t = (1.0 - (ev.CLOSE_RING_M - ev.BALLOON_BACK_M)) / 0.3          # when its balloon first comes inside the close ring
check("external: an opponent coming from the FRONT is left to chase() until it is inside the close ring, then it is "
      "tracked head-on (already facing it, so no turn)",
      first is None or (first[0] >= ring_t - 0.2 and abs(first[1].turn_deg) < 15), f"first flagged at {first[0]:.2f}s (ring at {ring_t:.2f}s)" if first else "never flagged")
first = sim_ext(still, {"A": lambda t: (-1.0 + 0.3 * t, 0.5)}, 6)
check("external: an opponent passing by that is not aimed at you is not a threat", first is None)
bad = sum(1 for pos in ((-0.6, 0.0), (-0.6, 0.5), (0.0, -0.7))
          if sim_ext(still, {"A": lambda t, p=pos: p}, 20, noise=0.008) is not None)
check("external: stationary opponents 0.6+ m away raise no alarm even with 8 mm noise", bad == 0)
first = sim_ext(lambda t: (0.0, 0.0, 180.0), {"A": approach((-1.0, 0.5), 0.3)}, 5)
check("external: 'behind' follows the follower's heading (an opponent ahead of a rover facing -x is not a threat until it is "
      "inside the close ring)", first is None or first[0] >= ring_t - 0.3,
      f"first flagged at {first[0]:.2f}s" if first else "never flagged")
w = ev.ExternalWatch()
for i in range(8):
    w.update({"x": 0, "y": 0, "yaw": 0}, {"A": (-0.1, 0)}, i / 30, wall=100.0 + i / 30)
check("external: data older than the stale limit is ignored", w.state(100.2).active and not w.state(101.5).active)
first = sim_ext(still, {"A": lambda t: (0.8, 0.1), "B": approach((-1.0, -0.4), 0.3)}, 5)
check("external: with two opponents it reports the one that is coming at you", first and first[1].who == "B")

# ------------------------------------------------------------------------------------------------ evade plan, fusion, speed cap
class M:
    def turn_degrees(self, d, sp): pass
    def stop(self): pass


e = ev.EvadeController(M(), read_ir=lambda: (30.0, 12.0))
check("evade: turn direction follows the side of the chaser (lidar offset +22.5 -> +157.5, -22.5 -> -157.5)",
      abs(e.plan(ev.Threat(True, 22.5)) - 157.5) < 1e-6 and abs(e.plan(ev.Threat(True, -22.5)) + 157.5) < 1e-6)
e = ev.EvadeController(M(), read_ir=lambda: (10.0, 40.0))
check("evade: dead behind, it turns toward the side with more room (IR)", e.plan(ev.Threat(True, 0.0, turn_deg=179.0)) == -180.0)
L = ev.Threat(True, 22.5, 300, 1.2, [], None, "lidar", "")
E = ev.Threat(True, 0, 400, 1.8, [], 150.0, "external", "B")
check("evade: fusion: either source triggers; the camera's exact bearing wins when both agree",
      ev.Brain.fuse(L, ev.Threat()).source == "lidar" and ev.Brain.fuse(ev.Threat(), E).source == "external"
      and ev.Brain.fuse(L, E).source == "both" and ev.Brain.fuse(L, E).turn_deg == 150.0 and not ev.Brain.fuse(ev.Threat(), ev.Threat()).active)
check("evade: warning time grows with the turn needed and shrinks never below latency + margin",
      ev.lead_time_s(180) > ev.lead_time_s(90) > ev.EVADE_LATENCY_S + ev.TTC_MARGIN_S)
sp = ev.evade_speed()
check("evade: the spin-time model matches the micromelon module's own turn calibration",
      all(abs(ev.turn_time_s(d) - __import__("micromelon.motors._motors", fromlist=["x"])._calcMotorSpeedsAndTime(sp, 0, d)["seconds"]) < 0.05 for d in (90, 153, 180)))

from micromelon import RoverController, Motors
from micromelon._robot_comms import MicromelonType as OPTYPE
seen = []
rc = RoverController()
rc.writeAttribute = lambda op, data, timeout=None: seen.append(list(data)) or True
rc.doMotorOperation = lambda op, data, timeout=None: seen.append(list(data))
tap = ev.MotionTap(OPTYPE.MOTOR_SET)
tap.install(rc)
spd = lambda b: (b - 256 if b > 127 else b) / 127 * 30
old_cap = ev.MAX_SPEED_CM_S
try:
    ev.set_max_speed(30)
    Motors.write(30, -30)
    check("speed cap: 30 cm/s passes through unchanged", abs(spd(seen[-1][0]) - 30) < 0.5 and abs(spd(seen[-1][1]) + 30) < 0.5)
    ev.set_max_speed(15)
    Motors.write(30, 30)
    check("speed cap: write(30,30) is clamped to 15 and the motion tap sees 15", abs(spd(seen[-1][0]) - 15) < 0.5 and abs(tap.forward_mm_s - 150) < 5)
    Motors.write(-25, 10)
    check("speed cap: values under the cap are kept (10 stays 10, -25 becomes -15)", abs(spd(seen[-1][1]) - 10) < 0.5 and abs(spd(seen[-1][0]) + 15) < 0.5)
    Motors.moveDistance(20, 30, 20, 30)
    check("speed cap: distance moves keep their distance but slow down", abs(spd(seen[-1][0]) - 15) < 0.5 and seen[-1][2:6] == [200, 0, 200, 0])
    Motors.turnDegrees(-30)
    check("motion tap: a turnDegrees call is seen as 'rotating' while it runs and not after", not tap.rotating)
    ev.set_max_speed(99)
    check("speed cap: values above the rover's limit are held at 30", ev.MAX_SPEED_CM_S == 30.0)
finally:
    ev.set_max_speed(old_cap)

# ------------------------------------------------------------------------------------------------ lidar packets + UDP
vals = [None if i % 5 == 0 else 100 + 37 * i for i in range(16)]
pkt = lx.make_packet(513, 123456, vals)
seq, ms, v = lx.parse_packet(pkt)
check("lidar_rx: packet round trip (sequence, time, distances, invalid zones)", seq == 513 and ms == 123456 and np.allclose(v, [np.nan if x is None else x for x in vals], equal_nan=True))
check("lidar_rx: junk and truncated packets are rejected", lx.parse_packet(b"hello") is None and lx.parse_packet(pkt[:-3]) is None and lx.parse_packet(b"XX" + pkt[2:]) is None)
got = []
rx = lx.LidarReceiver(0, lambda vals, t: got.append((vals.copy(), t)), bind="127.0.0.1")
tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
for i in range(20):
    tx.sendto(lx.make_packet(i, i * 33, [500] * 16), ("127.0.0.1", rx.port)); time.sleep(0.01)
time.sleep(0.2); rx.close()
check("lidar_rx: UDP receiver delivers every frame", len(got) == 20 and abs(got[-1][1] - 19 * 0.033) < 1e-6, f"{len(got)}/20")

# ------------------------------------------------------------------------------------------------ onboard camera maths
import cv2
Wd, Hd, FOV = 640, 480, 65.0
fpx = (Wd / 2) / math.tan(math.radians(FOV / 2))


def render(theta, bgr=(255, 60, 0)):
    img = np.full((Hd, Wd, 3), 90, np.uint8)
    cv2.circle(img, (int(round(Wd / 2 - fpx * math.tan(math.radians(theta)))), 240), 14, bgr, -1)
    return img


def onboard_yaw(theta, fov=FOV, bgr=(255, 60, 0)):
    tr = ob.OnboardTracker(fov)
    img = render(theta, bgr)
    x = int(round(Wd / 2 - fpx * math.tan(math.radians(theta))))
    h, s, vv = np.median(cv2.cvtColor(img[237:244, x - 3:x + 4], cv2.COLOR_BGR2HSV).reshape(-1, 3), axis=0)
    tr.set_color(h, s, vv)
    return tr.update(img, now=1.0)


check("onboard: angle is exact with the right FOV (worst error < 0.3 deg over -28..+28)", max(abs(onboard_yaw(t) - t) for t in range(-28, 29, 4)) < 0.3)
check("onboard: left is positive, right negative", onboard_yaw(20) > 15 and onboard_yaw(-20) < -15)
check("onboard: red (hue wrap-around) and green balloons work too", abs(onboard_yaw(15, bgr=(0, 0, 230)) - 15) < 0.5 and abs(onboard_yaw(15, bgr=(0, 200, 0)) - 15) < 0.5)
tr = ob.OnboardTracker(); tr.set_color(113, 255, 255)
a = tr.update(render(10), now=0.0); b = tr.update(np.full((Hd, Wd, 3), 90, np.uint8), now=0.2); c = tr.update(np.full((Hd, Wd, 3), 90, np.uint8), now=1.0)
check("onboard: a lost balloon is held briefly (0.2 s) then reported as None", abs(a - 10) < 0.5 and b is not None and c is None)


# ------------------------------------------------------------------------------------------------ new defence logic
# an opponent that STOPS moving is not a threat once it has been still for ACTIVE_HOLD_S; one that paused recently still is
def hover(t):
    return (-0.35, 0.0) if t < 1.0 else (-0.35, 0.0)
w = ev.ExternalWatch(); hits = []
for i in range(int(8 * 15)):
    t = i / 15
    f = {"x": 0.0, "y": 0.0, "yaw": 0.0}
    o = (-0.55 + 0.20 * min(t, 1.0), 0.0) if t < 8 else (-0.35, 0.0)          # moves in for 1 s, then stands still
    th = w.update(f, {"A": o}, t, wall=t); hits.append((t, th.active))
act_early = any(a for t, a in hits if 1.0 < t < 3.5); act_late = any(a for t, a in hits if t > 5.5)
check("defence: a hunter that has moved recently stays a threat while it pauses close by (ACTIVE_HOLD_S)", act_early)
check("defence: ... and stops being one once it has stood still for a few seconds", not act_late)
# running away from a slower hunter is allowed, but not when it is very close
def flee(gap, v_us=0.25, v_it=0.11):
    w = ev.ExternalWatch(); fl = []
    for i in range(int(6 * 15)):
        t = i / 15
        f = {"x": v_us * t, "y": 0.0, "yaw": 0.0}                                 # we drive forward (+x) at 25 cm/s
        o = (-gap + v_it * t, 0.0)                                                # it follows 'gap' behind at 11 cm/s
        th = w.update(f, {"B": o}, t, wall=t); fl.append(th.active)
    return fl
far = flee(0.62); near = flee(0.28)
check("defence: running away from a slower hunter that is 0.5+ m behind is NOT treated as a threat", not any(far[30:]), f"{sum(far[30:])} flagged frames")
check("defence: ... but a hunter inside HARD_RING_M still is, however you move", sum(near[:20]) > 5)
# the facing tracker: converges on a still opponent and keeps up with one circling at 30 deg/s
class W:
    def __init__(self): self.l = self.r = 0.0
    def write(self, l, r): self.l, self.r = l, r
    def stop(self): self.l = self.r = 0.0
def face_sim(circle_deg_s, secs=8.0, theta0=60.0, noise=4.0, lat=0.2):
    wm = W(); ft = ev.FacingTracker(wm); rng = np.random.default_rng(3)
    yaw_err = theta0; hist = []; dt = 0.05; errs = []
    for i in range(int(secs / dt)):
        t = i * dt
        w_deg = np.degrees((wm.r - wm.l) / ev.WHEELBASE_CM)                       # actual turn rate, deg/s (+ left)
        yaw_err += (circle_deg_s - w_deg) * dt                                    # bearing to the opponent relative to our heading
        hist.append((t, yaw_err))
        meas = [h for h in hist if h[0] <= t - lat]
        m = (meas[-1][1] if meas else theta0) + rng.normal(0, noise)
        if i % 3 == 0: ft_in = m                                                  # a new camera reading at ~7 Hz... held in between
        ft.step(ft_in if i else m, t)
        errs.append(abs(yaw_err))
    return np.mean(errs[-40:])
check("facing tracker: turns to face a still opponent from 60 deg off (final error under 8 deg)", face_sim(0.0) < 8, "%.1f deg" % face_sim(0.0))
check("facing tracker: keeps within 20 deg of an opponent circling at 30 deg/s with 4 deg of noise and 0.2 s delay", face_sim(30.0) < 20, "%.1f deg" % face_sim(30.0))

print("\nSUMMARY: %d/%d passed" % (sum(1 for r_ in results if r_[1]), len(results)))
failed = [r_ for r_ in results if not r_[1]]
for r_ in failed:
    print("   FAILED:", r_[0], r_[2])
sys.exit(1 if failed else 0)
