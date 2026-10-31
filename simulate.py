"""
simulate.py - run the WHOLE system in simulation and produce visual proof.

    python simulate.py                 run every scenario, check them, write sim_output/ (video, frames, plots)
    python simulate.py chase_around    run one scenario
    python simulate.py --no-video      skip the video (faster)

The real code under test: tracker.py (reads rendered camera images), plan.py, evade.py, pipeline.py, and the real
micromelon Motors module on top of a simulated rover link. Only the world, the sensors and the rover are simulated.
"""
import math
import os
import sys
import types

import cv2
import numpy as np

import evade as ev
import example_chase
import follow
import pipeline
import plan as pl
import sim_world as sw
import tracker as tk

CAM_LATENCY = 0.15
LIDAR_LATENCY = 0.10
OUT = "sim_output"


class SimEnd(BaseException):
    pass


def apply_measurements():
    """The numbers you would measure on the real rovers, for the simulated geometry (balloon 10 cm behind the
    centre, pin tip 10 cm ahead). See README 'The pop rule'."""
    ev.BALLOON_BACK_M = 0.10     # your balloon is 10 cm behind your centre
    ev.PIN_FROM_BALLOON_M = 0.20 # a rover's pin tip is 20 cm ahead of its own balloon
    ev.PIN_REACH_M = 0.10        # your centre -> their balloon when your pin touches it
    pl.BEHIND_M = ev.PIN_REACH_M
    ev.POP_REACH_MM = 80         # what the rear lidar reads at that moment
    ev.EVADE_LATENCY_S = 0.4     # camera 0.15 + lidar/network + Bluetooth (how long chase() blocks is added automatically)
    ev.EVADE_COOLDOWN_S = 1.2


class Sim:
    def __init__(self, scenario, seed=0, evade_on=True, max_speed=30.0, planning=True, camera_drift=True,
                 recorder=None, label=None, target="nearest", floor=False):
        apply_measurements()
        ev.set_max_speed(max_speed)
        self.label = label or scenario["name"]
        self.recorder = recorder
        self.rng = np.random.default_rng(seed + 100)
        f = sw.Agent("follower", *scenario["follower"], gains=(1.0, 0.985))
        opps = [sw.Opponent(n, x, y, yaw, kind, speed=spd, path=path, seed=seed + i)
                for i, (n, x, y, yaw, kind, spd, path) in enumerate(scenario["opponents"])]
        self.world = sw.World(f, opps)
        self.rig = sw.CameraRig(70.0, drift=camera_drift)
        self.renderer = sw.Renderer(seed=seed + 1)
        self.tr = tk.Tracker(size=sw.SIZE, fov_deg=70.0)
        self.scn = scenario
        self.tick_n = 0
        self.log = []
        self.cam_log = []
        self.mode = "chase"
        self.end_reason = None
        self.last_frame = None
        self.last_state = {}
        self.max_speed = max_speed
        self.evade_on = evade_on

        self.motors = sw.SimMotors(self)
        self.motors.__enter__()
        self.detector = ev.ChaserDetector(ev.GRID)
        self.ext = ev.ExternalWatch()
        self.planner = pl.Planner(pl.Arena(sw.ARENA, sw.NOGO), target) if planning else None
        self.pipe = pipeline.Pipeline(self.detector, self.ext, self.planner, example_chase.chase,
                                      follow.MicromelonMotors(), lambda: sw.ir_cm(self.world.snapshot_at(self.world.t)),
                                      clock=lambda: self.world.t,
                                      motion=lambda: (self.motors.tap.left, self.motors.tap.right),
                                      read_floor=(lambda: sw.floor_flags(self.world.follower)) if floor else None)
        if not evade_on:
            self.pipe.evader.ready = lambda now: False              # evasion switched off for the comparison
        handle, chase = self.pipe.evader.handle, self.pipe.brain.chase

        def handle_wrap(threat, now):
            self.mode = "evade"
            self.evade_t = (self.world.t, threat)
            try:
                return handle(threat, now)
            finally:
                self.mode = "chase"

        self.pipe.evader.handle = handle_wrap
        self.pipe.brain.chase = chase
        self._calibrate()

    # ------------------------------------------------------------------------------------ setup
    def _calibrate(self):
        snap0 = self.world.snapshot_at(0.0)
        cam0 = self.rig.cam(0.0)
        sim = self

        class Cap:
            def read(self):
                return True, sim.renderer.render(snap0, cam0)

        if not self.tr.lock_floor(Cap(), n=10):
            raise RuntimeError("simulated camera could not see the floor marker")
        self.tr._floor_t = 0.0
        frames = [sw.lidar_frame(snap0, self.rng) for _ in range(40)]
        self.detector.learn_mask(frames)                             # masks the follower's own balloon

    # ------------------------------------------------------------------------------------ rover link
    def set_wheels(self, l, r):
        self.world.follower.vl, self.world.follower.vr = l, r

    def advance(self, seconds):
        for _ in range(max(1, int(round(seconds / self.world.dt)))):
            self.tick()

    # ------------------------------------------------------------------------------------ one simulation step
    def tick(self):
        w = self.world
        w.step()
        t = w.t
        self.tick_n += 1
        if self.tick_n % 2 == 0:                                      # camera at 15 Hz
            sn = w.snapshot_at(t - CAM_LATENCY)
            self.last_frame = self.renderer.render(sn, self.rig.cam(sn.t))
            self.last_state = self.tr.update(self.last_frame, now=t)
            self.pipe.on_state(self.last_state, t)
            self.cam_log.append((self.last_state, sn.agents))             # estimate vs the truth AT THE IMAGE TIME
        if self.tick_n % (30 // ev.LIDAR_HZ) == 0:                    # the lidar: 30 Hz for 4x4, 15 Hz for 8x8
            sn2 = w.snapshot_at(t - LIDAR_LATENCY)
            tap = self.motors.tap
            self.pipe.on_lidar(sw.lidar_frame(sn2, self.rng), sn2.t, tap.forward_mm_s, tap.rotating, t)
        self._log(t)
        if self.recorder and self.tick_n % 2 == 0:
            self.recorder.add(self)
        if w.follower.popped:
            raise SimEnd("follower popped")
        if all(o.popped for o in w.opps):                             # the game is won: give the rover 3 s to notice and stop
            self.all_popped_t = getattr(self, "all_popped_t", t)
            if t - self.all_popped_t >= 3.0:
                raise SimEnd("all opponents popped")
        if self.scn.get("until") and self.scn["until"](self):
            raise SimEnd("goal reached")

    def _log(self, t):
        w = self.world
        f = w.follower
        est = self.last_state
        row = dict(t=t, x=f.x, y=f.y, yaw=math.degrees(f.yaw), mode=self.mode, vl=f.vl * f.gl, vr=f.vr * f.gr,
                   clear=sw.dist_point_poly(f.x, f.y, sw.NOGO),
                   lidar=self.detector.state(t).active, ext=self.ext.state(t).active,
                   yaw_cmd=self.pipe.yaw_to_leader,
                   opp={o.name: (o.x, o.y, o.balloon[0], o.balloon[1], o.popped) for o in w.opps})
        fe = est.get("follower")
        row["est"] = None if fe is None else (fe["x"], fe["y"], fe["yaw"])
        row["est_opp"] = {k: (v["x"], v["y"]) for k, v in est.items() if k != "follower"}
        row["threat_d"] = min([math.hypot(o.pin[0] - f.balloon[0], o.pin[1] - f.balloon[1]) for o in w.opps if not o.popped] or [9.0])
        self.log.append(row)

    # ------------------------------------------------------------------------------------ run + summary
    def run(self, duration):
        next_brain = 0.0
        try:
            while self.world.t < duration:
                self.tick()
                if self.world.t >= next_brain:
                    self.pipe.step()
                    next_brain = self.world.t + 0.05
            self.end_reason = "time limit"
        except SimEnd as e:
            self.end_reason = str(e)
        finally:
            self.motors.__exit__(None, None, None)
        return self.summary()

    def summary(self):
        w = self.world
        L = self.log
        pos_err, yaw_err, opp_err = [], [], []
        for est, truth in self.cam_log:                      # tracking accuracy, measured against the truth at the image time
            fe = est.get("follower")
            if fe is not None:
                tx, ty, tyaw, _ = truth["follower"]
                pos_err.append(math.hypot(fe["x"] - tx, fe["y"] - ty))
                yaw_err.append(abs(sw.wrap_deg(fe["yaw"] - math.degrees(tyaw))))
            for k, v in est.items():
                if k != "follower" and k in truth and not truth[k][3]:
                    ax, ay, ayaw, _ = truth[k]
                    bx, by = ax - sw.BALLOON_BACK * math.cos(ayaw), ay - sw.BALLOON_BACK * math.sin(ayaw)
                    opp_err.append(math.hypot(v["x"] - bx, v["y"] - by))
        pops = [e for e in w.events]
        s = dict(label=self.label, end=self.end_reason, duration=round(w.t, 1), follower_popped=w.follower.popped,
                 events=pops, nogo_frames=w.nogo_viol, tape_frames=w.tape_viol, min_nogo_clear=round(w.min_nogo_clear, 3),
                 evades=len(self.pipe.evader.log), max_wheel=round(max(max(abs(r["vl"]), abs(r["vr"])) for r in L), 1),
                 track_frac=round(sum(1 for e, _ in self.cam_log if "follower" in e) / max(len(self.cam_log), 1), 3),
                 pos_err_mm=(round(float(np.mean(pos_err)) * 1000, 1), round(float(np.percentile(pos_err, 95)) * 1000, 1)) if pos_err else None,
                 yaw_err_deg=(round(float(np.mean(yaw_err)), 2), round(float(np.percentile(yaw_err, 95)), 1)) if yaw_err else None,
                 opp_err_mm=(round(float(np.mean(opp_err)) * 1000, 1), round(float(np.percentile(opp_err, 95)) * 1000, 1)) if opp_err else None)
        return s


# ------------------------------------------------------------------------------------------------- scenarios
def popped(name):
    return lambda sim: any(o.name == name and o.popped for o in sim.world.opps)


SCENARIOS = {
    # the straight line to the target crosses the no-go square; the target patrols along the far side
    "chase_around": dict(name="chase_around", duration=120.0,
                         follower=(0.45, 0.30, 0.0),
                         opponents=[("A", 1.25, -0.35, 90.0, "patrol", 0.08, [(1.25, 0.35), (1.25, -0.35)])],
                         until=popped("A")),
    # follower starts right beside the no-go zone with the target on the far side of it
    "start_beside_nogo": dict(name="start_beside_nogo", duration=60.0,
                              follower=(0.80, -0.22, 90.0),
                              opponents=[("A", 0.50, 0.45, 0.0, "patrol", 0.08, [(1.10, 0.45), (0.50, 0.45)])],
                              until=popped("A")),
    # B hunts the follower from behind while the follower chases A (a still target whose back faces away from us)
    "hunter": dict(name="hunter", duration=60.0,
                   follower=(0.45, 0.30, 0.0),
                   opponents=[("A", 1.25, -0.30, 90.0, "static", 0.0, None),
                              ("B", 0.30, -0.45, 0.0, "hunter", 0.11, None)],
                   until=popped("A")),
    # B just wanders: it drives to random points round the arena and never aims at the follower. Shows how the follower's
    # path changes when an opponent crosses it by chance (compare with the "hunter" scenario, where B aims at it)
    "wanderer": dict(name="wanderer", duration=60.0,
                     follower=(0.45, 0.30, 0.0),
                     opponents=[("A", 1.25, -0.30, 90.0, "static", 0.0, None),
                                ("B", 0.30, -0.45, 0.0, "wander", 0.11, None)],
                     until=popped("A")),
    # three non-aggressive (static) opponents with the camera drifting: nothing should trigger an evasion, and
    # all three balloons should be tracked
    "three_calm": dict(name="three_calm", duration=60.0,
                       follower=(0.45, 0.30, 0.0),
                       opponents=[("A", 1.25, -0.30, 90.0, "static", 0.0, None),
                                  ("B", 0.45, -0.40, 0.0, "static", 0.0, None),
                                  ("C", 1.20, 0.45, 180.0, "static", 0.0, None)],
                       until=None),
}


# ------------------------------------------------------------------------------------------------- visuals
def draw_map(sim, size=(560, 450)):
    W, H = size
    s = 360.0
    ox = (W - 1.2 * s) / 2.0
    oy = (H - 1.13 * s) / 2.0
    xmin, ymax = sw.ARENA_RECT[0], sw.ARENA_RECT[3]
    P = lambda x, y: (int(round(ox + (x - xmin) * s)), int(round(oy + (ymax - y) * s)))
    img = np.full((H, W, 3), (30, 26, 22), np.uint8)
    cv2.fillPoly(img, [np.array([P(*p) for p in sw.ARENA], np.int32)], (52, 46, 40))
    a = sw.ARENA
    for (p, q), col in zip(zip(a, a[1:] + a[:1]), [(55, 55, 215), (6, 6, 6), (55, 55, 215), (6, 6, 6)]):
        cv2.line(img, P(*p), P(*q), col, 5, cv2.LINE_AA)
    if sim.planner is not None:                                  # where the planner will not send the follower: the tape and the
        for poly, col in ((sim.planner.free, (110, 110, 110)), (sim.planner.block, (110, 110, 190))):    # no-go zone, each grown by the robot's radius + margin
            pts = np.array([P(*p) for p in poly], np.int32)
            for i in range(len(pts)):
                a, b = pts[i], pts[(i + 1) % len(pts)]
                for u in np.arange(0.0, 1.0, 0.25):
                    p0 = (a + (b - a) * u).astype(int); p1 = (a + (b - a) * min(u + 0.12, 1.0)).astype(int)
                    cv2.line(img, tuple(p0), tuple(p1), col, 1, cv2.LINE_AA)                           # dashed outline
    ng = np.array([P(*p) for p in sw.NOGO], np.int32)
    cv2.fillPoly(img, [ng], (60, 60, 140))
    for i in range(-8, 12):
        cv2.line(img, (ng[:, 0].min() + i * 7, ng[:, 1].max()), (ng[:, 0].min() + i * 7 + 52, ng[:, 1].min()), (120, 120, 200), 1)
    cv2.polylines(img, [ng], True, (150, 150, 255), 2, cv2.LINE_AA)
    w, t = sim.world, sim.world.t
    L = sim.log
    # the whole path so far (thin) ...
    for name, col in (("follower", (255, 255, 255)), ("A", sw.BGR["A"]), ("B", sw.BGR["B"]), ("C", sw.BGR["C"])):
        pts = []
        for r in L[::8]:
            if name == "follower":
                pts.append(P(r["x"], r["y"]))
            elif name in r["opp"]:
                pts.append(P(r["opp"][name][0], r["opp"][name][1]))
        if len(pts) > 1 and name in ("follower", "B"):
            cv2.polylines(img, [np.array(pts, np.int32)], False, tuple(int(c * 0.45) for c in col), 1, cv2.LINE_AA)
    # trails (last 4 s)
    for name, col in (("follower", (255, 255, 255)), ("A", sw.BGR["A"]), ("B", sw.BGR["B"]), ("C", sw.BGR["C"])):
        pts = []
        for r in L[-120::4]:
            if name == "follower":
                pts.append(P(r["x"], r["y"]))
            elif name in r["opp"]:
                pts.append(P(r["opp"][name][0], r["opp"][name][1]))
        if len(pts) > 1:
            cv2.polylines(img, [np.array(pts, np.int32)], False, tuple(int(c * 0.6) for c in col), 1, cv2.LINE_AA)
    f = w.follower
    # lidar wedge (rear 60 degrees)
    lx, ly = f.x - 0.05 * math.cos(f.yaw), f.y - 0.05 * math.sin(f.yaw)
    wedge = [P(lx, ly)] + [P(lx + 0.6 * math.cos(f.yaw + math.pi + math.radians(d)), ly + 0.6 * math.sin(f.yaw + math.pi + math.radians(d))) for d in range(-30, 31, 6)]
    ov = img.copy()
    cv2.fillPoly(ov, [np.array(wedge, np.int32)], (60, 60, 230) if sim.detector.state(t).active else (150, 190, 60))
    img = cv2.addWeighted(ov, 0.28, img, 0.72, 0)
    # planned path
    p = sim.pipe.last_plan
    if p and p.path:
        pts = np.array([P(*q) for q in p.path], np.int32)
        cv2.polylines(img, [pts], False, (255, 184, 143), 2, cv2.LINE_AA)
        cv2.circle(img, P(*p.waypoint), 6, (255, 184, 143), 2, cv2.LINE_AA)
        cv2.drawMarker(img, P(*p.goal), (255, 184, 143), cv2.MARKER_CROSS, 12, 2, cv2.LINE_AA)
    # robots
    for ag in w.agents:
        c, sn = math.cos(ag.yaw), math.sin(ag.yaw)
        body = np.array([P(ag.x + dx * c - dy * sn, ag.y + dx * sn + dy * c) for dx, dy in
                         [(-sw.BODY_L / 2, -sw.BODY_W / 2), (sw.BODY_L / 2, -sw.BODY_W / 2), (sw.BODY_L / 2, sw.BODY_W / 2), (-sw.BODY_L / 2, sw.BODY_W / 2)]], np.int32)
        cv2.fillPoly(img, [body], (130, 136, 146))
        cv2.polylines(img, [body], True, (255, 255, 255) if ag.name == "follower" else (60, 64, 72), 2, cv2.LINE_AA)
        cv2.line(img, P(ag.x + 0.06 * c, ag.y + 0.06 * sn), P(*ag.pin), (240, 240, 240), 2, cv2.LINE_AA)
        bx, by = ag.balloon
        if ag.popped:
            q = P(bx, by)
            cv2.line(img, (q[0] - 8, q[1] - 8), (q[0] + 8, q[1] + 8), (90, 90, 255), 3)
            cv2.line(img, (q[0] + 8, q[1] - 8), (q[0] - 8, q[1] + 8), (90, 90, 255), 3)
        else:
            cv2.circle(img, P(bx, by), int(sw.BALLOON_R * s * 0.8), sw.BGR[ag.name], -1, cv2.LINE_AA)
        cv2.putText(img, ag.name[0].upper(), (P(ag.x, ag.y)[0] - 5, P(ag.x, ag.y)[1] + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (20, 20, 20), 1, cv2.LINE_AA)
    # tracker estimates (so you can see how close the REAL tracker gets to the truth)
    est = sim.last_state
    if "follower" in est:
        cv2.drawMarker(img, P(est["follower"]["x"], est["follower"]["y"]), (255, 255, 0), cv2.MARKER_CROSS, 14, 1, cv2.LINE_AA)
    for k, v in est.items():
        if k != "follower":
            cv2.circle(img, P(v["x"], v["y"]), 9, sw.BGR[k], 1, cv2.LINE_AA)
    # external threat
    es = sim.ext.state(t)
    if es.active and es.who:
        for o in w.opps:
            if o.name == es.who:
                cv2.circle(img, P(*o.balloon), 14, (91, 107, 255), 2, cv2.LINE_AA)
                cv2.line(img, P(*o.balloon), P(f.x, f.y), (91, 107, 255), 2, cv2.LINE_AA)
    src = []
    if sim.detector.state(t).active:
        src.append("lidar")
    if es.active:
        src.append("camera")
    col = (80, 80, 255) if sim.mode == "evade" else (120, 220, 120)
    cv2.putText(img, f"t={t:5.1f}s  {sim.mode.upper()}", (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2, cv2.LINE_AA)
    cv2.putText(img, "threat: " + (" + ".join(src) if src else "none"), (10, 44), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA)
    cv2.putText(img, f"speed cap {ev.MAX_SPEED_CM_S:.0f} cm/s", (10, H - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (160, 160, 160), 1, cv2.LINE_AA)
    cv2.putText(img, "dashed = keep-out (route limits)", (10, H - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 150, 200), 1, cv2.LINE_AA)
    cv2.putText(img, "TRUTH (cross/circles = what the tracker sees)", (170, H - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (160, 160, 160), 1, cv2.LINE_AA)
    for i, (te, txt) in enumerate(w.events[-3:]):
        cv2.putText(img, f"{te:5.1f}s  {txt.upper()}", (W - 230, 22 + 20 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (90, 90, 255), 2, cv2.LINE_AA)
    return img


def draw_timeline(sim, size, duration):
    W, H = size
    img = np.full((H, W, 3), (24, 20, 18), np.uint8)
    x0, x1 = 60, W - 20
    X = lambda t: int(x0 + (x1 - x0) * min(t / duration, 1.0))
    rows = {"mode": 30, "lidar": 56, "camera": 78}
    for name, y in rows.items():
        cv2.putText(img, name, (6, y + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (170, 170, 170), 1, cv2.LINE_AA)
        cv2.line(img, (x0, y + 10), (x1, y + 10), (50, 50, 50), 1)
    for r in sim.log[::2]:
        x = X(r["t"])
        cv2.line(img, (x, rows["mode"] - 8), (x, rows["mode"] + 8), (80, 80, 255) if r["mode"] == "evade" else (110, 190, 110), 2)
        if r["lidar"]:
            cv2.line(img, (x, rows["lidar"] - 6), (x, rows["lidar"] + 6), (200, 200, 60), 2)
        if r["ext"]:
            cv2.line(img, (x, rows["camera"] - 6), (x, rows["camera"] + 6), (255, 160, 90), 2)
    for te, txt in sim.world.events:
        cv2.line(img, (X(te), 8), (X(te), H - 8), (90, 90, 255), 1)
        cv2.putText(img, txt, (min(X(te) + 4, W - 150), 14), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (110, 110, 255), 1, cv2.LINE_AA)
    cv2.line(img, (X(sim.world.t), 4), (X(sim.world.t), H - 4), (255, 255, 255), 1)
    cv2.putText(img, "green = chasing   red = evading   yellow = rear lidar sees a chaser   orange = external camera sees a chaser",
                (x0, H - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (170, 170, 170), 1, cv2.LINE_AA)
    return img


class Recorder:
    def __init__(self, path, duration, fps=15, video=True):
        self.path, self.duration, self.fps, self.video = path, duration, fps, video
        self.size = (1360, 560)
        self.vw = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"mp4v"), fps, self.size) if video else None
        self.keyframes = {}
        self.first_blocked = self.first_evade = False

    def add(self, sim):
        if sim.last_frame is None:
            return
        cam = sim.last_frame.copy()
        if "follower" in sim.last_state:
            tk.draw_overlay(cam, sim.tr, sim.last_state)
        if sim.planner:
            pl.draw_plan(cam, sim.tr, sim.planner)
        cam = cv2.resize(cam, (800, 450), interpolation=cv2.INTER_AREA)
        cv2.putText(cam, "external camera (simulated image, REAL tracker + planner overlay)", (8, 440), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (235, 235, 235), 1, cv2.LINE_AA)
        top = np.hstack([cam, draw_map(sim)])
        frame = np.vstack([top, draw_timeline(sim, (1360, 110), self.duration)])
        p = sim.pipe.last_plan
        t = sim.world.t
        if "start" not in self.keyframes and t > 1.0:
            self.keyframes["start"] = frame.copy()
        if p and p.blocked and not self.first_blocked and t > 2.0:
            self.first_blocked = True
            self.keyframes["route round the no-go zone"] = frame.copy()
        if sim.mode == "evade" and not self.first_evade:
            self.first_evade = True
            self.keyframes["evasion triggered"] = frame.copy()
        for te, txt in sim.world.events:
            key = "pop: " + txt
            if key not in self.keyframes and t - te < 0.5:
                self.keyframes[key] = frame.copy()
        self.last = frame
        if self.vw:
            self.vw.write(frame)

    def close(self):
        if self.vw:
            self.vw.release()
        if getattr(self, "last", None) is not None:
            self.keyframes["end"] = self.last


def run_scenario(name, seed=0, video=True, **kw):
    scn = SCENARIOS[name]
    os.makedirs(OUT, exist_ok=True)
    rec = Recorder(f"{OUT}/{kw.get('label', name)}.mp4", scn["duration"], video=video)
    kw.pop("label", None)
    sim = Sim(scn, seed=seed, recorder=rec, **kw)
    summary = sim.run(scn["duration"])
    rec.close()
    return sim, summary, rec


def main(argv):
    names = [a for a in argv if not a.startswith("--")] or list(SCENARIOS)
    video = "--no-video" not in argv
    for n in names:
        sim, s, rec = run_scenario(n, video=video)
        print(n, s)


if __name__ == "__main__":
    main(sys.argv[1:])
