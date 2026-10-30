"""
sim_world.py - a physical simulation of the arena, to test the whole system without hardware.

What is simulated
  - the arena floor (red/black tape, no-go square, floor marker), robots with bodies, pins and balloons
  - opponents with simple behaviours (static / patrol / orbit / hunter) and pops when a pin meets a balloon
  - an external camera that MOVES, producing real images (ArUco markers drawn in perspective, coloured balloons)
    which the REAL tracker.py then has to detect
  - a rear lidar (4x4, with latency, noise, dropouts and your own balloon in view) and side IR
  - the rover link: the REAL micromelon Motors code runs on top of a fake link that moves the simulated robot,
    so speed caps, turn calibration and blocking moves behave as they would on the rover
"""
import math
import types
from collections import deque

import cv2
import numpy as np

import evade as ev
import plan as pl
import tracker as tk

SIZE = (1280, 720)
ARENA = [(0.20, -0.565), (1.40, -0.565), (1.40, 0.565), (0.20, 0.565)]       # floor frame, metres (inner tape edge)
NOGO_SIDE, NOGO_ROT_DEG = 0.15, 45.0        # the no-go square is turned 45 degrees (a diamond), centred at (0.80, 0)


def rotated_square(cx, cy, side, deg):
    """Corners of a square (counter-clockwise) turned by deg degrees about its centre."""
    a = math.radians(deg)
    h = side / 2.0
    pts = [(-h, -h), (h, -h), (h, h), (-h, h)]
    return [(cx + x * math.cos(a) - y * math.sin(a), cy + x * math.sin(a) + y * math.cos(a)) for x, y in pts]


NOGO = rotated_square(0.80, 0.0, NOGO_SIDE, NOGO_ROT_DEG)
TRACK_EFF_CM = 18.7              # effective track for spins: matches the Micromelon module's turn calibration
BODY_L, BODY_W, BODY_H = 0.12, 0.10, 0.08
PIN_TIP = 0.10                   # rover centre -> pin tip
BALLOON_BACK = 0.10              # rover centre -> balloon, backwards (a stick off the back, clear of the top marker)
BALLOON_Z, BALLOON_R = 0.15, 0.045
tk.TOP_MARKER_OFFSET = (0.02, 0.0)   # the follower's marker sits slightly forward of centre, away from its own balloon
BODY_CIRCLE = 0.07               # used for lidar returns and collisions
NOGO_VIOLATION_M = BODY_CIRCLE   # a violation is counted when the follower's centre is closer than this to the no-go zone ...
TAPE_VIOLATION_M = 0.06          # ... or than this to the inner edge of the tape (about half the rover's length). THE REAL PENALTY RULE IS
                                 # UNKNOWN: change these two to match it and every sweep and video follows.
COLLIDE = 0.13                   # centre-to-centre distance below which two robots push each other apart
POP_DIST = 0.045                 # pin tip within this of a balloon pops it

BGR = {"follower": (200, 0, 200), "A": (255, 90, 20), "B": (0, 200, 0), "C": (0, 220, 230)}   # magenta, blue, green, yellow


def wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


def rect_of(poly):
    xs, ys = [p[0] for p in poly], [p[1] for p in poly]
    return min(xs), min(ys), max(xs), max(ys)


NOGO_RECT = rect_of(NOGO)
ARENA_RECT = rect_of(ARENA)


def dist_point_rect(px, py, r):
    return math.hypot(max(r[0] - px, 0, px - r[2]), max(r[1] - py, 0, py - r[3]))


def dist_point_poly(px, py, poly):
    """Distance from a point to a convex polygon (0 if inside)."""
    n = len(poly)
    inside = all((poly[(i + 1) % n][0] - poly[i][0]) * (py - poly[i][1]) - (poly[(i + 1) % n][1] - poly[i][1]) * (px - poly[i][0]) >= 0
                 for i in range(n)) or all((poly[(i + 1) % n][0] - poly[i][0]) * (py - poly[i][1]) - (poly[(i + 1) % n][1] - poly[i][1]) * (px - poly[i][0]) <= 0
                                           for i in range(n))
    if inside:
        return 0.0
    best = 1e9
    for i in range(n):
        ax, ay = poly[i]
        bx, by = poly[(i + 1) % n]
        dx, dy = bx - ax, by - ay
        t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)))
        best = min(best, math.hypot(px - (ax + t * dx), py - (ay + t * dy)))
    return best


def ray_poly(ox, oy, dx, dy, poly):
    """Distance along a ray to a convex polygon, or None (0 if it starts inside)."""
    if dist_point_poly(ox, oy, poly) == 0.0:
        return 0.0
    best = None
    n = len(poly)
    for i in range(n):
        ax, ay = poly[i]
        ex, ey = poly[(i + 1) % n][0] - ax, poly[(i + 1) % n][1] - ay
        den = dx * ey - dy * ex
        if abs(den) < 1e-12:
            continue
        t = ((ax - ox) * ey - (ay - oy) * ex) / den
        u = ((ax - ox) * dy - (ay - oy) * dx) / den
        if t >= 0.0 and 0.0 <= u <= 1.0 and (best is None or t < best):
            best = t
    return best


def ray_circle(ox, oy, dx, dy, cx, cy, r):
    fx, fy = ox - cx, oy - cy
    b = fx * dx + fy * dy
    c = fx * fx + fy * fy - r * r
    disc = b * b - c
    if disc < 0:
        return None
    t = -b - math.sqrt(disc)
    return t if t >= 0 else (None if -b + math.sqrt(disc) < 0 else 0.0)


def ray_rect(ox, oy, dx, dy, r):
    t0, t1 = 0.0, 1e9
    for o, d, lo, hi in ((ox, dx, r[0], r[2]), (oy, dy, r[1], r[3])):
        if abs(d) < 1e-12:
            if o < lo or o > hi:
                return None
        else:
            a, b = (lo - o) / d, (hi - o) / d
            t0, t1 = max(t0, min(a, b)), min(t1, max(a, b))
            if t0 > t1:
                return None
    return t0


class Agent:
    def __init__(self, name, x, y, yaw_deg, gains=(1.0, 1.0)):
        self.name, self.x, self.y, self.yaw = name, x, y, math.radians(yaw_deg)
        self.vl = self.vr = 0.0                # wheel speeds, cm/s
        self.gl, self.gr = gains               # per-wheel gain (motors are never identical)
        self.popped = False
        self.turn_noise = 1.0

    def step(self, dt):
        vl, vr = self.vl * self.gl, self.vr * self.gr
        v = (vl + vr) / 2.0 / 100.0
        w = (vr - vl) / TRACK_EFF_CM
        self.yaw += w * dt
        self.x += v * math.cos(self.yaw) * dt
        self.y += v * math.sin(self.yaw) * dt

    @property
    def pin(self):
        return (self.x + PIN_TIP * math.cos(self.yaw), self.y + PIN_TIP * math.sin(self.yaw))

    @property
    def balloon(self):
        return (self.x - BALLOON_BACK * math.cos(self.yaw), self.y - BALLOON_BACK * math.sin(self.yaw))

    def pose(self):
        return (self.x, self.y, self.yaw, self.popped)


class Opponent(Agent):
    """kind: static | patrol | orbit | hunter | wander (drives to random points round the arena, ignoring the follower)"""

    def __init__(self, name, x, y, yaw_deg, kind="static", speed=0.10, path=None, seed=0):
        super().__init__(name, x, y, yaw_deg)
        self.kind, self.speed, self.path, self.wp = kind, speed, path or [], 0
        self.delay = 0.4                       # a hunter reacts to where the follower WAS 0.4 s ago, like a real controller
        self.planner = pl.Planner(pl.Arena(ARENA, NOGO), "T") if kind in ("hunter", "wander") else None
        self._wp, self._wp_t = None, -1e9
        self.rng = np.random.default_rng(seed)

    def _drive(self, err, v):
        """err: bearing error in radians (+ = target on the left); v: forward speed m/s."""
        w = max(-2.2, min(2.2, 2.5 * err))
        if abs(err) > 0.6:
            v = 0.0
        else:
            v *= 1.0 - abs(err) / 0.6
        self.vl = v * 100.0 - w * TRACK_EFF_CM / 2.0
        self.vr = v * 100.0 + w * TRACK_EFF_CM / 2.0

    def policy(self, world, dt):
        if self.popped or self.kind == "static":
            self.vl = self.vr = 0.0
            return
        if self.kind in ("patrol", "orbit"):
            tx, ty = self.path[self.wp]
            if math.hypot(tx - self.x, ty - self.y) < 0.06:
                self.wp = (self.wp + 1) % len(self.path)
                tx, ty = self.path[self.wp]
            err = math.atan2(math.sin(math.atan2(ty - self.y, tx - self.x) - self.yaw),
                             math.cos(math.atan2(ty - self.y, tx - self.x) - self.yaw))
            self._drive(err, self.speed)
        elif self.kind == "wander":
            if (self._wp is None or math.hypot(self._wp[0] - self.x, self._wp[1] - self.y) < 0.08
                    or world.t - self._wp_t > 7.0):
                for _ in range(100):                                   # a new random point inside the tape, clear of the no-go square
                    p = (self.rng.uniform(ARENA_RECT[0] + 0.15, ARENA_RECT[2] - 0.15),
                         self.rng.uniform(ARENA_RECT[1] + 0.15, ARENA_RECT[3] - 0.15))
                    if dist_point_poly(p[0], p[1], NOGO) > 0.2:
                        break
                self._wp, self._wp_t = p, world.t
            st = {"follower": {"x": self.x, "y": self.y, "yaw": math.degrees(self.yaw)}, "T": {"x": self._wp[0], "y": self._wp[1]}}
            p = self.planner.update(st, world.t)
            if p is None:
                self.vl = self.vr = 0.0
                return
            self._drive(math.radians(p.yaw_to_waypoint), self.speed)
        elif self.kind == "hunter":
            _, fx, fy, fyaw = world.follower_seen_by_hunter(self.delay)
            bx, by = fx - BALLOON_BACK * math.cos(fyaw), fy - BALLOON_BACK * math.sin(fyaw)
            st = {"follower": {"x": self.x, "y": self.y, "yaw": math.degrees(self.yaw)}, "T": {"x": bx, "y": by}}
            p = self.planner.update(st, world.t)
            if p is None:
                self.vl = self.vr = 0.0
                return
            self._drive(math.radians(p.yaw_to_waypoint), self.speed)


class World:
    def __init__(self, follower, opponents, dt=1 / 30.0):
        self.follower, self.opps = follower, opponents
        self.agents = [follower] + opponents
        self.dt, self.t = dt, 0.0
        self.snaps = deque(maxlen=120)
        self.f_hist = deque(maxlen=120)         # the follower's recent poses (a hunter sees them with a delay)
        self.events = []                # (t, text)
        self.nogo_viol = 0              # frames the follower's body overlapped the no-go zone
        self.tape_viol = 0              # frames the follower's body crossed the tape
        self.min_nogo_clear = 9.0
        self._record()

    def _record(self):
        self.snaps.append(types.SimpleNamespace(t=self.t, agents={a.name: a.pose() for a in self.agents}))
        f = self.follower
        self.f_hist.append((self.t, f.x, f.y, f.yaw))

    def follower_seen_by_hunter(self, delay):
        """The follower's pose as a hunter with `delay` seconds of reaction time sees it."""
        best = self.f_hist[0]
        for h in self.f_hist:
            if h[0] <= self.t - delay + 1e-9:
                best = h
        return best

    def snapshot_at(self, t):
        best = self.snaps[0]
        for s in self.snaps:
            if s.t <= t + 1e-9:
                best = s
        return best

    def step(self):
        dt = self.dt
        for o in self.opps:
            o.policy(self, dt)
        for a in self.agents:
            a.step(dt)
        live = [a for a in self.agents if not a.popped]
        for i, a in enumerate(live):                                   # robots can't drive through each other
            for b in live[i + 1:]:
                d = math.hypot(a.x - b.x, a.y - b.y)
                if 1e-6 < d < COLLIDE:
                    push = (COLLIDE - d) / 2.0
                    ux, uy = (a.x - b.x) / d, (a.y - b.y) / d
                    a.x, a.y, b.x, b.y = a.x + ux * push, a.y + uy * push, b.x - ux * push, b.y - uy * push
        self.t += dt
        self._record()
        for a in live:                                                 # pops
            for v in live:
                if a is not v and not v.popped and not a.popped:
                    px, py = a.pin
                    bx, by = v.balloon
                    if math.hypot(px - bx, py - by) < POP_DIST:
                        v.popped = True
                        self.events.append((self.t, f"{a.name} popped {v.name}"))
        f = self.follower
        c = dist_point_poly(f.x, f.y, NOGO)
        self.min_nogo_clear = min(self.min_nogo_clear, c)
        if c < NOGO_VIOLATION_M:
            self.nogo_viol += 1
        inner = min(f.x - ARENA_RECT[0], ARENA_RECT[2] - f.x, f.y - ARENA_RECT[1], ARENA_RECT[3] - f.y)
        if inner < TAPE_VIOLATION_M:
            self.tape_viol += 1


# --------------------------------------------------------------------------------------------- camera + renderer
class Cam:
    def __init__(self, K, cam_from_floor):
        self.K, self.T = K, cam_from_floor
        self.rvec = cv2.Rodrigues(cam_from_floor[:3, :3])[0]
        self.tvec = cam_from_floor[:3, 3].reshape(3, 1)

    def project(self, pts):
        pts = np.asarray(pts, np.float64).reshape(-1, 3)
        img, _ = cv2.projectPoints(pts, self.rvec, self.tvec, self.K, None)
        return img.reshape(-1, 2)

    def radius_px(self, centre, r):
        z = float((self.T[:3, :3] @ np.asarray(centre) + self.T[:3, 3])[2])
        return self.K[0, 0] * r / max(z, 1e-3)


class CameraRig:
    """An external camera held roughly above the arena that DRIFTS and swings a little, like a hand-held or bumped one."""

    def __init__(self, fov_deg=70.0, size=SIZE, drift=True):
        self.size, self.drift = size, drift
        self.K = tk.intrinsics_from_fov(size, fov_deg)

    def cam_from_floor(self, t):
        pos = np.array([0.80, -0.55, 1.75])
        target = np.array([0.80, 0.0, 0.0])
        if self.drift:
            pos = pos + [0.05 * math.sin(0.7 * t), 0.04 * math.sin(0.5 * t + 1.0), 0.03 * math.sin(0.3 * t)]
            target = target + [0.03 * math.sin(0.4 * t), 0.02 * math.sin(0.6 * t), 0.0]
        zc = target - pos
        zc /= np.linalg.norm(zc)
        xc = np.cross(zc, [0, 0, 1.0])
        xc /= np.linalg.norm(xc)
        yc = np.cross(zc, xc)
        T = np.eye(4)
        T[:3, :3] = np.stack([xc, yc, zc], 1)
        T[:3, 3] = pos
        return np.linalg.inv(T)

    def cam(self, t):
        return Cam(self.K, self.cam_from_floor(t))


class Renderer:
    def __init__(self, seed=0, noise=3.0):
        self.rng = np.random.default_rng(seed)
        self.noise = noise
        self.pool = [np.rint(self.rng.normal(0, noise, (SIZE[1], SIZE[0], 3))).astype(np.int16) for _ in range(5)] if noise > 0 else []
        d = cv2.aruco.getPredefinedDictionary(tk.DICTIONARY)
        self.markers = {i: cv2.cvtColor(cv2.aruco.generateImageMarker(d, i, 120), cv2.COLOR_GRAY2BGR)
                        for i in (tk.FLOOR_ID, tk.TOP_MARKER_ID)}

    def _marker(self, img, marker_id, corners3d, cam):
        c3 = np.asarray(corners3d, float)
        ctr = c3.mean(0)
        cv2.fillConvexPoly(img, cam.project(ctr + (c3 - ctr) * (4.0 / 3.0)).astype(np.int32), (235, 235, 235))
        px = cam.project(c3).astype(np.float32)
        x0, y0 = np.maximum(np.floor(px.min(0)).astype(int) - 2, 0)
        x1, y1 = np.minimum(np.ceil(px.max(0)).astype(int) + 2, [SIZE[0], SIZE[1]])
        if x1 <= x0 or y1 <= y0:
            return
        H = cv2.getPerspectiveTransform(np.float32([[0, 0], [119, 0], [119, 119], [0, 119]]), px - np.float32([x0, y0]))
        wh = (int(x1 - x0), int(y1 - y0))
        warped = cv2.warpPerspective(self.markers[marker_id], H, wh, flags=cv2.INTER_LINEAR)
        mask = cv2.warpPerspective(np.full((120, 120), 255, np.uint8), H, wh, flags=cv2.INTER_NEAREST)
        roi = img[y0:y1, x0:x1]
        roi[mask > 0] = warped[mask > 0]

    @staticmethod
    def _poly(img, cam, pts3, colour):
        cv2.fillConvexPoly(img, cam.project(pts3).astype(np.int32), colour, cv2.LINE_AA)

    def render(self, snap, cam):
        img = np.full((SIZE[1], SIZE[0], 3), (36, 30, 26), np.uint8)
        a = ARENA
        self._poly(img, cam, [(x, y, 0) for x, y in a], (58, 52, 46))
        # tape: red on the long edges, black on the short ones (inner edge of the arena)
        for (p, q), col in zip(zip(a, a[1:] + a[:1]), [(55, 55, 215), (6, 6, 6), (55, 55, 215), (6, 6, 6)]):
            px = cam.project([(p[0], p[1], 0), (q[0], q[1], 0)])
            cv2.line(img, tuple(px[0].astype(int)), tuple(px[1].astype(int)), col, 7, cv2.LINE_AA)
        self._poly(img, cam, [(x, y, 0) for x, y in NOGO], (70, 70, 150))
        # floor marker at the origin, flat
        h = tk.FLOOR_SIZE / 2.0
        self._marker(img, tk.FLOOR_ID, [(-h, h, 0), (h, h, 0), (h, -h, 0), (-h, -h, 0)], cam)
        for name, (x, y, yaw, popped) in snap.agents.items():
            c, s = math.cos(yaw), math.sin(yaw)

            def to_floor(dx, dy, z):
                return (x + dx * c - dy * s, y + dx * s + dy * c, z)

            foot = [to_floor(dx, dy, 0.0) for dx, dy in [(-BODY_L / 2, -BODY_W / 2), (BODY_L / 2, -BODY_W / 2),
                                                         (BODY_L / 2, BODY_W / 2), (-BODY_L / 2, BODY_W / 2)]]
            top = [(p[0], p[1], BODY_H) for p in foot]
            self._poly(img, cam, foot, (20, 22, 26))
            for i in range(4):                                         # side faces, then the top
                self._poly(img, cam, [foot[i], foot[(i + 1) % 4], top[(i + 1) % 4], top[i]], (92, 96, 104) if i % 2 else (78, 82, 90))
            self._poly(img, cam, top, (120, 126, 136))
            if name == "follower":
                M = tk.robot_from_marker(None)
                ob = tk.obj_points(tk.MARKER_SIZE)
                R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
                corners = [R @ (M @ np.array([px, py, 0.0, 1.0]))[:3] + np.array([x, y, 0.0]) for px, py, _ in ob]
                self._marker(img, tk.TOP_MARKER_ID, corners, cam)
            if not popped:
                bx, by = x - BALLOON_BACK * c, y - BALLOON_BACK * s
                base = cam.project([(bx, by, BODY_H)])[0]
                ctr3 = np.array([bx, by, BALLOON_Z])
                ctr = cam.project([ctr3])[0]
                cv2.line(img, tuple(base.astype(int)), tuple(ctr.astype(int)), (150, 150, 150), 2, cv2.LINE_AA)
                r = max(2, int(round(cam.radius_px(ctr3, BALLOON_R))))
                cv2.circle(img, tuple(ctr.astype(int)), r, BGR[name], -1, cv2.LINE_AA)
        img = cv2.GaussianBlur(img, (3, 3), 0.6)
        if self.noise > 0:
            img = np.clip(img.astype(np.int16) + self.pool[int(self.rng.integers(len(self.pool)))], 0, 255).astype(np.uint8)
        return img


# ------------------------------------------------------------------------------------------------------ sensors
def lidar_frame(snap, rng, lidar_back=0.05, own_balloon=True):
    """Rear 4x4 lidar, millimetres, row-major, nan = no reading. Column 0 = the rover's LEFT."""
    x, y, yaw, _ = snap.agents["follower"]
    ox, oy = x - lidar_back * math.cos(yaw), y - lidar_back * math.sin(yaw)
    out = np.full((4, 4), np.nan)
    for c in range(4):
        ang = yaw + math.pi - math.radians(22.5 - 15.0 * c)
        dx, dy = math.cos(ang), math.sin(ang)
        best = None
        for name, (ax, ay, _, popped) in snap.agents.items():
            if name == "follower":
                continue
            t = ray_circle(ox, oy, dx, dy, ax, ay, BODY_CIRCLE)
            if t is not None and (best is None or t < best):
                best = t
        if best is not None and best <= 3.5:
            mm = best * 1000.0 + rng.normal(0, 4.0)
            out[1, c] = out[2, c] = mm
    if own_balloon:
        out[3, 1] = out[3, 2] = 90.0 + rng.normal(0, 2.0)
    drop = rng.random((4, 4)) < 0.02
    out[drop] = np.nan
    return out.ravel()


TAPE_WIDTH_M = 0.04              # the tape band outside the inner edge; beyond it the (light) room floor


def floor_flags(follower):
    """What the three floor colour sensors report: True where a sensor is over the tape band."""
    import floor as floor_mod
    flags = []
    for i in range(3):
        sx, sy = floor_mod.sensor_world(follower.x, follower.y, math.degrees(follower.yaw), i)
        inner = min(sx - ARENA_RECT[0], ARENA_RECT[2] - sx, sy - ARENA_RECT[1], ARENA_RECT[3] - sy)
        flags.append(-TAPE_WIDTH_M < inner < 0.0)
    return flags


def ir_cm(snap):
    """(left, right) distance in cm from the follower's sides, to robots and the no-go square."""
    x, y, yaw, _ = snap.agents["follower"]
    res = []
    for side in (+1.0, -1.0):
        ang = yaw + side * math.pi / 2
        dx, dy = math.cos(ang), math.sin(ang)
        best = 0.8
        for name, (ax, ay, _, _) in snap.agents.items():
            if name != "follower":
                t = ray_circle(x, y, dx, dy, ax, ay, BODY_CIRCLE)
                if t is not None:
                    best = min(best, t)
        t = ray_poly(x, y, dx, dy, NOGO)
        if t is not None:
            best = min(best, t)
        res.append(int(best * 100))
    return tuple(res)


# -------------------------------------------------------------------------------------------- the rover link
class SimRC:
    """Stands in for the rover link. The REAL micromelon Motors code builds packets and calls these."""

    def __init__(self, sim):
        self.sim = sim
        self.rng = np.random.default_rng(11)

    def connectedRobotIsSimulated(self):
        return False

    def writeAttribute(self, op, data, timeout=None):
        l, r = ev.decode_motor_packet(list(data), False)
        self.sim.set_wheels(l, r)
        return True

    def doMotorOperation(self, op, data, timeout=None):
        d = list(data)

        def s16(lo, hi):
            v = lo | (hi << 8)
            return v - 65536 if v > 32767 else v

        ls, rs = d[0] / 127.0 * 30.0, d[1] / 127.0 * 30.0
        ld, rd = s16(d[2], d[3]) / 10.0, s16(d[4], d[5]) / 10.0           # mm -> cm
        sync = bool(d[6])
        tl = abs(ld) / ls if ls > 0 and ld != 0 else 0.0
        tr_ = abs(rd) / rs if rs > 0 and rd != 0 else 0.0
        dur = (min(tl, tr_) if sync and tl and tr_ else max(tl, tr_))
        gain = 1.0 + self.rng.normal(0, 0.03)                               # open-loop turns are never exact
        lv = math.copysign(ls, ld) * gain
        rv = math.copysign(rs, rd) * gain
        if dur <= 0:
            return
        t_first = min(x for x in (tl, tr_) if x > 0) if not sync else dur
        self.sim.set_wheels(lv if tl > 0 else 0.0, rv if tr_ > 0 else 0.0)
        self.sim.advance(t_first)
        if dur > t_first:
            self.sim.set_wheels(lv if tl > t_first else 0.0, rv if tr_ > t_first else 0.0)
            self.sim.advance(dur - t_first)
        self.sim.set_wheels(0.0, 0.0)


class SimMotors:
    """Context manager: point the real micromelon Motors code at the simulator."""

    def __init__(self, sim, tap_cls=ev.MotionTap):
        self.sim, self.tap_cls = sim, tap_cls
        self.tap = None

    def __enter__(self):
        import micromelon
        import micromelon._utils as mu
        import micromelon.motors._motors as mm
        from micromelon._robot_comms import MicromelonType as OPTYPE
        self._mm, self._mu = mm, mu
        self._saved = (mm.time, mu.sleep, getattr(micromelon, "delay", None))
        mm.time = types.SimpleNamespace(sleep=self.sim.advance)
        mu.sleep = self.sim.advance
        rc = mm._rc
        self._rc, self._rc_saved = rc, (rc.writeAttribute, rc.doMotorOperation, getattr(rc, "connectedRobotIsSimulated", None))
        link = SimRC(self.sim)
        rc.writeAttribute, rc.doMotorOperation, rc.connectedRobotIsSimulated = link.writeAttribute, link.doMotorOperation, link.connectedRobotIsSimulated
        self.tap = self.tap_cls(OPTYPE.MOTOR_SET)
        self.tap.install(rc)                                              # speed cap + motion estimate, as on the rover
        ev.install_speed_clamp(micromelon.Motors)                         # (and the same clamp follow.py installs)
        return self

    def __exit__(self, *exc):
        self._mm.time, self._mu.sleep = self._saved[0], self._saved[1]
        self._rc.writeAttribute, self._rc.doMotorOperation, self._rc.connectedRobotIsSimulated = self._rc_saved
