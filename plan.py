"""
plan.py - waypoint planning round the no-go zone and the tape boundary.

Works in the external camera's floor frame (metres), the same frame tracker.py reports.

What it does each step
  1. Pick a target opponent (the nearest by default).
  2. Choose a goal point BEHIND that opponent (where your pin meets its balloon), led forward a little so
     the camera/network delay doesn't make you chase where it was. If it is hardly moving its heading is
     unknown, so the goal is just the opponent itself.
  3. Find the shortest path to the goal that stays inside the arena and out of the no-go zone, with both
     grown by the rover's size. For a convex obstacle that is a few straight segments that bend only at the
     grown corners (a "visibility graph"), so it is exact and fast.
  4. Report the yaw to the NEXT waypoint (+ = left), which follow.py hands to your chase(yaw).
     Once you are close to the goal point it switches to aiming straight at the opponent.

The arena and no-go square come from arena.json (made with mark_arena.py).
"""
import json
import math
import os
from collections import deque
from dataclasses import dataclass, field

import numpy as np

import evade

# ----------------------------- CONFIG ---------------------------------------
ARENA_FILE = "arena.json"
ROBOT_RADIUS_M = 0.08      # half the rover's longest dimension, from its centre (MEASURE; err on the big side)
TAPE_MARGIN_M = 0.12       # extra clearance from the tape boundary
NOGO_MARGIN_M = 0.13       # extra clearance from the no-go zone. Bigger = safer but longer routes (was 0.03)
HYSTERESIS_M = 0.08        # keep the side you committed to unless the other side is this much shorter
LEAD_S = 0.4               # aim this far ahead of a moving opponent (camera + network delay)
MIN_SPEED = 0.08           # m/s: slower than this, an opponent's heading is unknown
BEHIND_M = evade.PIN_REACH_M   # goal distance behind the opponent's balloon point (where your pin meets its balloon)
TARGET_MODE = "adaptive"     # how a target is chosen: "sticky" (nearest, then fixate), "switch" (always the nearest, re-checked every
                            # frame), "active_first" (the nearest MOVING opponent first), "calm_first" (the nearest STILL one first), "adaptive" (sticky, but a still one first while 2+ are moving)
AVOID_R_M = 0.0             # >0 = route AROUND moving opponents that are not the target: a keep-out circle of this radius (m) on where each will be
                            # in AVOID_LEAD_S. 0 = off. A hunter is slower than we are, so keeping clear of it means it never gets to push us.
AVOID_LEAD_S = 0.8
STALL_S = 14.0              # give up on a target we haven't got closer to (by STALL_PROGRESS_M) for this long ...
STALL_PROGRESS_M = 0.05
SKIP_S = 20.0               # ... and don't pick it again for this long (if another opponent is in play)
ENGAGE_M = 0.25            # within this of the goal point, stop weaving and aim straight at the opponent
WAYPOINT_REACHED_M = 0.05  # a waypoint closer than this counts as reached (the yaw to a point a few cm away is noise)
HISTORY_S = 0.5            # window for measuring an opponent's velocity
# -----------------------------------------------------------------------------


def wrap(a):
    return evade.wrap(a)


def _cross(a, b):
    return a[0] * b[1] - a[1] * b[0]


def ccw(poly):
    P = [np.asarray(p, float) for p in poly]
    area = 0.5 * sum(_cross(P[i], P[(i + 1) % len(P)]) for i in range(len(P)))
    return P if area > 0 else P[::-1]


def offset_convex(poly, d):
    """Move every edge of a convex polygon outward by d (negative = inward). Returns CCW vertices."""
    P = ccw(poly)
    n = len(P)
    lines = []
    for i in range(n):
        e = P[(i + 1) % n] - P[i]
        e = e / np.linalg.norm(e)
        lines.append((P[i] + np.array([e[1], -e[0]]) * d, e))       # outward normal of a CCW edge
    out = []
    for i in range(n):
        p1, e1 = lines[i - 1]
        p2, e2 = lines[i]
        t = _cross(p2 - p1, e2) / _cross(e1, e2)
        out.append(p1 + t * e1)
    return out


def offset_rounded(poly, d, step_deg=30.0):
    """A convex polygon grown by d in every direction with ROUND corners (what a robot of radius d really sweeps out), as
    a convex polygon with a vertex every step_deg round each corner. Mitred corners (offset_convex) stick out by
    d*(sqrt(2)-1) more, which for a polygon turned 45 degrees closes the corridors beside it."""
    P = ccw(poly)
    n = len(P)
    out = []
    for i in range(n):
        e0 = P[i] - P[i - 1]
        e1 = P[(i + 1) % n] - P[i]
        a0 = math.atan2(-e0[0], e0[1])                      # outward normal of the edge into this vertex
        a1 = math.atan2(-e1[0], e1[1])                      # ... and of the edge out of it
        turn = (a1 - a0 + math.pi) % (2 * math.pi) - math.pi
        k = max(1, int(math.ceil(abs(math.degrees(turn)) / step_deg)))
        for j in range(k + 1):
            a = a0 + turn * j / k
            out.append(P[i] + d * np.array([math.cos(a), math.sin(a)]))
    return out


def inside(poly, p, eps=0.0):
    P = ccw(poly)
    p = np.asarray(p, float)
    return all(_cross(P[(i + 1) % len(P)] - P[i], p - P[i]) >= -eps for i in range(len(P)))


_INNER = {}


def crosses(poly, a, b):
    """True if the segment a-b passes through the INTERIOR of the convex polygon (touching is fine)."""
    hit = _INNER.get(id(poly))
    if hit is not None and hit[0] is poly:                           # the shrunk polygon is the same every call: cache it
        inner = hit[1]
    else:
        inner = offset_convex(poly, -1e-4)
        if len(_INNER) > 64:
            _INNER.clear()
        _INNER[id(poly)] = (poly, inner)
    a, b = np.asarray(a, float), np.asarray(b, float)
    d = b - a
    t0, t1 = 0.0, 1.0
    for i in range(len(inner)):
        p, q = inner[i], inner[(i + 1) % len(inner)]
        e = q - p
        nrm = np.array([-e[1], e[0]])                               # inward normal
        num, den = float(np.dot(nrm, a - p)), float(np.dot(nrm, d))
        if abs(den) < 1e-12:
            if num < 0:
                return False
        else:
            t = -num / den
            if den > 0:
                t0 = max(t0, t)
            else:
                t1 = min(t1, t)
        if t0 >= t1:
            return False
    return (t1 - t0) > 1e-9


def _closest_on_boundary(poly, p):
    P = ccw(poly)
    p = np.asarray(p, float)
    best, bd = None, 1e18
    for i in range(len(P)):
        a, b = P[i], P[(i + 1) % len(P)]
        ab = b - a
        t = max(0.0, min(1.0, float(np.dot(p - a, ab) / np.dot(ab, ab))))
        c = a + t * ab
        dd = float(np.linalg.norm(p - c))
        if dd < bd:
            best, bd = c, dd
    return best


def push_out(poly, p):
    """If p is inside the polygon move it just outside; otherwise leave it."""
    if not inside(poly, p):
        return np.asarray(p, float)
    c = _closest_on_boundary(poly, p)
    centre = np.mean(ccw(poly), axis=0)
    v = c - centre
    return c + 1e-3 * v / (np.linalg.norm(v) + 1e-12)


def pull_in(poly, p):
    """If p is outside the polygon move it just inside; otherwise leave it."""
    if inside(poly, p):
        return np.asarray(p, float)
    c = _closest_on_boundary(poly, p)
    centre = np.mean(ccw(poly), axis=0)
    v = centre - c
    return c + 1e-3 * v / (np.linalg.norm(v) + 1e-12)


@dataclass
class Arena:
    poly: list
    nogo: list = None


def load_arena(path=ARENA_FILE):
    """Read arena.json ({"arena": [[x,y]x4], "nogo": [[x,y]x4] or null}) or return None."""
    if not os.path.exists(path):
        return None
    d = json.load(open(path))
    return Arena([tuple(p) for p in d["arena"]], [tuple(p) for p in d["nogo"]] if d.get("nogo") else None)


@dataclass
class Plan:
    target: str = ""
    path: list = field(default_factory=list)       # [start, waypoint..., goal] in metres
    waypoint: tuple = None
    goal: tuple = None
    yaw_to_waypoint: float = None                  # degrees, + = left
    yaw_to_target: float = None                    # straight at the opponent, for reference
    engaged: bool = False
    blocked: bool = False                          # the straight line to the goal crosses the no-go zone
    dist_to_target: float = None


class Planner:
    def __init__(self, arena, target="nearest"):
        self.arena = arena
        self.free = offset_convex(arena.poly, -(ROBOT_RADIUS_M + TAPE_MARGIN_M))
        self.block = offset_rounded(arena.nogo, ROBOT_RADIUS_M + NOGO_MARGIN_M) if arena.nogo else None
        self.target_pref = target
        self._target = None
        self._best_d, self._best_t = None, None          # closest we have got to the current target, and when
        self._now = 0.0
        self.avoid = []                                   # keep-out polygons round moving non-target opponents (see AVOID_R_M)
        self._active_until = {}                           # name -> time until which it counts as an ACTIVE opponent
        self._skip = {}                                   # name -> time until which we won't choose it again (it stalled us)
        self._last_via = None
        self._hist = {}
        self.last = None

    # ---- geometry ---------------------------------------------------------------------
    def _free_point(self, p):
        p = np.asarray(p, float)
        for _ in range(3):
            if self.block is not None:
                p = push_out(self.block, p)
            p = pull_in(self.free, p)
        return p

    def route(self, start, goal):
        """Shortest path start -> goal inside the arena, round the no-go zone (and any keep-out circles). Returns (path, blocked)."""
        s = self._free_point(start)
        g = self._free_point(goal)
        obst = ([self.block] if self.block is not None else []) + [a for a in self.avoid if not inside(a, s) and not inside(a, g)]
        corners = []
        for k, poly in enumerate(obst):
            for c in poly:
                if inside(self.free, c) and not any(inside(o, c) for j, o in enumerate(obst) if j != k):
                    corners.append(c)
        nodes = [s, g] + corners
        n = len(nodes)

        def visible(i, j):
            return not any(crosses(o, nodes[i], nodes[j]) for o in obst)

        moved = float(np.linalg.norm(s - np.asarray(start, float))) > 1e-6     # start was too close: step out first
        if visible(0, 1):
            self._last_via = None
            path = [np.asarray(start, float), g]
            if moved:
                path.insert(1, s)
            return path, False
        # Dijkstra from the goal over every node except the start
        INF = 1e18
        dist = [INF] * n
        prev = [None] * n
        dist[1] = 0.0
        todo = set(range(1, n))
        while todo:
            u = min(todo, key=lambda k: dist[k])
            if dist[u] >= INF:
                break
            todo.discard(u)
            for v in todo:
                if visible(u, v):
                    w = dist[u] + float(np.linalg.norm(nodes[u] - nodes[v]))
                    if w < dist[v]:
                        dist[v], prev[v] = w, u
        best, best_cost = None, INF
        for k in range(1, n):
            if k == 1 or dist[k] >= INF or not visible(0, k):
                continue
            key = tuple(np.round(nodes[k], 3))
            cost = float(np.linalg.norm(s - nodes[k])) + dist[k]
            if self._last_via is not None and key != self._last_via:
                cost += HYSTERESIS_M
            if cost < best_cost:
                best, best_cost = k, cost
        if best is None:                                  # nothing reachable: head straight for the goal
            return ([np.asarray(start, float)] + ([s] if moved else []) + [g]), True
        self._last_via = tuple(np.round(nodes[best], 3))
        path = [np.asarray(start, float)] + ([s] if moved else [])
        k = best
        while k is not None:
            path.append(nodes[k])
            k = prev[k]
        return path, True

    # ---- opponents ------------------------------------------------------------------------
    def _velocity(self, name, t, x, y):
        h = self._hist.setdefault(name, deque())
        h.append((t, x, y))
        while h and t - h[0][0] > HISTORY_S:
            h.popleft()
        return evade._velocity(h)

    def _avoid_polys(self, target, opps, vel, now):
        """A keep-out octagon round where each MOVING opponent (other than the target) will be in AVOID_LEAD_S. Centres are rounded
        to 5 cm so the route does not flicker with every camera frame."""
        out = []
        for name, (x, y) in opps.items():
            if name == target or now >= self._active_until.get(name, -1e9):
                continue
            v = vel.get(name) or (0.0, 0.0)
            cx = round((x + v[0] * AVOID_LEAD_S) / 0.05) * 0.05
            cy = round((y + v[1] * AVOID_LEAD_S) / 0.05) * 0.05
            out.append([(cx + AVOID_R_M * math.cos(math.radians(a)), cy + AVOID_R_M * math.sin(math.radians(a)))
                        for a in range(0, 360, 45)])
        return out

    def _choose(self, f, opps):
        fx, fy = f["x"], f["y"]
        d = {k: math.hypot(v[0] - fx, v[1] - fy) for k, v in opps.items()}
        if self.target_pref != "nearest":
            return self.target_pref if self.target_pref in opps else None
        now = self._now
        if TARGET_MODE == "switch":                        # the original rule: always the nearest, switching only for a clear gain
            best = min(d, key=d.get)
            if self._target in d and d[self._target] <= d[best] + 0.10:
                best = self._target
            self._target = best
            return best
        self._skip = {k: t for k, t in self._skip.items() if t > now}
        if self._target in d:                              # fixate: keep the current target while it is still in play ...
            dt = d[self._target]
            if self._best_d is None or dt < self._best_d - STALL_PROGRESS_M:
                self._best_d, self._best_t = dt, now
                return self._target                                    # getting closer: stay on it
            if now - self._best_t > STALL_S and len(d) > 1:            # ... unless we have stopped getting closer to it
                self._skip[self._target] = now + SKIP_S                # (unreachable, or it keeps dodging us): try another
                self._target = None
            else:
                return self._target
        pool = {k: v for k, v in d.items() if k not in self._skip} or d
        active = {k: v for k, v in pool.items() if now < self._active_until.get(k, -1e9)}
        calm = {k: v for k, v in pool.items() if k not in active}
        if TARGET_MODE == "active_first" and active:       # hunt the hunter: the nearest opponent that is moving
            pool = active
        elif TARGET_MODE == "calm_first" and calm:         # leave the hunter alone: the nearest one that is not moving
            pool = calm
        elif TARGET_MODE == "adaptive" and calm and len(active) >= 2:   # two or more are moving: take a still one first
            pool = calm
        self._target = min(pool, key=pool.get)
        self._best_d, self._best_t = d[self._target], now
        return self._target

    def set_target(self, name):
        """Retarget (used when we had to evade that opponent). Ignored for a fixed --target or an unknown name."""
        if self.target_pref == "nearest" and name and name != self._target:
            self._target = name
            self._best_d, self._best_t = None, None

    # ---- main entry -----------------------------------------------------------------------
    def update(self, state, now):
        self._now = now
        """state: tracker.py state {"follower": {x,y,yaw}, "A": {x,y}, ...}. Returns a Plan or None."""
        f = state.get("follower")
        opps = {k: (v["x"], v["y"]) for k, v in state.items() if k != "follower"}
        for name in list(self._hist):
            if name not in opps:
                del self._hist[name]
        vel = {name: self._velocity(name, now, x, y) for name, (x, y) in opps.items()}
        for name, v in vel.items():                        # which opponents are ACTIVE (seen moving in the last few seconds)
            if v and math.hypot(*v) >= evade.RING_MIN_SPEED:
                self._active_until[name] = now + evade.ACTIVE_HOLD_S
        if not f or not opps:
            self.last = None
            return None
        name = self._choose(f, opps)
        if name is None:
            self.last = None
            return None

        self.avoid = self._avoid_polys(name, opps, vel, now) if AVOID_R_M > 0 else []
        F = np.array([f["x"], f["y"]], float)
        O = np.array(opps[name], float)
        v = vel[name]
        speed = math.hypot(*v) if v else 0.0
        if v and speed >= MIN_SPEED:
            heading = np.array(v) / speed
            ahead = O + np.array(v) * LEAD_S                   # where it will be once our commands land
            goal = ahead - heading * BEHIND_M
        else:
            goal = O.copy()                                    # heading unknown: aim at it

        yaw_t = wrap(math.degrees(math.atan2(O[1] - F[1], O[0] - F[0])) - f["yaw"])
        blocked_direct = self.block is not None and crosses(self.block, F, O)
        engaged = (not blocked_direct) and float(np.linalg.norm(F - goal)) <= ENGAGE_M
        if engaged:
            path, blocked = [F, O], False
            wp = O
        else:
            path, blocked = self.route(F, goal)
            i = 1
            while i < len(path) - 1 and float(np.linalg.norm(path[i] - F)) < WAYPOINT_REACHED_M:   # already there: look ahead
                i += 1
            wp = path[i]
        yaw_w = wrap(math.degrees(math.atan2(wp[1] - F[1], wp[0] - F[0])) - f["yaw"])
        self.last = Plan(name, [tuple(map(float, p)) for p in path], tuple(map(float, wp)),
                         tuple(map(float, goal)), yaw_w, yaw_t, engaged, blocked,
                         float(np.linalg.norm(O - F)))
        return self.last


def draw_plan(frame, tr, planner):
    """Overlay on the external camera view: arena, no-go zone (and its grown outline), the path, the waypoint."""
    import cv2
    from tracker import project_floor, _pt

    def poly3(poly):
        return [(float(p[0]), float(p[1]), 0.0) for p in poly]

    def line(pts, col, th=1):
        img = project_floor(tr, poly3(pts))
        for i in range(len(img)):
            cv2.line(frame, _pt(img[i]), _pt(img[(i + 1) % len(img)]), col, th, cv2.LINE_AA)

    line(planner.arena.poly, (160, 160, 160), 1)
    line(planner.free, (110, 110, 110), 1)
    if planner.arena.nogo:
        line(planner.arena.nogo, (146, 157, 255), 2)
        line(planner.block, (90, 100, 170), 1)
    p = planner.last
    if p and p.path:
        img = project_floor(tr, poly3(p.path))
        for i in range(len(img) - 1):
            cv2.line(frame, _pt(img[i]), _pt(img[i + 1]), (255, 184, 143), 3, cv2.LINE_AA)
        wp = project_floor(tr, [(p.waypoint[0], p.waypoint[1], 0.0)])[0]
        cv2.circle(frame, _pt(wp), 8, (255, 184, 143), 2, cv2.LINE_AA)
        g = project_floor(tr, [(p.goal[0], p.goal[1], 0.0)])[0]
        cv2.drawMarker(frame, _pt(g), (255, 184, 143), cv2.MARKER_CROSS, 14, 2, cv2.LINE_AA)
        txt = f"waypoint yaw {p.yaw_to_waypoint:+.0f}" + ("  (engaged)" if p.engaged else "")
        cv2.putText(frame, txt, (10, frame.shape[0] - 36), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (255, 184, 143), 2, cv2.LINE_AA)
