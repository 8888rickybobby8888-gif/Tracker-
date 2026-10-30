"""
pipeline.py - everything between the sensors and your chase(): shared by follow.py and the simulator, so what is
tested in simulation is the same code that runs on the rover.

    pipe = Pipeline(detector, ext, planner, chase, motors, read_ir, clock, publish)
    pipe.on_state(state, now)        # every external-camera frame (tracker.py's state)
    pipe.on_yaw(yaw)                 # every frame, when there is no planner (yaw straight at the nearest opponent)
    pipe.on_lidar(vals, t, forward_mm_s, rotating, wall)   # every lidar frame
    pipe.step()                      # brain: evade if threatened, otherwise chase(yaw)
"""
import math
import time

import numpy as np

import floor as floor_mod
import evade
import plan as pl

YAW_FILTER = True       # smooth the follower's heading using the wheel commands (the marker's yaw has glitches that last
                        # 0.3-0.7 s: the camera sometimes flips it by 30+ degrees)
YAW_ALPHA = 0.15        # how strongly each camera reading corrects the wheel-based heading
YAW_GATE_DEG = 18       # a reading this far from the wheel-based heading is treated as a glitch ...
YAW_GATE_S = 1.2        # ... unless it persists this long, then the camera is believed (the wheel estimate has drifted)
YAW_DELAY_S = 0.2       # how old a camera reading is (compared with where the wheels say we were THEN)
TAPE_FIX_MAX_M = 0.15       # a floor-sensor correction bigger than this is ignored (misread)
CONTACT_PROJECT_M = 0.34    # an opponent's balloon this close to our estimated position: assume we are in contact with it ...
CONTACT_CENTRE_M = 0.13     # ... robot centre to robot centre when two rovers touch (MEASURE on the real rovers)
CONTACT_BLEND = 0.15        # how much of the radial error to remove per camera frame
DEAD_RECKON_NEAR_M = 0.5    # "right beside us" for the longer dead-reckoning window below
DEAD_RECKON_CLOSE_S = 5.0   # ... and this long while an opponent is right beside us: that is when the camera tends to lose us
                        # (the opponent blocks its view) and when we most need to know where we are
DEAD_RECKON_S = 1.5     # if the follower's marker is hidden (say an opponent's balloon is in the way), carry its last
                        # pose forward from the wheel commands for up to this long, then give up


class Pipeline:
    def __init__(self, detector, ext, planner, chase, motors, read_ir, clock=time.time, publish=None, motion=None, read_floor=None):
        self.detector, self.ext, self.planner = detector, ext, planner
        self.read_floor = read_floor         # optional: () -> [left, middle, right] booleans "this floor sensor sees tape" (floor.py)
        self._floor_t = -1e9
        self._floor_flags = [False, False, False]
        self._pose_dr = False                # the current pose is dead-reckoned (marker hidden), so a tape sighting may correct it
        self.motion = motion                 # () -> (left_cm_s, right_cm_s) of the wheel command in force (MotionTap)
        self._dr = None                      # last follower pose (t, x, y, yaw_deg), carried forward while hidden
        self._yaw = None                     # filtered heading (deg)
        self._yaw_t = None
        self._yaw_hist = []                  # (time, filtered heading), to compare a delayed reading with its own time
        self._gate_since = None
        self._dr_last = None
        self.publish = publish or (lambda yaw, target_yaw: None)
        self.yaw_to_leader = None            # what chase(yaw) gets: yaw to the next waypoint (or the nearest opponent)
        self.yaw_to_target = None            # straight at the target
        self.last_plan = None
        self.evader = evade.EvadeController(motors, read_ir, clock)
        self.pose = None                      # (x, y, yaw_deg, time): the newest follower estimate
        self._escaping = False
        self.brain = evade.Brain(detector, self.evader, chase, external=ext, clock=clock, guard=self)

    def _set(self, yaw, target_yaw="same"):
        self.yaw_to_leader = yaw
        self.yaw_to_target = yaw if target_yaw == "same" else target_yaw
        self.publish(self.yaw_to_leader, self.yaw_to_target)

    def pause(self):
        self.brain.paused = True

    def resume(self):
        self.brain.paused = False

    def toggle_pause(self):
        self.brain.paused = not self.brain.paused
        return self.brain.paused

    def on_yaw(self, yaw):
        self._set(yaw)

    def _filter_yaw(self, st, now):
        """Wheel-based heading corrected gently by the camera: glitchy marker yaws (flips lasting a few frames) are
        ignored, a real change that persists is believed."""
        f = st.get("follower")
        if not YAW_FILTER or f is None or f.get("dead_reckoned"):
            return st
        meas = f["yaw"]
        if self._yaw is None or self._yaw_t is None or now - self._yaw_t > 1.0:
            self._yaw, self._yaw_hist, self._gate_since = meas, [], None
        else:
            dt = max(0.0, min(0.5, now - self._yaw_t))
            if self.motion is not None:
                left, right = self.motion()
                self._yaw = evade.wrap(self._yaw + math.degrees((right - left) / evade.WHEELBASE_CM) * dt)
            then = self._yaw                                   # the heading the wheels say we had YAW_DELAY_S ago
            for (th, yh) in reversed(self._yaw_hist):
                if th <= now - YAW_DELAY_S:
                    then = yh
                    break
            innov = evade.wrap(meas - then)
            if abs(innov) > YAW_GATE_DEG:
                if self._gate_since is None:
                    self._gate_since = now
                if now - self._gate_since < YAW_GATE_S:
                    innov = 0.0                                # a glitch, for now: keep the wheel-based heading
                else:
                    innov, self._gate_since = innov, None      # it has persisted: believe the camera
                    self._yaw = evade.wrap(self._yaw + innov)
                    innov = 0.0
            else:
                self._gate_since = None
            self._yaw = evade.wrap(self._yaw + YAW_ALPHA * innov)
        self._yaw_t = now
        self._yaw_hist.append((now, self._yaw))
        self._yaw_hist = [h for h in self._yaw_hist if now - h[0] < 1.0]
        st = dict(st)
        st["follower"] = dict(f, yaw=self._yaw)
        return st

    def _dead_reckon(self, st, now):
        """Keep a follower pose going while its marker is hidden, from the wheel commands."""
        f = st.get("follower")
        if f is not None:
            self._dr = (now, f["x"], f["y"], f["yaw"])
            self._dr_last = now
            return st
        limit = DEAD_RECKON_S
        if self.ext is not None and self._dr is not None:
            fx, fy = self._dr[1], self._dr[2]
            if any(math.hypot(v["x"] - fx, v["y"] - fy) <= DEAD_RECKON_NEAR_M for k, v in st.items() if k != "follower"):
                limit = DEAD_RECKON_CLOSE_S
        if self._dr is None or self.motion is None or now - self._dr[0] > limit:
            return st
        t0, x, y, yaw = self._dr
        dt = max(now - (self._dr_last if self._dr_last is not None else now), 0.0)
        left, right = self.motion()
        yaw += math.degrees((right - left) / evade.WHEELBASE_CM) * dt
        x += (left + right) / 200.0 * math.cos(math.radians(yaw)) * dt
        y += (left + right) / 200.0 * math.sin(math.radians(yaw)) * dt
        x, y = self._contact_fix(st, x, y, yaw)
        self._dr, self._dr_last = (t0, x, y, yaw), now
        if YAW_FILTER:                                         # keep the heading filter running through the gap, so a glitchy
            self._yaw, self._yaw_t = evade.wrap(yaw), now      # reading when the marker reappears (a robot half-covering it)
            self._yaw_hist.append((now, self._yaw))            # is checked against the wheels, not adopted as the truth
            self._yaw_hist = [h for h in self._yaw_hist if now - h[0] < 1.0]
        st = dict(st)
        st["follower"] = {"x": x, "y": y, "yaw": yaw, "age": 0.0, "dead_reckoned": True}
        return st

    def floor_tape(self, now):
        """Poll the floor colour sensors (at most 10 times a second). Returns the [left, middle, right] flags. When the pose is
        only dead-reckoned and a sensor is on the tape although the estimate puts it inside the arena, move the estimate out
        to the edge (the tape is a landmark)."""
        if self.read_floor is None:
            return [False, False, False]
        if now - self._floor_t >= 0.1:
            self._floor_t = now
            try:
                self._floor_flags = list(self.read_floor())
            except Exception:
                self._floor_flags = [False, False, False]
            if any(self._floor_flags) and self._pose_dr and self.planner is not None and self.pose is not None:
                self._tape_fix()
        return self._floor_flags

    def _tape_fix(self):
        x, y, yaw, t = self.pose
        arena = self.planner.arena
        for i, on in enumerate(self._floor_flags):
            if not on:
                continue
            sx, sy = floor_mod.sensor_world(x, y, yaw, i)
            p = np.array([sx, sy])
            if not pl.inside(arena.poly, p):
                continue                                          # the estimate already has this sensor on/over the tape
            c = pl._closest_on_boundary(arena.poly, p)
            shift = c - p
            if float(np.linalg.norm(shift)) > TAPE_FIX_MAX_M:
                continue                                          # too far to be believable: a misread
            x, y = x + float(shift[0]), y + float(shift[1])
        self.pose = (x, y, yaw, t)
        if self._dr is not None:
            self._dr = (self._dr[0], x, y, self._dr[3])

    def forward_speed(self):
        """Forward speed of the wheel command in force, cm/s (0 if unknown)."""
        if self.motion is None:
            return 0.0
        left, right = self.motion()
        return (left + right) / 2.0

    def _contact_fix(self, st, x, y, yaw):
        """While the marker is hidden, an opponent that was in contact with us pushes us about and the wheel commands know nothing
        of it (the position drifts about 3.5 cm/s). But the camera still sees ITS balloon, and in contact we are a fixed
        distance from it, in front of its pin: its centre is BALLOON_BACK_M ahead of its balloon, towards us, and the two
        centres are CONTACT_CENTRE_M apart. So pull our estimate radially to that distance from its balloon."""
        best = None
        for k, v in st.items():
            if k == "follower":
                continue
            d = math.hypot(x - v["x"], y - v["y"])
            if best is None or d < best[0]:
                best = (d, v["x"], v["y"])
        if best is None or best[0] > CONTACT_PROJECT_M or best[0] < 1e-6:
            return x, y
        d, bx, by = best
        want = evade.BALLOON_BACK_M + CONTACT_CENTRE_M
        k = CONTACT_BLEND * (want - d) / d
        return x + (x - bx) * k, y + (y - by) * k

    def escape(self, now):
        """BOUNDARY ESCAPE: None, or {"away": heading in degrees that leads out of trouble, "yaw": our heading} when the
        rover is too close to the no-go zone or the tape. Needs the arena (arena.json), else it never triggers."""
        return self._edge(now, evade.ESCAPE_NOGO_M, evade.ESCAPE_TAPE_M, True)

    def clear_behind(self, now, dist_m):
        """True if the floor dist_m straight behind the rover is free: inside the tape (with the planner's margins) and clear
        of the no-go zone. False when unknown (no arena or a stale pose)."""
        if self.planner is None or self.pose is None or now - self.pose[3] > 0.6:
            return False
        x, y, yaw, _ = self.pose
        for k in range(1, int(dist_m / 0.05) + 1):
            p = np.array([x - k * 0.05 * math.cos(math.radians(yaw)), y - k * 0.05 * math.sin(math.radians(yaw))])
            if not pl.inside(self.planner.free, p):
                return False
            if self.planner.block is not None and pl.inside(self.planner.block, p):
                return False
        return True

    def edge_push(self, now):
        """HOLD-GROUND zone: like escape() but starting further out (SOFT_*_M), and with "depth" 0..1 = how deep into that
        zone we are. Used while facing an opponent that is shoving us toward an edge."""
        return self._edge(now, evade.SOFT_NOGO_M, evade.SOFT_TAPE_M, False)

    def _edge(self, now, nogo_m, tape_m, latch):
        if self.planner is None or self.pose is None or now - self.pose[3] > 0.6:
            return None
        x, y, yaw, t_pose = self.pose
        if self.motion is not None:                    # the camera pose is old (image + processing delay): where are we NOW?
            left, right = self.motion()
            lead = (now - t_pose) + evade.POSE_LAG_S
            x += (left + right) / 200.0 * math.cos(math.radians(yaw)) * lead
            y += (left + right) / 200.0 * math.sin(math.radians(yaw)) * lead
        p = np.array([x, y])
        arena = self.planner.arena
        rel = evade.ESCAPE_RELEASE_M if (latch and self._escaping) else 0.0
        away = np.zeros(2)
        if arena.nogo:
            c = pl._closest_on_boundary(arena.nogo, p)
            inside = pl.inside(arena.nogo, p)
            d = 0.0 if inside else float(np.linalg.norm(p - c))
            thr = nogo_m + rel
            if d < thr:
                v = (p - np.mean(pl.ccw(arena.nogo), axis=0)) if inside else (p - c)
                n = float(np.linalg.norm(v))
                away += (v / n if n > 1e-9 else np.array([1.0, 0.0])) * (thr - d)
        c = pl._closest_on_boundary(arena.poly, p)
        d = float(np.linalg.norm(p - c)) * (1.0 if pl.inside(arena.poly, p) else -1.0)
        thr = tape_m + rel
        if d < thr:
            v = (p - c) if d >= 0 else (c - p)                # either way this points back into the arena
            n = float(np.linalg.norm(v))
            away += (v / n if n > 1e-9 else np.array([1.0, 0.0])) * (thr - d)
        if float(np.linalg.norm(away)) < 1e-9:
            if latch:
                self._escaping = False
            return None
        if latch:
            self._escaping = True
        return {"away": math.degrees(math.atan2(away[1], away[0])), "yaw": yaw,
                "depth": min(1.0, float(np.linalg.norm(away)) / max(nogo_m, tape_m))}

    def on_state(self, st, now):
        st = self._filter_yaw(st, now)
        st = self._dead_reckon(st, now)
        f = st.get("follower")
        if f is not None:
            self.pose = (f["x"], f["y"], f["yaw"], now)
            self._pose_dr = bool(f.get("dead_reckoned"))
        opps = {k: (v["x"], v["y"]) for k, v in st.items() if k != "follower"}
        if self.ext is not None:
            self.ext.update(st.get("follower"), opps, now, wall=now)
        if self.planner is not None:
            self.planner.set_target(self.brain.defended_who)   # whoever made us evade becomes the new target
            p = self.planner.update(st, now)
            self.last_plan = p
            self._set(p.yaw_to_waypoint if p else None, p.yaw_to_target if p else None)

    def on_lidar(self, vals, t, forward_mm_s, rotating, wall=None):
        self.detector.update(vals, t, forward_mm_s, rotating, wall=wall)

    def get_yaw(self):
        return self.yaw_to_leader

    def step(self):
        return self.brain.step(self.yaw_to_leader)
