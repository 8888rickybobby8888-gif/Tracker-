"""
evade.py - rear-lidar chaser detection and the "turn to face it" evasion.

Pure Python (numpy only). No hardware imports, so it can be tested with fake data.

Pieces
  ChaserDetector    rear lidar frames in -> "something is closing in from behind" out
  ExternalWatch     external-camera floor positions in -> "an opponent is coming at us, from ANY direction" out
  EvadeController   turns the rover to face the chaser (uses the side IR to pick a side when it is dead behind)
  MotionTap         learns how fast the rover is moving/turning by watching motor commands
  Brain             each step: evade if EITHER source reports a threat, otherwise call YOUR chase(yaw)

Conventions
  distances in millimetres; angles in degrees
  lidar offset: angle from the rover's REAR axis, positive toward the rover's LEFT
  turns: positive = LEFT (counter-clockwise), negative = right
  closing speed: how fast an object approaches the rover in the WORLD, with the rover's own motion removed:
      closing = own_forward_speed - d(distance)/dt
"""
import math
import time
from collections import deque
from dataclasses import dataclass, field

import numpy as np

# ----------------------------- CONFIG ---------------------------------------
# --- THE POP RULE: you only lose when an opponent's pin reaches your rear balloon. MEASURE THESE. ---
POP_REACH_MM = 100        # DEFENCE, rear lidar: what it reads when a chaser's pin is touching your balloon (MEASURE)
BALLOON_BACK_M = 0.10     # DEFENCE, camera: YOUR balloon sits this far BEHIND your rover's centre (MEASURE)
PIN_FROM_BALLOON_M = 0.20 # DEFENCE, camera: along a rover's heading, from its own balloon to its pin tip (MEASURE).
                          # The camera tracks opponents by their balloon, so this tells us where their pin is.
PIN_REACH_M = 0.12        # ATTACK: distance from YOUR centre to the opponent's balloon (as the camera sees it) at the
                          # moment YOUR pin touches it. plan.py aims at a point this far behind the opponent (MEASURE)
HIT_RADIUS_M = 0.06       # a pin tip within this of your balloon counts as a pop (balloon radius plus pin reach)
WHEELBASE_CM = 18.7       # EFFECTIVE track width for a spin on the spot. This is the Micromelon module's own turn
                          # calibration: each wheel travels 9.35 cm per radian of spin (5.25 cm half-track plus a 4.10 cm
                          # skid allowance), so a spin takes ~1.8x longer than the physical 10.5 cm track suggests.
                          # Check it on your rover: time a 360 degree spin and compare with turn_time_s(360).
EVADE_LATENCY_S = 0.4     # delay from "threat seen" to the wheels actually turning (camera + network + Bluetooth)
TTC_MARGIN_S = 0.4        # extra safety time on top of the turn time

GRID = 4                  # lidar resolution: 4 (4x4) or 8 (8x8). Must match the ESP32 sketch.
FOV_DEG = 60.0            # horizontal field of view (Pololu spec for the VL53L7CX)
COL0_IS_ROVER_LEFT = True # False if column 0 turns out to be the rover's RIGHT (do the hand-wave test)
USE_ROWS = None           # None = all rows, or e.g. [1, 2] to ignore rows that see the floor

MIN_MM = 30               # ignore readings closer than this (sensor minimum is ~20 mm)
MAX_MM = 1200             # ignore readings further than this: SET THIS TO ABOUT YOUR ARENA SIZE
DANGER_MM = None          # anything this close behind counts as a threat, however it moves. None = POP_REACH_MM + 50
DANGER_EXTRA_MM = 50
NEAR_MM = 300             # while the rover is spinning, a standout object this close counts
SALIENCE_MM = 120         # an object must be this much nearer than the background to count as one
CLOSING_MIN_MM_S = 150    # must approach at least this fast (world speed) to count as a chaser
TTC_TRIGGER_S = None      # None = automatic: time to turn round + latency + margin. Or set seconds to override.
CONFIRM_FRAMES = 2        # consecutive frames needed before declaring a threat
CLEAR_S = 0.4             # a threat stays active this long after the last hit
HISTORY_S = 0.3           # window used to measure how fast a distance is changing
STALE_S = 0.5             # lidar data older than this (wall clock) is ignored

# --- external camera (floor positions, METRES and seconds) ---
EXT_HISTORY_S = 0.5       # window used to measure velocities
EXT_MIN_SAMPLES = 5
EXT_MIN_SPAN_S = 0.25
EXT_MIN_SPEED = 0.08      # m/s: an opponent must be moving at least this fast to count as chasing
EXT_AIM_DEG = 40          # its velocity must point within this angle of the follower
EXT_CLOSING_MIN = 0.10    # m/s: closing speed between the two robots (both motions included)
EXT_TTC_S = None          # None = automatic (time to turn to face it + latency + margin). Or set seconds.
EXT_DANGER_M = None       # an opponent's balloon this close to YOUR balloon, and behind you, counts however it moves.
                          # None = PIN_FROM_BALLOON_M + 0.04
EXT_DANGER_EXTRA_M = 0.04
REAR_DANGER_DEG = 100     # "behind" for the rule above: further round than this from straight ahead
FRONT_SAFE_DEG = 40       # opponents within this angle of straight ahead are chase()'s job, not a threat
DEFEND = True             # while an opponent is circling close to you, keep FACING it (and don't chase) until it leaves
RING_MIN_SPEED = 0.05     # m/s: an opponent moving at least this fast is ACTIVE (it can attack). Must stay above the
                          # speed noise of your tracker or standing-still opponents look active (0.03 was too low).
HOLD_FACING_DEG = 15      # already facing it within this: just hold, no turn needed
RING_HOLD_S = 0.8         # a close opponent keeps you in defence for this long after it was last close
DEFEND_TURN_COOLDOWN_S = 0.25   # between small corrective turns while defending
CLOSE_RING_M = 0.40       # a MOVING opponent whose balloon is this close to yours, not ahead of you and not moving away,
                          # can swing round to your balloon even if it isn't heading straight at it: face it
ACTIVE_HOLD_S = 3.0       # an opponent seen moving counts as ACTIVE for this long, even if it pauses (a hunter stalks)
RING_OPEN_SPEED = 0.04    # ... unless you are pulling away from it faster than this (running from a slower hunter) ...
HARD_RING_M = 0.30        # ... but NEVER inside this distance (its balloon to yours): that close, a pin can reach you however
                          # you are moving, and a chase that swings your back toward it is exactly what it is waiting for
RING_AWAY_COS = 0.5       # ... or it is clearly moving AWAY from you (cos of the angle between its velocity and the line
                          # from you to it): a target you are chasing is not a threat
ESCAPE_NOGO_M = 0.09      # BOUNDARY ESCAPE: if the centre of the rover gets this close to the no-go zone (shoved by an opponent,
                          # or just drifting), stop whatever else it is doing and drive away from it. Top priority.
ESCAPE_TAPE_M = 0.08      # ... the same for the tape boundary (distance from the centre to the inside edge of the tape)
POSE_LAG_S = 0.2          # age of the camera's pose of the follower (image + processing): the boundary check looks this far ahead
ESCAPE_RELEASE_M = 0.03   # it keeps escaping until it is this much further clear (no flip-flopping at the edge)
ESCAPE_DRIVE = True       # when no opponent is close, drive away from the no-go zone / tape edge (no longer fights the defence)
SOFT_NOGO_M = 0.30        # HOLD GROUND: while FACING an opponent, from this far out (centre to the no-go zone) ...
SOFT_TAPE_M = 0.25        # ... or the tape, push back toward the opponent so it can't shove us over the edge
BACKOFF_SPEED = 28        # cm/s: facing a slower opponent that is in our face, back away from it (it is slower than us, so the gap opens)
BACKOFF_M = 0.35          # start backing off when its balloon is this close (to our centre) ...
BACKOFF_RELEASE_M = 0.55  # ... and keep going until the gap is this big, then let the normal chase run (it can't catch us)
BACKOFF_CLEAR_M = 0.50    # only if there is this much free floor behind us (inside the tape, outside the no-go zone)
HOLD_SPEED = 0            # cm/s of push at full depth (an opponent shoves at about 10 cm/s)
ESCAPE_NUDGE = False      # True = while facing an opponent near an edge, also move forward/back away from it.
                          # Tested: it drops survival against a hunter from 12/12 to 5/12, so it stays off.
ESCAPE_REVERSE_ONLY = False  # with ESCAPE_NUDGE: only back away from an edge (never drive TOWARD the opponent we are facing)
ESCAPE_SPEED = 20         # cm/s used for the escape drive (and for the forward/back nudge while facing an opponent)
TRACK_FACING = True       # while defending, keep FACING the opponent with continuous wheel commands (not discrete turns)
TRACK_KP = 1.8            # turn rate (deg/s) per degree of bearing error
TRACK_DELAY_S = 0.25      # camera + processing delay: the bearing it sees is this old, so it is carried forward by our own
                          # turning and the opponent's circling before use
TRACK_ALPHA = 0.5         # tracking filter: how much of each new bearing is believed (the rest is the prediction)
TRACK_BETA = 0.12         # ... and how fast the opponent's circling rate is learned
TRACK_GATE_DEG = 35       # a new bearing this far from the prediction is ignored (an occasional camera glitch) ...
TRACK_GATE_MAX = 2        # ... for at most this many readings in a row, then it is believed
TRACK_SLEW_DEG_S2 = 900   # the commanded turn rate may not change faster than this (no bang-bang)
TRACK_MAX_DEG_S = 170     # fastest turn rate it will command (also limited by EVADE_SPEED and the speed cap)
TRACK_DEADBAND_DEG = 3    # facing it this well and it is not circling: stay still
EXT_CONFIRM_FRAMES = 3
EXT_CLEAR_S = 0.5
EXT_STALE_S = 0.5         # external data older than this (wall clock) is ignored

MAX_SPEED_CM_S = 30.0     # GLOBAL SPEED CAP for the wheels, applied to EVERY motor command (your chase moves too).
                          # The rover itself tops out at 30. Change it live with set_max_speed(). Lower = safer.
LIDAR_BACK_M = 0.05       # the lidar sits this far behind the rover's centre
ALL_POPPED_CONFIRM_S = 1.5  # every opponent must stay gone this long before we believe the game is won and stop
GHOST_LOST_M = 0.30       # an opponent that disappears from the camera within this of you was probably just popped
LIDAR_MATCH_M = 0.30      # a lidar object within this of a tracked opponent's balloon IS that opponent
LIDAR_VETO_EXTRA_M = 0.15  # a still lidar object only counts if the camera sees an opponent within
                          # PIN_FROM_BALLOON_M + BALLOON_BACK_M + this of you
EVADE_SPEED = 25          # cm/s used for the evasive turn; never above MAX_SPEED_CM_S
EVADE_COOLDOWN_S = 1.2    # after an evasion, leave the threat alone for this long
BEHIND_DEG = 165          # a chaser this far round (or more) is "dead behind": pick the side by IR
ROTATING_CM_S = 5         # wheel speed difference above which the rover counts as turning
# -----------------------------------------------------------------------------


def set_max_speed(cm_s):
    """Change the global wheel speed cap (cm/s, 0-30). Takes effect on the very next motor command."""
    global MAX_SPEED_CM_S
    MAX_SPEED_CM_S = max(0.0, min(30.0, float(cm_s)))
    return MAX_SPEED_CM_S


def evade_speed():
    """Speed used for the evasive spin: EVADE_SPEED, but never above the global cap (and never zero)."""
    return max(1.0, min(EVADE_SPEED, MAX_SPEED_CM_S))


def wrap(a):
    """Wrap an angle to (-180, 180]."""
    a = (a + 180.0) % 360.0 - 180.0
    return 180.0 if a == -180.0 else a


def turn_time_s(turn_deg):
    """Seconds to spin on the spot by turn_deg at the evasion speed (a lower speed cap means a longer spin)."""
    return math.radians(abs(turn_deg)) * (WHEELBASE_CM / 2.0) / evade_speed()


EVADE_LATENCY_EXTRA_S = 0.0   # filled in by Brain: how long your chase() calls block (threats are only checked between them)


def lead_time_s(turn_deg):
    """How long before the pin would reach your balloon you must START turning: latency (including how long
    your chase() calls block) + the turn + margin."""
    return EVADE_LATENCY_S + EVADE_LATENCY_EXTRA_S + turn_time_s(turn_deg) + TTC_MARGIN_S


def _ttc_limit(override, turn_deg):
    return override if override is not None else lead_time_s(turn_deg)


@dataclass
class Threat:
    active: bool = False
    offset_deg: float = 0.0        # angle from the rear axis, + = rover's left
    min_mm: float = float("nan")
    ttc_s: float = float("nan")    # time to contact of the worst zone (nan if not from closing speed)
    zones: list = field(default_factory=list)
    turn_deg: float = None         # exact turn to face the threat (+ = left), when a camera supplied it
    source: str = ""               # "lidar", "external" or "both"
    who: str = ""                  # opponent name when known (external camera)
    static_only: bool = False      # lidar: every hit was "something is close", none was "something is closing in"


class ChaserDetector:
    def __init__(self, grid=GRID, mask=None):
        self.n = grid
        self.mask = np.zeros((grid, grid), bool) if mask is None else np.asarray(mask, bool)
        self._hist = {}
        self._confirm = 0
        self._last_hit_t = -1e9
        self._last_t = None
        self._state = Threat()
        self.last_update_wall = None

    # ---- geometry -----------------------------------------------------------
    def col_offset_deg(self, c):
        a = FOV_DEG / 2.0 - (c + 0.5) * FOV_DEG / self.n       # + = toward column 0's side
        return a if COL0_IS_ROVER_LEFT else -a

    # ---- calibration helper ---------------------------------------------------
    def learn_mask(self, frames, near_mm=250, fraction=0.8):
        """Call with frames recorded while nothing is behind the rover. Zones that are nearly always
        close (your own balloon, a stick, the body) get masked out."""
        arr = np.array([np.asarray(f, float).reshape(self.n, self.n) for f in frames])
        near = (np.nan_to_num(arr, nan=1e9) < near_mm).mean(axis=0)
        self.mask = near >= fraction
        return self.mask

    # ---- main entry --------------------------------------------------------------
    def update(self, frame_mm, t, own_forward_mm_s=0.0, rotating=False, wall=None):
        """frame_mm: n*n distances in mm (nan = no reading). t: sensor time in seconds."""
        n = self.n
        if self._last_t is not None and t < self._last_t:          # sender restarted
            self._hist.clear()
            self._confirm = 0
        self._last_t = t
        self.last_update_wall = time.time() if wall is None else wall

        d = np.asarray(frame_mm, float).reshape(n, n)
        rows = np.ones(n, bool)
        if USE_ROWS is not None:
            rows = np.zeros(n, bool)
            rows[list(USE_ROWS)] = True
        considered = (~self.mask) & rows[:, None]
        valid = considered & (d >= MIN_MM) & (d <= MAX_MM)         # nan comparisons are False

        eff = np.where(valid, d, MAX_MM)
        background = float(np.median(eff[considered])) if considered.any() else float(MAX_MM)

        hits = []
        live = set(zip(*[a.tolist() for a in np.nonzero(valid)]))
        for key in list(self._hist):
            if key not in live:
                del self._hist[key]

        for r, c in live:
            dist = float(d[r, c])
            h = self._hist.setdefault((r, c), deque())
            h.append((t, dist))
            while h and t - h[0][0] > HISTORY_S:
                h.popleft()
            slope = self._slope(h)
            salient = dist < background - SALIENCE_MM
            ttc = float("nan")
            hit = False
            kinetic = False
            if not rotating and salient and slope is not None:
                closing = own_forward_mm_s - slope
                if closing >= CLOSING_MIN_MM_S:
                    ttc = max(dist - POP_REACH_MM, 0.0) / closing
                    hit = ttc <= _ttc_limit(TTC_TRIGGER_S, 170)      # a rear chaser means turning about 170 deg
                    kinetic = hit
            if dist <= (DANGER_MM if DANGER_MM is not None else POP_REACH_MM + DANGER_EXTRA_MM):
                hit = True
            if rotating and salient and dist <= NEAR_MM:
                hit = True
            if hit:
                hits.append((r, c, dist, ttc, kinetic))

        if hits:
            self._confirm += 1
            self._last_hit_t = t
        else:
            self._confirm = 0

        if hits and self._confirm >= CONFIRM_FRAMES:
            w = np.array([1.0 / max(h[2], 1.0) for h in hits])
            ang = np.array([self.col_offset_deg(h[1]) for h in hits])
            ttcs = [h[3] for h in hits if not math.isnan(h[3])]
            self._state = Threat(True, float((w * ang).sum() / w.sum()),
                                 min(h[2] for h in hits), min(ttcs) if ttcs else float("nan"),
                                 [(h[0], h[1]) for h in hits], source="lidar", static_only=not any(h[4] for h in hits))
        elif self._state.active and (t - self._last_hit_t) < CLEAR_S:
            pass                                                     # hold until it has been quiet a while
        else:
            self._state = Threat()
        return self._state

    @staticmethod
    def _slope(h):
        if len(h) < 3 or h[-1][0] - h[0][0] < 0.08:
            return None
        ts = np.array([x[0] for x in h])
        ds = np.array([x[1] for x in h])
        ts = ts - ts.mean()
        den = float((ts * ts).sum())
        return float((ts * (ds - ds.mean())).sum() / den) if den > 0 else None      # mm/s

    def state(self, now=None):
        """Latest threat, or an inactive one if the lidar has gone quiet."""
        now = time.time() if now is None else now
        if self.last_update_wall is None or now - self.last_update_wall > STALE_S:
            return Threat()
        return self._state

    def lidar_alive(self, now=None):
        now = time.time() if now is None else now
        return self.last_update_wall is not None and now - self.last_update_wall <= STALE_S


def _velocity(hist):
    """Least-squares velocity (vx, vy) in m/s from a deque of (t, x, y), or None if too little data."""
    if len(hist) < EXT_MIN_SAMPLES or hist[-1][0] - hist[0][0] < EXT_MIN_SPAN_S:
        return None
    a = np.array(hist, float)
    t = a[:, 0] - a[:, 0].mean()
    den = float((t * t).sum())
    if den <= 0:
        return None
    return float((t * (a[:, 1] - a[:, 1].mean())).sum() / den), float((t * (a[:, 2] - a[:, 2].mean())).sum() / den)


class ExternalWatch:
    """Spots an opponent coming at the follower from ANY direction, using the external camera's floor
    positions (metres) for the follower (x, y, yaw) and each opponent (x, y).

    The camera tracks each opponent's BALLOON, so its pin tip is predicted PIN_FROM_BALLOON_M ahead of that point
    along its direction of travel. An opponent counts as a chaser when that pin tip is on course to reach YOUR
    balloon (BALLOON_BACK_M behind your centre) sooner than you can turn to face it (lead_time_s), with both
    robots' motion included. Opponents roughly straight ahead are ignored (facing them is chase()'s job), and
    something right behind you counts even if it is still."""

    def __init__(self):
        self._opp = {}
        self._fol = deque()
        self._follower = None
        self._confirm = 0
        self._last_hit_t = -1e9
        self._state = Threat()
        self.last_update_wall = None
        self._ring_until = -1e9
        self._active_until = {}                     # name -> time until which it counts as an ACTIVE opponent
        self._status = {}                           # name -> "live" | "popped" (vanished right beside us) | "lost" (vanished far off)
        self._gone_t = -1e9                         # when the last opponent vanished
        self._ghosts = []                           # (x, y) of opponents that vanished right beside us: almost surely popped

    def update(self, follower, opponents, t, wall=None):
        """follower: {"x","y","yaw"(deg)} or None.  opponents: {name: (x, y)}.  t: seconds."""
        self.last_update_wall = time.time() if wall is None else wall
        for name in list(self._opp):
            if name not in opponents:
                h = self._opp.pop(name)                   # lost: start its history afresh when it returns
                f = self._follower
                self._gone_t = t
                if h and f is not None and math.hypot(h[-1][1] - f["x"], h[-1][2] - f["y"]) <= GHOST_LOST_M:
                    self._ghosts.append((h[-1][1], h[-1][2]))     # gone while we were on top of it: it was popped,
                    self._status[name] = "popped"                 # but its body is still there for the lidar to see
                else:
                    self._status[name] = "lost"
        if follower is None:
            self._follower = None
            self._fol.clear()
            self._confirm = 0
            return self._decay(t)
        self._follower = follower
        if self._ghosts:                                  # an opponent that is back in view near a ghost was not popped
            self._ghosts = [g for g in self._ghosts
                            if all(math.hypot(g[0] - o[0], g[1] - o[1]) > LIDAR_MATCH_M for o in opponents.values())]
        self._fol.append((t, follower["x"], follower["y"]))
        while self._fol and t - self._fol[0][0] > EXT_HISTORY_S:
            self._fol.popleft()
        for name, (x, y) in opponents.items():
            self._status[name] = "live"
            h = self._opp.setdefault(name, deque())
            h.append((t, x, y))
            while h and t - h[0][0] > EXT_HISTORY_S:
                h.popleft()

        fvel = _velocity(self._fol) or (0.0, 0.0)
        fx, fy, fyaw = follower["x"], follower["y"], follower["yaw"]
        fc, fs = math.cos(math.radians(fyaw)), math.sin(math.radians(fyaw))
        balx, baly = fx - BALLOON_BACK_M * fc, fy - BALLOON_BACK_M * fs          # YOUR balloon
        danger_m = EXT_DANGER_M if EXT_DANGER_M is not None else PIN_FROM_BALLOON_M + EXT_DANGER_EXTRA_M
        hits = []
        for name, h in self._opp.items():
            px, py = h[-1][1], h[-1][2]                  # the opponent's balloon, which is what the camera tracks
            rx, ry = px - fx, py - fy
            d = math.hypot(rx, ry)
            if d < 1e-6:
                continue
            theta = wrap(math.degrees(math.atan2(ry, rx)) - fyaw)      # + = to the follower's left
            front = abs(theta) <= FRONT_SAFE_DEG
            dbal = math.hypot(px - balx, py - baly)
            ov = _velocity(h)
            speed = math.hypot(*ov) if ov else 0.0
            if speed >= RING_MIN_SPEED:
                self._active_until[name] = t + ACTIVE_HOLD_S
            active = t < self._active_until.get(name, -1e9)
            if not front and dbal <= danger_m and abs(theta) >= REAR_DANGER_DEG:    # right behind us, its pin could reach
                hits.append((name, dbal, 0.0, theta))
                continue
            if DEFEND and active and dbal <= CLOSE_RING_M:
                # An opponent that is (or recently was) moving and is close to your balloon can swing round to it, on
                # whichever side it is. Face it, unless you are clearly pulling away from it or it is moving away from you.
                rvx, rvy = (ov[0] - fvel[0], ov[1] - fvel[1]) if ov else (-fvel[0], -fvel[1])
                closing = -((px - balx) * rvx + (py - baly) * rvy) / max(dbal, 1e-6)       # + = getting closer
                away = (rx * ov[0] + ry * ov[1]) / (d * speed) if (ov and speed > 1e-3) else 0.0
                if (closing > -RING_OPEN_SPEED or dbal <= HARD_RING_M) and away < RING_AWAY_COS:
                    hits.append((name, dbal, max(dbal - 0.15, 0.0) / max(speed, EXT_MIN_SPEED), theta))
                    continue
            if front or ov is None or speed < EXT_MIN_SPEED:
                continue
            hx, hy = ov[0] / speed, ov[1] / speed                     # it points where it is going
            tipx, tipy = px + hx * PIN_FROM_BALLOON_M, py + hy * PIN_FROM_BALLOON_M
            qx, qy = tipx - balx, tipy - baly                          # its pin tip, relative to our balloon
            vx, vy = ov[0] - fvel[0], ov[1] - fvel[1]
            vv = vx * vx + vy * vy
            if vv < 1e-6:
                continue
            tca = max(-(qx * vx + qy * vy) / vv, 0.0)                  # when the tip comes closest to the balloon
            dmin = math.hypot(qx + vx * tca, qy + vy * tca)
            if dmin > HIT_RADIUS_M + 0.04:
                continue
            # at that moment its centre must be behind ours: it can't push through our body to reach the balloon
            if (px + ov[0] * tca - (fx + fvel[0] * tca)) * fc + (py + ov[1] * tca - (fy + fvel[1] * tca)) * fs > 0.03:
                continue
            if tca <= _ttc_limit(EXT_TTC_S, theta):
                hits.append((name, dbal, tca, theta))

        if hits:
            self._confirm += 1
            self._last_hit_t = t
        else:
            self._confirm = 0
        if hits and self._confirm >= EXT_CONFIRM_FRAMES:
            name, d, ttc, theta = min(hits, key=lambda h: h[2])
            self._state = Threat(True, offset_deg=wrap(180.0 - theta), min_mm=d * 1000.0, ttc_s=ttc,
                                 turn_deg=theta, source="external", who=name)
        else:
            self._decay(t)
        return self._state

    def any_active_within(self, radius_m, t):
        """True if an ACTIVE opponent (one seen moving in the last ACTIVE_HOLD_S) is within radius_m of the follower."""
        f = self._follower
        if f is None:
            return False
        return any(math.hypot(h[-1][1] - f["x"], h[-1][2] - f["y"]) <= radius_m and t < self._active_until.get(n, -1e9)
                   for n, h in self._opp.items() if h)

    def pop_pending(self, t):
        """True in the moment right after the LAST opponent vanished right beside us (probably popped, not yet confirmed):
        stand still instead of searching, so we don't wander into the tape while the camera makes up its mind."""
        return (bool(self._status) and not self._opp and all(s == "popped" for s in self._status.values())
                and t - self._gone_t < ALL_POPPED_CONFIRM_S)

    def all_popped(self, t):
        """True once EVERY opponent we have seen has vanished right beside us (popped) and none has been back in view for
        ALL_POPPED_CONFIRM_S. Any one that vanished far off (maybe just hidden) or reappears makes it False again."""
        return (bool(self._status) and not self._opp and all(s == "popped" for s in self._status.values())
                and t - self._gone_t >= ALL_POPPED_CONFIRM_S)

    def lidar_object_is_inactive(self, offset_deg, dist_mm, t):
        """True if the thing the REAR LIDAR sees (bearing offset_deg from the rear axis, + = rover's left, dist_mm away) is
        a tracked opponent that is NOT active (not seen moving lately): a parked or already popped opponent can't come at
        us, and a noisy lidar slope against one is a false alarm. False when no opponent is tracked near that spot
        (the camera may simply have lost it), so the lidar keeps its say."""
        f = self._follower
        if f is None:
            return False
        ang = math.radians(f["yaw"] + 180.0 - offset_deg)
        r = dist_mm / 1000.0 + LIDAR_BACK_M
        ox, oy = f["x"] + r * math.cos(ang), f["y"] + r * math.sin(ang)
        near = [n for n, h in self._opp.items() if h and math.hypot(h[-1][1] - ox, h[-1][2] - oy) <= LIDAR_MATCH_M]
        if near:
            return all(t >= self._active_until.get(n, -1e9) for n in near)
        return any(math.hypot(gx - ox, gy - oy) <= LIDAR_MATCH_M for gx, gy in self._ghosts)

    def distance(self, name):
        """Floor distance from the follower's centre to the opponent's tracked balloon, or None."""
        f, h = self._follower, self._opp.get(name)
        if f is None or not h:
            return None
        return math.hypot(h[-1][1] - f["x"], h[-1][2] - f["y"])

    def any_within(self, radius_m):
        """True if any currently tracked opponent is within radius_m of the follower."""
        f = self._follower
        if f is None:
            return False
        return any(math.hypot(h[-1][1] - f["x"], h[-1][2] - f["y"]) <= radius_m for h in self._opp.values() if h)

    def _decay(self, t):
        if not (self._state.active and (t - self._last_hit_t) < EXT_CLEAR_S):
            self._state = Threat()
        return self._state

    def state(self, now=None):
        now = time.time() if now is None else now
        if self.last_update_wall is None or now - self.last_update_wall > EXT_STALE_S:
            return Threat()
        return self._state

    def alive(self, now=None):
        now = time.time() if now is None else now
        return self.last_update_wall is not None and now - self.last_update_wall <= EXT_STALE_S


class FacingTracker:
    """While a threatening opponent is close, keep FACING it with continuous (non-blocking) wheel commands, so your
    balloon stays on the far side of you.

    The camera's bearing to the opponent is TRACK_DELAY_S old and noisy, so this runs a small tracking filter on it:
    it carries each reading forward by how far we have turned since and how fast the opponent circles us, ignores
    one-off glitches, learns the opponent's circling rate, and commands a smooth turn of Kp x error + that rate.
    Needs motors.write(left, right)."""

    def __init__(self, motors, clock=time.time):
        self.motors, self.clock = motors, clock
        self.active = False
        self._reset()

    def _reset(self):
        self._theta = None        # filtered bearing to the opponent NOW (deg, + = left)
        self._rate = 0.0          # how fast the opponent circles us (deg/s, world frame)
        self._w = 0.0             # the turn rate being commanded (deg/s, + = left)
        self._t = None
        self._t_meas = None       # when the last NEW camera reading arrived
        self._hist = deque()      # (time, commanded turn rate), to know how far we turned during the camera delay
        self._last_in = None
        self._bad = 0
        self._side = 0.0

    def _turned_since(self, t0, now):
        """Degrees we turned left between t0 and now, from the commands we sent."""
        total, prev_t, prev_w = 0.0, None, 0.0
        for (tt, w) in self._hist:
            if prev_t is not None:
                seg0, seg1 = max(prev_t, t0), min(tt, now)
                if seg1 > seg0:
                    total += prev_w * (seg1 - seg0)
            prev_t, prev_w = tt, w
        if prev_t is not None and now > max(prev_t, t0):
            total += prev_w * (now - max(prev_t, t0))
        return total

    def step(self, theta_meas, now, v_extra=0.0):
        """theta_meas: bearing of the opponent relative to your heading (+ = left), as last measured by the camera.
        v_extra: a forward (+) / backward (-) speed in cm/s to move at while turning (used to back away from an edge)."""
        theta_meas = wrap(theta_meas)
        dt = 0.05 if self._t is None else max(0.01, min(0.5, now - self._t))
        if self._theta is None:
            self._theta = theta_meas
            self._t_meas = now
        else:
            pred = wrap(self._theta + (self._rate - self._w) * dt)             # our turning shrinks it, its circling grows it
            if theta_meas != self._last_in:                                    # a NEW camera reading (not the same one again)
                fwd = wrap(theta_meas - self._turned_since(now - TRACK_DELAY_S, now) + self._rate * TRACK_DELAY_S)
                innov = wrap(fwd - pred)
                if abs(innov) > TRACK_GATE_DEG and self._bad < TRACK_GATE_MAX:
                    self._bad += 1
                    innov = 0.0
                else:
                    self._bad = 0
                self._theta = wrap(pred + TRACK_ALPHA * innov)
                dt_m = max(0.05, now - self._t_meas) if self._t_meas is not None else 0.15   # since the last NEW reading
                self._rate += TRACK_BETA * innov / dt_m
                self._rate = max(-120.0, min(120.0, self._rate))
                self._t_meas = now
            else:
                self._theta = pred
        self._last_in, self._t = theta_meas, now
        theta = self._theta
        if abs(theta) > 150 and self._side:                                    # dead behind: don't flip direction on noise
            theta = math.copysign(abs(theta), self._side)
        w = TRACK_KP * theta + self._rate
        if abs(theta) < TRACK_DEADBAND_DEG and abs(self._rate) < 8.0:
            w = 0.0
        w = max(-TRACK_MAX_DEG_S, min(TRACK_MAX_DEG_S, w))
        w = self._w + max(-TRACK_SLEW_DEG_S2 * dt, min(TRACK_SLEW_DEG_S2 * dt, w - self._w))     # smooth, no bang-bang
        half = WHEELBASE_CM / 2.0
        v = max(-evade_speed(), min(evade_speed(), math.radians(w) * half))      # cm/s of each wheel, spinning on the spot
        self._w = math.degrees(v / half)
        if abs(self._w) > 1.0:
            self._side = math.copysign(1.0, self._w)
        self._hist.append((now, self._w))
        while self._hist and now - self._hist[0][0] > 1.5:
            self._hist.popleft()
        cap = evade_speed()
        self.motors.write(max(-cap, min(cap, v_extra - v)), max(-cap, min(cap, v_extra + v)))
        self.active = True

    def stop(self):
        if self.active:
            try:
                self.motors.write(0.0, 0.0)
            except Exception:
                pass
        self.active = False
        self._reset()


class EvadeController:
    """Turns to face the chaser so your back (and balloon) is away from it."""

    def __init__(self, motors, read_ir=None, clock=time.time):
        self.motors = motors            # needs turn_degrees(deg_left_positive, speed) and stop()
        self.read_ir = read_ir          # optional: () -> (left_cm, right_cm)
        self.clock = clock
        self._cooldown_until = 0.0
        self.log = []

    def ready(self, now):
        return now >= self._cooldown_until

    def plan(self, threat):
        """Degrees to turn (+ = left) so the chaser ends up in front."""
        if threat.turn_deg is not None:             # a camera gave the exact direction of the chaser
            ang = wrap(threat.turn_deg)
        else:                                       # lidar only: from the rear column that is closing
            ang = wrap(180.0 - threat.offset_deg)   # direction of the chaser, counter-clockwise from forward
        if abs(ang) >= BEHIND_DEG:                  # right behind: choose the side with more room
            left = right = None
            if self.read_ir:
                try:
                    left, right = self.read_ir()
                except Exception:
                    pass
            ang = -180.0 if (left is not None and right is not None and right > left) else 180.0
        return ang

    def handle(self, threat, now):
        deg = self.plan(threat)
        self.log.append((now, deg, threat.source, threat.who))
        self.motors.turn_degrees(deg, evade_speed())   # blocks until the turn is done
        small = abs(deg) < 90 and threat.source in ("external", "both")
        self._cooldown_until = self.clock() + (DEFEND_TURN_COOLDOWN_S if (DEFEND and small) else EVADE_COOLDOWN_S)
        return deg


def decode_motor_packet(data, is_operation):
    """Decode a MOTOR_SET payload into (left_cm_s, right_cm_s). Speed bytes are signed 8-bit, scaled
    so 127 = 30 cm/s. For distance operations the direction is carried by the sign of the distances."""
    def s8(b):
        return b - 256 if b > 127 else b

    def s16(lo, hi):
        v = lo | (hi << 8)
        return v - 65536 if v > 32767 else v

    ls, rs = s8(data[0]) / 127.0 * 30.0, s8(data[1]) / 127.0 * 30.0
    if is_operation:
        ld, rd = s16(data[2], data[3]), s16(data[4], data[5])
        ls = abs(ls) * (1 if ld >= 0 else -1)
        rs = abs(rs) * (1 if rd >= 0 else -1)
    return ls, rs


def clamp_speed_bytes(data):
    """Return the motor payload with both wheel speeds limited to +/- MAX_SPEED_CM_S (speed bytes are signed,
    127 = 30 cm/s). Distances/other bytes are untouched, so a distance move just takes longer."""
    cap = max(0.0, min(30.0, MAX_SPEED_CM_S))
    limit = int(round(cap / 30.0 * 127))
    d = list(data)
    for i in (0, 1):
        v = d[i] - 256 if d[i] > 127 else d[i]
        d[i] = max(-limit, min(limit, v)) & 0xFF
    return d


class MotionTap:
    """Watches motor commands passing through the rover controller (wherever they come from, including
    your pasted block code) to estimate forward speed and whether the rover is turning. It also enforces the
    global speed cap MAX_SPEED_CM_S on every command."""

    def __init__(self, op_motor_set):
        self.op = op_motor_set
        self.left = 0.0
        self.right = 0.0

    def install(self, rc):
        orig_write, orig_op = rc.writeAttribute, rc.doMotorOperation

        def write_attr(op, data, *a, **k):
            if op == self.op:
                data = clamp_speed_bytes(data)                  # global speed cap, whoever sent the command
                self.left, self.right = decode_motor_packet(list(data), False)
            return orig_write(op, data, *a, **k)

        def do_op(op, data, *a, **k):
            if op == self.op:
                data = clamp_speed_bytes(data)
                self.left, self.right = decode_motor_packet(list(data), True)
            try:
                return orig_op(op, data, *a, **k)
            finally:
                self.left = self.right = 0.0          # the rover stops itself when the move completes

        rc.writeAttribute, rc.doMotorOperation = write_attr, do_op

    @property
    def forward_mm_s(self):
        return (self.left + self.right) / 2.0 * 10.0

    @property
    def rotating(self):
        return abs(self.left - self.right) > ROTATING_CM_S


class Brain:
    """One step: if something is closing in (lidar OR external camera), evade; otherwise run your chase(yaw)."""

    def __init__(self, detector, evader, chase, external=None, clock=time.time, guard=None):
        self.detector, self.evader, self.chase, self.external, self.clock = detector, evader, chase, external, clock
        self.guard = guard                                   # needs .escape(now) -> None or {"away": deg, "yaw": deg}
        motors = getattr(evader, "motors", None)
        self.facing = FacingTracker(motors, clock) if (TRACK_FACING and motors is not None and hasattr(motors, "write")) else None
        self._last_mode = None
        self.defended_who = ""                               # last opponent we had to defend against (the planner retargets it)
        self._backing = False                                # backing off from a slower opponent in our face
        self.paused = False                                  # the PAUSE button: wheels stopped, nothing else runs
        self._park_t = -1e9

    def _drive_away(self, esc):
        """Boundary escape: move away from the no-go zone / tape by the shortest way, forwards or backwards."""
        m = self.evader.motors
        if not hasattr(m, "write"):
            return
        err = wrap(esc["away"] - esc["yaw"])                 # + = the way out is to our left
        sign = 1.0 if abs(err) <= 100 else -1.0              # nearer to our back? then reverse out
        e = err if sign > 0 else wrap(err - 180.0)
        speed = min(ESCAPE_SPEED, evade_speed())
        if abs(e) > 35:                                      # not lined up: turn on the spot first
            w = max(-20.0, min(20.0, 0.5 * e))
            m.write(-w, w)
        else:
            steer = max(-6.0, min(6.0, 0.3 * e))
            m.write(sign * speed - steer, sign * speed + steer)

    def _park(self, mode):
        """Stand still (paused, or every opponent popped). The stop is re-sent about once a second, so a lost Bluetooth
        packet can't leave the wheels turning."""
        now = self.clock()
        if self._last_mode != mode or now - self._park_t >= 1.0:
            if self.facing is not None and self.facing.active:
                self.facing.stop()
            try:
                self.evader.motors.stop()
            except Exception:
                pass
            self._park_t = now
        self._last_mode = mode
        return mode

    def _halt(self):
        """Leaving 'chase' without a new move: stop, because chase() may have left the wheels running."""
        if self._last_mode in ("chase", "escape"):
            try:
                self.evader.motors.stop()
            except Exception:
                pass
        self._last_mode = "hold"

    @staticmethod
    def fuse(lidar, ext):
        """Evade if either source sees a threat. The external camera knows the exact direction, so it wins
        when both report; the lidar covers the rear when the external camera is blocked or has lost track."""
        if ext.active and lidar.active:
            return Threat(True, ext.offset_deg, min(ext.min_mm, lidar.min_mm), ext.ttc_s, ext.zones,
                          ext.turn_deg, "both", ext.who)
        if ext.active:
            return ext
        if lidar.active:
            return lidar
        return Threat()

    def step(self, yaw):
        now = self.clock()
        if self.paused:
            return self._park("paused")
        esc = self.guard.escape(now) if self.guard is not None else None
        lid = self.detector.state(now)
        if (lid.active and self.external is not None and self.external.alive(now)
                and self.external.lidar_object_is_inactive(lid.offset_deg, lid.min_mm, now)):
            lid = Threat()          # the camera says that object is an opponent that is parked or already popped: it can't
                                    # pop us unless WE swing our balloon into its pin, and turning away is exactly that move
        threat = self.fuse(lid, self.external.state(now) if self.external else Threat())
        if not threat.active and self.external is not None and (self.external.all_popped(now) or self.external.pop_pending(now)):
            return self._park("done")                      # every opponent is popped: the game is won, stand still
        facing_now = (threat.active and self.facing is not None and threat.source in ("external", "both")
                      and bool(threat.who))
        hunter_near = (esc is not None and self.external is not None
                       and self.external.any_active_within(CLOSE_RING_M + 0.15, now))
        if ESCAPE_DRIVE and esc is not None and not facing_now and not hunter_near:   # BOUNDARY ESCAPE: nothing to keep facing: get clear
            if self.facing is not None and self.facing.active:
                self.facing.stop()
            self._drive_away(esc)
            self._last_mode = "escape"
            return "escape"
        if threat.active:
            if threat.who:
                self.defended_who = threat.who
            turn = threat.turn_deg if threat.turn_deg is not None else wrap(180.0 - threat.offset_deg)
            if facing_now:
                theta = turn if abs(turn) < BEHIND_DEG else self.evader.plan(threat)   # dead behind: IR picks the side
                # an edge is near: keep facing the opponent but also back away from the edge along our own axis
                nudge = (ESCAPE_SPEED * math.cos(math.radians(wrap(esc["away"] - esc["yaw"])))
                         if (esc is not None and ESCAPE_NUDGE) else 0.0)
                if ESCAPE_REVERSE_ONLY:
                    nudge = min(nudge, 0.0)
                if HOLD_SPEED and self.guard is not None and hasattr(self.guard, "edge_push"):
                    soft = self.guard.edge_push(now)               # near an edge with an opponent in our face: hold our ground
                    if soft is not None:
                        nudge += HOLD_SPEED * soft["depth"] * math.cos(math.radians(wrap(soft["away"] - soft["yaw"])))
                if BACKOFF_SPEED and self.guard is not None and hasattr(self.guard, "clear_behind"):
                    d = self.external.distance(threat.who) if self.external is not None else None
                    if d is not None and (d < BACKOFF_M or (self._backing and d < BACKOFF_RELEASE_M)):
                        self._backing = self.guard.clear_behind(now, BACKOFF_CLEAR_M)
                    else:
                        self._backing = False
                    if self._backing:
                        nudge = -min(BACKOFF_SPEED, evade_speed())      # reverse, still facing it, until it falls behind
                self.facing.step(theta, now, nudge)
                self._last_mode = "track"
                return "track"
            if DEFEND and threat.source in ("external", "both") and abs(turn) <= HOLD_FACING_DEG:
                self._halt()
                return "hold"                                  # already facing it: don't turn away to chase
            if self.evader.ready(now):
                self.evader.handle(threat, now)
                self._last_mode = "evade"
                return "evade"
            if DEFEND:
                self._halt()
                return "hold"                                  # mid-cooldown with a threat about: don't chase into its pin
        elif self.facing is not None and self.facing.active:
            self.facing.stop()
        global EVADE_LATENCY_EXTRA_S
        t0 = self.clock()
        self.chase(yaw)
        took = max(self.clock() - t0, 0.0)
        EVADE_LATENCY_EXTRA_S = 0.7 * EVADE_LATENCY_EXTRA_S + 0.3 * took if took > 0 else EVADE_LATENCY_EXTRA_S * 0.9
        self._last_mode = "chase"
        return "chase"

    def run(self, stop_event, get_yaw, period=0.05):
        while not stop_event.is_set():
            try:
                self.step(get_yaw())
            except Exception as e:                      # keep the loop alive; show what went wrong
                print("brain error:", repr(e))
                time.sleep(0.5)
            time.sleep(period)
