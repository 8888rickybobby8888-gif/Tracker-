"""
settings.py - every tunable variable in one list, for the dashboard's Settings page.

Each entry names a module-level setting (for example evade.EVADE_LATENCY_S). The code reads these live, so a change takes effect on the next step,
with a few exceptions marked `restart` (read once at start-up). `Save` writes the values that differ from the defaults to settings.json, and
follow.py loads that file at start, so your calibration survives a restart. `Reset` goes back to the defaults in the code.

Nothing here was tuned on the real rover: the defaults are the simulator's, and the geometry numbers (marker size and height, pin reach,
balloon height, robot radius ...) are MEASUREMENTS you must make on your own rover.
"""
import importlib
import json
import math
import os
from dataclasses import dataclass

SETTINGS_FILE = "settings.json"


@dataclass
class S:
    key: str                  # "module.ATTR"
    group: str
    label: str
    help: str
    kind: str = "float"       # float | int | bool | choice
    lo: float = 0.0
    hi: float = 1.0
    step: float = 0.01
    unit: str = ""
    restart: bool = False     # read once at start-up: the change applies after a restart
    hook: str = ""            # something that must be rebuilt after the change ("planner", "tracker")
    choices: tuple = ()
    default: object = None


G_GEOM, G_CAM, G_POP, G_TIME = "Your rover and arena (measure these)", "Camera and tracking", "Pop geometry", "Speed and timing"
G_LID, G_EXT, G_FACE, G_ESC = "Rear lidar", "Chaser detection (external camera)", "Facing a chaser", "Boundary escape"
G_PLAN, G_PIPE, G_ONB = "Route planner", "Heading and dead reckoning", "Onboard camera"

SETTINGS = [
    # ---- geometry
    S("tracker.MARKER_SIZE", G_GEOM, "Follower marker size", "Side of the printed black square on each follower marker. Measure it with a ruler.", lo=0.02, hi=0.15, step=0.0025, unit="m"),
    S("tracker.FLOOR_SIZE", G_GEOM, "Floor marker size", "Side of the black square of floor marker 8.", lo=0.05, hi=0.4, step=0.005, unit="m"),
    S("tracker.MARKER_HEIGHT", G_GEOM, "Marker height", "Floor to the centre of the follower's markers.", lo=0.0, hi=0.25, step=0.005, unit="m", hook="tracker"),
    S("tracker.TOP_MARKER_HEIGHT", G_GEOM, "Top marker height", "Floor to the centre of the FLAT marker on top of the tower (ID 0).", lo=0.0, hi=0.4, step=0.005, unit="m", hook="tracker"),
    S("tracker.POST_RADIUS", G_GEOM, "Post radius", "Four-marker layout: rover centre to the face of a marker.", lo=0.0, hi=0.12, step=0.005, unit="m", hook="tracker"),
    S("tracker.TILT_DEG", G_GEOM, "Marker tilt", "Four-marker layout: how far each face leans back (0 = vertical).", lo=0.0, hi=45.0, step=1.0, unit="deg", hook="tracker"),
    S("tracker.MARKER_LAYOUT", G_GEOM, "Marker layout", "tower3 = flat ID 0 on top + IDs 1 left, 2 back, 3 right; tower = flat ID 0 + faces 1-4 (with front); post = faces 0-3; top = one flat marker.", kind="choice", choices=("tower3", "tower", "post", "top"), restart=True),
    S("tracker.LEADER_HEIGHT", G_GEOM, "Balloon height", "Floor to the centre of an opponent's balloon (same for all).", lo=0.03, hi=0.45, step=0.005, unit="m"),
    S("plan.ROBOT_RADIUS_M", G_GEOM, "Rover radius", "Half the rover's longest dimension, from its centre (err on the big side).", lo=0.03, hi=0.25, step=0.005, unit="m", hook="planner"),
    S("pipeline.CONTACT_CENTRE_M", G_GEOM, "Centre to centre in contact", "Distance between two rover centres when they touch.", lo=0.05, hi=0.3, step=0.005, unit="m"),
    # ---- camera / tracking
    S("tracker.MIN_BLOB_AREA", G_CAM, "Smallest balloon blob", "Colour blobs smaller than this many pixels are ignored.", kind="int", lo=10, hi=3000, step=10, unit="px"),
    S("tracker.COLOR_TOL_H", G_CAM, "Colour tolerance: hue", "A clicked balloon colour accepts this much hue either side (0-179). Widen it if a balloon is lost when the light changes. Applies to the NEXT colour you set.", kind="int", lo=2, hi=40, step=1),
    S("tracker.COLOR_TOL_S", G_CAM, "Colour tolerance: saturation", "How far the colour's saturation may differ (0-255). Pale or washed-out balloons (often yellow) need more.", kind="int", lo=10, hi=200, step=5),
    S("tracker.COLOR_TOL_V", G_CAM, "Colour tolerance: brightness", "How far the brightness may differ (0-255). More = still found in shadow and glare.", kind="int", lo=10, hi=200, step=5),
    S("tracker.ALPHA", G_CAM, "Position smoothing", "1 = none (jittery), lower = steadier but slower to follow.", lo=0.05, hi=1.0, step=0.05),
    S("tracker.HOLD_S", G_CAM, "Hold a lost robot", "Keep the last position this long after the markers are lost.", lo=0.1, hi=2.0, step=0.05, unit="s"),
    S("tracker.FLOOR_ALPHA", G_CAM, "Floor frame smoothing", "1 = none, lower = steadier origin but lags a moving camera.", lo=0.05, hi=1.0, step=0.05),
    S("tracker.MIN_FACING", G_CAM, "Smallest marker facing", "cos of the angle between the marker's normal and the view ray (0.35 = about 70 degrees). Lower accepts slanted markers.", lo=0.0, hi=0.95, step=0.05),
    S("tracker.MAX_REPROJ_PX", G_CAM, "Marker fit tolerance", "Drop a marker pose whose corners do not fit to within this many pixels.", lo=0.5, hi=10.0, step=0.5, unit="px"),
    # ---- pop geometry
    S("evade.POP_REACH_MM", G_POP, "Lidar pop reach", "What the rear lidar reads when a chaser's pin touches your balloon.", lo=30, hi=300, step=5, unit="mm"),
    S("evade.BALLOON_BACK_M", G_POP, "Your balloon behind centre", "Your balloon sits this far behind your rover's centre.", lo=0.0, hi=0.3, step=0.005, unit="m"),
    S("evade.PIN_FROM_BALLOON_M", G_POP, "Pin tip beyond its balloon", "Along a rover's heading, from its own balloon to its pin tip.", lo=0.0, hi=0.4, step=0.005, unit="m"),
    S("evade.PIN_REACH_M", G_POP, "Your pin reach", "From your centre to an opponent's balloon when your pin touches it.", lo=0.04, hi=0.3, step=0.005, unit="m"),
    S("plan.BEHIND_M", G_POP, "Goal behind the balloon", "The chase aims this far behind the opponent's balloon point (read once; usually = your pin reach).", lo=0.0, hi=0.3, step=0.005, unit="m"),
    S("evade.HIT_RADIUS_M", G_POP, "Hit radius", "A pin tip within this of a balloon counts as a pop.", lo=0.02, hi=0.2, step=0.005, unit="m"),
    S("evade.WHEELBASE_CM", G_POP, "Effective track width", "For a spin on the spot (the Micromelon module's own turn model).", lo=10.0, hi=30.0, step=0.1, unit="cm"),
    # ---- speed / timing
    S("evade.MAX_SPEED_CM_S", G_TIME, "Speed limit", "Global cap on every wheel command (the rover's maximum is 30).", lo=5.0, hi=30.0, step=1.0, unit="cm/s"),
    S("evade.EVADE_SPEED", G_TIME, "Evasive turn speed", "Wheel speed for the evasive turn.", lo=5.0, hi=30.0, step=1.0, unit="cm/s"),
    S("evade.EVADE_LATENCY_S", G_TIME, "Camera and network delay", "From 'threat seen' to the wheels turning (camera + network + Bluetooth).", lo=0.0, hi=1.5, step=0.05, unit="s"),
    S("evade.TTC_MARGIN_S", G_TIME, "Safety margin", "Extra time added to the warning a chaser needs.", lo=0.0, hi=1.5, step=0.05, unit="s"),
    S("evade.NO_CAMERA_STOP_S", G_TIME, "Camera silence limit", "No external camera picture for this long and the rover stops.", lo=0.2, hi=3.0, step=0.1, unit="s"),
    # ---- lidar
    S("evade.GRID", G_LID, "Lidar resolution", "4 = 4x4 zones, 8 = 8x8 zones. Must match LIDAR_GRID in the ESP32 sketch.", kind="choice", choices=(4, 8), restart=True),
    S("evade.MIN_MM", G_LID, "Nearest reading used", "Ignore readings closer than this.", kind="int", lo=10, hi=200, step=5, unit="mm"),
    S("evade.MAX_MM", G_LID, "Farthest reading used", "Ignore readings farther than this (about your arena size).", kind="int", lo=300, hi=3500, step=50, unit="mm"),
    S("evade.DANGER_EXTRA_MM", G_LID, "Danger margin", "Anything within the pop reach plus this counts as a threat however it moves.", kind="int", lo=0, hi=300, step=5, unit="mm"),
    S("evade.NEAR_MM", G_LID, "Near while spinning", "While the rover spins, a standout object this close counts.", kind="int", lo=100, hi=1000, step=10, unit="mm"),
    S("evade.SALIENCE_MM", G_LID, "Standout depth", "An object must be this much nearer than the background to count.", kind="int", lo=20, hi=500, step=5, unit="mm"),
    S("evade.CLOSING_MIN_MM_S", G_LID, "Closing speed to count", "Must approach at least this fast to count as a chaser.", kind="int", lo=20, hi=800, step=10, unit="mm/s"),
    S("evade.CONFIRM_FRAMES", G_LID, "Frames to confirm", "Consecutive lidar frames needed to declare a threat.", kind="int", lo=1, hi=8, step=1),
    S("evade.CLEAR_S", G_LID, "Threat hold", "A threat stays active this long after the last hit.", lo=0.1, hi=2.0, step=0.05, unit="s"),
    S("evade.STALE_S", G_LID, "Stale data limit", "Lidar data older than this (wall clock) is ignored.", lo=0.1, hi=2.0, step=0.05, unit="s"),
    # ---- external-camera chaser detection
    S("evade.EXT_MIN_SPEED", G_EXT, "Chaser's minimum speed", "An opponent must move at least this fast to count as chasing.", lo=0.0, hi=0.4, step=0.01, unit="m/s"),
    S("evade.EXT_AIM_DEG", G_EXT, "Aim tolerance", "Its velocity must point within this angle of you.", kind="int", lo=10, hi=90, step=5, unit="deg"),
    S("evade.EXT_CLOSING_MIN", G_EXT, "Closing speed to count", "Closing speed between the two robots.", lo=0.0, hi=0.5, step=0.01, unit="m/s"),
    S("evade.EXT_DANGER_EXTRA_M", G_EXT, "Danger margin", "A balloon this close to yours, and behind you, counts however it moves.", lo=0.0, hi=0.3, step=0.005, unit="m"),
    S("evade.REAR_DANGER_DEG", G_EXT, "What counts as behind", "Further round than this from straight ahead.", kind="int", lo=60, hi=170, step=5, unit="deg"),
    S("evade.FRONT_SAFE_DEG", G_EXT, "Front is the chase's job", "Opponents within this angle of straight ahead are not a threat.", kind="int", lo=10, hi=90, step=5, unit="deg"),
    S("evade.RING_MIN_SPEED", G_EXT, "Active opponent speed", "An opponent moving at least this fast is ACTIVE.", lo=0.0, hi=0.3, step=0.01, unit="m/s"),
    S("evade.CLOSE_RING_M", G_EXT, "Close ring", "A moving opponent this close keeps you in defence.", lo=0.1, hi=1.0, step=0.01, unit="m"),
    S("evade.HARD_RING_M", G_EXT, "Hard ring", "Inside this a pin can reach you however the opponent moves.", lo=0.1, hi=0.8, step=0.01, unit="m"),
    S("evade.ACTIVE_HOLD_S", G_EXT, "Stay-active time", "An opponent seen moving counts as active for this long.", lo=0.0, hi=10.0, step=0.5, unit="s"),
    S("evade.RING_HOLD_S", G_EXT, "Defence hold", "A close opponent keeps you defending this long after it was last close.", lo=0.0, hi=3.0, step=0.1, unit="s"),
    S("evade.HOLD_FACING_DEG", G_EXT, "Already facing it", "Facing it within this: just hold, no turn needed.", kind="int", lo=3, hi=45, step=1, unit="deg"),
    S("evade.EXT_CONFIRM_FRAMES", G_EXT, "Frames to confirm", "Consecutive camera frames needed to declare a chaser.", kind="int", lo=1, hi=8, step=1),
    S("evade.EXT_CLEAR_S", G_EXT, "Chaser hold", "A chaser stays active this long after the last hit.", lo=0.1, hi=2.0, step=0.05, unit="s"),
    # ---- facing
    S("evade.TRACK_FACING", G_FACE, "Continuous facing", "Keep facing a chaser with continuous wheel commands.", kind="bool", restart=True),
    S("evade.TRACK_KP", G_FACE, "Turn gain", "Turn rate (deg/s) per degree of bearing error.", lo=0.2, hi=6.0, step=0.1),
    S("evade.TRACK_DELAY_S", G_FACE, "Camera delay (facing)", "How old the bearing is; it is carried forward by our own turning.", lo=0.0, hi=1.0, step=0.01, unit="s"),
    S("evade.TRACK_ALPHA", G_FACE, "Filter alpha", "How much of each new bearing is believed.", lo=0.05, hi=1.0, step=0.05),
    S("evade.TRACK_BETA", G_FACE, "Filter beta", "How fast the opponent's circling rate is learned.", lo=0.0, hi=0.6, step=0.01),
    S("evade.TRACK_GATE_DEG", G_FACE, "Glitch gate", "A bearing this far from the prediction is ignored for a moment.", kind="int", lo=5, hi=90, step=5, unit="deg"),
    S("evade.TRACK_MAX_DEG_S", G_FACE, "Fastest turn", "Fastest turn rate it will command.", kind="int", lo=30, hi=360, step=10, unit="deg/s"),
    S("evade.TRACK_DEADBAND_DEG", G_FACE, "Dead band", "Facing it this well and it is not circling: stay still.", lo=0.0, hi=15.0, step=0.5, unit="deg"),
    # ---- escape
    S("evade.ESCAPE_NOGO_M", G_ESC, "No-go escape distance", "Escape when the rover's centre is this close to the no-go zone.", lo=0.05, hi=0.4, step=0.005, unit="m"),
    S("evade.ESCAPE_TAPE_M", G_ESC, "Tape escape distance", "The same for the tape boundary (from the inside edge).", lo=0.05, hi=0.4, step=0.005, unit="m"),
    S("evade.ESCAPE_RELEASE_M", G_ESC, "Escape release margin", "It keeps escaping until this much further clear.", lo=0.0, hi=0.15, step=0.005, unit="m"),
    S("evade.ESCAPE_SPEED", G_ESC, "Escape speed", "Wheel speed for the escape drive.", lo=5.0, hi=30.0, step=1.0, unit="cm/s"),
    S("evade.POSE_LAG_S", G_ESC, "Pose lag", "Age of the camera's pose of the follower: the boundary check looks this far ahead.", lo=0.0, hi=0.6, step=0.01, unit="s"),
    S("evade.SOFT_NOGO_M", G_ESC, "Soft no-go distance", "Hold-ground experiment: start pushing back from this far out.", lo=0.1, hi=0.6, step=0.01, unit="m"),
    S("evade.SOFT_TAPE_M", G_ESC, "Soft tape distance", "The same for the tape.", lo=0.1, hi=0.6, step=0.01, unit="m"),
    S("evade.LOST_KEEP_S", G_ESC, "Lost-opponent memory", "An opponent that vanished this recently still counts as beside us.", lo=0.0, hi=10.0, step=0.5, unit="s"),
    S("evade.HOLD_WHEN_LOST_M", G_ESC, "Stand still when lost", "No pose and an opponent within this: stand still instead of searching.", lo=0.1, hi=1.5, step=0.05, unit="m"),
    # ---- planner
    S("plan.TAPE_MARGIN_M", G_PLAN, "Tape margin", "Extra clearance from the tape boundary.", lo=0.0, hi=0.4, step=0.005, unit="m", hook="planner"),
    S("plan.NOGO_MARGIN_M", G_PLAN, "No-go margin (opponent moving)", "Clearance from the no-go zone while anything moves.", lo=0.0, hi=0.4, step=0.005, unit="m", hook="planner"),
    S("plan.NOGO_MARGIN_CALM_M", G_PLAN, "No-go margin (calm)", "Smaller clearance while nothing moves.", lo=0.0, hi=0.4, step=0.005, unit="m", hook="planner"),
    S("plan.HYSTERESIS_M", G_PLAN, "Side hysteresis", "Keep the side you committed to unless the other is this much shorter.", lo=0.0, hi=0.3, step=0.01, unit="m"),
    S("plan.LEAD_S", G_PLAN, "Lead time", "Aim this far ahead of a moving opponent.", lo=0.0, hi=1.5, step=0.05, unit="s"),
    S("plan.AVOID_LEAD_S", G_PLAN, "Avoid lead time", "How far ahead the keep-out round moving opponents looks.", lo=0.0, hi=2.0, step=0.05, unit="s"),
    S("plan.STALL_S", G_PLAN, "Give up a target after", "Give up a target we have not got closer to for this long.", lo=2.0, hi=60.0, step=1.0, unit="s"),
    S("plan.SKIP_S", G_PLAN, "Skip a target for", "Do not pick it again for this long.", lo=2.0, hi=90.0, step=1.0, unit="s"),
    S("plan.ENGAGE_M", G_PLAN, "Stop weaving within", "Within this of the goal point, aim straight at the opponent.", lo=0.05, hi=0.8, step=0.01, unit="m"),
    S("plan.WAYPOINT_REACHED_M", G_PLAN, "Waypoint reached", "A waypoint closer than this counts as reached.", lo=0.01, hi=0.3, step=0.005, unit="m"),
    # ---- pipeline
    S("pipeline.YAW_FILTER", G_PIPE, "Heading filter", "Smooth the follower's heading using the wheel commands.", kind="bool"),
    S("pipeline.YAW_ALPHA", G_PIPE, "Camera correction", "How strongly each camera heading corrects the wheel-based heading.", lo=0.0, hi=1.0, step=0.01),
    S("pipeline.YAW_GATE_DEG", G_PIPE, "Heading glitch gate", "A reading this far from the wheel heading is treated as a glitch ...", kind="int", lo=3, hi=90, step=1, unit="deg"),
    S("pipeline.YAW_GATE_S", G_PIPE, "... unless it persists", "... this long, then the camera is believed.", lo=0.1, hi=6.0, step=0.1, unit="s"),
    S("pipeline.YAW_DELAY_S", G_PIPE, "Camera heading delay", "How old a camera reading is.", lo=0.0, hi=0.8, step=0.01, unit="s"),
    S("pipeline.CONTACT_BLEND", G_PIPE, "Contact correction", "How much of the radial error to remove per frame when in contact.", lo=0.0, hi=1.0, step=0.01),
    S("pipeline.DEAD_RECKON_S", G_PIPE, "Dead-reckoning window", "Carry the last pose forward this long when the marker is hidden.", lo=0.0, hi=6.0, step=0.1, unit="s"),
    S("pipeline.DEAD_RECKON_CLOSE_S", G_PIPE, "... with an opponent beside us", "A longer window while an opponent is right beside us.", lo=0.0, hi=12.0, step=0.5, unit="s"),
    S("pipeline.DEAD_RECKON_NEAR_M", G_PIPE, "'Beside us' distance", "Right beside us for the longer window.", lo=0.1, hi=1.5, step=0.05, unit="m"),
    # ---- onboard
    S("evade.ONBOARD_FRESH_S", G_ONB, "Reading lifetime", "An onboard reading older than this is ignored.", lo=0.05, hi=2.0, step=0.05, unit="s"),
    S("evade.ONBOARD_DELAY_S", G_ONB, "Reading delay", "How old a reading is when it arrives (capture + network).", lo=0.0, hi=1.0, step=0.01, unit="s"),
    S("evade.ONBOARD_SUSTAIN_S", G_ONB, "Carry the facing for", "Keep facing an opponent the external camera lost, while we still see it close.", lo=0.0, hi=6.0, step=0.1, unit="s"),
    S("evade.ONBOARD_CLOSE_M", G_ONB, "Close range", "... within this range.", lo=0.1, hi=1.5, step=0.05, unit="m"),
    S("evade.ONBOARD_FRONT_DEG", G_ONB, "Front cone", "... and within this angle of straight ahead.", kind="int", lo=10, hi=120, step=5, unit="deg"),
    S("evade.ONBOARD_CHASE_M", G_ONB, "Chase range", "Onboard bearing blended into the chase yaw within this range.", lo=0.1, hi=1.5, step=0.05, unit="m"),
    S("evade.ONBOARD_CHASE_FRONT_DEG", G_ONB, "Chase front cone", "... and within this angle of straight ahead.", kind="int", lo=10, hi=120, step=5, unit="deg"),
    S("evade.ONBOARD_CHASE_W", G_ONB, "Chase blend weight", "Weight of the onboard bearing in the blend.", lo=0.0, hi=1.0, step=0.05),
    S("onboard.BALLOON_DIAM_M", G_ONB, "Balloon diameter", "Real balloon diameter, for the range from its apparent size.", lo=0.03, hi=0.3, step=0.005, unit="m"),
]
BY_KEY = {s.key: s for s in SETTINGS}


def _mod(s):
    return importlib.import_module(s.key.rsplit(".", 1)[0])


def _attr(s):
    return s.key.rsplit(".", 1)[1]


def available(s):
    try:
        getattr(_mod(s), _attr(s))
        return True
    except (ImportError, AttributeError):
        return False


for _s in SETTINGS:                                           # the defaults the program started with
    try:
        _s.default = getattr(_mod(_s), _attr(_s))
    except (ImportError, AttributeError):
        _s.default = None


def current(s):
    return getattr(_mod(s), _attr(s))


def _coerce(s, v):
    if s.kind == "bool":
        return bool(v)
    if s.kind == "choice":
        for c in s.choices:
            if str(c) == str(v):
                return c
        raise ValueError(f"{v!r} is not one of {list(s.choices)}")
    x = float(v)
    if math.isnan(x) or math.isinf(x):
        raise ValueError("not a number")
    x = min(max(x, s.lo), s.hi)
    return int(round(x)) if s.kind == "int" else x


def apply(key, v, hooks=None):
    """Set one variable (clamped to its range). `hooks` = {"planner": fn, "tracker": fn} to rebuild what depends on it.
    Returns {"value", "restart", "clamped"}."""
    s = BY_KEY[key]
    if not available(s):
        raise KeyError(f"{key} is not available in this program")
    new = _coerce(s, v)
    clamped = s.kind in ("float", "int") and float(v) != float(new)
    if key == "evade.MAX_SPEED_CM_S":
        importlib.import_module("evade").set_max_speed(new)
    elif key == "tracker.MARKER_LAYOUT":
        setattr(_mod(s), _attr(s), new)                          # takes effect when the tracker is made (restart)
    else:
        setattr(_mod(s), _attr(s), new)
    if s.hook and hooks and hooks.get(s.hook):
        hooks[s.hook]()
    return {"value": current(s), "restart": s.restart, "clamped": clamped}


def reset_one(key, hooks=None):
    s = BY_KEY[key]
    if available(s):
        setattr(_mod(s), _attr(s), s.default)
        if key == "evade.MAX_SPEED_CM_S":
            importlib.import_module("evade").set_max_speed(s.default)
        if s.hook and hooks and hooks.get(s.hook):
            hooks[s.hook]()


def reset_all(hooks=None):
    for s in SETTINGS:
        reset_one(s.key, hooks)


def listing():
    out = []
    for s in SETTINGS:
        if not available(s):
            continue
        out.append({"key": s.key, "group": s.group, "label": s.label, "help": s.help, "kind": s.kind, "lo": s.lo, "hi": s.hi, "step": s.step,
                    "unit": s.unit, "restart": s.restart, "choices": list(s.choices), "value": current(s), "default": s.default,
                    "changed": current(s) != s.default})
    return out


def changed():
    return {s.key: current(s) for s in SETTINGS if available(s) and current(s) != s.default}


def save(path=SETTINGS_FILE):
    d = changed()
    with open(path, "w") as f:
        json.dump(d, f, indent=1, sort_keys=True)
    return len(d)


def load_saved(path=SETTINGS_FILE, hooks=None):
    """Apply settings.json (if there is one). Returns (applied, skipped)."""
    if not os.path.exists(path):
        return 0, 0
    try:
        d = json.load(open(path))
    except (OSError, ValueError):
        return 0, 0
    ok = bad = 0
    for k, v in d.items():
        try:
            apply(k, v, hooks)
            ok += 1
        except (KeyError, ValueError, TypeError):
            bad += 1
    return ok, bad
