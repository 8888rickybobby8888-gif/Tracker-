"""
experiments.py - every experimental option in one list, so the dashboard can switch them on and off while the program runs.

Each entry names a module-level setting (for example evade.USE_ONBOARD_CHASE). The settings are read live by the code, so a change takes effect
on the next step. The `note` is what the simulator measured (docs/EXPERIMENTS.md has the details); all of it is simulation, none of it
was tried on the real rover.

Kinds:  "bool"    on/off
        "value"   off = `off` (usually 0), on = `on` (the value that was tried)
        "choice"  one of `choices`
"""
import importlib
from dataclasses import dataclass, field


@dataclass
class Exp:
    id: str
    label: str
    desc: str
    note: str
    mod: str
    attr: str
    kind: str = "bool"
    on: object = True
    off: object = False
    choices: list = field(default_factory=list)
    default: object = None


EXPERIMENTS = [
    Exp("onboard_defence", "Onboard camera in the defence",
        "When the rover's own camera sees the chaser, its bearing replaces the external one, and it carries the facing for 2 s if the external camera loses you.",
        "Simulator, hunter: survived 122/180 against 78/180 without it (good link, 15 fps). Needs --onboard.",
        "evade", "USE_ONBOARD"),
    Exp("onboard_chase", "Onboard camera in the chase yaw",
        "When the rover's own camera sees your target close (within 0.6 m) and ahead, its bearing is blended into the yaw your chase() gets.",
        "Simulator, 48 seeds x 5 scenarios: 173 wins with it, 172 without. Better in three_calm (21 against 16), worse in hunter (24 against 29) and wanderer (33 against 37), and more tape frames. Off by default. Needs --onboard.",
        "evade", "USE_ONBOARD_CHASE"),
    Exp("plane_marker", "Heading from the floor plane",
        "The follower's heading comes from where the marker's four corners land on the floor, not from solvePnP.",
        "Fixes yaw changing when the camera moves (78 degrees of error down to about 5). Fights: same as before within the noise.",
        "tracker", "PLANE_MARKER"),
    Exp("line_first", "Line first: escape the line even when an opponent is on you",
        "Driving clear of the tape or the no-go zone takes priority over facing the opponent.",
        "Zero line crossings in 16/16 full-render hunter runs, but only 4 wins and 6 pops (before: 11 to 14 wins).",
        "evade", "ESCAPE_IN_CONTACT"),
    Exp("escape_drive", "Drive away from the line when no opponent is close",
        "Back to the inside when the rover gets near the tape or the no-go zone.",
        "On by default. Off only to see what it does.",
        "evade", "ESCAPE_DRIVE"),
    Exp("defend", "Keep facing a close opponent",
        "While an opponent circles close to you, face it and hold instead of chasing.",
        "On by default (it is what turned many pops into wins).",
        "evade", "DEFEND"),
    Exp("retarget", "Target whoever made you evade",
        "After defending against an opponent, that opponent becomes the new chase target.",
        "On by default (the sticky targeting you asked for).",
        "pipeline", "RETARGET_ON_THREAT"),
    Exp("nudge", "Back away from the edge while facing an opponent",
        "While facing an opponent near an edge, also move forward or back away from the edge.",
        "Off by default: it did not improve the fight.",
        "evade", "ESCAPE_NUDGE"),
    Exp("backoff", "Back away from a slower opponent in your face",
        "Reverse at 28 cm/s when an opponent's balloon is within 0.35 m, until the gap is 0.55 m.",
        "Fixed seed 13 but over 16 full-render seeds: 4 wins and 10 pops against 11 wins and 3 pops.",
        "evade", "BACKOFF_SPEED", "value", 28, 0),
    Exp("sidestep", "Sidestep a slower opponent that is shoving you",
        "Pivot 90 degrees to the open side and drive off (26 cm/s) when in contact near an edge.",
        "Fast simulator, 300 runs: survived 221 against 251; the pops happen during the pivot.",
        "evade", "SIDESTEP_SPEED", "value", 26, 0),
    Exp("hold", "Hold ground near an edge",
        "Push back toward an opponent that is shoving you toward the edge (6 cm/s at full depth).",
        "Survival 245 down to 176 to 197 of 300 in the fast simulator.",
        "evade", "HOLD_SPEED", "value", 6, 0),
    Exp("avoid", "Route round moving opponents that are not the target",
        "A keep-out circle of 0.25 m on where each moving opponent will be in 0.8 s.",
        "No effect against one hunter (the opponent to avoid is the target). Use with the kite target mode.",
        "plan", "AVOID_R_M", "value", 0.25, 0.0),
    Exp("target_mode", "How the chase target is chosen",
        "sticky: nearest, then fixate. switch: always the nearest. adaptive, active_first, calm_first, kite: see docs/EXPERIMENTS.md.",
        "The targeting study found no method that beat another. sticky is the default you asked for.",
        "plan", "TARGET_MODE", "choice", None, None, ["sticky", "switch", "adaptive", "active_first", "calm_first", "kite"]),
]
BY_ID = {e.id: e for e in EXPERIMENTS}


def _mod(e):
    return importlib.import_module(e.mod)


def available(e):
    try:
        getattr(_mod(e), e.attr)
        return True
    except (ImportError, AttributeError):
        return False


for _e in EXPERIMENTS:                                      # remember the defaults the program started with
    try:
        _e.default = getattr(_mod(_e), _e.attr)
    except (ImportError, AttributeError):
        _e.default = None


def value(e):
    v = getattr(_mod(e), e.attr)
    if e.kind == "bool":
        return bool(v)
    if e.kind == "value":
        return v != e.off
    return v


def set_value(exp_id, v):
    """Apply one experiment. Returns the new value; raises KeyError / ValueError on a bad id or value."""
    e = BY_ID[exp_id]
    if not available(e):
        raise KeyError(f"{exp_id} is not available in this program")
    if e.kind == "choice":
        if v not in e.choices:
            raise ValueError(f"{v!r} is not one of {e.choices}")
        new = v
    else:
        on = bool(v)
        new = (True if on else False) if e.kind == "bool" else (e.on if on else e.off)
    setattr(_mod(e), e.attr, new)
    return value(e)


def reset():
    for e in EXPERIMENTS:
        if available(e):
            setattr(_mod(e), e.attr, e.default)


def snapshot():
    out = []
    for e in EXPERIMENTS:
        if not available(e):
            continue
        cur = value(e)
        dflt = (bool(e.default) if e.kind == "bool" else (e.default != e.off if e.kind == "value" else e.default))
        out.append({"id": e.id, "label": e.label, "desc": e.desc, "note": e.note, "kind": e.kind, "value": cur,
                    "default": dflt, "choices": e.choices})
    return out
