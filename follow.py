"""
follow.py - runs everything together.

    python follow.py --camera 1                           external camera + markers (tracker.py)
    python follow.py --onboard http://<esp32>:81/stream   the follower's own camera (onboard.py)
    python follow.py --onboard bridge --lidar-serial COM5 the follower's own camera AND lidar through the ESP-NOW bridge (no WiFi)
    python follow.py --camera 1 --onboard http://<esp32>:81/stream   TANDEM: both cameras (external primary, onboard sharpens the facing)
    python follow.py --camera 1 --onboard bridge --lidar-serial COM5  the same, camera + lidar over ESP-NOW
    add  --lidar-port 4210   (default) for the rear lidar;  --no-lidar to run without evasion

What runs
  main thread     camera window and vision. Publishes the variable  yaw_to_leader  (degrees, + = left, None = unseen).
                  With the external camera and an arena.json (from mark_arena.py) it is the yaw to the NEXT WAYPOINT
                  on a planned route round the no-go zone to a point behind your target; without them it points
                  straight at the nearest opponent. yaw_to_target always points straight at the target.
  lidar thread    receives the ESP32's lidar frames and watches the rear for a chaser (evade.py)
  brain thread    each step: a chaser closing in, seen by the rear lidar OR by the external camera (from ANY
                  direction) -> evade (turn to face it); otherwise call YOUR chase(yaw) from my_chase.py

With --onboard ALONE there is no external camera, so evasion uses the rear lidar only.

Press Esc in the video window to stop. The motors are stopped and the rover disconnected on exit.
Ctrl+C in the terminal also stops the rover.
"""
import argparse
import colorsys
import collections
import json
import math
import os
import threading
import time

from micromelon import *                                       # Motors, IR, RoverController, ...
from micromelon._robot_comms import MicromelonType as OPTYPE

import evade
import experiments
import lidar_rx
import settings
import my_chase
import plan
import pipeline

yaw_to_leader = None              # the variable your chase(yaw) gets: yaw to the next waypoint (or the nearest opponent)
yaw_to_target = None              # straight at the target, ignoring the route (for reference)
stop = threading.Event()
MASK_FILE = "lidar_mask.json"
OPEN_BROWSER = True               # open the dashboard in your browser at start (tests switch it off)
AUTO_CONNECT_S = 5.0              # the window counts down this long before connecting, so you can press R and change the rover ID
ROVER_ID = 1                      # YOUR rover's number (the ID on the rover / in the Micromelon app). --rover N overrides it



def _camera_arg(text):
    return int(text) if str(text).strip().isdigit() else str(text)


def _publish(yaw, target_yaw="same"):      # called by the vision code every frame
    global yaw_to_leader, yaw_to_target
    yaw_to_leader = yaw
    yaw_to_target = yaw if target_yaw == "same" else target_yaw


class MicromelonMotors:
    """The few motor actions evade.py needs, expressed with the micromelon module."""

    def turn_degrees(self, deg_left_positive, speed):
        Motors.turnDegrees(-deg_left_positive, speed)      # the module treats NEGATIVE degrees as a LEFT turn

    def write(self, left, right):
        Motors.write(left, right)                          # non-blocking: the wheels keep this speed until the next command

    def stop(self):
        Motors.write(0, 0)


def read_ir():
    left, right = IR.readAll()                             # [left, right] distances in cm
    return left, right


def read_floor_colours():
    """[[hue, r, g, b, bright] x 3] for the left, middle and right floor colour sensors."""
    return Colour.readAllSensors(CS.ALL)


def load_mask():
    if os.path.exists(MASK_FILE):
        import numpy as np
        mask = np.array(json.load(open(MASK_FILE)), bool)
        if mask.shape == (evade.GRID, evade.GRID):
            print("Loaded lidar mask:", [tuple(z) for z in zip(*mask.nonzero())])
            return mask
        print(f"{MASK_FILE} doesn't match GRID={evade.GRID}: ignoring it. Run learn_mask.py again.")
    else:
        print(f"No {MASK_FILE}: run learn_mask.py so your own balloon isn't mistaken for a chaser.")
    return None


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--onboard", metavar="SOURCE",
                    help="use the follower's own camera (ESP32 stream URL or index, or the word 'bridge' for frames sent through the ESP-NOW "
                         "bridge: needs --lidar-serial) instead of the external camera")
    ap.add_argument("--camera", type=_camera_arg, help="the EXTERNAL camera: its number (python tracker.py --scan lists them), or a URL for an IP camera / phone")
    ap.add_argument("--flip", action="store_true")
    ap.add_argument("--rover", type=int, default=ROVER_ID, help=f"rover ID for Bluetooth (default ROVER_ID at the top of follow.py = {ROVER_ID})")
    ap.add_argument("--lidar-port", type=int, default=4210)
    ap.add_argument("--lidar-serial", metavar="PORT", default=None,
                    help="get the lidar over USB serial from an ESP-NOW bridge ESP32 (e.g. COM5 or /dev/ttyUSB0) instead of WiFi UDP")
    ap.add_argument("--lidar-baud", type=int, default=921600, help="baud rate of the bridge (default 921600; must match espnow_bridge.ino)")
    ap.add_argument("--no-lidar", action="store_true", help="run without the rear lidar (no evasion!)")
    ap.add_argument("--max-speed", type=float, default=None,
                    help="global wheel speed cap in cm/s (default: MAX_SPEED_CM_S in evade.py, rover max 30)")
    ap.add_argument("--floor-sensors", action="store_true",
                    help="use the floor colour sensors as a tape detector (tape guard + position fix). NOT TESTED ON THE REAL ROVER: "
                         "start with the rover on bare floor inside the arena, it learns what the floor reads")
    ap.add_argument("--no-plan", action="store_true", help="don't plan routes: yaw points straight at the nearest opponent")
    ap.add_argument("--markers", choices=["post", "top"], default=None, help="follower marker layout: post = FOUR markers on the faces of a post (default), top = one flat marker on top")
    ap.add_argument("--window", action="store_true", help="use the plain OpenCV window (keys) instead of the browser dashboard")
    ap.add_argument("--no-browser", action="store_true", help="do not open the dashboard in the browser (open the printed address yourself)")
    ap.add_argument("--port", type=int, default=8765, help="dashboard port (default 8765; the next free one is used if it is busy)")
    ap.add_argument("--target", default="nearest", help='which opponent to chase: "nearest" (default) or a slot name A/B/C')
    args = ap.parse_args(argv)

    n_ok, n_bad = settings.load_saved()
    if n_ok or n_bad:
        print(f"Settings: loaded {n_ok} saved value(s) from {settings.SETTINGS_FILE}" + (f" ({n_bad} skipped: unknown or invalid)" if n_bad else ""))
    import tracker as _trk_layout
    if hasattr(_trk_layout, "set_layout"):
        _trk_layout.set_layout(args.markers or _trk_layout.MARKER_LAYOUT)
        print(f"Follower markers: {_trk_layout.MARKER_LAYOUT} ({'four markers on a post, IDs 0-3' if _trk_layout.MARKER_LAYOUT == 'post' else 'one flat marker, ID 0'}), {_trk_layout.MARKER_SIZE * 100:.1f} cm each")
    if args.max_speed is not None:
        evade.set_max_speed(args.max_speed)
    print(f"Max wheel speed: {evade.MAX_SPEED_CM_S:.0f} cm/s (applies to every motor command)")
    rc = RoverController()
    tap = evade.MotionTap(OPTYPE.MOTOR_SET)
    ready, conn = threading.Event(), {"err": None, "busy": False}
    use_dash = not args.window and not (args.onboard and args.camera is None)     # the dashboard is the default; --onboard on its own and --window keep the old window
    if use_dash and args.camera is None:
        args.camera = 0                                         # no --camera given: use camera 0 (add --camera N for another one)
        print("No --camera given: using camera 0. (Add --camera 1, 2 ... for another one.)")
    print("Starting " + ("the dashboard version" if use_dash else "the window version") + " ...")
    rid = {"id": args.rover, "typing": None, "go_at": time.time() + (max(AUTO_CONNECT_S, 8.0) if (use_dash and AUTO_CONNECT_S > 0) else AUTO_CONNECT_S)}
    events = collections.deque(maxlen=80)
    def log(tag, msg):
        events.append({"time": time.strftime("%H:%M:%S"), "tag": tag, "msg": str(msg)})
    run = {"started": not use_dash, "stopped": False}           # with the dashboard the rover waits for START    # the rover ID is chosen in the window

    def connect_rover():
        conn["busy"], conn["err"] = True, None
        try:
            print(f"Connecting to rover {rid['id']} by Bluetooth (the camera window / dashboard is already up)...")
            rc.connectBLE(rid["id"])
            rc.startRover()
            evade.install_speed_clamp(Motors)               # a chase that asks for 31 cm/s is clamped, not an error
            tap.install(rc)                                 # learn own speed/turning from motor commands
            if use_dash:
                pipe.pause()                                # stand still until START is pressed
            ready.set()
            log("ROVER", f"Rover {rid['id']} connected" + (": press START" if use_dash else ""))
            print(f"Rover {rid['id']} connected: " + ("press START in the dashboard." if use_dash else "the chase and the defence are ON."))
        except Exception as e:
            conn["err"] = e
            log("ERROR", f"Rover {rid['id']} connection failed: {e!r}")
            print(f"ROVER {rid['id']} CONNECTION FAILED:", repr(e), "-- press R in the window to change the ID, Enter to retry.")
        finally:
            conn["busy"] = False

    def connector():                                        # connects when the countdown ends or Enter is pressed
        while not stop.is_set() and not ready.is_set():
            if rid["typing"] is None and not conn["busy"] and conn["err"] is None and time.time() >= rid["go_at"]:
                connect_rover()
            time.sleep(0.1)

    tandem = bool(args.onboard) and args.camera is not None      # --camera N --onboard SRC: BOTH cameras, external is primary
    ext = None if (args.onboard and not tandem) else evade.ExternalWatch()      # external camera: chasers from any direction
    if ext is None:
        print("Onboard camera mode: evasion uses the rear lidar only (no external camera).")

    planner = None
    if (not args.onboard or tandem) and not args.no_plan:
        arena = plan.load_arena()
        if arena:
            planner = plan.Planner(arena, args.target)
            print("Waypoint planning ON: yaw now points at the next waypoint round the no-go zone.")
        else:
            print(f"No {plan.ARENA_FILE}: run mark_arena.py for waypoint planning. "
                  "Until then yaw points straight at the nearest opponent.")

    detector = evade.ChaserDetector(evade.GRID, mask=load_mask())
    read_floor = None
    if args.floor_sensors:
        import floor
        fs = floor.FloorSensors(read_floor_colours)
        try:
            base, spread = fs.calibrate(10, time.sleep)
            print(f"Floor sensors: bare floor reads brightness {base['bright']:.0f} (spread {spread:.0f}). "
                  "Anything much darker, or clearly red, counts as tape.")
            read_floor = fs.tape
        except Exception as e:
            print("Floor sensors NOT used (could not read them):", repr(e))
    ir_cache = {"left": None, "right": None, "t": 0.0}

    def read_ir_cached():                                   # the dashboard shows the last IR reading (no extra Bluetooth traffic)
        left, right = read_ir()
        ir_cache.update(left=left, right=right, t=time.time())
        return left, right

    pipe = pipeline.Pipeline(detector, ext, planner, my_chase.chase, MicromelonMotors(), read_ir_cached,
                             read_floor=read_floor,
                             publish=_publish, motion=lambda: (tap.left, tap.right))
    on_state = lambda st: pipe.safe_on_state(st, time.time()) if ready.is_set() else None     # called by tracker.py every frame
    brain = pipe.brain
    receiver = None
    jpeg_src = None
    if args.onboard == "bridge":                              # the follower's own camera comes through the ESP-NOW bridge, not WiFi
        if not args.lidar_serial:
            ap.error("--onboard bridge needs --lidar-serial PORT (the bridge's USB port carries the camera frames too)")
        import onboard
        jpeg_src = onboard.JpegFrameSource()
    feed = None
    if tandem:
        import onboard
        import tracker as _trk
        cam = jpeg_src if jpeg_src is not None else onboard.LatestFrame(args.onboard)
        feed = onboard.OnboardFeed(cam, onboard.MultiTracker(_trk.OPPONENT_HSV),
                                   lambda b, t: pipe.on_onboard(b, time.time()))
        print("TANDEM: external camera = primary, the follower's own camera sharpens the facing when the marker is hidden.")
    lid = {"vals": None, "n": 0, "t0": time.time()}

    def on_frame(vals, t):
        lid["vals"], lid["n"] = vals, lid["n"] + 1
        if ready.is_set():
            pipe.on_lidar(vals, t, tap.forward_mm_s, tap.rotating, time.time())
    if args.no_lidar:
        print("Running without the rear lidar: " + ("evasion uses the external camera only (blind to a chaser it can't see)."
              if not args.onboard else "WARNING: nothing will evade a chaser (onboard camera mode has no other source)."))
        if jpeg_src is not None:                              # ... but still read the camera frames from the bridge
            receiver = lidar_rx.SerialLidarReceiver(args.lidar_serial, lambda vals, t: None, baud=args.lidar_baud, on_jpeg=jpeg_src.push)
    elif args.lidar_serial:
        receiver = lidar_rx.SerialLidarReceiver(args.lidar_serial, on_frame, baud=args.lidar_baud,
                                                on_jpeg=jpeg_src.push if jpeg_src is not None else None)
        print(f"Listening for lidar{' and camera' if jpeg_src is not None else ''} on {args.lidar_serial} at {args.lidar_baud} baud (ESP-NOW bridge)")
    else:
        receiver = lidar_rx.LidarReceiver(args.lidar_port, on_frame)
        print(f"Listening for lidar on UDP port {args.lidar_port}")

    def _on_key(key):
        if not ready.is_set() and not conn["busy"]:         # rover ID entry (digits would otherwise pick the balloon slot)
            if rid["typing"] is None:
                if key in (ord("r"), ord("R")):
                    rid["typing"] = ""
                    return True
                if key in (13, 10):                         # Enter: connect now (or retry after a failure)
                    conn["err"], rid["go_at"] = None, 0.0
                    return True
            else:
                if ord("0") <= key <= ord("9") and len(rid["typing"]) < 3:
                    rid["typing"] += chr(key)
                elif key in (8, 127):
                    rid["typing"] = rid["typing"][:-1]
                elif key in (13, 10):
                    if rid["typing"]:
                        rid["id"] = int(rid["typing"])
                    rid["typing"], conn["err"], rid["go_at"] = None, None, 0.0
                return True
        if key in (ord("p"), ord(" ")):                     # PAUSE / RESUME: the rover stands still while paused
            print("PAUSED: rover stopped" if pipe.toggle_pause() else "RESUMED")

    def _status():
        if not ready.is_set():
            if conn["busy"]:
                return f"CONNECTING TO ROVER {rid['id']}..."
            if rid["typing"] is not None:
                return f"ROVER ID: {rid['typing']}_   (type the number, Enter = connect)"
            if conn["err"] is not None:
                return f"ROVER {rid['id']} FAILED (see terminal). R = change ID, Enter = retry"
            return f"ROVER {rid['id']}: connecting in {max(0.0, rid['go_at'] - time.time()):.0f}s.  R = change ID, Enter = now"
        return "PAUSED (P or space = resume)" if brain.paused else ("ALL OPPONENTS POPPED: stopped" if brain._last_mode == "done" else "")

    # ------------------------------------------------------------------ the browser dashboard (dashboard.py + dashboard.html)
    shared = {"jpg": None, "jpg_t": 0.0, "raw": None, "tr": None, "st": {}, "n": 0, "t0": time.time(), "fov": None, "on_jpg": None,
              "on_t": 0.0}

    def on_frame_cam(frame, raw, tr, st):                    # tracker.py hands over every annotated picture
        import cv2
        shared.update(raw=raw, tr=tr, st=st, n=shared["n"] + 1)
        if time.time() - shared["jpg_t"] >= 0.06:
            ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 72])
            if ok:
                shared["jpg"], shared["jpg_t"] = buf.tobytes(), time.time()

    def on_preview_cam(frame):                               # the picture while the floor marker is still being looked for
        import cv2
        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if ok:
            shared["jpg"], shared["jpg_t"] = buf.tobytes(), time.time()

    set_hooks = {"planner": lambda: planner.rebuild() if planner is not None else None,
                 "tracker": lambda: shared.__setitem__("rebuild_tr", True)}

    def on_tick(tr, cap):                                    # runs in the vision loop: changes that must happen there
        if shared.get("rebuild_tr") and hasattr(tr, "rebuild_geometry"):
            shared["rebuild_tr"] = False
            tr.rebuild_geometry()
        if planner is not None and getattr(tr, "tape_poly", None) is None and hasattr(tr, "set_tape"):
            tr.set_tape(planner.arena.poly)                  # lets a red balloon be told from the red tape
        v = shared.get("fov")
        if v is not None:
            shared["fov"] = None
            tr.set_fov(float(v))
            log("INFO", f"Field of view {float(v):.0f} deg: floor re-locked" if tr.lock_floor(cap) else f"Field of view {float(v):.0f} deg: could not re-lock the floor marker")

    def frame_for(name):
        import cv2
        if name == "onboard":
            if feed is None:
                return None
            fr, t = feed.cam.read()
            if fr is None:
                return None
            if time.time() - shared["on_t"] >= 0.06:
                ok, buf = cv2.imencode(".jpg", fr, [cv2.IMWRITE_JPEG_QUALITY, 72])
                if ok:
                    shared["on_jpg"], shared["on_t"] = buf.tobytes(), time.time()
            return shared["on_jpg"]
        return shared["jpg"]

    def hue_swatch(tr, slot):
        try:
            lo, hi = tr.opp_hsv[slot][0]
            h = (lo[0] + hi[0]) / 2.0
            r, g, b = colorsys.hsv_to_rgb(h / 180.0, 0.75, 0.95)
            return "#%02x%02x%02x" % (int(r * 255), int(g * 255), int(b * 255)), f"H {int(h)}"
        except Exception:
            return "#5b6b7f", "H -"

    def bearing_of(st, name):
        f, o = st.get("follower"), st.get(name)
        if not f or not o:
            return None
        return (math.degrees(math.atan2(o["y"] - f["y"], o["x"] - f["x"])) - f["yaw"] + 180.0) % 360.0 - 180.0

    def snapshot():
        now = time.time()
        tr, st = shared["tr"], dict(shared["st"] or {})
        # --- rover / run state
        if ready.is_set():
            rstate = "connected"
        elif conn["busy"]:
            rstate = "connecting"
        elif conn["err"] is not None:
            rstate = "failed"
        elif rid["go_at"] > now:
            rstate = "countdown"
        else:
            rstate = "idle"
        mode = brain._last_mode or "-"
        if not ready.is_set():
            rs = "waiting"
        elif mode == "done":
            rs = "done"
        elif not run["started"]:
            rs = "ready"
        elif run["stopped"]:
            rs = "stopped"
        elif brain.paused:
            rs = "paused"
        else:
            rs = "running"
        # --- cameras and lidar
        rates = shared.setdefault("rates", {})

        def rate(key, n):                                    # events per second over the last ~2 s
            t_prev, n_prev, r = rates.get(key, (now, n, 0.0))
            if now - t_prev >= 2.0:
                t_prev, n_prev, r = now, n, (n - n_prev) / (now - t_prev)
            elif key not in rates:
                t_prev, n_prev, r = now, n, 0.0
            rates[key] = (t_prev, n_prev, r)
            return r
        ext_alive = bool(ext.alive(now)) if ext is not None else (now - shared["jpg_t"] < 1.5 and shared["jpg"] is not None)
        chips = {"external": {"alive": ext_alive, "fps": rate("ext", shared["n"])}, "lidar": None, "onboard": None}
        if receiver is not None:
            chips["lidar"] = {"alive": bool(detector.lidar_alive(now)), "hz": rate("lid", lid["n"])}
        if feed is not None:
            chips["onboard"] = {"alive": bool(feed.alive(now)), "fps": rate("on", feed.frames)}
        # --- arena / positions / plan
        arena = None
        if planner is not None:
            xs = [p[0] for p in planner.arena.poly]
            ys = [p[1] for p in planner.arena.poly]
            arena = {"poly": [list(p) for p in planner.arena.poly], "nogo": [list(p) for p in planner.arena.nogo] if planner.arena.nogo else None,
                     "keepout": [list(p) for p in planner.block] if planner.block else None,
                     "summary": f"Saved: {max(xs) - min(xs):.2f} x {max(ys) - min(ys):.2f} m, " + ("no-go zone found." if planner.arena.nogo else "no no-go zone.")}
        f = st.get("follower")
        follower = {"x": f["x"], "y": f["y"], "yaw": f["yaw"]} if f else None
        opps = {n: {"x": v["x"], "y": v["y"], "seen": True} for n, v in st.items() if n != "follower"}
        plan_d = None
        if planner is not None:
            p = planner.last
            plan_d = {"target": (p.target if p else "") or "", "pref": planner.target_pref,
                      "path": [list(q) for q in p.path] if p and p.path else [], "waypoint": list(p.waypoint) if p and p.waypoint else None,
                      "goal": list(p.goal) if p and p.goal else None, "yaw_wp": p.yaw_to_waypoint if p else None,
                      "yaw_target": p.yaw_to_target if p else None, "blocked": bool(p.blocked) if p else False, "engaged": bool(p.engaged) if p else False}
        # --- lidar
        lidar = None
        if receiver is not None:
            v = lid["vals"]
            n = detector.n
            if v is not None and len(v) == n * n:
                cells = [float(x) if evade.MIN_MM <= x <= evade.MAX_MM else None for x in v]
                mask = [bool(m) for m in detector.mask.ravel()]
                valid = [c for c, m in zip(cells, mask) if c is not None and not m]
                lidar = {"n": n, "cells": cells, "mask": mask, "nearest_mm": min(valid) if valid else None,
                         "hz": chips["lidar"]["hz"] if chips["lidar"] else 0.0}
            else:
                lidar = {"n": n, "cells": [None] * (n * n), "mask": [bool(m) for m in detector.mask.ravel()], "nearest_mm": None, "hz": 0.0}
        # --- threat, timing
        T = brain.threat_now
        nz = lambda x: None if (x is None or (isinstance(x, float) and math.isnan(x))) else float(x)
        threat = {"active": bool(T.active), "who": T.who, "turn": nz(T.turn_deg), "offset": nz(T.offset_deg), "min_mm": nz(T.min_mm), "source": T.source}
        turn = abs(T.turn_deg) if (T.active and T.turn_deg is not None) else 180.0
        spin = evade.turn_time_s(turn)
        timing = {"pin_ext_m": evade.PIN_FROM_BALLOON_M, "pin_lidar_mm": evade.POP_REACH_MM, "delay_s": evade.EVADE_LATENCY_S,
                  "speed": evade.EVADE_SPEED, "spin_s": spin, "margin_s": evade.TTC_MARGIN_S,
                  "need_s": evade.EVADE_LATENCY_S + spin + evade.TTC_MARGIN_S}
        # --- slots, colours, "behind you"
        slots, colours, behind = {}, {}, ""
        if tr is not None:
            for sl in ("A", "B", "C"):
                sw, hue = hue_swatch(tr, sl)
                colours[sl] = sw
                b = bearing_of(st, sl)
                if b is None:
                    status = "Not seen"
                else:
                    status = f"In view, {b:+.0f} deg" if abs(b) <= 100 else f"Behind you, {b:+.0f} deg"
                    if abs(b) > 100 and not behind:
                        behind = sl
                if plan_d and plan_d["target"] == sl:
                    status += ". Current target"
                slots[sl] = {"swatch": sw, "hue": hue, "status": status}
        floor_flags = list(pipe._floor_flags) if (read_floor is not None and getattr(pipe, "_floor_flags", None) is not None) else None
        ir = {"left": ir_cache["left"], "right": ir_cache["right"], "age": now - ir_cache["t"]} if ir_cache["left"] is not None else None
        return {"now": now, "tandem": tandem,
                "rover": {"id": rid["id"], "state": rstate, "countdown": max(0.0, rid["go_at"] - now)},
                "run": {"state": rs, "mode": mode, "manual": brain.manual is not None},
                "chips": chips, "arena": arena, "follower": follower, "opps": opps, "plan": plan_d,
                "yaw": yaw_to_leader, "lidar": lidar, "threat": threat, "timing": timing, "slots": slots, "colours": colours,
                "behind": behind, "ir": ir, "floor": floor_flags,
                "settings": {"speed": evade.MAX_SPEED_CM_S, "fov": tr.fov_deg if tr is not None else 70.0, "camstop": brain.safety_camera_stop},
                "experiments": experiments.snapshot(),
                "log": list(events)}

    def command(c):
        k = c.get("cmd")
        if k == "connect":
            if not ready.is_set() and not conn["busy"]:
                try:
                    rid["id"] = int(c.get("id"))
                except (TypeError, ValueError):
                    return {"error": "bad rover id"}
                conn["err"], rid["go_at"] = None, 0.0
                log("ROVER", f"Connecting to rover {rid['id']}")
            return {"ok": True}
        if k == "experiment":                                # the Experiments panel: works before the rover is connected too
            try:
                v = experiments.set_value(str(c.get("id")), c.get("value"))
            except (KeyError, ValueError) as e:
                return {"error": str(e)}
            log("WARN", f"Experiment {c.get('id')}: {v}")
            return {"ok": True, "value": v}
        if k == "settings_list":
            return {"ok": True, "settings": settings.listing(), "saved_file": os.path.exists(settings.SETTINGS_FILE)}
        if k == "setting":
            try:
                r = settings.apply(str(c.get("key")), c.get("value"), set_hooks)
            except (KeyError, ValueError, TypeError) as e:
                return {"error": str(e)}
            log("WARN", f"Setting {c.get('key')} = {r['value']}" + (" (restart needed)" if r["restart"] else ""))
            return {"ok": True, **r}
        if k == "setting_reset":
            try:
                settings.reset_one(str(c.get("key")), set_hooks)
            except KeyError as e:
                return {"error": str(e)}
            return {"ok": True}
        if k == "settings_reset_all":
            settings.reset_all(set_hooks)
            log("INFO", "All settings back to the defaults in the code (not saved until you press Save)")
            return {"ok": True}
        if k == "settings_save":
            try:
                n = settings.save()
            except OSError as e:
                return {"error": f"could not write {settings.SETTINGS_FILE}: {e}"}
            log("INFO", f"Saved {n} changed setting(s) to {settings.SETTINGS_FILE}")
            return {"ok": True, "saved": n}
        if k == "experiments_reset":
            experiments.reset()
            log("INFO", "All experiments back to their defaults")
            return {"ok": True}
        if not ready.is_set():
            return {"error": "the rover is not connected yet"}
        if k == "start":
            run["started"], run["stopped"] = True, False
            pipe.resume()
            log("INFO", "START: the chase and the defence are running")
        elif k == "pause":
            if run["started"] and not run["stopped"]:
                log("INFO", "PAUSED: rover stopped" if pipe.toggle_pause() else "RESUMED")
        elif k == "stop":
            run["stopped"] = True
            brain.manual = None
            pipe.pause()
            try:
                Motors.write(0, 0)
            except Exception:
                pass
            log("INFO", "STOP: rover stopped (START to run again)")
        elif k == "mode":
            brain.manual = (0, 0) if c.get("mode") == "manual" else None
            log("INFO", "MANUAL drive" if brain.manual is not None else "AUTO: chase and defence")
        elif k == "drive":
            if brain.manual is not None:
                sp = 15
                brain.manual = {"fwd": (sp, sp), "back": (-sp, -sp), "left": (-12, 12), "right": (12, -12)}.get(c.get("dir"), (0, 0))
        elif k == "target":
            if planner is not None:
                planner.set_preference(str(c.get("name")))
                log("CHASE", f"Chase target: {planner.target_pref}")
        elif k == "speed":
            evade.set_max_speed(max(5.0, min(30.0, float(c.get("value")))))
            log("INFO", f"Speed limit {evade.MAX_SPEED_CM_S:.0f} cm/s")
        elif k == "fov":
            shared["fov"] = max(40.0, min(120.0, float(c.get("value"))))
        elif k == "safety":
            brain.safety_camera_stop = bool(c.get("camstop"))
            log("WARN" if not brain.safety_camera_stop else "INFO", "Camera fail-safe " + ("ON" if brain.safety_camera_stop else "OFF: the rover keeps driving if the camera goes quiet"))
        elif k == "colour":
            import cv2
            import numpy as np
            slot = c.get("slot")
            fr = shared["raw"] if c.get("cam") != "onboard" else (feed.cam.read()[0] if feed is not None else None)
            if slot in ("A", "B", "C") and fr is not None:
                h, w = fr.shape[:2]
                x, y = int(float(c["x"]) * w), int(float(c["y"]) * h)
                patch = cv2.cvtColor(fr[max(0, y - 4):y + 5, max(0, x - 4):x + 5], cv2.COLOR_BGR2HSV).reshape(-1, 3)
                hh, ss, vv = np.median(patch, axis=0)
                if c.get("cam") == "onboard" and feed is not None:
                    from tracker import hsv_ranges_around
                    feed.tr.hsv[slot] = hsv_ranges_around(int(hh), int(ss), int(vv))
                elif shared["tr"] is not None:
                    shared["tr"].set_opponent_color(slot, hh, ss, vv)
                log("INFO", f"Slot {slot} colour set: H={hh:.0f} S={ss:.0f} V={vv:.0f}")
        elif k == "quit":
            log("INFO", "Quit")
            stop.set()
        else:
            return {"error": "unknown command"}
        return {"ok": True}

    dash = None
    if use_dash:
        import dashboard
        dash = dashboard.Dashboard(snapshot, frame_for, command, port=args.port)
        print("=" * 70)
        print(f"  DASHBOARD:  {dash.url}")
        if OPEN_BROWSER and not args.no_browser:
            print("  Opening it in your browser. If nothing opens, copy the address above into a browser on THIS computer.")
            if not dash.open_browser():
                print("  (could not start a browser from here: open the address above yourself)")
        else:
            print("  Open that address in a browser on THIS computer.")
        print("=" * 70)
        log("INFO", "Dashboard started. Waiting for the rover to connect.")

        def eventer():                                       # turns mode changes into log lines
            last = None
            while not stop.is_set():
                m = brain._last_mode
                if m != last and m is not None:
                    tgt = planner.last.target if (planner is not None and planner.last) else ""
                    text = {"chase": ("CHASE", f"Chasing {tgt or 'the nearest opponent'}" + (f", yaw {yaw_to_leader:+.0f} deg" if yaw_to_leader is not None else "")),
                            "defend": ("EVADE", f"Defending: facing {brain.defended_who or 'the chaser'}"),
                            "evade": ("EVADE", f"Evading {brain.defended_who or 'a chaser'}"),
                            "escape": ("EVADE", "Boundary escape: driving away from the line"),
                            "tape": ("WARN", "Floor sensor saw tape: backing off"),
                            "no-camera": ("WARN", "No external camera picture: rover stopped"),
                            "done": ("INFO", "All opponents popped: standing still"),
                            "error": ("ERROR", "Error in the brain loop: wheels stopped (see terminal)")}.get(m)
                    if text:
                        log(*text)
                    last = m
                time.sleep(0.2)
        threading.Thread(target=eventer, daemon=True).start()

    def brain_when_ready():
        while not stop.is_set() and not ready.wait(0.2):
            pass                                            # nothing may drive until the rover is connected
        if ready.is_set():
            brain.run(stop, lambda: yaw_to_leader)

    threads = [threading.Thread(target=brain_when_ready, daemon=True)]
    if receiver or ext or feed:
        def watchdog():
            warned = {}
            time.sleep(5)
            while not stop.is_set():
                sources = []
                if receiver:
                    sources.append(("lidar", detector.lidar_alive()))
                if ext:
                    sources.append(("external camera", ext.alive()))
                if feed:
                    sources.append(("onboard camera", feed.alive()))
                for name, alive in sources:
                    if not alive and not warned.get(name):
                        print(f"WARNING: no {name} data. That part of the evasion is OFF until it comes back.")
                        warned[name] = True
                    elif alive and warned.get(name):
                        print(f"{name} data is back.")
                        warned[name] = False
                time.sleep(1)
        threads.append(threading.Thread(target=watchdog, daemon=True))
    for t in threads:
        t.start()
    threading.Thread(target=connector, daemon=True).start()          # the window below opens straight away

    try:
        if args.onboard and not tandem:
            import onboard
            onboard.main(None if jpeg_src is not None else args.onboard, on_yaw=_publish, cam=jpeg_src)
        else:
            import tracker
            tracker.main(args.camera, True if args.flip else None,
                         on_yaw=None if planner else _publish, on_state=on_state,
                         on_key=_on_key, status=_status,
                         on_draw=(lambda frame, tr: plan.draw_plan(frame, tr, planner)) if planner else None,
                         headless=use_dash, on_frame=on_frame_cam if use_dash else None,
                         should_stop=stop.is_set, on_tick=on_tick if use_dash else None,
                         on_preview=on_preview_cam if use_dash else None)
    finally:
        stop.set()
        if dash:
            dash.close()
        for t in threads:
            t.join(timeout=2)
        if feed:
            feed.close()
        if receiver:
            receiver.close()
        if ready.is_set():
            try:
                Motors.write(0, 0)                          # make sure it isn't left driving
            except Exception:
                pass
            for fn in (rc.stopRover, rc.end):
                try:
                    fn()
                except Exception:
                    pass


if __name__ == "__main__":
    main()
