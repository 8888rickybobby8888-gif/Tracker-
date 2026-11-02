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
import json
import os
import threading
import time

from micromelon import *                                       # Motors, IR, RoverController, ...
from micromelon._robot_comms import MicromelonType as OPTYPE

import evade
import lidar_rx
import my_chase
import plan
import pipeline

yaw_to_leader = None              # the variable your chase(yaw) gets: yaw to the next waypoint (or the nearest opponent)
yaw_to_target = None              # straight at the target, ignoring the route (for reference)
stop = threading.Event()
MASK_FILE = "lidar_mask.json"
AUTO_CONNECT_S = 5.0              # the window counts down this long before connecting, so you can press R and change the rover ID
ROVER_ID = 1                      # YOUR rover's number (the ID on the rover / in the Micromelon app). --rover N overrides it



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
    ap.add_argument("--camera", type=int)
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
    ap.add_argument("--target", default="nearest", help='which opponent to chase: "nearest" (default) or a slot name A/B/C')
    args = ap.parse_args(argv)

    if args.max_speed is not None:
        evade.set_max_speed(args.max_speed)
    print(f"Max wheel speed: {evade.MAX_SPEED_CM_S:.0f} cm/s (applies to every motor command)")
    rc = RoverController()
    tap = evade.MotionTap(OPTYPE.MOTOR_SET)
    ready, conn = threading.Event(), {"err": None, "busy": False}
    rid = {"id": args.rover, "typing": None, "go_at": time.time() + AUTO_CONNECT_S}    # the rover ID is chosen in the window

    def connect_rover():
        conn["busy"], conn["err"] = True, None
        try:
            print(f"Connecting to rover {rid['id']} by Bluetooth (the camera window is already up)...")
            rc.connectBLE(rid["id"])
            rc.startRover()
            evade.install_speed_clamp(Motors)               # a chase that asks for 31 cm/s is clamped, not an error
            tap.install(rc)                                 # learn own speed/turning from motor commands
            ready.set()
            print(f"Rover {rid['id']} connected: the chase and the defence are ON.")
        except Exception as e:
            conn["err"] = e
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
    pipe = pipeline.Pipeline(detector, ext, planner, my_chase.chase, MicromelonMotors(), read_ir,
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
    on_frame = lambda vals, t: pipe.on_lidar(vals, t, tap.forward_mm_s, tap.rotating, time.time()) if ready.is_set() else None
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
                         on_draw=(lambda frame, tr: plan.draw_plan(frame, tr, planner)) if planner else None)
    finally:
        stop.set()
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
