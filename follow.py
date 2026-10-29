"""
follow.py - runs everything together.

    python follow.py --camera 1                           external camera + markers (tracker.py)
    python follow.py --onboard http://<esp32>:81/stream   the follower's own camera (onboard.py)
    add  --lidar-port 4210   (default) for the rear lidar;  --no-lidar to run without evasion

What runs
  main thread     camera window and vision. Publishes the variable  yaw_to_leader  (degrees, + = left, None = unseen).
                  With the external camera and an arena.json (from mark_arena.py) it is the yaw to the NEXT WAYPOINT
                  on a planned route round the no-go zone to a point behind your target; without them it points
                  straight at the nearest opponent. yaw_to_target always points straight at the target.
  lidar thread    receives the ESP32's lidar frames and watches the rear for a chaser (evade.py)
  brain thread    each step: a chaser closing in, seen by the rear lidar OR by the external camera (from ANY
                  direction) -> evade (turn to face it); otherwise call YOUR chase(yaw) from my_chase.py

With --onboard there is no external camera, so evasion uses the rear lidar only.

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
                    help="use the follower's own camera (ESP32 stream URL or index) instead of the external camera")
    ap.add_argument("--camera", type=int)
    ap.add_argument("--flip", action="store_true")
    ap.add_argument("--rover", type=int, default=1, help="rover ID for Bluetooth (default 1)")
    ap.add_argument("--lidar-port", type=int, default=4210)
    ap.add_argument("--no-lidar", action="store_true", help="run without the rear lidar (no evasion!)")
    ap.add_argument("--max-speed", type=float, default=None,
                    help="global wheel speed cap in cm/s (default: MAX_SPEED_CM_S in evade.py, rover max 30)")
    ap.add_argument("--no-plan", action="store_true", help="don't plan routes: yaw points straight at the nearest opponent")
    ap.add_argument("--target", default="nearest", help='which opponent to chase: "nearest" (default) or a slot name A/B/C')
    args = ap.parse_args(argv)

    if args.max_speed is not None:
        evade.set_max_speed(args.max_speed)
    print(f"Max wheel speed: {evade.MAX_SPEED_CM_S:.0f} cm/s (applies to every motor command)")
    rc = RoverController()
    rc.connectBLE(args.rover)
    rc.startRover()

    tap = evade.MotionTap(OPTYPE.MOTOR_SET)
    tap.install(rc)                                         # learn own speed/turning from motor commands

    ext = None if args.onboard else evade.ExternalWatch()      # external camera: chasers from any direction
    if ext is None:
        print("Onboard camera mode: evasion uses the rear lidar only (no external camera).")

    planner = None
    if not args.onboard and not args.no_plan:
        arena = plan.load_arena()
        if arena:
            planner = plan.Planner(arena, args.target)
            print("Waypoint planning ON: yaw now points at the next waypoint round the no-go zone.")
        else:
            print(f"No {plan.ARENA_FILE}: run mark_arena.py for waypoint planning. "
                  "Until then yaw points straight at the nearest opponent.")

    detector = evade.ChaserDetector(evade.GRID, mask=load_mask())
    pipe = pipeline.Pipeline(detector, ext, planner, my_chase.chase, MicromelonMotors(), read_ir,
                             publish=_publish, motion=lambda: (tap.left, tap.right))
    on_state = lambda st: pipe.on_state(st, time.time())     # called by tracker.py every frame
    brain = pipe.brain
    receiver = None
    if args.no_lidar:
        print("Running without the rear lidar: " + ("evasion uses the external camera only (blind to a chaser it can't see)."
              if not args.onboard else "WARNING: nothing will evade a chaser (onboard camera mode has no other source)."))
    else:
        receiver = lidar_rx.LidarReceiver(
            args.lidar_port,
            lambda vals, t: pipe.on_lidar(vals, t, tap.forward_mm_s, tap.rotating, time.time()))
        print(f"Listening for lidar on UDP port {args.lidar_port}")

    threads = [threading.Thread(target=brain.run, args=(stop, lambda: yaw_to_leader), daemon=True)]
    if receiver or ext:
        def watchdog():
            warned = {}
            time.sleep(5)
            while not stop.is_set():
                sources = []
                if receiver:
                    sources.append(("lidar", detector.lidar_alive()))
                if ext:
                    sources.append(("external camera", ext.alive()))
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

    try:
        if args.onboard:
            import onboard
            onboard.main(args.onboard, on_yaw=_publish)
        else:
            import tracker
            tracker.main(args.camera, True if args.flip else None,
                         on_yaw=None if planner else _publish, on_state=on_state,
                         on_draw=(lambda frame, tr: plan.draw_plan(frame, tr, planner)) if planner else None)
    finally:
        stop.set()
        for t in threads:
            t.join(timeout=2)
        if receiver:
            receiver.close()
        try:
            Motors.write(0, 0)                              # make sure it isn't left driving
        except Exception:
            pass
        rc.stopRover()
        rc.end()


if __name__ == "__main__":
    main()
