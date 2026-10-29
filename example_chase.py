"""
example_chase.py - a complete, working chase() written the way a block program would be. Paste the idea into
my_chase.py, or copy this file over it. The simulator (simulate.py) drives the rover with exactly this code.

yaw = degrees to turn to face the next waypoint (+ = left, - = right), or None if nothing is seen.

chase() is called about 20 times a second and does NOT wait: each call just sets the wheel speeds (they keep that speed
until the next call), steering in proportion to yaw. That keeps the rover moving at a steady 22 cm/s, which matters: a
chase that does 'one short move, then wait for the camera' averages under 10 cm/s, slower than a hunter, and is also blind
to threats for the whole move.
"""
from micromelon import *


def chase(yaw):
    if yaw is None:
        Motors.write(-12, 12)                       # nothing seen: spin left slowly, looking around
        return
    if abs(yaw) > 25:                               # well off target: turn almost on the spot first
        w = max(-20, min(20, 0.5 * yaw))            # + yaw = target on the left = turn left = left wheel back, right forward
        Motors.write(-w, w)
    else:                                           # roughly lined up: drive, steering gently toward it
        steer = max(-8, min(8, 0.35 * yaw))         # + yaw: slow the left wheel
        Motors.write(22 - steer, 22 + steer)
