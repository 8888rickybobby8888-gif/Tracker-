"""
my_chase.py - YOUR part. Put the chasing moves here.

chase(yaw) is called about 20 times a second whenever nothing is trying to catch you
(evading is handled for you in evade.py and takes over automatically).

    yaw  = how many degrees you must turn to face the opponent
           positive = the opponent is on your LEFT, negative = on your RIGHT, 0 = dead ahead
           None     = the opponent isn't seen right now (always check for this first!)

How to use it with the Micromelon editor
  1. Build a move in blocks (for example "turn toward the opponent, then drive forward").
  2. Copy the Python the editor shows next to the blocks.
  3. Paste it inside chase() below, and use the variable  yaw  wherever the blocks used a fixed angle.

Speed: every motor command is limited to evade.MAX_SPEED_CM_S (default 30, the rover's maximum) no matter what
  you write here, so you can't go faster than the cap. To use it as a number: Motors.write(evade.MAX_SPEED_CM_S, ...)

Rules so it keeps working
  - Keep each call SHORT (a quick turn or a short drive, well under a second). chase() is called again
    straight afterwards, so many small steps beat one long move. A long blocking move can't be
    interrupted, so an evasion would have to wait for it to finish.
  - Don't connect to the rover or call start/stop/end here. follow.py already did.
  - Turning: in the Micromelon module a NEGATIVE angle turns LEFT (Motors.turnDegrees(-20) = 20 degrees left).
    yaw is POSITIVE for "opponent on the left", so the sign flips: left turn = turnDegrees(-yaw).
"""
from micromelon import *
import evade                      # evade.MAX_SPEED_CM_S is the global speed cap (it is enforced for you anyway)


def chase(yaw):
    if yaw is None:
        return                                          # opponent not seen: do nothing (or put a search move here)
    a = min(abs(yaw), 60)                               # angle off, capped: full turn speed from 60 degrees on
    t = a / 60 * 30                                     # adaptive turn speed (your Math.scale(a, 0, 60, 0, 30))
    fwd = 22 * (1 - a / 60)                             # drive less the further off you are
    if yaw > 0:
        Motors.write(fwd - t, fwd + t)                  # opponent on the left: steer left
    else:
        Motors.write(fwd + t, fwd - t)                  # opponent on the right: steer right
