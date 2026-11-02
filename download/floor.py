"""
floor.py - the rover's three FLOOR COLOUR SENSORS (left, middle, right, under the front) as a tape detector.

OPT-IN: `python follow.py --camera 1 --floor-sensors`. NOT TESTED ON THE REAL ROVER: the Micromelon docs say the sensor
readings "need calibrating for your situation", so this learns what the BARE FLOOR reads at start-up (start with the rover
on bare floor inside the arena) and calls anything much darker (black tape) or much redder (red tape) "tape".

What it gives the rest of the system (see pipeline.py):
  - a TAPE GUARD: if a sensor sees tape while the rover drives forward, it backs off at once, whatever the camera says;
  - a POSITION FIX: the tape under a sensor is a landmark at a known place, so when the follower's marker is hidden and its
    position is only dead-reckoned, a sensor on the tape pulls the estimate back to the arena edge.
The sensors face forward, so they do not see the tape behind the rover while it is being pushed backwards.
"""
import math

import numpy as np

# Where the three sensors are, in the rover frame (x forward, y left), metres: left, middle, right. MEASURE.
SENSOR_XY = [(0.05, 0.03), (0.05, 0.0), (0.05, -0.03)]
DARK_RATIO = 0.45      # a brightness below this x the floor's is black tape
RED_RATIO = 1.5        # red tape: red at least this x both green and blue ...
RED_MIN_DIFF = 30      # ... and at least this much stronger (0-255), so a faint reddish floor is not tape
CONFIRM = 2            # consecutive reads needed before a sensor counts as "on tape"


def classify(reading, floor):
    """reading / floor: [hue, red, green, blue, brightness] (Micromelon CS.ALL). True if it looks like tape."""
    r, g, b, bright = reading[1], reading[2], reading[3], reading[4]
    if bright < DARK_RATIO * floor["bright"]:
        return True
    other = max(g, b)
    return r > RED_RATIO * max(other, 1) and (r - other) >= RED_MIN_DIFF and floor["red_excess"] < RED_MIN_DIFF


class FloorSensors:
    def __init__(self, read):
        """read() -> [[hue, r, g, b, bright] x 3] for left, middle, right (micromelon Colour.readAllSensors())."""
        self.read = read
        self.floor = None
        self._seen = [0, 0, 0]

    def calibrate(self, n=10, sleep=lambda s: None):
        """Learn what bare floor reads. Rover on bare floor, inside the arena, away from the tape."""
        rows = []
        for _ in range(n):
            rows.extend(self.read())
            sleep(0.05)
        a = np.array(rows, float)
        med = np.median(a, axis=0)
        self.floor = {"bright": float(med[4]), "red_excess": float(med[1] - max(med[2], med[3]))}
        spread = float(np.std(a[:, 4]))
        return self.floor, spread

    def tape(self):
        """[left, middle, right] booleans: this sensor has seen tape CONFIRM reads in a row."""
        if self.floor is None:
            return [False, False, False]
        out = []
        for i, rd in enumerate(self.read()):
            self._seen[i] = self._seen[i] + 1 if classify(rd, self.floor) else 0
            out.append(self._seen[i] >= CONFIRM)
        return out


def sensor_world(x, y, yaw_deg, i):
    """World position of sensor i (0 left, 1 middle, 2 right) for a rover at (x, y) facing yaw_deg."""
    sx, sy = SENSOR_XY[i]
    a = math.radians(yaw_deg)
    return x + sx * math.cos(a) - sy * math.sin(a), y + sx * math.sin(a) + sy * math.cos(a)
