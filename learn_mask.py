"""
learn_mask.py - tells the chaser detector which lidar zones are always blocked by YOUR OWN rover
(the balloon on its back, a stick, the body), so they are never mistaken for a chaser.

    python learn_mask.py            (ESP32 must be sending lidar data; keep the area behind the rover EMPTY)

Writes lidar_mask.json, which follow.py loads automatically.
"""
import json
import sys
import time

import evade
import lidar_rx

PORT = 4210
frames = []
rx = lidar_rx.LidarReceiver(PORT, lambda vals, t: frames.append(vals.copy()))
print("Listening for the lidar for 4 seconds. Keep everything away from behind the rover...")
time.sleep(4)
rx.close()
if len(frames) < 20:
    sys.exit(f"Only {len(frames)} frames received. Is the ESP32 sending to port {PORT}?")
det = evade.ChaserDetector(evade.GRID)
mask = det.learn_mask(frames)
json.dump(mask.astype(int).tolist(), open("lidar_mask.json", "w"))
print(f"{len(frames)} frames. Masked zones (row, col): {[tuple(z) for z in zip(*mask.nonzero())]}")
print("Saved lidar_mask.json")
