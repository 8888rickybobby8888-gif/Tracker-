"""
tests/plot_paths.py - draw the follower's path with and without a randomly wandering opponent, on the fast simulator.

    python tests/plot_paths.py [--seeds 6] [--out sim_output/wanderer_paths.png]

Thin white lines = the follower's path with NO opponent in the way (just the static target A); coloured lines = its path
with B wandering at random; the dot is where B's balloon was when the follower's route first bent away from the
no-opponent route. Shows how an opponent that doesn't even aim at the follower still changes its trajectory.
"""
import math, os, sys
import cv2
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import fastsim as F
import simulate as S
import sim_world as sw

SCALE, PAD = 420.0, 40
W = int(1.2 * SCALE) + 2 * PAD + 10
H = int(1.13 * SCALE) + 2 * PAD + 30


def P(x, y):
    return (int(PAD + (x - sw.ARENA_RECT[0]) * SCALE), int(PAD + (sw.ARENA_RECT[3] - y) * SCALE))


def scenario(with_b):
    opps = [("A", 1.25, -0.30, 90.0, "static", 0.0, None)]
    if with_b:
        opps.append(("B", 0.30, -0.45, 0.0, "wander", 0.11, None))
    return dict(name="wanderer" if with_b else "alone", duration=60.0, follower=(0.45, 0.30, 0.0), opponents=opps, until=S.popped("A"))


def main():
    n = int(sys.argv[sys.argv.index("--seeds") + 1]) if "--seeds" in sys.argv else 6
    out = sys.argv[sys.argv.index("--out") + 1] if "--out" in sys.argv else "sim_output/wanderer_paths.png"
    img = np.full((H, W, 3), (30, 26, 22), np.uint8)
    cv2.fillPoly(img, [np.array([P(*p) for p in sw.ARENA], np.int32)], (52, 46, 40))
    a = sw.ARENA
    for (p, q), col in zip(zip(a, a[1:] + a[:1]), [(55, 55, 215), (6, 6, 6), (55, 55, 215), (6, 6, 6)]):
        cv2.line(img, P(*p), P(*q), col, 4, cv2.LINE_AA)
    cv2.fillPoly(img, [np.array([P(*p) for p in sw.NOGO], np.int32)], (60, 60, 140))
    cv2.polylines(img, [np.array([P(*p) for p in sw.NOGO], np.int32)], True, (150, 150, 255), 2, cv2.LINE_AA)
    sim0, _ = F.run(scenario(False), seed=0)
    base = [P(r["x"], r["y"]) for r in sim0.log[::6]]
    cv2.polylines(img, [np.array(base, np.int32)], False, (255, 255, 255), 2, cv2.LINE_AA)
    palette = [(91, 107, 255), (62, 207, 158), (242, 201, 76), (255, 160, 90), (200, 120, 255), (120, 220, 255)]
    rows = []
    for sd in range(n):
        sim, s = F.run(scenario(True), seed=sd)
        col = palette[sd % len(palette)]
        fp = [P(r["x"], r["y"]) for r in sim.log[::6]]
        bp = [P(r["opp"]["B"][0], r["opp"]["B"][1]) for r in sim.log[::6]]
        cv2.polylines(img, [np.array(bp, np.int32)], False, tuple(int(c * 0.45) for c in col), 1, cv2.LINE_AA)   # B's wander (dim)
        cv2.polylines(img, [np.array(fp, np.int32)], False, col, 2, cv2.LINE_AA)                               # the follower (bright)
        cv2.circle(img, fp[-1], 5, col, -1, cv2.LINE_AA)
        # how far the path strayed from the no-opponent path
        bx = np.array([[r["x"], r["y"]] for r in sim0.log]); dev = 0.0
        for r in sim.log:
            dev = max(dev, float(np.min(np.hypot(bx[:, 0] - r["x"], bx[:, 1] - r["y"]))))
        rows.append((sd, s["end"], s["duration"], dev))
    cv2.circle(img, P(1.25, -0.40), 7, sw.BGR["A"], -1, cv2.LINE_AA)
    cv2.putText(img, "A", (P(1.25, -0.40)[0] + 10, P(1.25, -0.40)[1] + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.5, sw.BGR["A"], 1, cv2.LINE_AA)
    cv2.putText(img, "white = path with no opponent in the way", (PAD, H - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (230, 230, 230), 1, cv2.LINE_AA)
    cv2.putText(img, "bright = follower with B wandering at random (dim = B's own path)", (PAD, H - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (230, 230, 230), 1, cv2.LINE_AA)
    os.makedirs(os.path.dirname(out) or ".", exist_ok=True)
    cv2.imwrite(out, img)
    for sd, end, dur, dev in rows:
        print(f"seed {sd}: {end} at {dur:.1f} s, path strayed up to {dev * 100:.0f} cm from the no-opponent path")


if __name__ == "__main__":
    main()
