"""
mark_arena.py - tell the planner where the arena and the no-go square are, by clicking on the external camera view.

    python mark_arena.py --camera 1            (add --flip if your image is mirrored; same options as tracker.py)

1. Keep the floor marker in view. Its position is re-solved every frame, so the camera may move while you click:
   each click is turned into a floor position at that moment and the dots stay stuck to the floor.
2. Click the 4 corners of the arena: the INNER corners of the tape, where the rover can actually drive.
3. Click the 4 corners of the no-go square, at floor level (the base of the square, not its top).
   (If there is no no-go zone, press N to skip it.)
Keys: U undo the last click | Enter or S save arena.json | Esc quit without saving

The arena is saved relative to the FLOOR MARKER, so the floor marker must not move between now and match day
(tape it down). The camera can go anywhere. The FOV must be right (tune it with [ and ] in tracker.py first).
"""
import json
import math
import sys

import cv2
import numpy as np

import tracker


def order_ccw(points):
    """Sort 4 points counter-clockwise around their centre (so click order doesn't matter)."""
    c = np.mean(points, axis=0)
    return [tuple(map(float, p)) for p in sorted(points, key=lambda p: math.atan2(p[1] - c[1], p[0] - c[0]))]


def pixels_to_floor(tr, pixels):
    """Pixel clicks -> floor points (metres) on the floor plane z = 0, using the CURRENT floor frame."""
    out = []
    for px in pixels:
        p = tr._pixel_to_floor(px, 0.0)
        if p is None:
            raise ValueError("a click doesn't hit the floor (is it above the horizon?)")
        out.append(p)
    return out


def main(camera=None, flip=None, out_file="arena.json"):
    cap, size = tracker.open_camera(camera, flip)
    tr = tracker.Tracker(size=size)
    print("Looking for the floor marker: keep it in view...")
    if not tr.lock_floor(cap):
        sys.exit("Could not see the floor marker.")

    pending = []                   # pixel clicks from the mouse, converted to floor points in the loop
    pts = []                       # floor points (x, y) in metres, in click order
    skip_nogo = {"v": False}

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            pending.append((x, y))

    cv2.namedWindow("mark arena")
    cv2.setMouseCallback("mark arena", on_mouse)
    note = ""
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        tr.track_floor(frame)                                   # the camera may be moving
        status, _ = tr.floor_status()
        while pending:
            px = pending.pop(0)
            want = 4 if skip_nogo["v"] else 8
            if len(pts) >= want:
                continue
            if status in ("lost", "waiting"):
                note = "Floor marker not in view: click ignored"
                continue
            try:
                pts.append(pixels_to_floor(tr, [px])[0])
                note = ""
            except ValueError as e:
                note = str(e)
        # draw the points where they sit on the floor NOW (they follow the floor if the camera moves)
        if pts:
            img = tracker.project_floor(tr, [(x, y, 0.0) for x, y in pts])
            for i, c in enumerate(img):
                col = (255, 200, 0) if i < 4 else (120, 140, 255)
                cv2.circle(frame, tracker._pt(c), 6, col, -1, cv2.LINE_AA)
                cv2.putText(frame, str(i + 1), (int(c[0]) + 8, int(c[1]) - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2, cv2.LINE_AA)
            for lo, hi, col in ((0, 4, (255, 200, 0)), (4, 8, (120, 140, 255))):
                grp = img[lo:hi]
                if len(grp) >= 2:
                    cv2.polylines(frame, [np.array(grp, np.int32).reshape(-1, 1, 2)], len(grp) == 4, col, 1, cv2.LINE_AA)
        if len(pts) < 4:
            msg = f"Click arena corner {len(pts) + 1} of 4 (inner edge of the tape)"
        elif skip_nogo["v"] or len(pts) == 8:
            msg = "Press Enter or S to save (U = undo)"
        else:
            msg = f"Click no-go corner {len(pts) - 3} of 4 (N = no no-go zone)"
        cv2.putText(frame, msg, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(frame, f"floor: {status}" + (f"   {note}" if note else ""), (10, 62), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (80, 200, 255) if status != "ok" or note else (180, 180, 180), 2, cv2.LINE_AA)
        cv2.imshow("mark arena", frame)
        key = cv2.waitKey(30) & 0xFF
        if key == 27:
            print("Quit without saving.")
            break
        if key == ord("u") and pts:
            pts.pop()
            skip_nogo["v"] = False
        if key == ord("n") and len(pts) == 4:
            skip_nogo["v"] = True
        if key in (13, ord("s")) and (len(pts) == 8 or (len(pts) == 4 and skip_nogo["v"])):
            arena = order_ccw(pts[:4])
            nogo = order_ccw(pts[4:8]) if len(pts) == 8 else None
            json.dump({"arena": arena, "nogo": nogo}, open(out_file, "w"), indent=1)
            xs, ys = [p[0] for p in arena], [p[1] for p in arena]
            print(f"Saved {out_file}. Arena about {max(xs) - min(xs):.2f} x {max(ys) - min(ys):.2f} m in the floor frame.")
            if nogo:
                print("No-go zone corners:", [tuple(round(v, 3) for v in p) for p in nogo])
            print("Sanity check: do those sizes match a tape measure? If not, fix the FOV in tracker.py first.")
            break
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", type=int)
    ap.add_argument("--flip", action="store_true")
    a = ap.parse_args()
    main(a.camera, True if a.flip else None)
