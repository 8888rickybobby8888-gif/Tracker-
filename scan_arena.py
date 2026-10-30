"""
scan_arena.py - SCAN the arena: find the tape boundary and the no-go zone automatically and save arena.json.

    python scan_arena.py --camera 1          (add --flip if the picture is mirrored; same options as tracker.py)

How to use it
  1. Put the floor marker where it will stay (taped down) and keep it in view. Point the camera at the whole arena.
  2. Clear the arena of robots if you can (robots the tracker can see are ignored, but a clear floor is safest).
  3. Watch the overlay: GREEN = the arena (inner edge of the tape), ORANGE = the no-go zone. It averages many frames
     and shows how steady the result is. Press ENTER (or S) to save arena.json.
  Keys: R = start the averaging again | M = show what the scan sees (tape and objects) | Esc = quit without saving
  If it can't find the arena (tape gaps, strong glare, arena partly out of view) it says why. You can always fall back
  to clicking the corners with mark_arena.py. It works best on a floor that is LIGHTER than the black tape; on a very
  dark floor set USE_BLACK_TAPE = False below and rely on the red tape.

What it does: it re-draws the floor as seen from straight above (using the floor marker, so the camera can sit at any
angle), finds the RED and BLACK tape, takes the inner edge of the tape ring as the arena, then looks inside the arena
for anything that is not floor-coloured (a taped square or a coloured mat) and takes the biggest square-ish one as the
no-go zone. Corners are saved relative to the floor marker, like mark_arena.py does.
"""
import json
import math
import sys
import time
from collections import deque

import cv2
import numpy as np

import tracker as tk
from mark_arena import order_ccw

# ----------------------------- CONFIG ---------------------------------------
SCAN_PX_PER_M = 250            # resolution of the top-down view (250 = 4 mm per pixel)
RED_HUE = 10                   # red tape: hue within this of 0 (OpenCV hue 0-179) ...
RED_MIN_S, RED_MIN_V = 130, 90   # ... and vivid enough
BLACK_MAX_V = 40               # black tape: darker than this
FLOOR_DELTA = 28               # inside the arena, a colour this far from the floor's counts as "something there"
MIN_NOGO_SIDE_M = 0.06         # the no-go zone is at least this wide ...
MAX_NOGO_SIDE_M = 0.60         # ... and at most this wide
MAX_ASPECT = 2.0               # and roughly square (long side / short side)
MIN_FILL = 0.65                # how well it fills its bounding rectangle (hollow tape squares count as filled)
ROBOT_CLEARANCE_M = 0.14       # ignore anything this close to a robot the tracker can see
MIN_ARENA_SIDE_M = 0.30
AVERAGE_FRAMES = 15            # how many scans are averaged before the result is trusted
STEADY_MM = 25                 # corners must agree within this (spread) to be called steady
USE_BLACK_TAPE = True          # set False if your floor is as dark as the black tape (then only red tape is used)
MAX_DARK_FRACTION = 0.35       # warn if more than this share of the view is "black": the floor is probably too dark
# -----------------------------------------------------------------------------


class ScanError(Exception):
    pass


def _extent(tr, size):
    """Bounding box (xmin, xmax, ymin, ymax) in metres of the floor the camera can see."""
    w, h = size
    pts = []
    for px in [(0, 0), (w / 2, 0), (w, 0), (0, h / 2), (w, h / 2), (0, h), (w / 2, h), (w, h)]:
        p = tr._pixel_to_floor(px, 0.0)
        if p is not None and math.hypot(p[0], p[1]) < 6.0:
            pts.append(p)
    if len(pts) < 4:
        raise ScanError("the camera view does not reach the floor (check the camera angle and the floor marker)")
    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
    return min(xs), max(xs), min(ys), max(ys)


def rectify(frame, tr):
    """Top-down view of the floor plane. Returns (view, valid_mask, to_floor, to_px): to_floor(u, v) -> (x, y) metres."""
    xmin, xmax, ymin, ymax = _extent(tr, (frame.shape[1], frame.shape[0]))
    s = SCAN_PX_PER_M
    W, H = int((xmax - xmin) * s), int((ymax - ymin) * s)
    if W < 50 or H < 50 or W * H > 40e6:
        raise ScanError("the visible floor area is an odd size")
    T = tr.cam_from_floor
    H_fi = tr.K @ np.column_stack([T[:3, 0], T[:3, 1], T[:3, 3]])      # floor (x, y, 1) -> image pixel
    A = np.array([[s, 0, -xmin * s], [0, -s, ymax * s], [0, 0, 1.0]])    # floor -> top-down view pixel
    M = A @ np.linalg.inv(H_fi)
    view = cv2.warpPerspective(frame, M, (W, H), flags=cv2.INTER_LINEAR)
    valid = cv2.warpPerspective(np.full(frame.shape[:2], 255, np.uint8), M, (W, H), flags=cv2.INTER_NEAREST)
    valid = cv2.erode(valid, np.ones((5, 5), np.uint8)) > 0
    return view, valid, (lambda u, v: (xmin + u / s, ymax - v / s)), (lambda x, y: ((x - xmin) * s, (ymax - y) * s))


def tape_mask(view, valid):
    hsv = cv2.cvtColor(view, cv2.COLOR_BGR2HSV)
    h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    red = ((h <= RED_HUE) | (h >= 180 - RED_HUE)) & (s >= RED_MIN_S) & (v >= RED_MIN_V)
    black = (v <= BLACK_MAX_V) if USE_BLACK_TAPE else np.zeros_like(red)
    m = ((red | black) & valid).astype(np.uint8) * 255
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    return cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))


def _fit_quad(contour):
    """4 corners (float, pixel coords) of a roughly four-sided contour: corners from approxPolyDP, then refined by
    fitting a straight line to each side and intersecting neighbours."""
    peri = cv2.arcLength(contour, True)
    quad = None
    for eps in np.linspace(0.01, 0.06, 11):
        ap = cv2.approxPolyDP(contour, eps * peri, True).reshape(-1, 2)
        if len(ap) == 4:
            quad = ap.astype(np.float64)
            break
    if quad is None:
        raise ScanError("the tape ring is not four-sided (gaps in the tape, an object over it, or part of it out of view)")
    pts = contour.reshape(-1, 2).astype(np.float64)
    idx = [int(np.argmin(np.linalg.norm(pts - q, axis=1))) for q in quad]
    order = np.argsort(idx)
    quad, idx = quad[order], [idx[i] for i in order]
    lines = []
    n = len(pts)
    for i in range(4):
        a, b = idx[i], idx[(i + 1) % 4]
        seg = pts[a:b + 1] if b > a else np.vstack([pts[a:], pts[:b + 1]])
        k = max(2, int(len(seg) * 0.12))
        seg = seg[k:-k] if len(seg) > 4 * k else seg
        vx, vy, x0, y0 = cv2.fitLine(seg.astype(np.float32), cv2.DIST_L2, 0, 0.01, 0.01).ravel()
        lines.append((np.array([x0, y0], np.float64), np.array([vx, vy], np.float64)))
    out = []
    for i in range(4):
        (p1, d1), (p2, d2) = lines[i - 1], lines[i]
        den = d1[0] * d2[1] - d1[1] * d2[0]
        if abs(den) < 1e-6:
            raise ScanError("two sides of the tape ring are parallel where they should meet")
        t = ((p2[0] - p1[0]) * d2[1] - (p2[1] - p1[1]) * d2[0]) / den
        out.append(p1 + t * d1)
    return np.array(out)


def scan_frame(frame, tr, exclude=()):
    """Find the arena (and no-go zone) in one frame. exclude: floor points (x, y) of robots to ignore.
    Returns {"arena": [4 corners], "nogo": [4 corners] or None, "debug": {...}} in the floor frame (metres).
    Raises ScanError with a plain-English reason if it can't."""
    view, valid, to_floor, to_px = rectify(frame, tr)
    tape = tape_mask(view, valid)
    warnings = []
    dark = float(np.mean((cv2.cvtColor(view, cv2.COLOR_BGR2HSV)[..., 2] <= BLACK_MAX_V)[valid])) if valid.any() else 0.0
    if USE_BLACK_TAPE and dark > MAX_DARK_FRACTION:
        warnings.append(f"{dark * 100:.0f}% of the view is as dark as the black tape: the floor may be too dark "
                        "(try a lighter floor, or set USE_BLACK_TAPE = False)")
    cnts, hier = cv2.findContours(tape, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
    if hier is None or not len(cnts):
        raise ScanError("no red or black tape found (is the arena in view? is the lighting very different?)")
    hier = hier[0]
    best, best_hole, best_area = None, None, 0
    for i, c in enumerate(cnts):
        if hier[i][3] != -1 or hier[i][2] == -1:                      # want an outer ring that has a hole
            continue
        j, hole = hier[i][2], None
        while j != -1:                                                  # biggest hole of this ring
            if hole is None or cv2.contourArea(cnts[j]) > cv2.contourArea(hole):
                hole = cnts[j]
            j = hier[j][0]
        a = cv2.contourArea(hole)
        if a > best_area:
            best, best_hole, best_area = c, hole, a
    if best_hole is None:
        raise ScanError("found tape but no closed ring: there is a gap, or an edge of the arena is out of view")
    quad_px = _fit_quad(best_hole)
    arena = [to_floor(u, v) for u, v in quad_px]
    sides = [math.dist(arena[i], arena[(i + 1) % 4]) for i in range(4)]
    if min(sides) < MIN_ARENA_SIDE_M:
        raise ScanError(f"the ring found is only {min(sides) * 100:.0f} cm across: probably not the arena")
    for u, v in quad_px:
        if not (0 <= u < view.shape[1] and 0 <= v < view.shape[0]):
            raise ScanError("part of the arena is outside the camera view")

    # ---- the no-go zone: anything inside the arena that is tape-coloured or not floor-coloured
    inside = np.zeros(tape.shape, np.uint8)
    cv2.fillPoly(inside, [np.round(quad_px).astype(np.int32)], 255)
    inside = cv2.erode(inside, np.ones((9, 9), np.uint8))
    floor_pixels = view[(inside > 0) & (tape == 0)]
    nogo, nogo_px, cand_info = None, None, []
    if len(floor_pixels) > 500:
        floor_col = np.median(floor_pixels, axis=0)
        dev = np.linalg.norm(view.astype(np.float32) - floor_col.astype(np.float32), axis=2) > FLOOR_DELTA
        obj = ((dev | (tape > 0)) & (inside > 0)).astype(np.uint8) * 255
        obj = cv2.morphologyEx(obj, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        obj = cv2.morphologyEx(obj, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
        n, lab, stats, cent = cv2.connectedComponentsWithStats(obj)
        s = SCAN_PX_PER_M
        scored = []
        for k in range(1, n):
            if stats[k, cv2.CC_STAT_AREA] < (MIN_NOGO_SIDE_M * s) ** 2 * 0.5:
                continue
            cx, cy = to_floor(*cent[k])
            if any(math.hypot(cx - ex, cy - ey) < ROBOT_CLEARANCE_M for ex, ey in exclude):
                continue
            cs, _ = cv2.findContours((lab == k).astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            c = max(cs, key=cv2.contourArea)
            hull = cv2.convexHull(c)
            (rx, ry), (rw, rh), ang = cv2.minAreaRect(hull)
            lo, hi = sorted((rw / s, rh / s))
            fill = cv2.contourArea(hull) / max(rw * rh, 1.0)
            aspect = hi / max(lo, 1e-6)
            ok = lo >= MIN_NOGO_SIDE_M and hi <= MAX_NOGO_SIDE_M and aspect <= MAX_ASPECT and fill >= MIN_FILL
            cand_info.append((round(lo, 3), round(hi, 3), round(fill, 2), ok))
            if ok:
                scored.append((fill / aspect, cv2.boxPoints(((rx, ry), (rw, rh), ang))))   # the squarest, fullest one wins
        scored.sort(key=lambda t: -t[0])
        if scored:
            nogo_px = scored[0][1]
        if len(scored) > 1:
            warnings.append(f"{len(scored)} square-ish objects inside the arena: clear the robots so the right one is "
                            "picked as the no-go zone")
        if nogo_px is not None:
            nogo = [to_floor(u, v) for u, v in nogo_px]
    return {"arena": order_ccw(arena), "nogo": order_ccw(nogo) if nogo else None,
            "debug": {"view": view, "tape": tape, "quad_px": quad_px, "nogo_px": nogo_px, "candidates": cand_info,
                      "warnings": warnings}}


def _align(polys):
    """Give every scan's corners the same starting corner as the first scan's. Each scan lists its corners counter-clockwise
    from an arbitrary corner; for a square turned 45 degrees (a diamond) noise decides which corner comes first, and the
    median of mismatched corners would be nonsense."""
    ref = np.asarray(polys[0], float)
    out = []
    for p in polys:
        p = np.asarray(p, float)
        k = min(range(len(p)), key=lambda s: float(np.sum(np.linalg.norm(np.roll(p, -s, axis=0) - ref, axis=1))))
        out.append(np.roll(p, -k, axis=0))
    return np.array(out)


def combine(results):
    """Median of several scans, plus how much they disagree (mm, worst corner spread)."""
    A = _align([r["arena"] for r in results])                            # (n, 4, 2)
    arena = np.median(A, axis=0)
    spread = float(np.max(np.linalg.norm(A - arena, axis=2))) * 1000
    with_nogo = [r["nogo"] for r in results if r["nogo"]]
    nogo = None
    if len(with_nogo) > len(results) / 2:
        N = _align(with_nogo)
        nogo = np.median(N, axis=0)
        spread = max(spread, float(np.max(np.linalg.norm(N - nogo, axis=2))) * 1000)
    return ([tuple(map(float, p)) for p in arena], [tuple(map(float, p)) for p in nogo] if nogo is not None else None, spread)


def save(arena, nogo, path="arena.json"):
    json.dump({"arena": arena, "nogo": nogo}, open(path, "w"), indent=1)


def main(camera=None, flip=None, out_file="arena.json"):
    cap, size = tk.open_camera(camera, flip)
    tr = tk.Tracker(size=size)
    print("Looking for the floor marker: keep it in view...")
    if not tr.lock_floor(cap):
        sys.exit("Could not see the floor marker.")
    recent = deque(maxlen=AVERAGE_FRAMES)
    last_scan, reason, show_mask = 0.0, "scanning...", False
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        st = tr.update(frame)                                              # keeps the floor frame fresh (camera may move)
        now = time.time()
        if now - last_scan > 0.25:
            last_scan = now
            robots = [(v["x"], v["y"]) for k, v in st.items()]
            try:
                recent.append(scan_frame(frame, tr, exclude=robots))
                reason = ""
            except ScanError as e:
                recent.clear()
                reason = str(e)
            except cv2.error as e:
                recent.clear()
                reason = f"image problem: {e}"
        arena = nogo = None
        spread = None
        if recent:
            arena, nogo, spread = combine(list(recent))
            img = tk.project_floor(tr, [(x, y, 0.0) for x, y in arena])
            cv2.polylines(frame, [np.round(img).astype(np.int32).reshape(-1, 1, 2)], True, (0, 220, 0), 3, cv2.LINE_AA)
            if nogo:
                img = tk.project_floor(tr, [(x, y, 0.0) for x, y in nogo])
                cv2.polylines(frame, [np.round(img).astype(np.int32).reshape(-1, 1, 2)], True, (0, 150, 255), 3, cv2.LINE_AA)
        steady = spread is not None and len(recent) >= AVERAGE_FRAMES and spread <= STEADY_MM
        if arena:
            xs, ys = [p[0] for p in arena], [p[1] for p in arena]
            msg = (f"Arena {max(xs) - min(xs):.2f} x {max(ys) - min(ys):.2f} m, "
                   + ("no-go zone found" if nogo else "no no-go zone found")
                   + f" | {len(recent)}/{AVERAGE_FRAMES} scans, spread {spread:.0f} mm"
                   + ("  -> ENTER to save" if steady else "  (wait for it to settle)"))
            col = (0, 255, 0) if steady else (0, 200, 255)
        else:
            msg, col = "Can't find the arena: " + reason, (80, 80, 255)
        cv2.putText(frame, msg, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, col, 2, cv2.LINE_AA)
        if recent and recent[-1]["debug"]["warnings"]:
            cv2.putText(frame, "! " + recent[-1]["debug"]["warnings"][0][:110], (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        (0, 165, 255), 2, cv2.LINE_AA)
        cv2.putText(frame, "ENTER save | R rescan | M what the scan sees | Esc quit", (10, size[1] - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA)
        cv2.imshow("scan arena", frame)
        if show_mask and recent:
            dbg = recent[-1]["debug"]
            vis = cv2.cvtColor(dbg["tape"], cv2.COLOR_GRAY2BGR)
            cv2.polylines(vis, [np.round(dbg["quad_px"]).astype(np.int32).reshape(-1, 1, 2)], True, (0, 220, 0), 2)
            if dbg["nogo_px"] is not None:
                cv2.polylines(vis, [np.round(dbg["nogo_px"]).astype(np.int32).reshape(-1, 1, 2)], True, (0, 150, 255), 2)
            cv2.imshow("what the scan sees", vis)
        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            print("Quit without saving.")
            break
        if key == ord("r"):
            recent.clear()
        if key == ord("m"):
            show_mask = not show_mask
            if not show_mask:
                cv2.destroyWindow("what the scan sees")
        if key in (13, ord("s")):
            if not arena:
                print("Nothing to save yet:", reason)
            elif not steady:
                print(f"Not steady yet ({len(recent)}/{AVERAGE_FRAMES} scans, spread {spread:.0f} mm). Hold still a moment, then press ENTER.")
            else:
                save(arena, nogo, out_file)
                xs, ys = [p[0] for p in arena], [p[1] for p in arena]
                print(f"Saved {out_file}: arena about {max(xs) - min(xs):.2f} x {max(ys) - min(ys):.2f} m"
                      + (", no-go zone found." if nogo else ", NO no-go zone found (add it with mark_arena.py if there is one)."))
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
