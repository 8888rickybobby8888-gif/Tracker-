"""
tracker.py - floor-frame tracking of a follower (one ArUco marker flat on top, or four on a post) and a
up to three opponents (colour blobs, e.g. balloons).

Output: one number, the yaw to the leader = degrees the follower must turn to face the NEAREST
opponent (+ = left/CCW, - = right). 0 means it is pointing straight at it. The state also holds every
opponent's floor position ("A", "B", "C") for the evasion logic.

Floor frame (re-solved from the floor marker every frame, so the camera may move): origin at the centre of the floor marker (ID FLOOR_ID), x/y along
the marker's right/up edges, z up. Yaw is counter-clockwise from +x, seen from above.
Robot frame: x forward, y left, z up, origin on the floor under the robot centre.

Use from your own code:

    from tracker import Tracker
    tr = Tracker()
    tr.lock_floor(cap)                 # once, to find the floor marker; update() then re-solves it every frame
    state = tr.update(frame)           # call every frame
    print(tr.yaw_to_leader(state))     # e.g. +5.2   (None if either robot isn't seen)

Run directly for a live overlay:  python tracker.py   (1/2/3 pick opponent slot, click a balloon to set its colour, M = mask, F = re-lock, [ ] = FOV, Esc = quit)
"""
import math
import sys
import time

import cv2
import numpy as np

# ----------------------------- CONFIG ---------------------------------------
CAMERA_INDEX = 0                  # which camera to use. Run `python tracker.py --list` to find your rear one,
                                  # then `python tracker.py --camera N` (or set it here).
FLIP_HORIZONTAL = False           # set True (or use --flip) if the camera image is mirrored; a mirrored
                                  # image makes markers undetectable and flips the yaw sign
FRAME_SIZE = (1280, 720)          # requested capture size (the camera may give something else)
FOV_DEG = 70.0                    # camera HORIZONTAL field of view; no chessboard calibration is used.
                                  # Typical webcams are 60-80. Tune live with [ and ] if overlay boxes look off.

DICTIONARY = cv2.aruco.DICT_4X4_50
FLOOR_ID = 8                      # marker lying flat on the floor = world origin
FLOOR_SIZE = 0.15                 # side length of the printed black square, metres. BIGGER = steadier origin and angle
FLOOR_TRACKING = True             # re-solve the floor frame (origin + angle) from the floor marker on EVERY frame, so
                                  # the camera may move. False = lock it once at start (camera must then stay still).
FLOOR_ALPHA = 0.5                 # smoothing of that pose: 1 = none (jittery), lower = steadier but lags a moving camera
FLOOR_HOLD_S = 0.5                # if the floor marker is hidden, keep the last floor frame this long, then report nothing
FLOOR_JUMP_DEG = 25               # a sudden change bigger than this (or FLOOR_JUMP_M) must be seen on 2 frames in a row
FLOOR_JUMP_M = 0.25               # before it is believed: one-frame glitches are ignored, a real bump is accepted
MARKER_SIZE = 0.05                # side length of EACH follower marker, metres (measure the printed black square; bigger = more accurate; 4 to 6 cm works on a post)

# Follower marker layout.
#   "top":  ONE marker lying flat on top of the follower, its top edge pointing to the rover's FRONT. Best when the
#           external camera looks down from above (it works at an angle too, but yaw gets noisier).
#   "post": four markers on the faces of a post (make_markers.py prints them), for a low side-on camera.
MARKER_LAYOUT = "tower3"          # "tower3" = flat ID 0 on top + faces 1 left, 2 back, 3 right (default); "tower" = flat ID 0 + faces 1-4 incl. front; "post" = four faces only; "top" = one flat marker. follow.py --markers
TOP_MARKER_ID = 0
TOP_MARKER_OFFSET = (0.0, 0.0)    # "top": marker centre from the rover's centre: (forward, left), metres (MEASURE)
RED_GUARD = True                  # a RED balloon looks like the red tape: for red-like slots, ignore the tape band (needs arena.json) and long thin blobs
TAPE_BAND_M = (-0.02, 0.08)       # where the tape lies relative to the arena edge, metres (inside, outside); the arena corners are the INNER edge of the tape
LOCK_PREVIEW = True               # show the live picture (with a message) while the floor marker is being looked for
PLANE_MARKER = True               # a marker that lies flat on the rover gets its heading from the floor plane, not from solvePnP (no 30-60 degree flips at oblique camera angles)
MARKER_HEIGHT = 0.08              # floor -> centre of the SIDE-FACE markers, metres (MEASURE)
TOP_MARKER_HEIGHT = 0.08          # floor -> centre of the FLAT top marker, metres (MEASURE: for the "tower" layout this is the very top of the tower, higher than MARKER_HEIGHT)
POST_RADIUS = 0.04                # "post" only: robot centre -> face of a marker, metres
TILT_DEG = 20.0                   # "post" only: backwards lean of each face (0 = vertical)

# marker ID -> (robot name, face angle). "post": angle around the robot (0 = front, +90 = left, 180 = back,
# 270 = right), IDs counter-clockwise seen from above. "top": a single marker, angle None.
FACES = {}


def set_layout(layout):
    """Choose the follower marker layout. Call it BEFORE a Tracker is made.
      "post":  four markers on the faces of a post, IDs 0 front, 1 left, 2 back, 3 right.
      "tower3": ID 0 FLAT on top of the tower (its top edge to the rover's front) + IDs 1 left, 2 back, 3 right. No front face (the pin side). Needs markers 0-3 only.
      "tower": the same four faces as IDs 1 front, 2 left, 3 back, 4 right, and ID 0 FLAT on top of the tower.
      "top":   one flat marker, ID TOP_MARKER_ID."""
    global MARKER_LAYOUT
    if layout not in ("post", "top", "tower", "tower3"):
        raise ValueError('layout must be "tower3", "tower", "post" or "top"')
    MARKER_LAYOUT = layout
    FACES.clear()
    if layout == "top":
        FACES[TOP_MARKER_ID] = ("follower", None)
    elif layout == "tower3":
        FACES.update({TOP_MARKER_ID: ("follower", None), 1: ("follower", 90), 2: ("follower", 180), 3: ("follower", 270)})
    elif layout == "tower":
        FACES.update({TOP_MARKER_ID: ("follower", None), 1: ("follower", 0), 2: ("follower", 90), 3: ("follower", 180), 4: ("follower", 270)})
    else:
        FACES.update({0: ("follower", 0), 1: ("follower", 90), 2: ("follower", 180), 3: ("follower", 270)})


set_layout(MARKER_LAYOUT)

# Opponents = colour blobs (balloons), up to three slots. Press 1/2/3 in the live view to pick a slot, then click
# that balloon to set its colour. Or edit the HSV ranges (OpenCV HSV: H 0-179, S/V 0-255).
# Red needs two ranges because hue wraps around 0, and red tape would confuse it: use blue/green/yellow.
OPPONENT_HSV = {
    "A": [((100, 120, 60), (130, 255, 255))],       # blue
    "B": [((40, 100, 60), (80, 255, 255))],         # green
    "C": [((20, 120, 100), (35, 255, 255))],        # yellow
}
LEADER_HEIGHT = 0.15              # height of the balloon CENTRE above the floor, metres (MEASURE; same for all)
MIN_BLOB_AREA = 150               # pixels; smaller blobs are ignored as noise

# Overlay: follower footprint (length along heading, width, height) in metres - set to your rover
ROBOT_BOX = {"follower": (0.14, 0.11, 0.09)}
ARROW_LEN = 0.25                  # yaw arrow length on the floor, metres
COLORS = {"follower": (0, 220, 0), "A": (255, 150, 0), "B": (0, 200, 0), "C": (0, 220, 255)}   # BGR

# Filtering / rejection
MAX_REPROJ_PX = 3.0               # drop poses whose corners don't fit the model
MIN_FACING = 0.35                 # cos(angle) between marker normal and view ray (~70 deg)
ALPHA = 0.4                       # smoothing: 1 = none, lower = smoother
HOLD_S = 0.4                      # keep last estimate this long after markers are lost
# -----------------------------------------------------------------------------


def intrinsics_from_fov(size, fov_deg):
    """Pinhole camera matrix from image size and horizontal FOV (no lens distortion)."""
    w, h = size
    f = (w / 2.0) / math.tan(math.radians(fov_deg) / 2.0)
    return np.array([[f, 0, w / 2.0], [0, f, h / 2.0], [0, 0, 1]], np.float64)


def hsv_ranges_around(h, sat, val, dh=10, ds=70, dv=70):
    """HSV range(s) around a sampled colour. Handles hue wrap-around (reds)."""
    lo_s, hi_s = max(50, sat - ds), min(255, sat + ds)
    lo_v, hi_v = max(40, val - dv), min(255, val + dv)
    lo_h, hi_h = h - dh, h + dh
    if lo_h < 0:
        return [((0, lo_s, lo_v), (hi_h, hi_s, hi_v)), ((180 + lo_h, lo_s, lo_v), (179, hi_s, hi_v))]
    if hi_h > 179:
        return [((lo_h, lo_s, lo_v), (179, hi_s, hi_v)), ((0, lo_s, lo_v), (hi_h - 180, hi_s, hi_v))]
    return [((lo_h, lo_s, lo_v), (hi_h, hi_s, hi_v))]


def is_red_like(ranges):
    """True if an HSV range set covers red (hue wraps round 0 / 179)."""
    return any(lo[0] <= 8 or hi[0] >= 172 for lo, hi in ranges)


def balloon_shaped(c):
    """A balloon is a compact, roundish blob. The tape (a long thin strip, or an L at a corner) is not. c = an OpenCV contour."""
    area = cv2.contourArea(c)
    if area <= 0:
        return False
    (_, _), (w, h), _ = cv2.minAreaRect(c)
    if min(w, h) < 1e-6 or max(w, h) / min(w, h) > 2.4:
        return False
    (_, _), r = cv2.minEnclosingCircle(c)
    return area / (np.pi * r * r + 1e-6) >= 0.45


def obj_points(s):
    h = s / 2.0
    # TL, TR, BR, BL in marker coordinates (x right, y up, z out of the face)
    return np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], np.float32)


def robot_from_marker(angle_deg):
    """4x4 transform mapping marker coordinates -> robot coordinates for a face
    at `angle_deg` around the robot, leaning back by TILT_DEG. angle_deg None = one marker lying flat on top."""
    if angle_deg is None:                      # flat on top: marker "up" edge = rover forward, face normal = up
        M = np.eye(4)
        M[:3, 0], M[:3, 1], M[:3, 2] = [0, -1, 0], [1, 0, 0], [0, 0, 1]
        M[:3, 3] = [TOP_MARKER_OFFSET[0], TOP_MARKER_OFFSET[1], TOP_MARKER_HEIGHT]
        return M
    a, t = math.radians(angle_deg), math.radians(TILT_DEG)
    n = np.array([math.cos(a) * math.cos(t), math.sin(a) * math.cos(t), math.sin(t)])  # face normal
    x = np.cross([0, 0, 1], n)
    x /= np.linalg.norm(x)                                                              # marker right
    y = np.cross(n, x)                                                                  # marker up
    M = np.eye(4)
    M[:3, 0], M[:3, 1], M[:3, 2] = x, y, n
    M[:3, 3] = [POST_RADIUS * math.cos(a), POST_RADIUS * math.sin(a), MARKER_HEIGHT]
    return M


class Tracker:
    def __init__(self, K=None, D=None, size=FRAME_SIZE, fov_deg=FOV_DEG):
        self.size, self.fov_deg = size, fov_deg
        self.K = K if K is not None else intrinsics_from_fov(size, fov_deg)
        self.D = D if D is not None else np.zeros(5)

        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(DICTIONARY), params)

        self.M_inv = {mid: np.linalg.inv(robot_from_marker(ang)) for mid, (_, ang) in FACES.items()}
        self.floor_from_cam = None
        self.cam_from_floor = None
        self.floor_rvec = self.floor_tvec = None
        self._floor_t = None             # time the floor frame was last updated from the floor marker
        self._floor_pending = None       # a suspicious jump waiting for confirmation
        self._filt = {}          # robot -> [x, y, sin(yaw), cos(yaw), timestamp]
        self.last_det = {}
        self.opp_hsv = {k: list(v) for k, v in OPPONENT_HSV.items()}
        self.tape_poly = None
        self.opp_bbox = {}

    def set_opponent_color(self, name, h, sat, val):
        self.opp_hsv[name] = hsv_ranges_around(int(h), int(sat), int(val))

    def rebuild_geometry(self):
        """Recompute the marker-on-rover transforms after MARKER_HEIGHT, POST_RADIUS, TILT_DEG or the layout changed (Settings page)."""
        self.M_inv = {mid: np.linalg.inv(robot_from_marker(ang)) for mid, (_, ang) in FACES.items()}

    def set_tape(self, arena_poly):
        """The arena corners (floor frame, the INNER edge of the tape). Lets a red balloon be told from the red tape."""
        self.tape_poly = [tuple(p) for p in arena_poly] if arena_poly else None

    def _tape_mask(self, shape):
        """Image mask (255 = tape band) of the tape round the arena, from the known corners and the locked floor frame."""
        m = np.zeros(shape, np.uint8)
        if not self.tape_poly or self.floor_from_cam is None:
            return m
        cx = sum(p[0] for p in self.tape_poly) / len(self.tape_poly)
        cy = sum(p[1] for p in self.tape_poly) / len(self.tape_poly)
        a, b = TAPE_BAND_M
        n = len(self.tape_poly)
        for i in range(n):
            p, q = np.array(self.tape_poly[i]), np.array(self.tape_poly[(i + 1) % n])
            d = q - p
            d = d / (np.linalg.norm(d) + 1e-9)
            nrm = np.array([-d[1], d[0]])
            if np.dot(nrm, p - np.array([cx, cy])) < 0:          # outward normal
                nrm = -nrm
            quad = [p + nrm * a, q + nrm * a, q + nrm * b, p + nrm * b]
            img = project_floor(self, [(x[0], x[1], 0.0) for x in quad])
            cv2.fillConvexPoly(m, np.round(img).astype(np.int32), 255)
        return m

    def opponent_mask(self, frame, name):
        hsv = cv2.cvtColor(cv2.GaussianBlur(frame, (7, 7), 0), cv2.COLOR_BGR2HSV)
        mask = np.zeros(hsv.shape[:2], np.uint8)
        for lo, hi in self.opp_hsv[name]:
            mask |= cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
        k = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
        if RED_GUARD and self.tape_poly and is_red_like(self.opp_hsv[name]):
            mask[self._tape_mask(mask.shape) > 0] = 0            # a red balloon must not be confused with the red tape
        return mask

    def _detect_blob(self, frame, name):
        """Largest blob of this opponent's colour -> ((cx, cy), (x, y, w, h)) or None. For a RED slot the blob must also be balloon-shaped."""
        if frame is None:
            return None
        cnts, _ = cv2.findContours(self.opponent_mask(frame, name), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return None
        red = RED_GUARD and is_red_like(self.opp_hsv[name])
        for c in sorted(cnts, key=cv2.contourArea, reverse=True):
            if cv2.contourArea(c) < MIN_BLOB_AREA:
                return None
            if red and not balloon_shaped(c):
                continue
            m = cv2.moments(c)
            return (m["m10"] / m["m00"], m["m01"] / m["m00"]), cv2.boundingRect(c)
        return None

    def _pixel_to_floor(self, px, height):
        """Cast the camera ray through pixel px and intersect it with the horizontal plane z = height
        (floor frame). Returns (x, y) or None if the ray doesn't hit it."""
        n = cv2.undistortPoints(np.array([[px]], np.float64), self.K, self.D).reshape(2)
        ray = self.floor_from_cam[:3, :3] @ np.array([n[0], n[1], 1.0])
        origin = self.floor_from_cam[:3, 3]
        if ray[2] >= -1e-6:
            return None
        t = (height - origin[2]) / ray[2]
        if t <= 0:
            return None
        p = origin + t * ray
        return p[0], p[1]

    def set_fov(self, fov_deg):
        """Change the assumed FOV. The floor frame must be re-locked afterwards."""
        self.fov_deg = fov_deg
        self.K = intrinsics_from_fov(self.size, fov_deg)
        self.floor_from_cam = None

    # ---- detection / single-marker pose ------------------------------------
    def _detect(self, frame):
        corners, ids, _ = self.detector.detectMarkers(frame)
        if ids is None:
            return {}
        return {int(i): c.reshape(4, 2).astype(np.float32) for c, i in zip(corners, ids.flatten())}

    def _pose(self, corners, size):
        pts = obj_points(size)
        ok, rvec, tvec = cv2.solvePnP(pts, corners, self.K, self.D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            return None
        # Polish the closed-form (IPPE) answer by minimising the reprojection error. This matters most when the
        # marker faces the camera almost squarely (an overhead camera): the closed form is least reliable there.
        try:
            rvec, tvec = cv2.solvePnPRefineLM(pts, corners, self.K, self.D, rvec, tvec)
        except cv2.error:
            pass
        proj, _ = cv2.projectPoints(pts, rvec, tvec, self.K, self.D)
        err = float(np.linalg.norm(proj.reshape(-1, 2) - corners, axis=1).mean())
        T = np.eye(4)
        T[:3, :3] = cv2.Rodrigues(rvec)[0]
        T[:3, 3] = tvec.ravel()
        return T, err                                    # T: camera <- marker


    def _plane_pose(self, corners):
        """Floor <- marker for a marker that lies FLAT (horizontal, at TOP_MARKER_HEIGHT): cast each corner's camera ray onto that plane and take
        the position and heading straight from the four floor points. The usual single-marker solve (solvePnP) can flip between two
        look-alike answers when the marker is small or seen at a slant, which turned the heading by 30-60 degrees whenever the camera was
        moved to a low side angle; this cannot, because the marker's tilt is known instead of guessed."""
        pts = [self._pixel_to_floor(tuple(c), TOP_MARKER_HEIGHT) for c in corners]
        if any(p is None for p in pts):
            return None
        p = np.array(pts, float)
        u = (p[1] + p[2]) / 2.0 - (p[0] + p[3]) / 2.0          # marker +x (left edge -> right edge) on the floor
        n = float(np.linalg.norm(u))
        if n < 0.5 * MARKER_SIZE or n > 1.5 * MARKER_SIZE:     # the four points don't make a marker-sized square: bad detection
            return None
        th = math.atan2(u[1], u[0])
        c, sn = math.cos(th), math.sin(th)
        c0 = p.mean(axis=0)
        T = np.eye(4)
        T[:3, :3] = np.array([[c, -sn, 0.0], [sn, c, 0.0], [0.0, 0.0, 1.0]])
        T[:3, 3] = [c0[0], c0[1], TOP_MARKER_HEIGHT]
        return T

    def _pose_upright(self, corners, size, M_inv):
        """Pose of a marker (camera <- marker, reprojection error) for a marker whose place on the ROVER is known. A single square marker has two
        look-alike solutions (the 'flip' that turns the heading by tens of degrees when seen at a slant); of the two, take the one that leaves the
        ROVER standing upright on the floor. Falls back to the normal solve."""
        pts = obj_points(size)
        try:
            n, rvecs, tvecs, errs = cv2.solvePnPGeneric(pts, corners, self.K, self.D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        except cv2.error:
            return self._pose(corners, size)
        if not n:
            return self._pose(corners, size)
        best, best_up = None, -2.0
        for rv, tv, e in zip(rvecs, tvecs, errs):
            T = np.eye(4)
            T[:3, :3] = cv2.Rodrigues(rv)[0]
            T[:3, 3] = np.asarray(tv).ravel()
            up = float((self.floor_from_cam @ T @ M_inv)[2, 2])      # z component of the rover's up axis in the floor frame (1 = upright)
            if up > best_up:
                best, best_up = (rv, tv, T), up
        rv, tv, T = best
        try:
            rv, tv = cv2.solvePnPRefineLM(pts, corners, self.K, self.D, rv, tv)
            T[:3, :3] = cv2.Rodrigues(rv)[0]
            T[:3, 3] = np.asarray(tv).ravel()
        except cv2.error:
            pass
        proj, _ = cv2.projectPoints(pts, cv2.Rodrigues(T[:3, :3])[0], T[:3, 3].reshape(3, 1), self.K, self.D)
        return T, float(np.linalg.norm(proj.reshape(-1, 2) - corners, axis=1).mean())

    # ---- floor frame ---------------------------------------------------------
    def _set_floor(self, T, now):
        """T: camera <- floor frame."""
        self.cam_from_floor = T
        self.floor_from_cam = np.linalg.inv(T)
        self.floor_rvec = cv2.Rodrigues(T[:3, :3])[0]
        self.floor_tvec = T[:3, 3].reshape(3, 1)
        self._floor_t = now

    @staticmethod
    def _orthonormal(M):
        U, _, Vt = np.linalg.svd(M)
        R = U @ Vt
        if np.linalg.det(R) < 0:
            U[:, -1] *= -1
            R = U @ Vt
        return R

    @staticmethod
    def _pose_gap(A, B):
        """(rotation difference in degrees, translation difference in metres) between two 4x4 poses."""
        dR = A[:3, :3] @ B[:3, :3].T
        ang = math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(dR) - 1.0) / 2.0))))
        return ang, float(np.linalg.norm(A[:3, 3] - B[:3, 3]))

    def _update_floor(self, det, now):
        """Re-solve the floor frame from the floor marker in this frame. Returns True if it was updated."""
        if FLOOR_ID not in det:
            return False
        res = self._pose(det[FLOOR_ID], FLOOR_SIZE)
        if res is None or res[1] > MAX_REPROJ_PX:
            return False
        T = res[0]
        if float(np.dot(T[:3, 2], -T[:3, 3] / np.linalg.norm(T[:3, 3]))) < 0.2:      # seen too edge-on to trust
            return False
        if self.cam_from_floor is None or self._floor_t is None or now - self._floor_t > FLOOR_HOLD_S:
            self._set_floor(T, now)                                  # first sight, or back after a loss
            self._floor_pending = None
            return True
        ang, dist = self._pose_gap(T, self.cam_from_floor)
        if ang > FLOOR_JUMP_DEG or dist > FLOOR_JUMP_M:
            pend = self._floor_pending
            if pend is not None and self._pose_gap(T, pend)[0] < 10 and self._pose_gap(T, pend)[1] < 0.1:
                self._set_floor(T, now)                              # seen twice: the camera really moved
                self._floor_pending = None
                return True
            self._floor_pending = T                                  # once: could be a glitch, wait
            return False
        self._floor_pending = None
        a = FLOOR_ALPHA
        Tn = np.eye(4)
        Tn[:3, :3] = self._orthonormal((1 - a) * self.cam_from_floor[:3, :3] + a * T[:3, :3])
        Tn[:3, 3] = (1 - a) * self.cam_from_floor[:3, 3] + a * T[:3, 3]
        self._set_floor(Tn, now)
        return True

    def track_floor(self, frame, now=None):
        """Update the floor frame from one frame (for tools that don't call update())."""
        now = time.time() if now is None else now
        return self._update_floor(self._detect(frame), now)

    def floor_status(self, now=None):
        """("waiting" | "ok" | "held" | "lost", seconds since the floor marker was last used)."""
        now = time.time() if now is None else now
        if self._floor_t is None:
            return "waiting", None
        age = now - self._floor_t
        if not FLOOR_TRACKING:
            return "locked", age
        return ("ok" if age <= 0.15 else "held" if age <= FLOOR_HOLD_S else "lost"), age

    def lock_floor(self, cap, n=40, timeout=10.0):
        """Acquire the floor frame by averaging the floor marker pose over n good frames. With FLOOR_TRACKING on
        (the default) update() keeps re-solving it every frame afterwards; with it off this fixes the frame and
        the camera must not move."""
        Rs, ts, t0 = [], [], time.time()
        while len(Rs) < n and time.time() - t0 < timeout:
            ok, frame = cap.read()
            if not ok:
                continue
            det = self._detect(frame)
            if FLOOR_ID in det:
                res = self._pose(det[FLOOR_ID], FLOOR_SIZE)
                if res and res[1] < MAX_REPROJ_PX:
                    Rs.append(res[0][:3, :3])
                    ts.append(res[0][:3, 3])
        if len(Rs) < n // 2:
            return False
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = self._orthonormal(np.mean(Rs, axis=0)), np.mean(ts, axis=0)
        self._set_floor(T, time.time())
        self._floor_pending = None
        self._filt.clear()
        return True

    # ---- per-frame update ----------------------------------------------------
    def update(self, frame, now=None):
        now = time.time() if now is None else now
        self.last_det = self._detect(frame)
        if FLOOR_TRACKING:
            self._update_floor(self.last_det, now)                  # origin + angle, re-solved every frame
        if self.floor_from_cam is None:
            raise RuntimeError("no floor frame yet: call lock_floor() first")
        if FLOOR_TRACKING and now - self._floor_t > FLOOR_HOLD_S:
            return self.state(now)                                  # floor marker hidden too long: don't guess
        obs = {}
        for mid, corners in self.last_det.items():
            if mid not in FACES:
                continue
            res = self._pose(corners, MARKER_SIZE) if FACES[mid][1] is None else self._pose_upright(corners, MARKER_SIZE, self.M_inv[mid])
            if res is None:
                continue
            T, err = res
            if err > MAX_REPROJ_PX:
                continue
            ray = -T[:3, 3] / np.linalg.norm(T[:3, 3])   # marker -> camera
            facing = float(np.dot(T[:3, 2], ray))
            if facing < MIN_FACING:                      # too oblique: pose is unreliable
                continue
            Tf = self.floor_from_cam @ T @ self.M_inv[mid]   # floor <- robot
            if PLANE_MARKER and FACES[mid][1] is None:       # the marker lies FLAT on the rover: use that knowledge (see _plane_pose)
                Tp = self._plane_pose(corners)
                if Tp is not None:
                    Tf = Tp @ self.M_inv[mid]
            w = facing ** 2 / (err + 0.5)
            obs.setdefault(FACES[mid][0], []).append(
                (Tf[0, 3], Tf[1, 3], math.atan2(Tf[1, 0], Tf[0, 0]), w))

        for robot, o in obs.items():
            W = sum(a[3] for a in o)
            x = sum(a[0] * a[3] for a in o) / W
            y = sum(a[1] * a[3] for a in o) / W
            s = sum(math.sin(a[2]) * a[3] for a in o) / W
            c = sum(math.cos(a[2]) * a[3] for a in o) / W
            self._smooth(robot, x, y, s, c, now)

        for name in self.opp_hsv:
            hit = self._detect_blob(frame, name)
            if hit is not None:
                (cx, cy), bbox = hit
                p = self._pixel_to_floor((cx, cy), LEADER_HEIGHT)
                if p is not None:
                    self.opp_bbox[name] = bbox
                    self._smooth(name, p[0], p[1], 0.0, 1.0, now)
        return self.state(now)

    @staticmethod
    def opponents(state):
        """{name: (x, y)} of every opponent currently seen."""
        return {k: (v["x"], v["y"]) for k, v in state.items() if k != "follower"}

    @staticmethod
    def nearest_opponent(state):
        """Name of the opponent nearest the follower, or None."""
        f = state.get("follower")
        opps = Tracker.opponents(state)
        if not (f and opps):
            return None
        return min(opps, key=lambda k: math.hypot(opps[k][0] - f["x"], opps[k][1] - f["y"]))

    @staticmethod
    def yaw_to_leader(state):
        """Degrees the follower must turn to face the NEAREST opponent (+ = turn left/CCW, - = right),
        in -180..180. None if the follower or every opponent is unseen."""
        f = state.get("follower")
        name = Tracker.nearest_opponent(state)
        if name is None:
            return None
        l = state[name]
        bearing = math.degrees(math.atan2(l["y"] - f["y"], l["x"] - f["x"]))
        return _wrap(bearing - f["yaw"])

    def _smooth(self, robot, x, y, s, c, now):
        p = self._filt.get(robot)
        if p is None:
            self._filt[robot] = [x, y, s, c, now]
        else:
            a = ALPHA
            self._filt[robot] = [p[0] + a * (x - p[0]), p[1] + a * (y - p[1]),
                                 p[2] + a * (s - p[2]), p[3] + a * (c - p[3]), now]

    def state(self, now=None):
        now = time.time() if now is None else now
        out = {}
        for robot, (x, y, s, c, t) in self._filt.items():
            if now - t <= HOLD_S:
                out[robot] = {"x": x, "y": y, "age": now - t}
                if robot == "follower":
                    out[robot]["yaw"] = math.degrees(math.atan2(s, c))
        return out


# ----------------------------- OVERLAY ---------------------------------------
BOX_EDGES = [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4),
             (0, 4), (1, 5), (2, 6), (3, 7)]


def _wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def project_floor(tr, pts):
    """Floor-frame 3D points (N,3) -> pixel coords (N,2) using the locked floor pose."""
    img, _ = cv2.projectPoints(np.asarray(pts, np.float32), tr.floor_rvec, tr.floor_tvec, tr.K, tr.D)
    return img.reshape(-1, 2)


def _pt(p):
    return (int(round(p[0])), int(round(p[1])))


def draw_box(frame, tr, x, y, yaw_deg, size, color, thickness=2):
    """3D wireframe box on the floor, centred at (x, y), rotated by yaw."""
    l, w, h = size
    a = math.radians(yaw_deg)
    c, s = math.cos(a), math.sin(a)
    local = [(-l / 2, -w / 2), (l / 2, -w / 2), (l / 2, w / 2), (-l / 2, w / 2)]
    pts = [(x + px * c - py * s, y + px * s + py * c, z) for z in (0.0, h) for px, py in local]
    img = project_floor(tr, pts)
    for i, j in BOX_EDGES:
        cv2.line(frame, _pt(img[i]), _pt(img[j]), color, thickness, cv2.LINE_AA)
    return img                      # corner pixels (0-3 bottom, 4-7 top) for label placement


def draw_overlay(frame, tr, state):
    """Draw tracked markers, follower yaw arrow + box, leader box, and the follow line."""
    # detected markers (thin outline) so you can see what the tracker is using
    for mid, c in tr.last_det.items():
        col = (160, 160, 160) if mid in FACES or mid == FLOOR_ID else (80, 80, 80)
        cv2.polylines(frame, [c.astype(np.int32).reshape(-1, 1, 2)], True, col, 1, cv2.LINE_AA)
    cv2.drawFrameAxes(frame, tr.K, tr.D, tr.floor_rvec, tr.floor_tvec, FLOOR_SIZE)

    f = state.get("follower")

    for name in tr.opp_hsv:
        l = state.get(name)
        if l and name in tr.opp_bbox:
            bx, by, bw, bh = tr.opp_bbox[name]
            col = COLORS[name]
            cv2.rectangle(frame, (bx - 6, by - 6), (bx + bw + 6, by + bh + 6), col, 2, cv2.LINE_AA)
            cv2.putText(frame, f"OPP {name}", (bx - 6, by - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 2, cv2.LINE_AA)
            foot = project_floor(tr, [(l["x"], l["y"], 0.0)])[0]            # point on the floor under it
            cv2.circle(frame, _pt(foot), 5, col, -1, cv2.LINE_AA)

    if f:
        col = COLORS["follower"]
        img = draw_box(frame, tr, f["x"], f["y"], f["yaw"], ROBOT_BOX["follower"], col)
        a = math.radians(f["yaw"])
        tip = (f["x"] + ARROW_LEN * math.cos(a), f["y"] + ARROW_LEN * math.sin(a), 0.0)
        p = project_floor(tr, [(f["x"], f["y"], 0.0), tip])
        cv2.arrowedLine(frame, _pt(p[0]), _pt(p[1]), col, 3, cv2.LINE_AA, tipLength=0.25)
        top = img[4:].min(axis=0)
        cv2.putText(frame, f"FOLLOWER yaw {f['yaw']:.0f}", (int(top[0]) - 40, int(top[1]) - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 2, cv2.LINE_AA)

    err = tr.yaw_to_leader(state)
    if err is not None:
        l = state[tr.nearest_opponent(state)]
        q = project_floor(tr, [(f["x"], f["y"], 0.0), (l["x"], l["y"], 0.0)])
        cv2.line(frame, _pt(q[0]), _pt(q[1]), (0, 255, 255), 1, cv2.LINE_AA)
        m = _pt((q[0] + q[1]) / 2)
        cv2.putText(frame, f"{err:+.0f} deg", (m[0] + 8, m[1] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2, cv2.LINE_AA)
    return frame


class Cam:
    """Thin wrapper around cv2.VideoCapture that can mirror the image back."""

    def __init__(self, cap, flip):
        self.cap, self.flip = cap, flip

    def read(self):
        ok, frame = self.cap.read()
        if ok and self.flip:
            frame = cv2.flip(frame, 1)
        return ok, frame

    def release(self):
        self.cap.release()


def _open_raw(index):
    """Open camera `index`; on Windows try DirectShow first, then the default backend."""
    backends = [cv2.CAP_DSHOW, cv2.CAP_ANY] if (sys.platform.startswith("win") and not isinstance(index, str)) else [cv2.CAP_ANY]
    for be in backends:
        cap = cv2.VideoCapture(index, be)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_SIZE[0])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_SIZE[1])
        ok, frame = cap.read()
        if ok:
            return cap, frame
        cap.release()
    return None, None


def camera_arg(text):
    """A --camera value: a number (0, 1, 2 ...) or a URL / file path (a phone's IP camera, an RTSP or MJPEG stream, a video file)."""
    return int(text) if str(text).strip().isdigit() else str(text)


def scan_cameras(max_index=10):
    """Non-interactive: try camera 0..max_index-1, print which open and their size, and save camera_N.jpg so you can SEE which is which."""
    found = []
    for i in range(max_index):
        cap, frame = _open_raw(i)
        if cap is None:
            continue
        cap.release()
        cv2.imwrite(f"camera_{i}.jpg", frame)
        found.append(i)
        print(f"camera {i}: {frame.shape[1]}x{frame.shape[0]}  (picture saved as camera_{i}.jpg: open it to see which camera this is)")
    print(f"\nUse one with:  python follow.py --camera {found[0]}" if found else "No cameras found.")


def open_camera(index=None, flip=None):
    index = CAMERA_INDEX if index is None else index
    cap, frame = _open_raw(index)
    if cap is None:
        sys.exit(f"Could not open camera {index}. Run `python tracker.py --list` to see what is available "
                 "(and close other apps that use the camera, e.g. the Camera app).")
    h, w = frame.shape[:2]
    return Cam(cap, FLIP_HORIZONTAL if flip is None else flip), (w, h)


def pick_camera(max_index=8):
    """Show each camera in turn so you can identify the rear one."""
    found = []
    for i in range(max_index):
        cap, frame = _open_raw(i)
        if cap is None:
            continue
        found.append(i)
        print(f"camera {i}: {frame.shape[1]}x{frame.shape[0]}")
        key = 255
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            view = frame.copy()
            cv2.putText(view, f"camera {i}: Y = use this one, any other key = next", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.imshow("pick camera", view)
            key = cv2.waitKey(30) & 0xFF
            if key != 255:
                break
        cap.release()
        if key == ord("y"):
            cv2.destroyAllWindows()
            print(f"\nUse it with:  python tracker.py --camera {i}")
            return
    if found:
        cv2.destroyAllWindows()
    print("No more cameras." if found else "No cameras found.")


def main(camera=None, flip=None, on_yaw=None, on_state=None, on_draw=None, on_key=None, status=None,
         headless=False, on_frame=None, should_stop=None, on_tick=None, on_preview=None):
    """headless=True: no OpenCV window (the dashboard shows the picture). on_frame(frame, raw, tr, st) gets every annotated frame,
    the raw one, the Tracker and the state. should_stop() -> True ends the loop. on_tick(tr, cap) runs once per loop (before the read)."""
    cap, size = open_camera(camera, flip)
    tr = Tracker(size=size)
    print(f"Camera {size[0]}x{size[1]}, assumed FOV {tr.fov_deg:.0f} deg")
    print("Locking floor frame - keep the floor marker visible and the camera still...")
    if not headless:
        cv2.namedWindow("tracker")                                  # the window is up BEFORE the floor lock, so you can see what the camera sees

    class _Preview:                                                  # shows the live picture, with a message, while the floor marker is being looked for
        def __init__(self, c):
            self.c, self.n = c, 0

        def read(self):
            ok, fr = self.c.read()
            if ok and LOCK_PREVIEW:
                self.n += 1
                if self.n % 2 == 0:
                    shown = fr.copy()
                    cv2.putText(shown, f"Looking for the floor marker (ID {FLOOR_ID}): keep it in view and the camera still", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2, cv2.LINE_AA)
                    if headless:
                        if on_preview is not None:
                            on_preview(shown)
                    else:
                        cv2.imshow("tracker", shown)
                        cv2.waitKey(1)
            return ok, fr

    def lock():
        """Look for the floor marker (with the live picture and a message) until it is found. False = told to stop."""
        tries = 0
        while not tr.lock_floor(_Preview(cap), timeout=5.0):
            tries += 1
            print(f"Still looking for the floor marker (ID {FLOOR_ID}) ... is it flat, in view and well lit? (attempt {tries})")
            if (should_stop is not None and should_stop()) or (not headless and cv2.waitKey(1) & 0xFF == 27):
                return False
        return True

    if not lock():
        cap.release()
        if not headless:
            cv2.destroyAllWindows()
        return
    print("Floor found (re-solved every frame, so the camera may move: keep the floor marker in view).  1/2/3 pick opponent slot, then click its balloon to set the colour | M mask | F re-lock | [ ] FOV | Esc quit")

    latest = {"frame": None}
    active = {"slot": "A"}

    def on_mouse(event, x, y, flags, param):
        fr = latest["frame"]
        if event == cv2.EVENT_LBUTTONDOWN and fr is not None:
            patch = cv2.cvtColor(fr[max(0, y - 4):y + 5, max(0, x - 4):x + 5], cv2.COLOR_BGR2HSV).reshape(-1, 3)
            h, sat, val = np.median(patch, axis=0)
            tr.set_opponent_color(active["slot"], h, sat, val)
            print(f"opponent {active['slot']} colour set from click: H={h:.0f} S={sat:.0f} V={val:.0f}")

    if not headless:
        cv2.setMouseCallback("tracker", on_mouse)
    show_mask = False
    while True:
        if should_stop is not None and should_stop():
            break
        if on_tick is not None:
            try:
                new = on_tick(tr, cap)
            except Exception as e:
                new = None
                print("on_tick error:", repr(e))
            if new is not None:                                      # the dashboard switched the external camera
                new_cap, new_size = new
                try:
                    cap.release()
                except Exception:
                    pass
                cap = new_cap
                if tuple(new_size) != tuple(tr.size):
                    tr.size = tuple(new_size)
                    tr.K = intrinsics_from_fov(tr.size, tr.fov_deg)
                tr.floor_from_cam = None
                tr._floor_t = None
                print(f"External camera switched ({new_size[0]}x{new_size[1]}): looking for the floor marker again ...")
                if not lock():
                    break
                continue
        ok, frame = cap.read()
        if not ok:
            break
        latest["frame"] = frame.copy()
        st = tr.update(frame)
        if on_yaw:
            on_yaw(tr.yaw_to_leader(st))      # called every frame with degrees-to-nearest-opponent or None
        if on_state:
            on_state(st)                      # full state: follower pose + every opponent position
        if show_mask and not headless:
            cv2.imshow("opponent mask", tr.opponent_mask(frame, active["slot"]))
        draw_overlay(frame, tr, st)
        fstat, _ = tr.floor_status()
        if fstat == "lost":
            cv2.putText(frame, "FLOOR MARKER LOST: tracking paused", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (80, 80, 255), 2, cv2.LINE_AA)
        elif fstat == "held":
            cv2.putText(frame, "floor marker hidden: holding last frame", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (80, 200, 255), 2, cv2.LINE_AA)
        if on_draw:
            on_draw(frame, tr)                # e.g. plan.py draws the arena, no-go zone and planned path
        err = tr.yaw_to_leader(st)
        txt = f"yaw to nearest {err:+.1f} deg" if err is not None else "follower or opponent not seen"
        cv2.putText(frame, txt, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(frame, f"FOV {tr.fov_deg:.0f} | slot {active['slot']} (1/2/3) | click balloon = set colour | M mask",
                    (10, size[1] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA)
        if status:
            try:
                msg = status()                # e.g. "PAUSED (press P to resume)"
            except Exception as e:            # a display helper must never take the vision loop (and the rover with it) down
                msg = f"status error: {e!r}"[:60]
            if msg:
                cv2.putText(frame, msg, (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (80, 80, 255), 2, cv2.LINE_AA)
        if on_frame is not None:
            try:
                on_frame(frame, latest["frame"], tr, st)
            except Exception as e:
                print("on_frame error:", repr(e))
        if headless:
            time.sleep(0.001)
            key = 255
        else:
            cv2.imshow("tracker", frame)
            key = cv2.waitKey(1) & 0xFF
        if key == 27:
            break
        if on_key and key != 255:
            try:
                if on_key(key):               # e.g. follow.py: P = pause / resume the rover. Returning True = key used up
                    key = 255
            except Exception as e:
                print("on_key error:", repr(e))
        if key == ord("m"):
            show_mask = not show_mask
            if not show_mask:
                cv2.destroyWindow("opponent mask")
        if key in (ord("1"), ord("2"), ord("3")):
            active["slot"] = "ABC"[key - ord("1")]
            print("active opponent slot:", active["slot"])
        if key in (ord("["), ord("]"), ord("f")):
            if key == ord("["):
                tr.set_fov(max(30.0, tr.fov_deg - 2))
            elif key == ord("]"):
                tr.set_fov(min(120.0, tr.fov_deg + 2))
            print("re-lock:", tr.lock_floor(cap), f"(FOV {tr.fov_deg:.0f})")
    cap.release()
    if not headless:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(description="Follower yaw-to-leader tracker")
    ap.add_argument("--camera", type=camera_arg, help="camera index, or a URL / file for an IP camera (default: CAMERA_INDEX in the file)")
    ap.add_argument("--list", action="store_true", help="preview each camera to find the right index")
    ap.add_argument("--scan", action="store_true", help="list every camera that opens, with its size, and save camera_N.jpg for each")
    ap.add_argument("--flip", action="store_true", help="mirror the image horizontally")
    args = ap.parse_args()
    if args.scan:
        scan_cameras()
    elif args.list:
        pick_camera()
    else:
        main(args.camera, True if args.flip else None)
