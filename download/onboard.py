"""
onboard.py - yaw to the leader from the follower's OWN camera (e.g. an ESP32-CAM).

No markers, no floor frame, no external camera. The camera is on the follower, so the
leader's balloon appears left/right of the image centre by exactly the angle the follower
has to turn:

    yaw_to_leader = atan( (image_centre_x - balloon_x) / focal_length_px )

    + = leader is on the LEFT (turn left / CCW),  - = on the right,  None = not seen.

Run:  python onboard.py --source http://<esp32-ip>:81/stream
      python onboard.py --source 0          # any USB webcam, for testing
Keys / mouse: click the balloon to set its colour | M mask | [ ] adjust FOV | Esc quit

From your own code:  onboard.main(source, on_yaw=callback)   (see follow.py --onboard)
"""
import math
import sys
import threading
import time

import cv2
import numpy as np

from tracker import hsv_ranges_around        # same click-to-set-colour helper as tracker.py

# ----------------------------- CONFIG ---------------------------------------
STREAM_URL = "http://192.168.4.1:81/stream"   # ESP32-CAM default MJPEG stream; use your camera's IP
FOV_DEG = 65.0              # camera HORIZONTAL field of view. TUNE THIS (see README). Wide lenses are ~120+
MOUNT_YAW_DEG = 0.0         # camera points this many degrees LEFT of the robot's forward (+ = left)
ROTATE_180 = False          # True if the ESP32-CAM is mounted upside down
FLIP_HORIZONTAL = False     # True if the image is mirrored (it would swap left/right!)

LEADER_HSV = [((100, 120, 60), (130, 255, 255))]    # default blue; click the balloon to change
MIN_BLOB_AREA = 80          # pixels (small, since the leader may be far away)
ALPHA = 0.5                 # smoothing: 1 = none, lower = smoother
HOLD_S = 0.4                # keep the last value this long after the balloon is lost
# -----------------------------------------------------------------------------


def _wrap(a):
    return (a + 180.0) % 360.0 - 180.0


class LatestFrame:
    """Reads the stream in a background thread and only keeps the newest frame, so a slow
    loop never falls behind a buffered, laggy MJPEG stream."""

    def __init__(self, source):
        self.source = int(source) if str(source).isdigit() else source
        self.frame, self.t = None, 0.0
        self._lock = threading.Lock()
        self._stop = False
        self._cap = cv2.VideoCapture(self.source)
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        fails = 0
        while not self._stop:
            ok, f = self._cap.read()
            if not ok:
                fails += 1
                if fails > 30:                       # stream dropped: reconnect
                    self._cap.release()
                    time.sleep(0.5)
                    self._cap = cv2.VideoCapture(self.source)
                    fails = 0
                time.sleep(0.02)
                continue
            fails = 0
            if ROTATE_180:
                f = cv2.rotate(f, cv2.ROTATE_180)
            if FLIP_HORIZONTAL:
                f = cv2.flip(f, 1)
            with self._lock:
                self.frame, self.t = f, time.time()

    def read(self):
        with self._lock:
            return (None, 0.0) if self.frame is None else (self.frame.copy(), self.t)

    def release(self):
        self._stop = True
        self._cap.release()


class JpegFrameSource:
    """A camera whose frames are PUSHED in as JPEG bytes (from the ESP-NOW bridge, lidar_rx.SerialLidarReceiver(on_jpeg=source.push)).
    Same read()/release() as LatestFrame, so onboard.main can use either."""

    def __init__(self):
        self.frame, self.t = None, 0.0
        self.bad = 0
        self._lock = threading.Lock()

    def push(self, jpeg):
        f = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        if f is None:
            self.bad += 1
            return
        if ROTATE_180:
            f = cv2.rotate(f, cv2.ROTATE_180)
        if FLIP_HORIZONTAL:
            f = cv2.flip(f, 1)
        with self._lock:
            self.frame, self.t = f, time.time()

    def read(self):
        with self._lock:
            return (None, 0.0) if self.frame is None else (self.frame.copy(), self.t)

    def release(self):
        pass


class OnboardTracker:
    def __init__(self, fov_deg=FOV_DEG):
        self.fov_deg = fov_deg
        self.hsv = list(LEADER_HSV)
        self.bbox = None
        self.blob = None                 # (cx, cy) of the latest detection
        self._s = self._c = self._t = None   # smoothed unit vector of the angle, last-seen time

    def set_color(self, h, s, v):
        self.hsv = hsv_ranges_around(int(h), int(s), int(v))

    def mask(self, frame):
        hsv = cv2.cvtColor(cv2.GaussianBlur(frame, (5, 5), 0), cv2.COLOR_BGR2HSV)
        m = np.zeros(hsv.shape[:2], np.uint8)
        for lo, hi in self.hsv:
            m |= cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
        k = np.ones((3, 3), np.uint8)
        return cv2.morphologyEx(cv2.morphologyEx(m, cv2.MORPH_OPEN, k), cv2.MORPH_CLOSE, k)

    def focal_px(self, width):
        return (width / 2.0) / math.tan(math.radians(self.fov_deg) / 2.0)

    def update(self, frame, now=None):
        """Returns yaw_to_leader in degrees (+ = left) or None if the leader isn't seen."""
        now = time.time() if now is None else now
        h, w = frame.shape[:2]
        cnts, _ = cv2.findContours(self.mask(frame), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        c = max(cnts, key=cv2.contourArea) if cnts else None
        if c is not None and cv2.contourArea(c) >= MIN_BLOB_AREA:
            m = cv2.moments(c)
            cx, cy = m["m10"] / m["m00"], m["m01"] / m["m00"]
            self.blob, self.bbox = (cx, cy), cv2.boundingRect(c)
            a = math.atan2((w / 2.0) - cx, self.focal_px(w)) + math.radians(MOUNT_YAW_DEG)
            s, co = math.sin(a), math.cos(a)
            if self._s is None:
                self._s, self._c = s, co
            else:
                self._s += ALPHA * (s - self._s)
                self._c += ALPHA * (co - self._c)
            self._t = now
        if self._t is not None and now - self._t <= HOLD_S:
            return math.degrees(math.atan2(self._s, self._c))
        self.blob = self.bbox = None
        return None


BALLOON_DIAM_M = 0.09       # real balloon diameter, for range from apparent size (MEASURE)


class MultiTracker:
    """TANDEM mode: bearing AND range of every opponent balloon (one HSV range per slot, the same slots/colours as tracker.py) in the
    follower's own picture. bearings(frame) -> {slot: (bearing_deg relative to the rover's front, + = left; range_m)}."""

    def __init__(self, hsv_by_slot, fov_deg=FOV_DEG, min_area=MIN_BLOB_AREA):
        self.hsv, self.fov_deg, self.min_area = {k: list(v) for k, v in hsv_by_slot.items()}, fov_deg, min_area
        self._one = OnboardTracker(fov_deg)

    def bearings(self, frame):
        h, w = frame.shape[:2]
        f = (w / 2.0) / math.tan(math.radians(self.fov_deg) / 2.0)
        out = {}
        for slot, ranges in self.hsv.items():
            self._one.hsv = ranges
            cnts, _ = cv2.findContours(self._one.mask(frame), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not cnts:
                continue
            c = max(cnts, key=cv2.contourArea)
            if cv2.contourArea(c) < self.min_area:
                continue
            m = cv2.moments(c)
            cx = m["m10"] / m["m00"]
            width_px = max(cv2.boundingRect(c)[2], 1)
            brg = math.degrees(math.atan2((w / 2.0) - cx, f)) + MOUNT_YAW_DEG
            out[slot] = (_wrap(brg), BALLOON_DIAM_M * f / width_px)
        return out


class OnboardFeed:
    """Runs MultiTracker on a camera (LatestFrame / JpegFrameSource) in a background thread and calls on_bearings(bearings, t) with the time the
    picture ARRIVED. The external camera stays the primary source; this only sharpens the facing (see evade.Brain)."""

    def __init__(self, cam, tracker, on_bearings, hz=15.0):
        self.cam, self.tr, self.cb, self.dt = cam, tracker, on_bearings, 1.0 / hz
        self._stop, self.last_t, self.frames = False, 0.0, 0
        threading.Thread(target=self._run, daemon=True).start()

    def alive(self, now=None):
        return self.last_t > 0 and (time.time() if now is None else now) - self.last_t < 1.5

    def _run(self):
        seen = 0.0
        while not self._stop:
            frame, t = self.cam.read()
            if frame is None or t == seen:
                time.sleep(0.01)
                continue
            seen = t
            try:
                self.cb(self.tr.bearings(frame), t)
                self.last_t, self.frames = t, self.frames + 1
            except Exception as e:                        # a bad picture must never stop the follower
                print("onboard feed:", repr(e))
            time.sleep(self.dt)

    def close(self):
        self._stop = True


def draw_overlay(frame, tr, yaw):
    h, w = frame.shape[:2]
    cv2.line(frame, (w // 2, 0), (w // 2, h), (90, 90, 90), 1, cv2.LINE_AA)          # straight ahead
    if tr.bbox is not None and yaw is not None:
        x, y, bw, bh = tr.bbox
        cv2.rectangle(frame, (x - 6, y - 6), (x + bw + 6, y + bh + 6), (0, 140, 255), 2, cv2.LINE_AA)
        cv2.line(frame, (w // 2, int(tr.blob[1])), (int(tr.blob[0]), int(tr.blob[1])),
                 (0, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(frame, f"{yaw:+.0f} deg", (x - 6, y - 14), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0, 255, 255), 2, cv2.LINE_AA)
    txt = f"yaw to leader {yaw:+.1f} deg" if yaw is not None else "leader not seen"
    cv2.putText(frame, txt, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)
    cv2.putText(frame, f"FOV {tr.fov_deg:.0f} | click balloon = set colour | M mask",
                (10, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA)


def main(source=None, on_yaw=None, cam=None):
    """cam: an already-made camera object (read() -> (frame, time), release()), e.g. a JpegFrameSource fed by the ESP-NOW bridge."""
    src = STREAM_URL if source is None else source
    cam, tr = (cam if cam is not None else LatestFrame(src)), OnboardTracker()
    latest, show_mask = {"f": None}, False

    def on_mouse(event, x, y, flags, param):
        f = latest["f"]
        if event == cv2.EVENT_LBUTTONDOWN and f is not None:
            patch = cv2.cvtColor(f[max(0, y - 3):y + 4, max(0, x - 3):x + 4], cv2.COLOR_BGR2HSV).reshape(-1, 3)
            h, s, v = np.median(patch, axis=0)
            tr.set_color(h, s, v)
            print(f"leader colour set: H={h:.0f} S={s:.0f} V={v:.0f}")

    cv2.namedWindow("onboard")
    cv2.setMouseCallback("onboard", on_mouse)
    print("Waiting for the first frame from", src)
    last_t = 0.0
    while True:
        frame, t = cam.read()
        if frame is None or t == last_t:
            if cv2.waitKey(5) & 0xFF == 27:
                break
            continue
        last_t = t
        latest["f"] = frame.copy()
        yaw = tr.update(frame)
        if on_yaw:
            on_yaw(yaw)
        if show_mask:
            cv2.imshow("leader mask", tr.mask(frame))
        draw_overlay(frame, tr, yaw)
        cv2.imshow("onboard", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            break
        if key == ord("m"):
            show_mask = not show_mask
            if not show_mask:
                cv2.destroyWindow("leader mask")
        elif key == ord("["):
            tr.fov_deg = max(30.0, tr.fov_deg - 2); print("FOV", tr.fov_deg)
        elif key == ord("]"):
            tr.fov_deg = min(180.0, tr.fov_deg + 2); print("FOV", tr.fov_deg)
    cam.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(description="Yaw to leader from the follower's onboard camera")
    ap.add_argument("--source", help="stream URL or camera index (default STREAM_URL in the file)")
    main(ap.parse_args().source)
