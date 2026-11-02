"""
lidar_rx.py - receives the rear-lidar frames the ESP32 sends: over WiFi UDP (LidarReceiver) or over USB serial from an ESP-NOW bridge
(SerialLidarReceiver, see espnow_bridge/espnow_bridge.ino).

Packet (little-endian), matching lidar_udp.h:
    2 bytes  b"LD"
    2 bytes  sequence number (uint16)
    1 byte   number of zones n (16 or 64)
    4 bytes  sensor time in milliseconds (uint32)
    n * 2    distance in mm per zone (uint16), 0xFFFF = no valid reading
"""
import socket
import struct
import threading
import time

import numpy as np

HEADER = struct.Struct("<2sHBI")
INVALID = 0xFFFF


def make_packet(seq, ms, dists_mm):
    """Build a packet (used by tests and by the fake sender). dists_mm: iterable, None/nan = invalid."""
    vals = []
    for v in dists_mm:
        vals.append(INVALID if v is None or v != v else int(max(0, min(65534, round(v)))))
    return HEADER.pack(b"LD", seq & 0xFFFF, len(vals), ms & 0xFFFFFFFF) + struct.pack("<%dH" % len(vals), *vals)


def parse_packet(data):
    """Returns (seq, ms, distances as a float array with nan for invalid) or None if the packet is bad."""
    if len(data) < HEADER.size:
        return None
    magic, seq, n, ms = HEADER.unpack_from(data)
    if magic != b"LD" or n not in (16, 64) or len(data) != HEADER.size + 2 * n:
        return None
    vals = np.array(struct.unpack_from("<%dH" % n, data, HEADER.size), float)
    vals[vals == INVALID] = np.nan
    return seq, ms, vals


class LidarReceiver:
    """Listens for packets in a background thread and calls on_frame(distances, sensor_time_s)."""

    def __init__(self, port, on_frame, bind="0.0.0.0"):
        self.on_frame = on_frame
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((bind, port))
        self.sock.settimeout(0.2)
        self.port = self.sock.getsockname()[1]
        self.frames = 0
        self.bad = 0
        self.last_rx = None
        self._stop = False
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        while not self._stop:
            try:
                data, _ = self.sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break
            pkt = parse_packet(data)
            if pkt is None:
                self.bad += 1
                continue
            _, ms, vals = pkt
            self.frames += 1
            self.last_rx = time.time()
            self.on_frame(vals, ms / 1000.0)

    def close(self):
        self._stop = True
        self.sock.close()


# ----------------------------------------------------------------------------------------------------------------------
# ESP-NOW bridge over USB serial. A computer cannot speak ESP-NOW itself, so a SECOND ESP32 (plugged into the computer by USB)
# receives the lidar ESP32's ESP-NOW packets and forwards each one down the serial line, framed so a lost byte can be resynced:
#     0xA5 0x5A | length (1 byte) | the same "LD" packet as above | CRC-8 of that packet (poly 0x07)
SYNC = b"\xA5\x5A"


def crc8(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def frame_bytes(payload):
    """Frame one "LD" packet for the serial line (used by the tests and the fake bridge; the real one is the ESP32 sketch)."""
    payload = bytes(payload)
    return SYNC + bytes([len(payload)]) + payload + bytes([crc8(payload)])


class FrameParser:
    """Feed it whatever bytes arrive (any chunking, with garbage or lost bytes in between); it returns each complete, CRC-correct payload."""

    def __init__(self):
        self.buf = bytearray()
        self.bad = 0                                  # frames dropped for a bad length or CRC

    def feed(self, data):
        self.buf += data
        out = []
        while True:
            i = self.buf.find(SYNC)
            if i < 0:
                del self.buf[:max(0, len(self.buf) - 1)]            # keep a possible half sync byte
                break
            if i > 0:
                del self.buf[:i]
            if len(self.buf) < 3:
                break
            n = self.buf[2]
            if n == 0 or n > 200:                       # not a plausible length: this was not a frame start
                del self.buf[:1]
                self.bad += 1
                continue
            if len(self.buf) < 3 + n + 1:
                break                                   # wait for the rest
            payload = bytes(self.buf[3:3 + n])
            if crc8(payload) == self.buf[3 + n]:
                out.append(payload)
                del self.buf[:3 + n + 1]
            else:
                del self.buf[:1]                         # bad CRC: skip this sync byte and look for the next frame
                self.bad += 1
        return out


CAM_HEADER = struct.Struct("<2sHBB")           # b"CM", frame id (uint16), chunk index, chunk count
CAM_CHUNK_BYTES = 190                          # JPEG bytes per ESP-NOW packet (header 6 + 190 = 196, under the 200 byte frame limit)


def make_camera_chunks(frame_id, jpeg):
    """Split one JPEG into ESP-NOW sized packets (used by the tests and the fake bridge; the real sender is cam_espnow.h)."""
    jpeg = bytes(jpeg)
    parts = [jpeg[i:i + CAM_CHUNK_BYTES] for i in range(0, len(jpeg), CAM_CHUNK_BYTES)] or [b""]
    return [CAM_HEADER.pack(b"CM", frame_id & 0xFFFF, i, len(parts)) + p for i, p in enumerate(parts)]


class CamAssembler:
    """Puts the chunks of one camera frame back together. A frame with a lost chunk is dropped (ESP-NOW broadcast has no retransmit);
    a chunk of a NEWER frame abandons the unfinished one. Chunks may arrive in any order. Returns the JPEG bytes when complete."""

    def __init__(self):
        self.frame_id = None
        self.parts = {}
        self.count = 0
        self.dropped = 0                            # frames abandoned incomplete or failing the JPEG check

    def feed(self, payload):
        if len(payload) < CAM_HEADER.size:
            return None
        magic, fid, idx, count = CAM_HEADER.unpack_from(payload)
        if magic != b"CM" or count == 0 or idx >= count:
            return None
        if self.frame_id is None or fid != self.frame_id:
            if self.parts:
                self.dropped += 1                    # the old frame never completed
            self.frame_id, self.parts, self.count = fid, {}, count
        self.parts[idx] = payload[CAM_HEADER.size:]
        if len(self.parts) < self.count:
            return None
        jpeg = b"".join(self.parts[i] for i in range(self.count))
        self.parts = {}
        self.frame_id = None
        if not (jpeg[:2] == b"\xff\xd8" and jpeg[-2:] == b"\xff\xd9"):      # not a whole JPEG: corrupt, drop it
            self.dropped += 1
            return None
        return jpeg


class SerialLidarReceiver:
    """Reads framed lidar packets from a USB serial port in a background thread and calls on_frame(distances, sensor_time_s).
    Same interface as LidarReceiver. `opener(port, baud)` can be replaced (the tests give it a fake port)."""

    def __init__(self, port, on_frame, baud=921600, opener=None, on_jpeg=None):
        if opener is None:
            import serial                                 # pyserial: pip install pyserial
            opener = lambda p, b: serial.Serial(p, b, timeout=0.1)
        self.on_frame = on_frame
        self.on_jpeg = on_jpeg                            # optional: the ESP32's camera frames (JPEG bytes) come down the same line
        self.cam = CamAssembler()
        self.jpegs = 0
        self.ser = opener(port, baud)
        self.parser = FrameParser()
        self.frames = 0
        self._bad_packets = 0
        self.last_rx = None
        self._stop = False
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    @property
    def bad(self):
        """Frames dropped for a bad length or CRC, plus framed packets that were not a lidar packet."""
        return self.parser.bad + self._bad_packets

    def _run(self):
        while not self._stop:
            try:
                data = self.ser.read(256)
            except Exception:
                break
            if not data:
                continue
            for payload in self.parser.feed(data):
                if payload[:2] == b"CM":                  # a piece of a camera frame, not a lidar frame
                    jpeg = self.cam.feed(payload)
                    if jpeg is not None and self.on_jpeg is not None:
                        self.jpegs += 1
                        self.on_jpeg(jpeg)
                    continue
                pkt = parse_packet(payload)
                if pkt is None:
                    self._bad_packets += 1
                    continue
                _, ms, vals = pkt
                self.frames += 1
                self.last_rx = time.time()
                self.on_frame(vals, ms / 1000.0)

    def close(self):
        self._stop = True
        try:
            self.ser.close()
        except Exception:
            pass
