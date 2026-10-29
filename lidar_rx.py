"""
lidar_rx.py - receives the rear-lidar frames the ESP32 sends over UDP.

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
