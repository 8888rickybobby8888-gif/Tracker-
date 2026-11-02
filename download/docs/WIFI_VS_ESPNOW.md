# WiFi or ESP-NOW for the lidar and the onboard camera?

**Nothing here was measured on real hardware.** The link properties are published/typical figures and my estimates; the effect of a
slower or lossier link on the *defence* comes from the fast simulator (section 4). Measure on your boards before trusting a number.

What has to travel from the follower's ESP32 to the computer:

| Stream | Size | Rate we want |
|---|---|---|
| Rear lidar, 8x8 | 64 distances, ~140 B | 15 Hz |
| Onboard camera (QVGA JPEG, ~6-10 KB) | 30-50 chunks of <=190 B | 10-15 fps |

## 0. You have two ESP32s: what that means
- **ESP-NOW uses both:** #1 on the rover (camera + lidar on the SAME board, as `cam_espnow.h` / `lidar_udp.h` are written) and #2 as the
  bridge on the computer's USB. Nothing is left over, so the lidar and the camera must share the one rover board (and its one radio).
- **WiFi uses one:** the rover board only; the second ESP32 stays spare (or can be a second lidar/camera board if you want one).
- You cannot give the camera and the lidar their own ESP-NOW boards with only two; that would need three (two senders + one bridge).

## 1. The two options as built here

- **WiFi (default).** The rover-side ESP32 joins the computer's network. Lidar goes as UDP (`lidar_udp.h` -> `lidar_rx.LidarReceiver`),
  the camera as an MJPEG HTTP stream (`onboard.LatestFrame`).
- **ESP-NOW.** The rover-side ESP32 sends raw 2.4 GHz packets to a second ESP32 (the bridge, `espnow_bridge/espnow_bridge.ino`)
  plugged into the computer by USB at 921600 baud. Lidar = `LD` packet, camera = `CM` chunks (`cam_espnow.h`, `lidar_rx.CamAssembler`).
  No router, no IP, no pairing handshake.

## 2. Comparison

| | WiFi (UDP + MJPEG) | ESP-NOW + USB bridge |
|---|---|---|
| Needs a router / hotspot | Yes. Venue WiFi is often crowded or blocks device-to-device | **No** |
| Extra hardware | None | **Second ESP32** (you have one) |
| Setup | SSID/password in the sketch, find the IP | Flash two sketches; the bridge shows up as a COM port |
| Latency (estimate) | 5-30 ms typical, **spikes to 100-300 ms** when the channel is busy or a power-save/roam happens | 2-10 ms, much steadier (no AP, no TCP) |
| Throughput | Plenty (MJPEG 15 fps is no problem) | ~400-500 packets/s: a 40-chunk frame takes ~80-100 ms of airtime, so **5-10 fps** QVGA at best |
| Reliability | UDP drops under congestion; MJPEG over TCP stalls then jumps | Broadcast has **no retransmit**: a lost chunk = a lost camera frame (lidar frame is one packet, so cheap) |
| Camera frame loss at 5% packet loss | low (TCP resends, but adds lag) | `1-(0.95)^40` = **~87% of frames lost** with 40 chunks; at 1% loss ~33%. Make frames smaller (fewer chunks) or accept lower fps |
| Interference with other teams | Shares the venue's channel; many robots+phones = trouble | Also 2.4 GHz, but fixed channel you pick, tiny packets, and no AP to drop you |
| Range | AP-dependent | Line of sight in a small arena: no issue |
| Failure mode | Silent stall (we detect `alive()` and warn) | Same; CRC-8 on the serial frames drops corrupt data (tested) |
| Camera + lidar together | Two independent flows | Share one radio: camera chunks can delay a lidar packet; send lidar first / cap camera fps |
| Works with the ESP32 also being the lidar host | Yes | Yes (both in `lidar_udp.h` by a `#define`) |

## 3. Decision: WiFi (ESP-NOW stays an optional extra)
The user's choice: **use WiFi**; ESP-NOW was only a concept. With WiFi the second ESP32 isn't needed, and the simulator's best case
(15 fps, ~0.12 s) is what WiFi normally delivers. Setup: put the rover ESP32 on a hotspot (the computer's, or a phone's) that nothing
else uses, then `python follow.py --camera 1 --onboard http://<esp32-ip>:81/stream` (lidar UDP arrives on port 4210 as before).
Watch for `WARNING: no onboard camera data`; if it keeps appearing, move the hotspot's channel or drop `--onboard`.
The rest of this section is the original reasoning, kept for reference.

## 3b. Original recommendation

- **Lidar: ESP-NOW.** One small packet at 15 Hz, steady low latency, no router, a lost frame costs nothing. This is the clear win.
- **Onboard camera: depends on the venue.**
  - Quiet 2.4 GHz and you can run your own hotspot -> **WiFi** gives smoother video (more fps, no all-or-nothing frames).
  - Crowded venue / no router allowed -> **ESP-NOW**, but keep frames small (QVGA, JPEG quality ~20, <= 25 chunks) and treat the camera as
    a bonus: in tandem mode the defence only *uses* it for the facing when the external camera loses the marker, and falls back by itself.
- **Safest field setup:** lidar on ESP-NOW (always), camera on whichever of the two you can verify in the venue; run `follow.py` with
  `--camera N --onboard <SRC>` and watch for the `WARNING: no onboard camera data` line (the watchdog prints it).

## 4. Does the link quality change the fight? (fast simulator)

`tests/fastsim.py` models the onboard camera as bearing + range of each opponent balloon in view (+-1 degree, +-10% range), delivered with a
frame rate, a latency and a loss rate. The follower fights one hunter. "Survived" = own balloon not
popped, "won" = popped the opponent first, "tape/no-go" = frames over the limits.

Hunter scenario, 24 seeds per row (fast simulator, same code; only the onboard link differs):

| Onboard camera link | fps | latency | frames lost | survived | won | zero-violation runs |
|---|---|---|---|---|---|---|
| none (external camera only) | - | - | - | 11/24 | 7/24 | 20/24 |
| ideal | 15 | 0.12 s | 0% | **21/24** | **15/24** | 19/24 |
| WiFi-like | 15 | 0.15 s | 5% | 17/24 | 8/24 | 23/24 |
| ESP-NOW-like, 1% packet loss | 8 | 0.10 s | 33% | 10/24 | 7/24 | 18/24 |
| ESP-NOW-like, 5% packet loss | 8 | 0.10 s | 87% | 15/24 | 7/24 | 18/24 |
| congested WiFi | 6 | 0.30 s | 20% | 15/24 | 11/24 | 20/24 |

A larger 12-seed sweep over all hunter variants (15 Hz, 0.12 s, no loss) gave survived 122/180 vs 78/180 and won 72/180 vs 45/180 without the onboard camera.

**Reading it honestly:** with 24 seeds one row has roughly +-10 percentage points of noise, so the rows below "ideal" cannot be told apart from each
other (the 5%-loss ESP-NOW row beating the 1%-loss one is noise). What the data does support: (1) a good onboard camera clearly helps (about
2x survival over external-only, in both sweeps); (2) the benefit shrinks as fps drops and frames are lost, and a poor link is roughly
the external-only result, not worse; (3) no link profile hurt the line behaviour (tape/no-go frames are not worse). So the choice of link
matters mainly through fps and lost frames, which is why the recommendation is whichever link you can make deliver >= 10 fps with little
loss in your venue.


Read it as a guide to *relative* sensitivity, not as hardware numbers: the simulator's marker-hiding and heading-error models are
calibrated against the full renderer, but the onboard camera's field of view, its mounting height above the other robot's body and the
true latency are assumptions.

## 5. What to measure on the real hardware (10 minutes)

1. Lidar: count frames/second arriving in `follow.py` for 60 s over each link (`detector.lidar_alive()` plus a counter).
2. Camera: frames/second that decode (`JpegFrameSource.bad` counts corrupt ones) with the rover driving and a second robot close by.
3. Latency: wave a balloon in front of the onboard camera while recording the screen at 60 fps and compare to the tracker overlay.
4. If the camera is below ~8 fps or latency spikes above ~200 ms, drop it from the loop (`--camera N` only); the defence is unchanged.
