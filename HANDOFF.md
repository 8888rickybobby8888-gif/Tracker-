# Handoff: Micromelon "pop the balloon" follower robot

Written at the end of a long working session. Everything described as "tested" was tested **in simulation only**. Nothing has run on the real camera, rover or ESP32.

> **Note:** this repo currently holds only this handoff. The code it describes (`tracker.py`, `evade.py`, `pipeline.py`, `follow.py`, simulators, tests, etc.) lives in a separate working folder and has NOT been copied here yet.

---

## 1. The goal and the rules

- A **Micromelon Rover** (the "follower") must **pop an opponent's balloon, which sits on the opponent's back, with a pin on its own front, without having its own back balloon popped**. Up to **3 opponents**, each with a differently coloured balloon (blue / green / yellow recommended; **avoid red**: it looks like the tape).
- The arena is small, bordered by **red and black tape**, with a **square no-go zone** in the middle.
- "Caught" means **balloon popped** (touching is not a loss). Whether entering the no-go zone or crossing the tape is penalised, and how, is **not known**: treat both as bad.
- The user (Richard) programs the rover with **blocks in the Micromelon editor** and says he is **not very good at Python**. He writes the chase moves; the system provides `yaw` and does the defence. He is allowed a **marker** on the follower but **cannot use the gyro**. He likes short, direct answers and wants things shown/tested, not just described.

## 2. Hardware and what each part does

| Part | Role | Status |
|---|---|---|
| **External camera** = the computer's back camera, **not fixed** (may move) | Floor positions of the follower and every opponent; follower heading from a marker | Built (tracker.py) |
| **Follower marker**: ONE flat ArUco marker (DICT_4X4_50, ID 0, 8 cm) lying on top, top edge pointing to the front | Follower position + yaw | Built. A 4-faced post layout also exists (`MARKER_LAYOUT="post"`) |
| **Floor marker**: ID 8, 15 cm, taped flat, defines the origin | Floor frame is re-solved from it EVERY frame so the camera can move | Built |
| **ESP32 WROVER** camera board (pin map probably `CAMERA_MODEL_WROVER_KIT`; exact board name unknown) | Streams video (CameraWebServer example) and carries the lidar | Camera stream = stock example; lidar code written but **never compiled on real hardware** |
| **Rear lidar** Pololu VL53L7CX 4x4 mode, 60 deg FOV, 3.5 m, mounted at the BACK | Detects something closing in from behind | Built (evade.py + lidar_rx.py), ESP32 side untested |
| **Micromelon built-in sensors**: IR left+right (point sideways), ultrasonic (front), 3 floor colour sensors, encoders | IR used to pick a turn side; the rest unused so far | IR used; others not |
| **Rover balloon** on its back, **pin** on its front | Game geometry | Measurements still needed |
| SD card (1 GB) on the ESP32 | Not used | - |

Control link: **computer is the brain, Bluetooth to the rover** (`micromelon` Python module). ESP32 -> computer over **WiFi UDP** (lidar). A fallback where the ESP32 drives the rover over the expansion header **UART** was researched (115200 baud, packets `0x55 opcode type len payload`; header is I2C *or* UART, not both; pinout/voltage NOT found) but **not built**.

## 3. Architecture (software)

```
external camera -> tracker.py ---------------------------------> state {follower:{x,y,yaw}, A/B/C:{x,y}}
rear lidar (ESP32, UDP) -> lidar_rx.py -> evade.ChaserDetector --\
                                                                 v
state -> pipeline.py: heading filter, dead reckoning, escape check -> evade.ExternalWatch + plan.Planner
                                                                 v
                          evade.Brain.step():  escape?  ->  defend (FacingTracker / evade turn)  ->  chase(yaw)  ->  (search)
                                                                 v
              your chase(yaw) in my_chase.py (from Micromelon blocks) -> micromelon Motors -> Bluetooth -> rover
```

Priority each 0.05 s step (evade.Brain): **boundary escape** (only when no opponent is close) > **defend** (an opponent is threatening: keep facing it) > **chase** (user's `chase(yaw)`, where `yaw` = degrees to the NEXT WAYPOINT, + = left, None = nothing seen) > search (user's job when yaw is None).

### File map (in the original working folder)

| File | What it is |
|---|---|
| `tracker.py` | External-camera tracker: floor frame from marker 8 (per frame, glitch gating), follower pose from its marker, up to 3 opponents by balloon colour (click-to-set colour, keys 1/2/3), live view, `on_state/on_yaw/on_draw` hooks. No lens calibration: **assumes a field of view** (`FOV_DEG`, tune with `[` `]`). |
| `plan.py` | Waypoint planner: shortest route inside the tape and round the no-go zone (grown by `ROBOT_RADIUS_M`+margins), goal point BEHIND the opponent, `yaw_to_waypoint`. |
| `mark_arena.py` / `scan_arena.py` | Save `arena.json` (arena + no-go corners in the floor frame): click corners, or auto-scan the red/black tape and the no-go square. |
| `evade.py` | Defence logic: lidar `ChaserDetector`, camera `ExternalWatch`, `FacingTracker`, `EvadeController`, `MotionTap` (own speed estimate + global speed cap), `Brain`. Most tunables at the top. |
| `pipeline.py` | Shared glue used by `follow.py` and the simulator: yaw heading filter, dead reckoning, boundary-escape check, wires everything into `Brain`. |
| `follow.py` | Entry point (`--camera N`, `--onboard URL`, `--lidar-port`, `--no-lidar`, `--no-plan`, `--target`, `--max-speed`, `--rover`). |
| `my_chase.py` | **The user's file.** `chase(yaw)` goes here. `example_chase.py` is a complete working version (continuous, non-blocking). |
| `onboard.py` | Alternative mode: yaw to ONE balloon from the ESP32's own camera. |
| `lidar_rx.py`, `lidar_udp.h`, `lidar_test/lidar_test.ino`, `learn_mask.py` | Lidar UDP receiver; ESP32 sender; bring-up sketch; learns which lidar zones are the follower's own balloon. |
| `make_markers.py`, `markers/`, `markers_sheet.pdf` | Printable markers (floor ID 8, follower ID 0) at true size. |
| `simulate.py`, `sim_world.py` | Full-system simulator (renders camera pictures, real tracker/planner/evasion, real `micromelon.Motors` on a simulated rover, a hunter AI). |
| `tests/` | `test_units.py`, `test_gui.py`, `test_follow.py`, `test_scan.py`; `fastsim.py` (~20x faster, replays measured tracker noise); `sweep_hunter.py`, `sweep_scenarios.py`, `compare_sweeps.py` and saved `sweep_*.json` / `scen_*.json`. |
| `run_tests.py` | One-command runner (`python run_tests.py`, add `--sim` for the slow scenarios). |
| `README.md` | User-facing how-to. Partly stale (see section 7). |
| Dashboard preview | Static mock-up: https://claude.ai/artifact/P9xthM5gVy14zLq9CxsgYx |

## 4. How to run and test

```
pip install -r requirements.txt            # opencv-contrib, numpy, micromelon, pillow
python make_markers.py                      # print markers_sheet.pdf at 100%, matte paper, glue flat
python tracker.py --list                    # find the camera; --camera N; --flip if mirrored; tune FOV with [ ]
python scan_arena.py --camera N             # or mark_arena.py (clicks); writes arena.json
python follow.py --camera N                 # run everything (add --max-speed, --target A, --no-plan ...)

python run_tests.py                         # compile + unit + screen/script + follow + scan tests
python run_tests.py --sim                   # + the full simulator scenarios (slow)
python simulate.py hunter --no-video        # one scenario
python tests/sweep_hunter.py --label X      # 14 hunter cases x 10 seeds on the fast simulator (~4 min)
python tests/compare_sweeps.py baseline X   # side-by-side with the saved baseline
```

Last known test status (all simulated): unit 57/57 (not re-run after the last default change, `ESCAPE_DRIVE=True`), screen/script 13/13, `follow.py` 8/8, arena scan 14/14. ESP32 sketches were only syntax-checked against stand-in headers written from memory.

## 5. Key facts learned

**Micromelon module (v1.2.1)**
- Wheel speeds are **cm/s, limited to -30..30**. `Motors.write(l, r)` is **non-blocking** and persists until the next command; `Motors.write(l, r, secs)` blocks then stops; `turnDegrees`/`moveDistance` block.
- **Negative degrees = LEFT turn** in `turnDegrees`/`turn`. Our `yaw` is + for left, so left turn = `turnDegrees(-yaw)`.
- A spin on the spot uses an effective radius of 9.35 cm, so a 180 deg spin at 25 cm/s takes **1.18 s, not 0.66 s**. `evade.WHEELBASE_CM = 18.7` encodes this.
- `rc.end()` calls `sys.exit`; `RoverController()` registers a SIGINT handler (create in main thread); Ctrl+C stops the rover. `IR.readAll()` returns `[left, right]` in cm; `Ultrasonic.read()` cm; `Colour` gives `[left, middle, right]`.
- No "read my speed" call: `evade.MotionTap` decodes packets to estimate speed/turning and enforces the global speed cap (`evade.MAX_SPEED_CM_S`, `--max-speed`).

**Vision**
- No chessboard calibration: intrinsics from `FOV_DEG` (default 70, a guess). Tune live.
- Camera not fixed: floor frame re-solved every frame from marker 8 (smoothing `FLOOR_ALPHA`, hold 0.5 s if hidden, single-frame glitches ignored, real bumps accepted after 2 frames). Bigger floor marker = steadier origin.
- Opponent position = balloon pixel -> floor point using assumed balloon height (`LEADER_HEIGHT` 0.15 m, **must be measured**; 3 cm wrong is ~5 cm of position error).
- Flat top marker works best overhead (sim: ~0.7 deg mean yaw error at 1.5 m up, 8 cm); at 45 deg an 8 cm marker is poor, use 10 cm or the post layout. Pose solve = IPPE + LM refinement.
- Marker yaw glitches for 0.3-0.7 s about once per 11 s: `pipeline._filter_yaw` uses wheel commands to ignore them.

**Defence design**
- Threat is judged on the opponent's pin tip vs our balloon (`BALLOON_BACK_M`, `PIN_FROM_BALLOON_M`, `HIT_RADIUS_M`; lidar uses `POP_REACH_MM`). Warning time = latency + spin time + margin (`lead_time_s`).
- Facing a chaser is the main defence. A continuous facing controller beat discrete turns.
- An opponent that moved in the last 3 s is "active". `RING_MIN_SPEED` must stay above tracker speed noise (0.05 now; 0.03 gave 84% false alarms).
- A blocking chase is dangerous (rover blind during the move). `example_chase.py` writes wheel speeds continuously and never waits.

## 6. Last session's changes and evidence (simulation)

Continuous `example_chase`; activity memory + defence on any side + never exempt inside `HARD_RING_M=0.30`; `FacingTracker`; heading filter in `pipeline.py`; boundary escape drive (`ESCAPE_DRIVE=True`, only when no opponent is close). Rejected: escape/nudge while an opponent is near (`ESCAPE_NUDGE`, off): survival fell 12/12 -> 5/12.

Fast sim, 14 hunter cases x 10 seeds:

| | before | after |
|---|---|---|
| average survival | 74% | 83% |
| average "won" | 13% | 36% |
| lone 0.11 m/s hunter + target | survive 10/10, won 2/10 | survive 10/10, won 5/10 |
| 15% camera dropouts | survive 2/10 | survive 8/10 |
| camera noise x2 | survive 4/10 | survive 8/10 |
| non-hunter scenarios (time to pop target) | 32 s / 40 s | 11 s / 8 s |

Full renderer, one seed each: `chase_around` goal 7.6 s; `start_beside_nogo` 10.7 s; `hunter` goal 31.6 s (222 no-go frames, 50 tape frames); `three_calm` **follower popped by C at 14.9 s** (regression). Before the changes `hunter` lost in 5/5 runs.

## 7. Known problems (priority order)

1. **Shoved onto the tape / into the no-go zone while defending.** Tape time averages 400-1,000 frames per 60 s run in several hunter cases; no-go 100-170 frames. Escape is disabled while a hunter is near (it cost survival). Ideas: sidestep along the edge while facing, route defence away from edges earlier, a rule from the user about what counts.
2. **`three_calm` regression**: ~6.5 defensive events (baseline ~2.6), 62 tape frames in fast sim; popped by "calm" opponent C in the full renderer. Investigate first.
3. **Two hunters**: survive only 2/10. **Hunter as fast as follower (0.30 m/s)**: survive 6/10 (baseline 10/10).
4. A persistent hunter is a stalemate.
5. Fast and full simulators disagree; full renderer only run one seed per scenario.
6. Not validated on hardware: Bluetooth command rate at 20 writes/s, camera latency/noise, ESP32 lidar code, FOV, balloon height, pop-geometry numbers.
7. Docs/tests stale: `README.md` and `my_chase.py` header describe discrete turns / "keep each call short" (new advice: `Motors.write` and return); `run_tests.py --sim` still expects `hunter` to be a "known weakness"; dashboard preview lacks the new defence; README mentions renamed constants.
8. Smaller: `mark_arena.py` uses the NEXT frame's pose (~13 mm error); scan accuracy limited by floor-marker pose; scan assumes floor lighter than black tape; lidar left/right orientation must be checked (`COL0_IS_ROVER_LEFT`).

## 8. Not built / ideas

Tape sensing by camera or floor colour sensors; avoiding the front of an opponent when approaching; evasion aware of the no-go zone; combining onboard ESP32 camera with the external one; retreat-while-facing / kiting; a real dashboard; match recording; the UART fallback; front distance from ultrasonic; multi-floor-marker tracker.

## 9. Gotchas about the working folder

- Another session edited the same folder between turns (created `pipeline.py`, `sim_world.py`, `simulate.py`, `example_chase.py`; changed `evade.py`, `follow.py`, `plan.py`, `README.md`). Check modification times before editing.
- Simulator is the main safety net: `tests/fastsim.py` for sweeps, `simulate.py` to confirm; copy the tree first to get an unbiased baseline.
- `tests/sweep_*.json` are saved results (`baseline` = before hunter work, `after` = now, `iter1-5` intermediate).
- Sandbox is single-core; the 4 full-render scenarios take ~5-10 min; run detached (`setsid nohup`).
- Checked on Python 3.12 with `opencv-contrib-python` 4.x/5.x, numpy, micromelon 1.2.1, pillow, pymupdf (tests only).

## 10. Open questions for the user

1. What happens if the follower enters the no-go zone or crosses the tape (loss, penalty, nothing)?
2. How is the no-go square marked on the real floor (taped outline or coloured mat)?
3. Exact ESP32 board name, and where the lidar is mounted relative to the follower's balloon.
4. Measurements: `PIN_FROM_BALLOON_M`, `BALLOON_BACK_M`, `PIN_REACH_M`, `POP_REACH_MM`, `ROBOT_RADIUS_M`, balloon height (`LEADER_HEIGHT`), camera delay (`EVADE_LATENCY_S`), true camera FOV.
5. Is the external camera overhead or angled, and how big is the top of the rover (marker size)?
6. Fight, flee or kite a persistent hunter? A strategy decision the code cannot make.
