# Handoff: Micromelon "pop the balloon" follower robot

Current state of the project. Read this first, then `README.md` (how to run things), `docs/CODE_TOUR.md` (every file in depth) and `docs/EXPERIMENTS.md` (what was tried and
the numbers). **Everything described as "tested" was tested in simulation only. Nothing has run on the real camera, rover, ESP32
or lidar.**

## 1. The goal and the rules
- A **Micromelon Rover** (the "follower") must **pop an opponent's balloon (on the opponent's back) with a pin on its own front,
  without having its own back balloon popped**. Up to **3 opponents**, differently coloured balloons (blue / green / yellow; not red,
  it looks like the tape).
- The arena is small, bordered by **red and black tape**, with a **square no-go zone in the middle that is turned 45 degrees (a
  diamond)**. "Caught" = balloon popped. What entering the no-go zone or crossing the tape costs is **unknown**: treated as bad.
- The user (Richard) programs the rover with **blocks in the Micromelon editor** and says he is not good at Python. He writes the
  chase moves; the system supplies `yaw` and the defence. A marker is allowed on the follower; the gyro is not. He likes short, direct
  answers and wants things shown and tested.

## 2. Hardware
| Part | Role | Status |
|---|---|---|
| External camera (the computer's back camera, may move) | positions of the follower and every opponent; follower heading from a marker | built (`tracker.py`) |
| Follower marker: one flat ArUco (DICT_4X4_50, ID 0, 8 cm) on top, top edge to the front | follower position + yaw | built |
| Floor marker ID 8 (15 cm), taped flat | defines the origin, re-solved every frame | built |
| ESP32 WROVER camera board + rear lidar (VL53L7CX 4x4, 60 deg, 3.5 m) | rear chaser detection over WiFi UDP | code written, never compiled on real hardware |
| Micromelon IR (sides), ultrasonic (front), **3 floor colour sensors**, encoders | IR picks a turn side; colour sensors now an opt-in tape detector (`--floor-sensors`); the rest unused | IR used; colour sensors untested on hardware |

Control link: the computer is the brain, Bluetooth to the rover (`micromelon` module).

## 3. Architecture
```
external camera -> tracker.py -> state {follower:{x,y,yaw}, A/B/C:{x,y}}
rear lidar (ESP32, UDP) -> lidar_rx.py -> evade.ChaserDetector
floor colour sensors (opt-in) -> floor.py
                      |
                      v
 pipeline.py: heading filter (wheel-based, gates glitchy camera headings, also through hidden-marker gaps), dead reckoning
              (+ contact anchoring), boundary escape, floor-sensor fix  ->  evade.ExternalWatch + plan.Planner
                      |
                      v
 evade.Brain.step(), in priority order: paused > no camera frames (stop) > tape guard > boundary escape > defend (face the
 threat) > every opponent popped (stand still) > chase(yaw) (or stand still if our marker is hidden with an opponent beside us)
                      |
                      v
 your chase(yaw) in my_chase.py -> micromelon Motors (speeds clamped to the cap) -> Bluetooth -> rover
```
`yaw` = degrees to the NEXT WAYPOINT (+ = left), `None` = nothing seen.

### Files
| File | What it is |
|---|---|
| `tracker.py` | external-camera tracker, live view (keys: click = colour, 1/2/3 slot, M mask, [ ] FOV, F re-lock, P/space pause, Esc) |
| `plan.py` | planner: route round the no-go zone (round-cornered keep-out), goal point behind the target, **target selection** (`TARGET_MODE`, default `sticky`) |
| `evade.py` | defence (`ExternalWatch`, `FacingTracker`, `EvadeController`, `MotionTap`, `Brain`, fail-safes). Constants at the top |
| `pipeline.py` | glue: heading filter, dead reckoning (+ contact anchoring), escape, floor-sensor fix, retarget-on-threat |
| `floor.py` | floor colour sensors as a tape detector (opt-in, uncalibrated for the real rover) |
| `follow.py` | entry point (`--camera N`, `--onboard URL`, `--floor-sensors`, `--max-speed`, `--target`, `--no-plan`, `--no-lidar`, ...) |
| `my_chase.py` / `example_chase.py` | the user's chase / a working example (continuous, non-blocking, with a search that creeps) |
| `mark_arena.py` / `scan_arena.py` | save `arena.json` (clicks, or auto-scan; scan aligns corners so a diamond averages correctly) |
| `onboard.py`, `lidar_rx.py`, `lidar_udp.h`, `lidar_test/`, `learn_mask.py` | onboard-camera mode, lidar receiver, ESP32 sketches, own-balloon mask |
| `simulate.py`, `sim_world.py` | full-render simulator (renders camera pictures; scenarios: chase_around, start_beside_nogo, hunter, three_calm, wanderer) |
| `tests/` | `test_units.py`, `test_gui.py`, `test_follow.py`, `test_scan.py`; `fastsim.py` (no pictures, models marker occlusion and heading errors near opponents); `psweep`-style sweeps; `targeting_study.py`; `plot_paths.py` |
| `docs/` | `EXPERIMENTS.md` (what was tried, numbers), dashboard preview images |
| `sim_output/` | videos and key frames |

## 4. How to run and test
See the Quick start at the top of `README.md`. In short: `python follow.py --camera N`; `python run_tests.py` (add `--sim` for the
full-render scenarios); `python simulate.py hunter` (video in `sim_output/`).

## 5. Key facts learned
- Micromelon: wheel speeds are cm/s, **-30..30, and the module RAISES outside that** (clamped now by `evade.install_speed_clamp`).
  `Motors.write(l, r)` is non-blocking. Negative degrees = left turn in `turnDegrees`; our `yaw` is + for left. A spin on the spot has an
  effective track of 18.7 cm (a 180 turn at 25 cm/s takes 1.18 s). There is no "read my speed" call: `MotionTap` decodes the packets.
- Vision: no lens calibration (assumed FOV, tune with [ ]); balloon height must be measured (`LEADER_HEIGHT`). **With a robot beside it
  the marker is hidden about 27% of the time (bursts of ~1.9 s) and its heading is wrong by more than 15 degrees about 21% of the time**
  (30-57 degrees for a while). That is why the heading filter keeps running on the wheel commands through gaps and gates the camera
  against them: it turned `hunter` full-render results from 4 wins/3 pops into 7 wins/1 pop.
- Dead reckoning drifts mainly because the opponent shoves us (the wheels know nothing of it). Contact anchoring (pull the estimate to
  the contact distance from its balloon) cut the position error after 3-5 s from 17 cm to 5 cm.
- A popped robot's body stays on the floor: the lidar still sees it. The camera-based "popped" test is "its balloon vanished at our pin
  tip"; a robot that vanishes beside us was just hidden.
- The planner keep-out must be grown with ROUND corners: a mitred corner on a 45 degree diamond closed the corridors beside it.
- **Target fixation had a bug** (it fell through to "nearest" whenever we were making progress, which also overwrote the retarget). Fixed. The
  targeting study was re-run after the fix and NO method beat another (docs/EXPERIMENTS.md); results quoted elsewhere that were produced before
  the fix (the 16-seed full-render `hunter` numbers in section 6) may shift and need re-measuring.

## 5a. The dashboard is the real interface
`dashboard.py` (stdlib HTTP server, 127.0.0.1, per-run token on every POST) + `dashboard.html` (the mock-up layout, live). `follow.py --camera N` starts it,
runs `tracker.main(headless=True, on_frame=...)`, and the rover waits for START. `follow.py` builds `snapshot()` / `command()`; `evade.Brain` gained
`manual`, `threat_now`, `safety_camera_stop`; `plan.Planner.set_preference`. `--window` keeps the OpenCV window. Tested with a fake rover and a fake
tracker (`tests/test_follow.py`), and the page was rendered at desktop and phone width; never run with the real camera or rover.

`experiments.py` = the registry behind the dashboard's Experiments panel. `evade.USE_ONBOARD_CHASE` (onboard bearing blended into the chase yaw) is OFF: 48-seed sim showed no net gain.

## 5b. Tandem mode (external + onboard camera)
`follow.py --camera N --onboard SRC|bridge`. `onboard.MultiTracker` (bearing+range per slot) -> `OnboardFeed` thread -> `Pipeline.on_onboard` -> `evade.Brain`
(onboard bearing replaces the external `turn_deg` for the threat; carries the facing for <=2 s when the external camera lost us). Fast sim, hunter:
survived 122/180 vs 78/180 without. WiFi vs ESP-NOW: `docs/WIFI_VS_ESPNOW.md` (user has exactly 2 ESP32s: rover + bridge). Hardware-untested.

## 6. Results (simulation)
Full renderer, `hunter` scenario, 16 seeds (older code, before the fixation fix): about 11-14 wins, 2-3 pops, 0-2 time-outs; zero violations in
most runs. `chase_around`, `start_beside_nogo`, `wanderer`: clean. `three_calm`: all popped, 0-24 tape frames. Details and every experiment:
`docs/EXPERIMENTS.md`. The fast simulator is calibrated to the full renderer for marker occlusion and heading errors, but it is still only a
guide: judge a change on full-render runs over many seeds.

## 7. Open problems
1. **Hunter fights are a coin flip** (about 70-85% wins against one hunter). Seed 13 (full render): the follower targets the hunter B (nearest,
   still at t=0), they meet head-on, B shoves the passive follower past the tape and pops it. Back-off fixes that seed but loses overall;
   sidestep, hold-ground, route-avoidance and "hunter last" were tried and are off (`docs/EXPERIMENTS.md`). Untried: wait ~1 s before the first
   target, use the opponent's heading.
2. Nothing here is validated on the real camera, rover, Bluetooth link (20 writes/s), lidar, or floor colour sensors. All pop-geometry numbers
   (`PIN_REACH_M`, `BALLOON_BACK_M`, `POP_REACH_MM`, robot radius, balloon height, camera delay, FOV, floor-sensor positions) must be measured.
3. Dead reckoning has no lidar/IR correction; yaw drifts to about 5 degrees.
4. The violation limits in the simulator (6 cm tape, 7 cm no-go, from the centre) are guesses at the real penalty rule (`sim_world.py`).
5. A lost Bluetooth link and a camera that sends frozen frames are not handled.

## 8. Open questions for the user
1. What exactly does entering the no-go zone or crossing the tape cost?
2. How is the no-go diamond marked on the real floor (tape outline or coloured mat)?
3. Exact ESP32 board, and where the lidar sits relative to the follower's balloon.
4. The measurements in 7.2, and whether the external camera is overhead or angled (and the size of the rover's top).
5. Fight, flee or kite a persistent hunter? That is a strategy decision the code cannot make for him.

## 9. Process notes
- A stop hook asks for everything to be committed and pushed after each turn; branch `claude/peaceful-bardeen-rg0sv5`.
- Run long jobs detached and from a SNAPSHOT of the repo (`git archive HEAD | tar -x -C /tmp/snap`), so edits do not leak into runs in progress.
- Do not use `pkill -f <pattern>` inside a command whose own text contains the pattern: it kills the shell.
