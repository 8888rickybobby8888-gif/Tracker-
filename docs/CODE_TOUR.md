# Code tour: every file, in depth

About 6,100 lines. Everything here was checked against the code, but ALL of it has only run in simulation. Numbers in `code font` are the
constants as they are now (they live at the top of each file).

Contents: 0 the big picture · 1 one camera frame, step by step · 2 `follow.py` · 3 `tracker.py` · 4 `pipeline.py` · 5 `plan.py` · 6 `evade.py` ·
7 your chase code · 8 the lidar chain (ESP32, `lidar_rx.py`, `learn_mask.py`) · 9 `floor.py` · 10 setup tools · 11 `onboard.py` ·
12 the simulator · 13 tests and studies · 14 conventions · 15 known gaps

---------------------------------------------------------------------------------------------------------------------

## 0. The big picture
```
 external camera ──► tracker.py ──► state {follower:{x,y,yaw}, A/B/C:{x,y}}  (floor frame, metres)
                                         │
 rear lidar (ESP32) ─UDP─► lidar_rx.py ──┼──────────────► ChaserDetector (evade.py)
 floor colour sensors (opt-in) ─► floor.py
                                         ▼
                              pipeline.Pipeline.on_state  (called on every camera frame)
                                heading filter · dead reckoning · escape check · retarget
                                  │                      │
                                  ▼                      ▼
                         ExternalWatch (evade.py)   Planner (plan.py)
                         "who can pop me?"          "which way to the target?"
                                  └──────────┬───────────┘
                                             ▼
                              Brain.step()  every 0.05 s, in its own thread
                         paused? > no camera? > tape? > edge? > defend > won? > chase(yaw)
                                             ▼
                          your chase(yaw) ─► micromelon Motors ─► Bluetooth ─► rover
```
Two ideas run through everything:
1. **The camera is the primary sense**, the lidar and wheel commands are backups. The camera sees every robot's position on the floor, so
   the defence can predict where an opponent's pin will be. But it is delayed (~0.15-0.25 s) and often loses the follower's marker.
2. **The wheel commands are a sensor too.** There is no "read my speed" call on the rover, so `MotionTap` decodes every motor packet; the
   heading filter and dead reckoning integrate those commands to know where the rover is when the camera can't tell them.

---------------------------------------------------------------------------------------------------------------------

## 1. One camera frame, step by step (what happens ~15-30 times a second)
1. `tracker.main` grabs a frame and calls `Tracker.update(frame)`.
2. The floor marker (ID 8) is found and the floor frame is re-solved (so a moving camera is fine); the follower's marker (ID 0) gives its
   x, y, yaw; each opponent's balloon is found by colour and projected onto the floor. Result: the `state` dict.
3. `follow.py` passes it to `Pipeline.safe_on_state(state, now)` → `on_state`:
   a. `_filter_yaw`: replace the camera's heading with the wheel-based heading, nudged by the camera unless the camera disagrees by more
      than 18° (then it is a glitch, unless it persists 1.2 s).
   b. `_dead_reckon`: if the marker was NOT seen, carry the last pose forward from the wheel commands (1.5 s; 5 s with an opponent beside us),
      anchored to the opponent's balloon when in contact.
   c. `ExternalWatch.update(...)`: is any opponent a threat? Which are active / popped / lost?
   d. `Planner.update(...)`: pick the target, build the route, produce `yaw` to the next waypoint → published for `chase()`.
4. Separately, about every 0.05 s, the **brain thread** runs `Brain.step()`: it reads the threat state, decides what the wheels do, and
   either runs the defence or calls your `chase(yaw)`.
5. Separately again, lidar packets arrive over UDP (30 Hz) → `Pipeline.on_lidar` → `ChaserDetector.update`.

---------------------------------------------------------------------------------------------------------------------

## 2. `follow.py` (210 lines): the entry point
`main(argv)`:
- parses `--camera N | --onboard URL | --rover ID | --lidar-port | --no-lidar | --max-speed | --target | --no-plan | --floor-sensors | --flip`.
- creates the `RoverController`, connects Bluetooth, `startRover()`.
- `evade.install_speed_clamp(Motors)`: replaces `Motors.write` with a version that CLAMPS to ±cap, because micromelon raises outside -30..30.
- `evade.MotionTap(...).install(rc)`: wraps the controller's motor calls to read and cap every command (including your chase code's).
- builds `ExternalWatch`, the `Planner` (if `arena.json` exists), the `ChaserDetector` (loading `lidar_mask.json` if present), optional
  `FloorSensors` (calibrates on bare floor), and the `Pipeline`.
- starts threads: the **brain** (`Brain.run`), the **lidar receiver**, a **watchdog** that prints a warning when the lidar or camera go quiet.
- runs `tracker.main(...)` in the main thread with `on_state=pipe.safe_on_state`, `on_key` (P / space = pause), and a status line.
- `finally:` stops the threads and writes `Motors.write(0, 0)`.
Also: `MicromelonMotors` (the 3 actions the defence needs: `turn_degrees`, `write`, `stop`), `read_ir` (side IR in cm), `read_floor_colours`.

---------------------------------------------------------------------------------------------------------------------

## 3. `tracker.py` (653 lines): the external-camera vision
**Floor frame.** The floor marker (ID 8, 15 cm) defines the origin; x and y run along its edges, z up. `_update_floor` solves the marker's pose
every frame (`solvePnP` IPPE, then Levenberg-Marquardt polish), rejects poses seen too edge-on, smooths with `FLOOR_ALPHA = 0.5`, ignores a
one-frame jump bigger than 25° / 0.25 m (a real bump must be seen twice), and holds the last frame `FLOOR_HOLD_S = 0.5` s if the marker is hidden.
`lock_floor` averages 40 frames at start-up.

**Follower.** `_pose` for marker 0 (8 cm, flat on top, its top edge toward the rover's front); `robot_from_marker` maps marker to robot
coordinates (`TOP_MARKER_OFFSET`, `MARKER_HEIGHT`). Poses with reprojection error > 3 px or seen from > ~70° off-normal are dropped. A "post"
layout with four markers also exists. Position and heading are smoothed (`ALPHA = 0.4`, heading as sin/cos so it wraps) and held `HOLD_S = 0.4` s.

**Opponents.** `opponent_mask` = HSV range per slot (A blue, B green, C yellow; set by clicking a balloon in the live view), blurred and
cleaned; `_detect_blob` takes the biggest blob (≥ 150 px); `_pixel_to_floor` casts the camera ray through the blob centre and intersects it
with the horizontal plane at `LEADER_HEIGHT` (0.15 m, MEASURE it: 3 cm wrong is ~5 cm of position error). The tracked point is the BALLOON, not the robot.

**No lens calibration.** Intrinsics come from `FOV_DEG = 70` (a guess), tuned live with `[` and `]`.

**`state(now)`** returns `{"follower": {x, y, yaw, age}, "A": {x, y, age}, ...}` for everything seen within the hold time.
**`main`**: the live window (overlay, mask view, keys 1/2/3, click-to-set-colour, M, F re-lock, `[ ]` FOV, P pause, Esc), with the
`on_state / on_yaw / on_draw / on_key / status` hooks; a raising `on_key` or `status` is caught and printed so it can't kill the vision loop.

---------------------------------------------------------------------------------------------------------------------

## 4. `pipeline.py` (323 lines): the glue
`Pipeline.__init__` builds the `EvadeController`, the `Brain` (with itself as the `guard`), and holds the pose and filter state.

**`_filter_yaw`** (the single most valuable piece, per the experiments): keeps a wheel-integrated heading `_yaw`; each camera reading is
compared with where the wheels say the rover was `YAW_DELAY_S = 0.2` s ago. Within `YAW_GATE_DEG = 18°` it nudges `_yaw` by
`YAW_ALPHA = 0.15` of the difference; beyond it, it is a glitch and ignored, unless it persists `YAW_GATE_S = 1.2` s, then the camera is
believed (the wheel estimate has drifted). The filter keeps running through marker gaps, so when the marker returns with a heading that a
robot half-covering it has made 30-57° wrong, it is checked, not adopted. Without this, hunter results were much worse.

**`_dead_reckon`**: while the marker is hidden, integrate `(left+right)/2` and `(right-left)/WHEELBASE` from `MotionTap`'s numbers for up
to `DEAD_RECKON_S = 1.5` s (`DEAD_RECKON_CLOSE_S = 5.0` s with an opponent within 0.5 m). **`_contact_fix`**: an opponent in contact shoves us,
which the wheels can't know, so the estimate is pulled radially to the contact distance from its balloon (`BALLOON_BACK_M + CONTACT_CENTRE_M`,
blending 15% per frame). That took the position error after 3-5 s from 17 cm to 5 cm.

**`on_state`**: the sequence in section 1. **`safe_on_state`** wraps it: on any exception the rover stops, yaw becomes `None`, and the next
frame retries. **Boundary helpers**: `escape(now)` (centre within `ESCAPE_NOGO_M`/`ESCAPE_TAPE_M` of an edge → `{away, yaw}`), `edge_push`
(the wider "soft" zone for the optional hold-ground), `clear_toward(now, rel_deg, dist)` (is the floor that way free?), `clear_behind`.
**Floor sensors**: `floor_tape(now)` polls at ≤ 10 Hz and, if the pose is only dead-reckoned and a sensor is on tape although the estimate
puts it inside the arena, `_tape_fix` shifts the estimate to the edge (max 15 cm). **Pause**: `pause / resume / toggle_pause` set `brain.paused`.
**Retarget**: if `RETARGET_ON_THREAT`, `planner.set_target(brain.defended_who)` before every plan.

---------------------------------------------------------------------------------------------------------------------

## 5. `plan.py` (442 lines): where to go and who to go for
**Geometry helpers**: `offset_convex` (mitred offset), **`offset_rounded`** (offset with ROUND corners every 30°: what a round robot sweeps
out; mitred corners on a 45° diamond close the corridors beside it), `inside`, `crosses` (does a segment cross a convex polygon; the shrunk
polygon is cached), `push_out`, `pull_in`, `_closest_on_boundary`. `Arena` holds the tape polygon and the no-go polygon from `arena.json`.

**`Planner.__init__`** builds: `free` = the tape polygon pulled in by `ROBOT_RADIUS_M (0.08) + TAPE_MARGIN_M (0.12)`; and two keep-outs
round the no-go zone, `block_threat` (radius 0.08 + `NOGO_MARGIN_M` 0.13) and `block_calm` (0.08 + `NOGO_MARGIN_CALM_M` 0.09). `self.block` is
the one in force: the big one while any opponent is moving (or was in the last 3 s), the small one otherwise. (A plain smaller keep-out was
a cliff: see `docs/EXPERIMENTS.md`.)

**`route(start, goal)`**: pushes the start and goal out of the keep-out and into `free`; if the straight line is clear, that's the path;
otherwise a visibility graph over the keep-out's vertices, Dijkstra from the goal, and the first leg chosen with `HYSTERESIS_M = 0.08` so it
doesn't flip sides when two routes tie. Returns `(path, blocked)`.

**`update(state, now)`**:
1. keeps each opponent's recent positions → a velocity (`HISTORY_S = 0.5`); an opponent moving ≥ `RING_MIN_SPEED` is "active" for 3 s;
2. picks the keep-out size; 3. `_choose` picks the target; 4. the goal point: if the target moves ≥ `MIN_SPEED = 0.08` m/s, a point
`BEHIND_M` behind where it will be in `LEAD_S = 0.4` s (so your pin meets its balloon); if not, its heading is unknown, so the balloon itself;
5. if nothing blocks the straight line and we're within `ENGAGE_M = 0.25` of the goal, aim straight at the target; else follow the route;
6. returns a `Plan` with `yaw_to_waypoint` (what `chase()` gets), `yaw_to_target`, the path, `blocked`, `engaged`, the target's name.

**`_choose` (target selection)**: `TARGET_MODE = "sticky"`: the nearest at first, then it STAYS on it while we are getting closer
(by `STALL_PROGRESS_M = 0.05`); if we have not gained for `STALL_S = 14` s and another opponent exists, it skips this one for `SKIP_S = 20` s.
`set_target(name)` switches to an opponent we had to defend against. Other modes (`switch`, `adaptive`, `active_first`, `calm_first`) exist for
the study; no mode was better (see experiments). `--target A` pins one colour slot. **`AVOID_R_M`** (route round moving non-targets) is off.
`draw_plan` draws the arena, keep-out, path and waypoint on the live view.

---------------------------------------------------------------------------------------------------------------------

## 6. `evade.py` (1,002 lines): the defence
### 6.1 Constants block (top)
Pop geometry (**MEASURE**): `BALLOON_BACK_M` your balloon behind your centre; `PIN_FROM_BALLOON_M` an opponent's balloon to its pin tip;
`PIN_REACH_M` your centre to its balloon when your pin touches it; `POP_REACH_MM` what the rear lidar reads when a pin touches your balloon;
`HIT_RADIUS_M`. `WHEELBASE_CM = 18.7` is the effective track for a spin on the spot (the micromelon module's own calibration, a 180° turn at
25 cm/s takes 1.18 s). `EVADE_LATENCY_S = 0.4` (camera + network + Bluetooth) and `TTC_MARGIN_S = 0.4` set how early a turn must start.

### 6.2 `ChaserDetector`: the rear lidar
Takes a 4×4 frame in millimetres. Zones that `learn_mask` marked are ignored. For each live zone it keeps ~0.3 s of readings and fits a
slope (mm/s). A zone is a "hit" when (a) it is salient (≥ `SALIENCE_MM = 120` nearer than the background median), and its closing speed
(own forward speed minus the range slope) is ≥ `CLOSING_MIN_MM_S = 150` and the time to contact is under the time needed to turn round, or
(b) it is simply very close (`DANGER_MM`, pop reach + 50 mm), or (c) we are spinning and something salient is within `NEAR_MM`. Two hits in a row
(`CONFIRM_FRAMES = 2`) make a `Threat` (bearing from the column, `static_only` if nothing was actually closing); it stays on `CLEAR_S = 0.4` s.

### 6.3 `ExternalWatch`: the camera threat logic (the main defence input)
`update(follower, opponents, t)` per frame. For each tracked opponent it works out: its bearing and distance to YOUR BALLOON (not your centre),
its velocity (0.5 s window), and whether it is **active** (moved ≥ 0.05 m/s in the last 3 s; a hunter that pauses is still dangerous).
Then in order:
1. **Behind-us danger**: not in front of us (`FRONT_SAFE_DEG = 40`), its balloon within `PIN_FROM_BALLOON_M + 0.04` of ours and behind us
   → a hit however it moves.
2. **Close ring**: active, within `CLOSE_RING_M = 0.40` of our balloon, not moving away from us, and not being outrun faster than
   0.04 m/s (but ALWAYS inside `HARD_RING_M = 0.30`) → a hit. (We face it.)
3. **Predicted pop**: its pin tip (balloon + `PIN_FROM_BALLOON_M` along its velocity) is on course to come within `HIT_RADIUS_M` of our balloon,
   with both motions included, sooner than we can turn to face it.
Three frames in a row (`EXT_CONFIRM_FRAMES`) make a `Threat` with the exact turn angle and the name (`who`).
It also keeps per-opponent **status** (`live` / `popped` / `lost`), **ghosts**, and the helpers the Brain uses:
- an opponent that vanishes counts as **popped** only if its balloon vanished within `POP_TIP_M = 0.16` of OUR pin tip and ahead of us
  (`_vanished_at_pin_tip`); otherwise it was merely hidden (`lost`);
- popped robots leave a **ghost** (their body stays on the floor and the lidar sees it); `lidar_object_is_inactive` lets the Brain veto a lidar
  threat that is really a parked or popped body;
- `all_popped(t)` / `pop_pending(t)`: every opponent seen has been popped and stayed gone 1.5 s → the game is won;
- `any_near_last_seen`, `distance`, `any_within`, `age`, `alive`.

### 6.4 `FacingTracker`: the main defence while an opponent is close
Keeps your FRONT toward it with smooth wheel commands (balloon on the far side). The camera bearing is `TRACK_DELAY_S = 0.25` s old, so it
runs an alpha-beta filter (`TRACK_ALPHA = 0.5`, `TRACK_BETA = 0.12`): the filtered bearing is predicted forward by our own turning (it
remembers the commands it sent) and the opponent's learned circling rate, a new reading far off the prediction (`TRACK_GATE_DEG = 35`) is
ignored for at most 2 readings in a row, and the commanded turn rate is `TRACK_KP (1.8) × bearing + circling rate`, slew-limited and capped at
170°/s. It never flips direction on noise when the opponent is dead behind. Optional `v_extra` adds forward/back motion (used by the off-by-
default hold-ground and back-off).

### 6.5 `EvadeController`: the quick turn
For a chaser from behind: `plan(threat)` gives the turn angle (the camera's exact bearing, or the lidar column's); dead behind (`BEHIND_DEG = 165`)
it uses the side IR sensors to pick the side with more room; `handle` blocks while `Motors.turnDegrees` spins at `EVADE_SPEED = 25`.

### 6.6 `MotionTap` and the speed cap
`decode_motor_packet` parses the rover's packets; `MotionTap.install(rc)` wraps `writeAttribute` and `doMotorOperation` so every motor command,
from anywhere, is clamped to `MAX_SPEED_CM_S` and recorded as `left`, `right` (`forward_mm_s`, `rotating`). `install_speed_clamp` does the same for
the Python-level `Motors.write` so an out-of-range speed is clamped instead of raising.

### 6.7 `Brain`: who is in charge each 0.05 s
`step(yaw)` in priority order:
1. **paused** → stand still (`_park`, which re-sends the stop every second).
2. **no camera frame for 0.8 s** (`NO_CAMERA_STOP_S`) → stop; resumes by itself.
3. **floor-sensor tape guard** (opt-in): tape under a sensor while driving forward → reverse 0.5 s.
4. Read the threat: the lidar's, vetoed if it is really a parked/popped body; fused with the camera's (the camera wins: it has the exact angle).
5. **Boundary escape**: near the tape or no-go zone and no opponent close → drive away (it is deliberately off while a hunter is close, because
   fighting it from an edge was safer than turning your back).
6. **Defend**: a camera threat with a name → `FacingTracker`; a lidar-only threat → the quick turn (with cooldown).
7. **All opponents popped** (or the last one just vanished beside us) → stand still.
8. **No pose with an opponent beside us** → stand still (don't search blindly).
9. Otherwise call **your `chase(yaw)`**.
`run(stop_event, get_yaw)` loops at 20 Hz; any exception stops the wheels and is printed; 3 in a row keep it stopped until a step works.
Off by default (tested, not helpful): `BACKOFF_SPEED`, `SIDESTEP_SPEED`, `HOLD_SPEED`.

---------------------------------------------------------------------------------------------------------------------

## 7. Your code: `my_chase.py`, `example_chase.py`
`chase(yaw)` is called ~20 times a second and must NOT wait. `yaw` = degrees to the next waypoint (+ = left), `None` = nothing seen. The
example turns on the spot if |yaw| > 18°, else drives at 22 cm/s steering with `0.5 × yaw` (capped ±7 so a wheel never exceeds 30). With
`yaw = None` it searches: mostly spinning on the spot, and every ~1.7 s creeping a short way in a small circle (so it leaves a spot where a
popped robot is hiding its marker). Speeds beyond the cap are clamped, not an error.

---------------------------------------------------------------------------------------------------------------------

## 8. The lidar chain
- **`lidar_udp.h` / `lidar_test/lidar_test.ino`** (ESP32, **never compiled on real hardware**): read the VL53L7CX with the SparkFun library and
  broadcast UDP packets on port 4210: `'L' 'D'`, sequence (uint16), zone count (1 byte), sensor time (uint32 ms), then `zones × uint16` distances
  (0xFFFF = none), little-endian. `lidar_test.ino` is the bring-up sketch (wiring in its header).
- **`lidar_rx.py`** (78 lines): `LidarReceiver(port, callback)` runs a thread that parses those packets (`make_packet` is the matching encoder
  used by the tests) and calls `callback(values_mm, sensor_time)`; `follow.py` forwards that to `Pipeline.on_lidar` with the rover's own speed.
- **`learn_mask.py`** (28 lines): with nothing behind the rover, listens 4 s, and any zone reading < 250 mm in ≥ 80% of frames (your own
  balloon and stick) is masked; saves `lidar_mask.json` (4×4 of 0/1). `ChaserDetector` excludes masked zones from both detection and the
  background estimate. Re-run it whenever the balloon or lidar moves. In the simulator the mask is learned at start-up.
- `COL0_IS_ROVER_LEFT` in `evade.py`: check with a hand-wave test that column 0 really is the rover's left.

---------------------------------------------------------------------------------------------------------------------

## 9. `floor.py` (69 lines): the floor colour sensors as a tape detector (opt-in, `--floor-sensors`)
`FloorSensors(read)`: `calibrate` takes ~10 readings on bare floor (median brightness and red excess); `tape()` returns `[left, middle, right]`
booleans: a reading is tape if much darker than the floor (< 0.45 × its brightness) or clearly red (≥ 1.5 × green and blue and ≥ 30 more), and a
sensor needs 2 reads in a row. `SENSOR_XY` (assumed 5 cm ahead, ±3 cm) and `sensor_world` give each sensor's floor position. Used for the tape
guard (Brain) and the dead-reckoned position fix (Pipeline). It looks forward, so it never sees the tape behind a rover that is being shoved backwards.

---------------------------------------------------------------------------------------------------------------------

## 10. Setup tools
- **`make_markers.py`**: prints `markers_sheet.pdf` (A4 at true size, 300 dpi) and `markers/marker_<id>.png` from `tracker.py`'s sizes.
- **`mark_arena.py`**: click the 4 inner tape corners and the 4 no-go corners (at floor level) on the live view; each click is converted to a
  floor position at that moment, so a drifting camera is fine; saves `arena.json` (relative to the floor marker). `U` undo, `N` skip the no-go zone.
- **`scan_arena.py`** (338 lines): `rectify` re-draws the floor from straight above using the floor marker; `tape_mask` finds red and black
  tape; `_fit_quad` fits the inner edge of the tape ring as the arena; inside it, the squarest, fullest non-floor-coloured blob is the no-go zone
  (`minAreaRect`, so any rotation works, including your 45° diamond); robots the tracker sees are excluded. `combine` takes the median over ~15
  scans, **`_align` first gives every scan's corners the same starting corner** (for a diamond, noise decides which corner comes first), and
  reports how steady it is. Warns about gaps in the tape, a dark floor, several square-ish objects.

---------------------------------------------------------------------------------------------------------------------

## 11. `onboard.py` (201 lines): the alternative onboard-camera mode
`follow.py --onboard URL`: the ESP32-CAM on the follower itself. No markers or floor frame: a balloon's left/right offset from the image
centre IS the angle to turn (`atan((cx - x) / focal_px)`). Click to set the colour, `[ ]` to tune the FOV. Evasion then uses the lidar only (no
external camera). It is not combined with the external camera.

---------------------------------------------------------------------------------------------------------------------

## 12. The simulator
- **`sim_world.py`** (535 lines): `World` steps `Agent`s (a differential-drive body, a pin 0.10 m ahead, a balloon 0.10 m behind) at 30 Hz with
  push-apart collisions (`COLLIDE = 0.13`) and pops when a pin comes within `POP_DIST = 0.045` of a balloon. `Opponent.policy` implements
  `static`, `patrol`, `orbit`, `hunter` (aims at where the follower's balloon will be, reacting to where the follower WAS 0.4 s ago, using the
  same planner) and `wander` (random points). `CameraRig` is a camera that drifts; `Renderer` draws ArUco markers, balloons, the tape and the no-go
  diamond in perspective so the REAL `tracker.py` has to find them. `lidar_frame` (4×4, latency, noise, dropouts, your own balloon), `ir_cm`,
  `floor_flags`. `SimRC`/`SimMotors` run the real micromelon `Motors` code on top of a fake link. The no-go zone is a 0.15 m square turned
  45°; `NOGO_VIOLATION_M` (0.07) and `TAPE_VIOLATION_M` (0.06) are the guessed violation limits.
- **`simulate.py`** (446 lines): `Sim` wires the world to the real pipeline (camera at 15 Hz with 0.15 s latency, lidar 30 Hz with 0.10 s) and
  logs everything; scenarios `chase_around`, `start_beside_nogo`, `hunter`, `three_calm`, `wanderer`; `draw_map` (top-down truth, trails, plan,
  lidar wedge, keep-out outlines), `draw_timeline`, `Recorder` (the video and key frames), `run_scenario`.
- **`tests/fastsim.py`**: the same simulation without drawing pictures (tracker output = truth + replayed measured noise, delay and dropouts), about
  20× faster. It also models the two effects measured on the full renderer: the follower's marker hidden in bursts when an opponent is within
  0.45 m (~27% of the time), and wrong headings (> 15° about 21% of the time within 0.3 m).

## 13. Tests and studies
`run_tests.py` (`--sim` adds the full scenarios) runs: compile everything; `tests/test_units.py` (planner geometry and routing, lidar and camera
threat logic, facing tracker, fail-safes, heading filter, dead reckoning, popped detection, targeting, speed clamp, floor sensors);
`tests/test_gui.py` (live view keys, mouse, pause key, `mark_arena`, `onboard`, `learn_mask` over real UDP); `tests/test_follow.py` (`follow.py`
end to end with a fake rover, camera and ESP32); `tests/test_scan.py` (the arena scan). Studies: `tests/sweep_hunter.py`, `tests/targeting_study.py`,
`tests/plot_paths.py`; results and every experiment in `docs/EXPERIMENTS.md`.

## 14. Conventions
Floor frame in metres, origin at the floor marker, yaw counter-clockwise from +x. Robot frame: x forward, y left. Turns: + = left. Lidar offset:
angle from the rover's REAR axis, + toward its left. micromelon `turnDegrees`: NEGATIVE = left, so a left turn is `turnDegrees(-yaw)`.
Wheel speeds in cm/s, -30..30.

## 15. Known gaps
Nothing is validated on the real camera, rover, Bluetooth link, ESP32 or lidar. All geometry constants must be measured. Against one hunter the
follower wins roughly 70-85% of fights in simulation (seed 13 loses: shoved out and popped). Dead reckoning has no lidar/IR correction. The real
penalty rule is unknown. A lost Bluetooth link and a frozen camera picture are not handled. See `HANDOFF.md` and `docs/EXPERIMENTS.md`.
