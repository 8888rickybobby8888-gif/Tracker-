# Follower/leader tracker (yaw + degrees to leader)

## Quick start

### How to run it (real rover)
One-time setup:

    pip install -r requirements.txt             # opencv-contrib, numpy, micromelon, pillow
    python make_markers.py                      # print markers_sheet.pdf at 100%; floor marker ID 8, follower marker ID 0
    python tracker.py --list                    # find the external camera (press Y on the right one)
    python scan_arena.py --camera 1             # saves arena.json (tape + no-go square); or mark_arena.py to click the corners

Each match:

    python follow.py --camera 1                 # external camera + rear lidar + your chase code (my_chase.py)

- Put your chase moves in `my_chase.py` (`example_chase.py` is a complete working version to copy from).
- In the live window: click a balloon to set its colour (keys 1/2/3 pick the slot), `P` or space = **pause / resume**
  (the rover stands still while paused), `[` `]` tune the field of view, `F` re-lock the floor, `Esc` quit.
- It stops by itself once every opponent has been popped. Ctrl+C in the terminal also stops the rover.
- Useful options: `--max-speed 20` (cap the wheels), `--target A` (always chase slot A), `--no-lidar`, `--no-plan`,
  `--rover 2` (Bluetooth ID), `--flip` (mirrored image), `--floor-sensors` (use the rover's three floor colour sensors as a tape
  detector: start with the rover on bare floor inside the arena. NOT tested on the real rover). Details are further down.

### What happens when something goes wrong (fail-safes)
- **Ctrl+C**, or the live window closing, stops the rover.
- **Pause** (`P` / space) stops it until you press it again.
- **The external camera stops delivering frames** for 0.8 s: the rover stops and says so. It resumes by itself when frames return.
- **An error in the brain loop, including in your `chase()`**: the wheels are stopped at once (they never keep their last command),
  the error is printed, and it retries every half second. After 3 errors in a row it prints that it is stopped and stays stopped
  until a step works again.
- **The rear lidar goes quiet**: a warning is printed and only the lidar part of the evasion is off. The rover keeps going on the
  camera alone.
- **The follower's marker is hidden** (a robot in the way): its position and heading are carried on from the wheel commands for up to
  1.5 s (5 s with an opponent beside it). After that, with an opponent beside it, it stands still. With nothing beside it, `chase(None)` runs.
- **Not covered:** a lost Bluetooth link (whatever the Micromelon module does), and a camera that keeps sending frames of a frozen picture.

### How to run the simulator (no camera, rover or ESP32 needed)

    python run_tests.py                         # compile + unit + screen/script + follow.py + arena-scan tests (1-2 minutes)
    python run_tests.py --sim                   # plus the full-render scenarios (several more minutes)
    python simulate.py hunter                   # one scenario with a video: chase_around, start_beside_nogo, hunter, three_calm
    python simulate.py hunter --no-video        # same, without the video (faster)

`simulate.py` writes the video and key frames to `sim_output/`. The video shows the simulated external camera (real
tracker and planner overlay), a top-down truth map and a timeline. The summary it prints counts no-go and tape violations
and whether the follower was popped. Real code under test: tracker, planner, evasion and the Micromelon Motors module;
only the world, the sensors and the rover are simulated.

Many runs at once, on the fast simulator (no pictures, about 20x faster, replays measured tracker noise):

    python tests/sweep_hunter.py --label mine   # 14 hunter cases x 10 seeds, writes tests/sweep_mine.json
    python tests/compare_sweeps.py baseline mine

The simulator only tests the software, not the real hardware (camera latency, Bluetooth rate, lidar, field of view).
It is also kinder than the full renderer for the hunter case: judge changes on `simulate.py` runs over several seeds
(the `run_scenario(name, seed=N)` function in `simulate.py`), not on the fast simulator alone.

---

No robot control, no chessboard calibration.
- FOLLOWER: one ArUco marker lying flat on top -> position + yaw
- LEADER: a coloured balloon (colour only) -> position on the floor
Outputs ONE number: the yaw to the leader (degrees the follower must turn to face it,
+ = left/CCW, - = right, 0 = pointing straight at it), with an overlay.

## Install
    pip install -r requirements.txt

## 1. Print markers (DICT_4X4_50)
Print `markers_sheet.pdf` at 100% / actual size (not "fit to page") on matte paper, then
measure the black square with a ruler. Regenerate with `python make_markers.py` if you change sizes.
- ID 8: the floor marker (15 cm), lying flat on the floor and TAPED DOWN: it is the world origin and every position is
  measured from it. The camera does NOT have to stay still: it re-solves the origin and angle from this marker on
  every frame, so keep it in view and make it big (a bigger floor marker gives a steadier origin).
- ID 0: the follower's marker. Stick it flat on top of the rover (stiff card, matte) with its TOP EDGE pointing to the
  rover's FRONT. Make it as big as the top surface allows: bigger = more accurate yaw.
- Camera view: this works best when the external camera looks down from above. At an angle (like 45 degrees) it still
  works but yaw gets noisier, so use the biggest marker that fits (10 cm is much better than 5 cm), or switch
  `MARKER_LAYOUT` to `"post"` in tracker.py for a four-faced post and a low side-on camera.

## 2. Set values at the top of tracker.py
- MARKER_SIZE, FLOOR_SIZE: printed black-square side length, metres
- MARKER_HEIGHT: height of the marker's centre above the floor; TOP_MARKER_OFFSET: how far the marker's centre is
  forward/left of the rover's centre (both in metres, MEASURE). `POST_RADIUS`/`TILT_DEG` only matter for the post layout.
- ROBOT_BOX: follower length/width/height for the overlay
- LEADER_HEIGHT: height of the leader balloon's CENTRE above the floor (measure it carefully)
- FOV_DEG: camera HORIZONTAL field of view (webcams are typically 60-80)

## 3. Pick your camera and run
    python tracker.py --list         # previews each camera; press Y on the rear one
    python tracker.py --camera 1     # run with that camera (or set CAMERA_INDEX in tracker.py)
    python tracker.py --camera 1 --flip   # only if the image is mirrored (markers won't detect when mirrored)
If a camera won't open, close other apps using it (e.g. the Windows Camera app).
Rear cameras can have a different field of view than the front one: re-tune FOV_DEG with [ ].
Keys / mouse: click the leader balloon to set its colour | M show colour mask |
F re-lock floor | [ ] adjust FOV | Esc quit

The floor frame (origin + angle) is re-solved from the floor marker on EVERY frame, so the camera may move. If the
marker is hidden it holds the last frame for 0.5 s, then pauses tracking and says so on screen until it sees it again.
A one-frame glitch is ignored; a real bump of the camera is accepted after two frames. Press F to re-acquire.
Tuning: `FLOOR_TRACKING`, `FLOOR_ALPHA`, `FLOOR_HOLD_S`, `FLOOR_SIZE` at the top of tracker.py (set `FLOOR_TRACKING = False`
if you ever want to lock it once instead).

## Things that affect accuracy
- LEADER_HEIGHT: the leader's floor position comes from where the camera ray through the
  balloon meets a plane at that height. A wrong height shifts it along the view direction.
- FOV_DEG: no calibration, so focal length is assumed. Tune with [ ] until the overlay
  boxes/yaw arrow line up with the real robots.
- Leader colour must be unlike the floor, walls and the follower (including its balloon, if any).
- Keep the balloon at a fixed height on the leader (a rigid stick, not floating on a string).

## Use from your own code
    from tracker import Tracker, draw_overlay
    tr = Tracker(size=(1280, 720), fov_deg=70); tr.lock_floor(cap)
    state = tr.update(frame)
    print(tr.yaw_to_leader(state))   # degrees, or None if either robot isn't seen
    draw_overlay(frame, tr, state)

## Running everything: follow.py (your chase code + automatic evasion)
    python follow.py --onboard http://<esp32-ip>:81/stream        (follower's own camera)
    python follow.py --camera 1                                   (external camera + markers)
    --lidar-port 4210 (default)   --no-lidar (no evasion!)   --rover 1 (Bluetooth ID)

- YOU write `chase(yaw)` in `my_chase.py` (paste Python from your blocks; `yaw` is degrees to turn to face the
  opponent, + = left, None = not seen). Set wheel speeds and return (see `example_chase.py`): do not wait inside it.
- I wrote the evasion (`evade.py`). It has two sources and evades if EITHER reports a threat:
  - the rear lidar: something close and closing in from behind (covers the rear even if the external camera is blocked);
  - the external camera: any opponent that is moving, aimed at you, and predicted to reach you within ~2 s, from ANY
    direction (including the sides the lidar can't see). It also knows the exact direction, so the turn is precise.
  The rover turns to face the chaser (if it is dead behind, the side IR picks which way). Opponents roughly straight
  ahead are not evaded: facing them is chase()'s job. Evasion overrides `chase()` while it runs.
- With `--camera` (external) `yaw` points at the NEAREST of up to three opponents. In the live view press 1/2/3 to pick
  an opponent slot, then click that balloon to set its colour (use blue/green/yellow: red clashes with the red tape).
  With `--onboard` there is no external camera, so evasion uses the lidar only.
- Motor facts (checked against the micromelon source): speeds are cm/s in -30..30; `Motors.turnDegrees` and
  `Motors.turn` treat NEGATIVE as a LEFT turn, while `yaw` is POSITIVE for an opponent on the left, so a left turn
  is `Motors.turnDegrees(-yaw)`. Ctrl+C in the terminal stops the rover.

### Lidar setup (ESP32 side is untested)
1. Wire the lidar and test it alone with `lidar_test/lidar_test.ino`.
2. Add `lidar_udp.h` to your CameraWebServer sketch (3 small edits, see the top of the file).
3. Do the hand-wave test: wave a hand behind the rover on its LEFT. If the zones that react are on the wrong
   side, set `COL0_IS_ROVER_LEFT = False` in `evade.py`.
4. Set `MAX_MM` in `evade.py` to about your arena size so the room beyond the tape is ignored.
5. With the area behind the rover EMPTY: `python learn_mask.py` (masks your own balloon/stick). 
6. Tune in the Micromelon simulator before the real rover.

**Option: no WiFi, ESP-NOW through a second ESP32 (untested).** A computer cannot speak ESP-NOW itself, so a second ESP32 plugged in by USB
is the receiver:
1. In `lidar_udp.h` uncomment `#define LIDAR_USE_ESPNOW 1` (and don't connect that board to a WiFi network: ESP-NOW needs one channel).
2. Flash `espnow_bridge/espnow_bridge.ino` to the second ESP32 and plug it into the computer. Keep `ESPNOW_CHANNEL` and `LIDAR_ESPNOW_CHANNEL` equal.
3. `pip install pyserial`, then `python follow.py --camera 1 --lidar-serial COM5` (your port; `--lidar-baud` if you changed the baud).
It forwards the same lidar packet, framed (`A5 5A`, length, packet, CRC-8), so a lost byte is skipped and the stream resyncs. `GRID` (8x8 vs 4x4) works the same, and the mask is learned with
`python learn_mask.py --serial COM5`.

**The ESP32's own camera through ESP-NOW too (untested, only for `--onboard` mode).** The bridge's USB line can carry the camera as well:
include `cam_espnow.h` in the lidar ESP32's sketch (set `PIXFORMAT_JPEG`, `FRAMESIZE_QVGA`, `jpeg_quality 20`; call `camEspnowSend()` about every 100 ms; its
header lists the steps), then `python follow.py --onboard bridge --lidar-serial COM5`. Each JPEG is cut into 196-byte packets, framed like the lidar,
and reassembled on the computer; a frame that loses a packet is dropped (broadcast ESP-NOW has no retransmit). Expect roughly 5-10 frames a second at
QVGA (estimates, not measured): ESP-NOW's default 1 Mbit/s carries about 400-500 such packets a second, the serial line about 90 KB/s. In the normal
external-camera mode the ESP32's camera is not used at all, so this only matters if you want the follower's-eye view without WiFi.

### The pop rule (measure these, they set when evasion starts)
You only lose when an opponent's pin reaches your REAR balloon, so touching isn't a loss. At the top of `evade.py`:
- `BALLOON_BACK_M` (defence): how far behind your rover's centre your own balloon sits.
- `PIN_FROM_BALLOON_M` (defence): along a rover's heading, the distance from its balloon to its pin tip. The camera tracks
  opponents by their balloon, so this is how it works out where an attacker's pin is. It predicts whether that pin tip is
  on course to hit YOUR balloon, including an attacker circling in from the side.
- `PIN_REACH_M` (attack): with your pin touching the other rover's balloon, measure from YOUR rover's centre to that balloon.
- `POP_REACH_MM` (lidar): with a chaser's pin touching your balloon, what the rear lidar reads.
- `EVADE_LATENCY_S`: delay from "threat seen" to the wheels turning (camera, WiFi, Bluetooth). Guess 0.4 s; measure it.
The evasion starts early enough to finish the turn before the pin arrives: latency + the time the spin takes
(from `WHEELBASE_CM`, which is the module's effective 18.7 cm, and `EVADE_SPEED`) + `TTC_MARGIN_S`. A 180 degree turn needs about 2.0 s of warning at the defaults,
a 90 degree turn about 1.4 s. Something merely beside you only counts if it is moving at you; something right
behind you (within the pop reach plus a margin, `REAR_DANGER_DEG`) counts even if it is still. Those 'too close' distances now follow
your measured `BALLOON_BACK_M`, `PIN_FROM_BALLOON_M` / `POP_REACH_MM` automatically.
Other numbers: `CLOSING_MIN_MM_S`, `DANGER_MM`, `EXT_AIM_DEG`, `FRONT_SAFE_DEG`, `EVADE_SPEED`, `EVADE_COOLDOWN_S`.
Set `TTC_TRIGGER_S` / `EXT_TTC_S` to a number of seconds only if you want to override the automatic timing.

### What the defence does once an opponent is on you
The quick turn above is for something coming from behind. An opponent that is close and MOVING (or moved in the last 3 s) is
instead FACED continuously (`FacingTracker`): smooth wheel commands keep your front toward it, so your balloon stays on the far side.
- Inside `HARD_RING_M` an opponent always counts, however you move. `RING_MIN_SPEED` (0.05 m/s) must stay above the camera's speed noise.
- The camera loses your marker about a quarter of the time with an opponent beside you: your position and heading are then carried on
  from the wheel commands (up to 1.5 s, 5 s with an opponent beside you; `pipeline.py`), the heading is checked against the wheels
  when the marker comes back, and with no pose at all the rover stands still rather than search.
- **The line comes first** (`ESCAPE_IN_CONTACT = True`): whenever the rover gets within `ESCAPE_TAPE_M` (0.12 m) of the tape or `ESCAPE_NOGO_M`
  (0.13 m) of the no-go zone, it drives away from the line even with an opponent on it. That cut violations a lot and cost survival against
  hunters (numbers in `docs/EXPERIMENTS.md`); set `ESCAPE_IN_CONTACT = False` to fight on instead.
- Off by default and left in the code, because they lost more than they won in simulation (`docs/EXPERIMENTS.md`): `BACKOFF_SPEED`,
  `SIDESTEP_SPEED`, `HOLD_SPEED`, and `plan.AVOID_R_M`.

## Max speed
`MAX_SPEED_CM_S` in `evade.py` is a global cap on the wheel speed. It is enforced on EVERY motor command, including the
ones in your `my_chase.py` (they are clamped on the way to the rover), and it can't go above the rover's own limit of 30.
- Set it in the file, with `python follow.py --max-speed 18`, or in code with `evade.set_max_speed(18)` (takes effect
  on the next command, so a dashboard slider could change it live).
- `EVADE_SPEED` (the speed of the evasive spin) never goes above it. A lower cap means a slower spin, so evasion has to
  start earlier: at a 15 cm/s cap a rear chaser needs about 1.8 s of warning instead of 1.4 s (the code works this out).
- Distance moves (`Motors.moveDistance`, `turnDegrees`) keep their distances but run slower, so they take longer.

## Waypoint planning (round the no-go zone, to a point behind the opponent)
Needs the external camera (`--camera`). Set it up once:

    python scan_arena.py --camera 1       SCAN it automatically: finds the tape boundary and the no-go zone, press ENTER to save
    python mark_arena.py --camera 1       or click the 4 inner tape corners, then the 4 corners of the no-go square, Enter
                                          (the no-go square may be turned, e.g. 45 degrees: scan and click both handle any rotation)

**Scanning (`scan_arena.py`).** Keep the floor marker in view and clear the arena of robots (robots the tracker can see are
ignored, but a clear floor is safest). It re-draws the floor from straight above using the floor marker, so the camera can sit
at any angle, finds the RED and BLACK tape, takes the inner edge of the tape ring as the arena, and takes the biggest
square-ish thing inside the arena that is not floor-coloured (a taped square or a coloured mat) as the no-go zone. The live
view shows GREEN = arena, ORANGE = no-go zone, averages 15 scans and says when it has settled; ENTER saves, R restarts,
M shows what it sees, Esc quits. If it can't find the arena it says why (a gap in the tape, part of the arena out of view,
no tape visible) and you can fall back to `mark_arena.py`. Notes: it works best on a floor that is lighter than the black
tape (on a very dark floor set `USE_BLACK_TAPE = False`); and corners are only as exact as the camera's estimate of the
floor marker's pose (1-4 cm at the far corners in simulation, a bigger floor marker helps), which is fine for the planner's margins.

That writes `arena.json`. After that `follow.py --camera 1` plans automatically: `yaw` (what your `chase(yaw)` gets)
now points at the NEXT WAYPOINT on the shortest route that stays inside the tape and out of the no-go zone, aimed at a
point behind the target (where your pin meets its balloon). Close to that point it aims straight at the opponent.
`yaw_to_target` (a variable in follow.py) always points straight at the target, for reference.
- `--target A` chases a particular colour slot (default `nearest`); `--no-plan` switches planning off.
- The live view draws the arena, the no-go zone (and its grown outline), the path, the next waypoint and the goal.
- Tuning numbers are at the top of `plan.py`: `ROBOT_RADIUS_M` (MEASURE: half the rover's longest side), `NOGO_MARGIN_M` and
  `TAPE_MARGIN_M` (how far the route keeps from the no-go zone and the tape on top of the radius). `NOGO_MARGIN_M` = 0.13 applies while any
  opponent is moving; while nothing moves the smaller `NOGO_MARGIN_CALM_M` = 0.09 is used (the keep-out is then 0.54 m across instead of 0.62 m).
  Do not just shrink `NOGO_MARGIN_M`: 0.12 instead of 0.13 took violation-free runs from 74% to 54% in simulation, `BEHIND_M`, `ENGAGE_M`, `LEAD_S`. The no-go zone is grown with ROUND corners (what a round
  robot sweeps out), so a turned square still leaves room to pass.
- **Which opponent is the target** (`plan.TARGET_MODE`, default `"sticky"`): the nearest, then it STAYS on it (no flipping between two
  opponents at similar distances); it switches to an opponent the rover had to defend against (`pipeline.RETARGET_ON_THREAT`); and it
  gives up on a target it has not got closer to for 14 s (for 20 s, if another is in play). Other modes: `"switch"` (always the nearest),
  `"adaptive"`, `"active_first"`, `"calm_first"`. In simulation none of them beat the others (`docs/EXPERIMENTS.md`).
- Your `chase(yaw)` should set the wheel speeds and return (like `example_chase.py`), not do "a short move, then wait": while it waits
  the rover cannot see a threat, and the camera's picture is a little old so a wait-and-look chase wobbles. Wheel speeds beyond the cap
  are clamped, not an error.
- If the opponent is hardly moving its heading is unknown, so it aims at the opponent itself (the front, possibly).

## Connecting the rover's ESP32 to a WiFi hotspot (the WiFi route; untested on hardware)
The ESP32 joins the hotspot as an ordinary WiFi client. The repo's files don't do the joining: the stock `CameraWebServer` example does, and
`lidar_udp.h` plugs into it.
1. Make a **2.4 GHz** hotspot (the computer's mobile hotspot or a phone's). The ESP32 cannot use 5 GHz.
2. Arduino IDE: *File > Examples > ESP32 > Camera > CameraWebServer*. At the top set `const char* ssid = "YourHotspot";` and
   `const char* password = "YourPassword";`, and pick your camera model.
3. Add `lidar_udp.h` to that sketch folder and make the three small edits described at the top of the file (`lidarBegin()` in `setup()`,
   the lidar send in `loop()`).
4. Flash it and open the Serial Monitor at 115200 baud. On boot it prints the board's address: `Camera Ready! Use 'http://192.168.x.x'`.
5. Put the computer on the same hotspot (if the hotspot is on the computer, it already is).
6. Run `python follow.py --camera 1 --onboard http://192.168.x.x:81/stream` (camera stream = port 81).

The lidar needs nothing more: it broadcasts UDP on port 4210 and `follow.py` listens there by default. If the hotspot blocks broadcast (some
phone hotspots do), set `LIDAR_DEST` in `lidar_udp.h` to the computer's IP. The board's address can change on each reconnect: reserve a fixed
address for it in the hotspot's settings if you can. Tandem details and WiFi vs ESP-NOW: `docs/WIFI_VS_ESPNOW.md`.

## Tandem: external camera + the follower's own camera (untested on hardware)
```
python follow.py --camera 1 --onboard http://<esp32>:81/stream
python follow.py --camera 1 --onboard bridge --lidar-serial COM5      # camera + lidar over ESP-NOW (2 ESP32s: rover + bridge)
```
The external camera stays primary (positions, planning). The follower's own camera measures bearing and range of each balloon (same
A/B/C colours as `tracker.OPPONENT_HSV`) and, when it sees the threatening opponent, its bearing replaces the external one (fresher, exact, not
affected by a hidden marker); if the external camera loses us it carries the facing on for up to 2 s while the opponent is close and in front.
If the onboard camera stops, the watchdog warns and the defence is the external-only one. In the fast simulator survival roughly doubled
(see `docs/WIFI_VS_ESPNOW.md`, which also compares WiFi with ESP-NOW for the lidar and the camera).

## Follower with its own camera (ESP32-CAM): onboard.py
If the camera is on the follower, use `onboard.py` instead of `tracker.py`: no markers, floor marker
or external camera. The balloon's left/right position in the image IS the yaw to the leader.

    python onboard.py --source http://<esp32-ip>:81/stream
    python follow.py --onboard http://<esp32-ip>:81/stream     (with the rover program)

- Click the balloon to set its colour; M shows the mask.
- FOV_DEG must be the camera's HORIZONTAL field of view. Check it: put the balloon at a measured angle
  (protractor, string from the lens) and press [ or ] until the readout matches. The error grows with angle
  (tested with the FOV 10 deg out: about 1 deg error at 5 deg, 3 deg at 15 deg, 4 deg at 25 deg), but it is
  always 0 at 0, so a follower that keeps turning until the reading is ~0 is not sensitive to FOV.
- Mount the camera pointing straight ahead (or set MOUNT_YAW_DEG). If the picture is upside down or
  mirrored, set ROTATE_180 / FLIP_HORIZONTAL (a mirrored image swaps left and right).
- Positive = leader on the left; use Motors.turnDegrees(-yaw_to_leader) as before.

## Testing in simulation
    python run_tests.py          # compile + unit tests + screen/script tests (1-2 minutes)
    python run_tests.py --sim    # also the full-system simulator scenarios (several more minutes)
    python simulate.py hunter    # one scenario, writes a video and plots to sim_output/ (add --no-video to skip the video)
Everything above runs with no camera, rover or ESP32. It does not test real hardware.
