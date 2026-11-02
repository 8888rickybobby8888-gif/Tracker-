# What was tried, and what happened

All numbers are from the simulator (never the real rover). "Fast sim" = `tests/fastsim.py`, 20 seeds, the 15 `hunter` cases
(300 runs). "Full render" = `simulate.py`, one seed per run, the `hunter` scenario.

## Kept
| Change | Evidence |
|---|---|
| Wider planner margins (tape 0.12, no-go 0.13) and a tighter example chase | zero-violation runs 36/180 -> 133/180 (fast sim, early) |
| Planner zone with ROUND corners (needed for the 45 degree no-go diamond) | with mitred corners the diamond's zone closed the corridors beside it and there was no route round it |
| Lidar ignores a popped/parked opponent right beside us | stopped an endless spin-away loop next to a popped robot |
| Heading filter keeps running through a hidden-marker gap | full render, hunter seeds 0-7: 4 wins/3 pops -> 7 wins/1 pop (the camera's heading was 30-57 degrees wrong when a robot half-covered the marker) |
| Contact-anchored dead reckoning | position error after 3-5.5 s hidden: 17 cm -> 5 cm. Fast sim survival 224 -> 245 of 300 (wins 198 -> 185) |
| Remember an opponent that just vanished beside us; stand still instead of searching | removes the blind search that turned our balloon to its pin |
| Fail-safes: stop on no camera frames, stop the wheels on repeated errors | unit tests |

## Tried and left OFF (code is still there)
| Change | Evidence |
|---|---|
| Hold ground (push back toward an opponent that is shoving us at the edge), `evade.HOLD_SPEED` | full render seeds 13/7/4/10: 6 cm/s 1 win of 4, 10 cm/s 0 of 4. Fast sim, only near the edge: survival 245 -> 197/196/176 of 300, zero-violation runs unchanged |
| Back away from a slower opponent in our face, `evade.BACKOFF_SPEED` | full render (older code): 3 pops in 8 seeds vs 1. Fast sim (newer code): 245 survived, 193 won (baseline 185), within noise |
| Floor colour sensors (`--floor-sensors`) | fast sim: no change (246 vs 245 survived). The shoves push the rover BACKWARDS, and the sensors face forward |

## Still failing
- `hunter` seed 13 (full render): the hunter shoves the passive follower about 0.5 m past the tape, the camera loses it outside the
  arena, then it is popped. Seeds 2 and 6 behave the same way in some runs.
- Survival against a hunter is about 80% in the fast sim and 70-88% over 16 full-render seeds, depending on the code version.
- Nothing has run on the real camera, rover, ESP32 or lidar.

## Which target to go for (tests/targeting_study.py; fast sim, 5 scenarios with 2-3 opponents)
**CORRECTION.** The first version of this study (and the claim that "adaptive" was best) ran on code with a BUG: the planner only stayed on its target
while it was NOT getting closer to it; whenever it was making progress it fell through to "pick the nearest" again, and that also overwrote the
"retarget the opponent we defended against" rule. The big differences between methods in that study (e.g. 25 vs 49 survived of 60 against one
hunter) were artifacts of it. Found by a wiring test, fixed in `plan.py`, and the study re-run (30 seeds x 5 scenarios = 150 runs per method):

| Method | Survived | Avg % popped | Cleared all | Avg clear time | No violation |
|---|---|---|---|---|---|
| switch: always the nearest (the original rule) | 104 | 60% | 84 | 30.9 s | 116 |
| **sticky: nearest, then stay on it (+ give up if stalled)** | 108 | 61% | 87 | 30.7 s | 120 |
| sticky + retarget the opponent we defended against | 105 | 60% | 85 | 31.5 s | 114 |
| hunt the moving opponent first, + retarget | 104 | 60% | 83 | 32.9 s | 109 |
| still opponent first | 110 | 62% | 86 | 30.1 s | 118 |
| still opponent first + retarget | 105 | 60% | 84 | 31.1 s | 115 |
| adaptive + retarget | 105 | 60% | 85 | 31.5 s | 114 |

**Result: with fixation working, no method is better than another.** Everything is within 104-110 survived and 83-87 cleared of 150, which is
inside the noise (about +-4). In particular staying on a target, switching to the attacker, and hunting the hunter first or last make no
measurable difference in these scenarios. The default is therefore the simplest rule you specified: `TARGET_MODE = "sticky"` plus
`RETARGET_ON_THREAT = True` (stay on the nearest; when we had to defend against an opponent, make it the target). Hunting the hunter first is
still slightly slower (32.9 s) and has the fewest violation-free runs (109), so avoid it.

## Seed 13 (full render, `hunter`): what it is and what was tried
What happens: at t=0 the hunter B is the NEAREST opponent and still, so the follower targets it (and fixates). B starts moving, the two meet
head-on at about 3 s, and the fight is close to a coin flip: sometimes the follower pops B as it swings past, sometimes B shoves the passive
follower 0.5 m past the tape, the camera loses it outside the arena, and it is popped. Seed 13 is one of the bad outcomes with the default
settings.

| Tried | Result |
|---|---|
| Back away from B (`BACKOFF_SPEED=28`) | Fixes seed 13 (win, 0 violations) but over 16 full-render seeds: **4 wins / 10 pops** vs the default's **11 wins / 3 pops**. Off. |
| Sidestep (`SIDESTEP_SPEED=26`): pivot 90 degrees to the open side and drive off | Fast sim, 300 runs: survived 221 vs 251, but violation-free runs 238 vs 207. The pops happen DURING the pivot (the balloon swings next to B's pin). Off. |
| Hold ground (`HOLD_SPEED`) | Survival 245 -> 176-197 of 300. Off. |
| Route round moving non-target opponents (`plan.AVOID_R_M`) | No effect: here the opponent to avoid IS the target. Off. |
| Take a still opponent first, leave the hunter (`calm_first`) | One hunter + 2 still: 25 vs 49 survived of 60. The follower is safer fighting the hunter than ignoring it. |

So against ONE hunter the follower wins roughly 70-85% of the fights (it depends on the code version and seed set), and every change I made
to the fight itself traded survival for something else. What would be left is a different kind of idea (not committing to the nearest
opponent for the first second, or using the opponent's heading when it is known), not tuning.

### Seed 13 re-run with the current code (line-first + tandem), full render, `hunter`, 60 s
| Setup | Result |
|---|---|
| default (external camera only) | no pops either way, 0 tape / 0 no-go frames, **time-out** (no longer shoved out and popped: line-first escape fixed that part) |
| `calm_first`, `kite`, `active_first` | same: time-out, 0 violations |
| tandem, good link (15 Hz, 0.12 s) | **win**: popped B at 15.3 s and A at 21.5 s, 0 violations |
| tandem, ESP-NOW-like link (8 Hz, 33% frames lost) | time-out, 0 violations |
One seed only: it shows what changed for this seed, not a rate.

## Making the keep-out zone round the no-go diamond smaller
Asked for: "a little smaller". The keep-out is the robot radius (0.08) plus `NOGO_MARGIN_M`.
| `NOGO_MARGIN_M` (fixed) | Survived | Won | Violation-free runs |
|---|---|---|---|
| 0.13 (before) | 325/380 | 264 | 269 |
| 0.10 | 295 | 192 | 110 |
| 0.08 | 295 | 197 | 102 |
Finer steps, 12 seeds: 0.13 -> 168/228 violation-free; 0.12 -> 124; 0.115 -> 92; 0.11 -> 98. So it is a cliff, not a slope: with a smaller zone the
route hugs the diamond and the hunter's shoves (and the steering overshoot) put the rover over the no-go limit (`hunter` scenario: 0 -> 55-175
no-go frames). A fixed smaller zone is therefore NOT acceptable.

What was kept: a TWO-SIZE zone. While any opponent is moving (or was in the last 3 s) the full 0.13 margin is used; while nothing moves, `NOGO_MARGIN_CALM_M`
= 0.09 (0.54 m across instead of 0.62 m, 12% smaller). Same sweep, 12 seeds: 163/228 violation-free vs 168/228 for the fixed 0.13 (inside the noise),
calm scenarios unchanged. The shoves only happen when something is moving, so the smaller zone costs nothing there.

## Line first: never cross the tape or enter the no-go zone, even when it costs a fight
Asked for: "don't go over the line, that's what matters". The boundary escape used to be OFF while an opponent was on us. Now
`ESCAPE_IN_CONTACT = True`: the escape (drive away from the line) also runs in contact, and starts earlier (`ESCAPE_TAPE_M` 0.12, `ESCAPE_NOGO_M`
0.13 instead of 0.08 / 0.09) and faster (`ESCAPE_SPEED` 28). Fast sim, 12 seeds x 19 cases (228 runs):

| Setting | Violation-free runs | Survived | Won | `hunter` scenario, tape frames (avg) |
|---|---|---|---|---|
| before (escape off in contact) | 163 | 191 | 154 | 280 |
| escape in contact, old margins | 163 | 151 | 127 | 54 |
| + margins 0.10 / 0.11 | 179 | 142 | 99 | 16 |
| **+ margins 0.12 / 0.13, speed 28 (now the default)** | **200** | **126** | **93** | **2** |
| + margins 0.15 / 0.16 | 216 | 99 | 59 | 2 |

It is a straight trade: the earlier and harder the rover leaves a fight to get clear of the line, the fewer violations and the more often it is popped
(it turns away from the opponent, or pushes into it, and its balloon is exposed). The non-hunter scenarios only gain (`chase_around` 26 -> 0 tape frames,
`three_calm` 82 -> 22, same wins). The cost is entirely in fights against hunters (`hunter` scenario: 9 of 12 survived -> 4 of 12). To change the balance,
edit `ESCAPE_TAPE_M` / `ESCAPE_NOGO_M` / `ESCAPE_IN_CONTACT` at the top of `evade.py`.
