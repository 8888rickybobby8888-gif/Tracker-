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
