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

## Which target to go for (tests/targeting_study.py; fast sim, 5 scenarios with 2-3 opponents, 60 seeds each = 300 runs per method)
| Method | Survived | Cleared all opponents | Avg clear time | Runs with no violation |
|---|---|---|---|---|
| Fixate on the nearest, retarget the attacker (old default "sticky") | 220 | 173 | 31.6 s | 214 |
| Take a STILL opponent first, always ("calm_first") | 199 | 166 | 30.8 s | 226 |
| **Adaptive (now the default): as sticky, but take a still one first while 2+ are moving** | **220** | **180** | 31.2 s | 220 |

By scenario (survived / cleared of 60):
| Scenario | sticky | calm_first | adaptive |
|---|---|---|---|
| 1 hunter + 2 still | 49 / 24 | 25 / 11 | 42 / 19 |
| hunter + wanderer + still | 35 / 21 | 31 / 21 | 30 / 23 |
| 2 hunters + 1 still | 28 / 21 | 38 / 31 | 40 / 31 |
| 3 still, 3 patrolling | tied | tied | tied |

- Going for the hunter FIRST is the worst method (20-run study: slowest, 67 of 100 runs violation-free vs 76-79).
- Retargeting the opponent we had to defend against helps when we avoid the hunter first, and is neutral otherwise.
- Adaptive matches the best method for two hunters and loses a little (42 vs 49 survived of 60) against a single hunter. That gap is about
  1.2 standard errors, so it may be noise. "Active" means seen moving at 0.05 m/s or more in the last 3 s, so it depends on the camera's
  speed noise staying below that (unmeasured on the real camera).
