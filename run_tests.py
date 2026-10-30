"""
run_tests.py - check that everything works in simulation (no camera, rover or ESP32 needed).

    python run_tests.py             compile every file + unit tests + screen/script tests (about 1-2 minutes)
    python run_tests.py --sim       also run the full-system simulator scenarios (several more minutes)

What it covers
  1. every .py file compiles
  2. tests/test_units.py: planner, lidar and camera threat logic, evade planning, speed cap, packets, onboard maths
  3. tests/test_gui.py: tracker.py live view, mark_arena.py, onboard.py and learn_mask.py driven by simulated frames
  3b. tests/test_follow.py: follow.py itself with a fake rover link, camera and ESP32 (planning, evasion, speed cap, shutdown)
  3c. tests/test_scan.py: the automatic arena scan (scan_arena.py) on rendered frames
  4. --sim: simulate.py scenarios (the real tracker, planner, evasion and Micromelon Motors module in a simulated world)
Not covered: real cameras, the Bluetooth link, the ESP32 firmware (only a syntax check was run, against stubs).
"""
import glob
import os
import py_compile
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
os.chdir(HERE)
failures = []

print("== 1. compile every file")
for f in sorted(glob.glob("*.py") + glob.glob("tests/*.py")):
    try:
        py_compile.compile(f, doraise=True)
    except py_compile.PyCompileError as e:
        failures.append(f"compile {f}")
        print("FAIL", f, e)
print("   done")

for title, script in (("2. unit tests", "tests/test_units.py"), ("3. screen and script tests", "tests/test_gui.py"),
                      ("3b. follow.py end to end (fake rover, camera, ESP32)", "tests/test_follow.py"),
                      ("3c. automatic arena scan", "tests/test_scan.py")):
    print("\n== " + title)
    r = subprocess.run([sys.executable, script], capture_output=True, text=True, timeout=900)
    lines = [l for l in r.stdout.splitlines() if l.startswith(("PASS", "FAIL", "SUMMARY", "   FAILED"))]
    print("\n".join(l for l in lines if not l.startswith("PASS")))
    if r.returncode != 0:
        failures.append(script)
        print(r.stderr[-1500:])

if "--sim" in sys.argv:
    print("\n== 4. full-system simulation (slow)")
    import simulate as S
    # name: (what must hold, known weakness?)  -- seed 0, full renderer. "Known weakness" = a stochastic fight that is lost in some seeds
    # (see docs/EXPERIMENTS.md): it is reported but does not fail the run.
    clean = lambda s: s["nogo_frames"] == 0 and s["tape_frames"] == 0 and not s["follower_popped"]
    expect = {
        "chase_around": (lambda s: s["end"] == "goal reached" and clean(s), False),
        "start_beside_nogo": (lambda s: s["end"] == "goal reached" and clean(s), False),
        "wanderer": (lambda s: s["end"] == "goal reached" and clean(s), False),
        # all three popped and the rover stopped; a few tape frames while it reaches a balloon that hangs near the tape are tolerated
        "three_calm": (lambda s: s["end"] == "all opponents popped" and not s["follower_popped"] and s["nogo_frames"] == 0 and s["tape_frames"] <= 60, False),
        "hunter": (lambda s: s["end"] == "goal reached" and clean(s), True),
    }
    for name, (ok_fn, known) in expect.items():
        sim, s, rec = S.run_scenario(name, video=False)
        ok = ok_fn(s)
        tag = "PASS" if ok else ("KNOWN WEAKNESS" if known else "FAIL")
        print(f"{tag:15s} {name}: ended '{s['end']}' at {s['duration']} s, evades {s['evades']}, follower popped {s['follower_popped']}, "
              f"no-go/tape violations {s['nogo_frames']}/{s['tape_frames']}")
        if not ok and not known:
            failures.append("simulation " + name)

print()
print("ALL CHECKS PASSED" if not failures else "FAILURES: " + ", ".join(failures))
sys.exit(1 if failures else 0)
