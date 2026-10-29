"""
tests/sweep_hunter.py - many fast simulations of the follower against hunters (and other opponents), with a table of results.

    python tests/sweep_hunter.py [--quick] [--label NAME]       writes sweep_<label>.json next to this file's parent
"""
import json, os, sys, time
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import fastsim as F
import simulate as S

def hunter(speed=0.11, start=(0.30, -0.45, 0.0), with_target=True, n_hunters=1):
    opps = []
    if with_target:
        opps.append(("A", 1.25, -0.30, 90.0, "static", 0.0, None))
    opps.append(("B", start[0], start[1], start[2], "hunter", speed, None))
    if n_hunters > 1:
        opps.append(("C", 1.20, 0.40, 180.0, "hunter", speed, None))
    return dict(name="hunter", duration=60.0, follower=(0.45, 0.30, 0.0), opponents=opps,
                until=S.popped("A") if with_target else None)

CASES = {
    # name: (scenario factory kwargs, sim kwargs)
    "hunter 0.11, target":        (dict(speed=0.11), {}),
    "hunter 0.08, target":        (dict(speed=0.08), {}),
    "hunter 0.15, target":        (dict(speed=0.15), {}),
    "hunter 0.20, target":        (dict(speed=0.20), {}),
    "hunter 0.30 (as fast as us)": (dict(speed=0.30), {}),
    "hunter 0.11, NO target (pure survival)": (dict(speed=0.11, with_target=False), {}),
    "hunter 0.20, NO target (pure survival)": (dict(speed=0.20, with_target=False), {}),
    "hunter starts behind us":    (dict(speed=0.11, start=(0.45, -0.05, 90.0)), {}),
    "two hunters 0.11 + target":  (dict(speed=0.11, n_hunters=2), {}),
    "hunter, camera noise x2":     (dict(speed=0.11), dict(noise_scale=2.0)),
    "hunter, 15% camera dropouts": (dict(speed=0.11), dict(dropout=0.15)),
    "hunter, slow camera (0.30 s)": (dict(speed=0.11), dict(cam_latency=0.30)),
    "hunter, no lidar":           (dict(speed=0.11), dict(lidar_on=False)),
    "hunter, speed cap 15":       (dict(speed=0.11), dict(max_speed=15.0)),
}

def main():
    quick = "--quick" in sys.argv
    label = sys.argv[sys.argv.index("--label") + 1] if "--label" in sys.argv else "run"
    seeds = range(4 if quick else 10)
    only = [a for a in sys.argv[1:] if a.startswith("--only=")]
    out = {}
    print(f"{'case':44s} {'survived':>9s} {'won (A popped)':>15s} {'median pop time':>16s} {'tape fr':>8s} {'nogo fr':>8s} {'evades':>7s}")
    for name, (skw, simkw) in CASES.items():
        if only and only[0][7:] not in name: continue
        rows = []
        t0 = time.time()
        for sd in seeds:
            sim, s = F.run(hunter(**skw), seed=sd, **simkw)
            popped_t = s["duration"] if s["follower_popped"] else None
            rows.append(dict(seed=sd, survived=not s["follower_popped"], won=s["end"] == "goal reached", t=s["duration"],
                             tape=s["tape_frames"], nogo=s["nogo_frames"], evades=s["evades"]))
        n = len(rows)
        surv = sum(r["survived"] for r in rows); won = sum(r["won"] for r in rows)
        pops = sorted(r["t"] for r in rows if not r["survived"])
        med = f"{pops[len(pops)//2]:.0f} s" if pops else "-"
        print(f"{name:44s} {surv:>4d}/{n:<4d} {won:>9d}/{n:<5d} {med:>16s} {int(np.mean([r['tape'] for r in rows])):>8d} {int(np.mean([r['nogo'] for r in rows])):>8d} {int(np.mean([r['evades'] for r in rows])):>7d}   ({time.time()-t0:.0f}s)", flush=True)
        out[name] = rows
    json.dump(out, open(os.path.join(os.path.dirname(os.path.abspath(__file__)), f"sweep_{label}.json"), "w"), indent=1)

if __name__ == "__main__":
    main()
