"""
tests/targeting_study.py - which way of choosing the target works best? Fast simulator, several scenarios with 2-3 opponents,
every run lasts until all opponents are popped, the follower is popped, or 60 s.

    python tests/targeting_study.py [--seeds 20] [--only "sticky + retarget threat,calm_first + retarget"]

Methods compared (plan.TARGET_MODE, pipeline.RETARGET_ON_THREAT):
  nearest, re-checked every frame ("switch")        the original rule
  nearest then fixate ("sticky")                    + give up on a target we are not getting closer to
  sticky + retarget the opponent we had to defend against
  hunt the hunter first ("active_first")            the nearest MOVING opponent, then the rest
  leave the hunter alone ("calm_first")             the nearest STILL opponent first, the hunter last
"""
import os, sys, time
from multiprocessing import Pool
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

CONFIGS = {
    "switch (original)":              ("switch", False),
    "sticky":                         ("sticky", False),
    "sticky + retarget threat":       ("sticky", True),
    "active_first + retarget":        ("active_first", True),
    "calm_first":                     ("calm_first", False),
    "calm_first + retarget":          ("calm_first", True),
}

def scenarios():
    import simulate as S
    f = (0.45, 0.30, 0.0)
    A = ("A", 1.25, -0.30, 90.0, "static", 0.0, None)
    C_static = ("C", 1.20, 0.45, 180.0, "static", 0.0, None)
    B_static = ("B", 0.45, -0.40, 0.0, "static", 0.0, None)
    return {
        "3 still": dict(name="s1", duration=60.0, follower=f, opponents=[A, B_static, C_static], until=None),
        "1 hunter + 2 still": dict(name="s2", duration=60.0, follower=f, opponents=[A, ("B", 0.30, -0.45, 0.0, "hunter", 0.11, None), C_static], until=None),
        "hunter + wanderer + still": dict(name="s3", duration=60.0, follower=f, opponents=[A, ("B", 0.30, -0.45, 0.0, "hunter", 0.11, None), ("C", 1.20, 0.40, 180.0, "wander", 0.11, None)], until=None),
        "2 hunters + 1 still": dict(name="s4", duration=60.0, follower=f, opponents=[A, ("B", 0.30, -0.45, 0.0, "hunter", 0.11, None), ("C", 1.20, 0.40, 180.0, "hunter", 0.11, None)], until=None),
        "3 patrolling": dict(name="s5", duration=60.0, follower=f, opponents=[("A", 1.25, -0.35, 90.0, "patrol", 0.08, [(1.25, 0.35), (1.25, -0.35)]),
                                                                              ("B", 0.45, -0.40, 0.0, "patrol", 0.08, [(0.45, -0.40), (0.45, 0.40)]),
                                                                              ("C", 1.15, 0.45, 180.0, "patrol", 0.08, [(1.15, 0.45), (0.95, -0.45)])], until=None),
    }

def run(job):
    cfg, scn, seed = job
    import plan, pipeline, fastsim as F
    plan.TARGET_MODE, pipeline.RETARGET_ON_THREAT = CONFIGS[cfg]
    sim, s = F.run(scenarios()[scn], seed=seed)
    mine = sum(1 for _, t in s["events"] if t.startswith("follower popped"))
    cleared = s["end"] == "all opponents popped"
    return dict(cfg=cfg, scn=scn, seed=seed, survived=not s["follower_popped"], popped=mine, n=len(scenarios()[scn]["opponents"]),
                cleared=cleared, t=s["duration"], viol=s["tape_frames"] + s["nogo_frames"])

def main():
    n = int(sys.argv[sys.argv.index("--seeds") + 1]) if "--seeds" in sys.argv else 20
    only = sys.argv[sys.argv.index("--only") + 1].split(",") if "--only" in sys.argv else list(CONFIGS)
    for c in list(CONFIGS):
        if c not in only:
            del CONFIGS[c]
    jobs = [(c, sc, sd) for c in CONFIGS for sc in scenarios() for sd in range(n)]
    t0 = time.time()
    with Pool(4) as p:
        rows = p.map(run, jobs, chunksize=2)
    print(f"{len(rows)} runs in {time.time() - t0:.0f} s\n")
    print(f"{'method':28s} {'scenario':26s} {'survived':>9s} {'avg popped':>11s} {'cleared all':>12s} {'avg clear s':>12s} {'no viol':>8s}")
    totals = {}
    for c in CONFIGS:
        for sc in scenarios():
            rs = [r for r in rows if r["cfg"] == c and r["scn"] == sc]
            ct = [r["t"] for r in rs if r["cleared"]]
            print(f"{c:28s} {sc:26s} {sum(r['survived'] for r in rs):>6d}/{len(rs):<2d} {np.mean([r['popped'] / r['n'] for r in rs]) * 100:>10.0f}% "
                  f"{sum(r['cleared'] for r in rs):>9d}/{len(rs):<2d} {np.mean(ct) if ct else float('nan'):>12.1f} {sum(r['viol'] == 0 for r in rs):>5d}/{len(rs)}")
        rs = [r for r in rows if r["cfg"] == c]
        totals[c] = (sum(r["survived"] for r in rs), np.mean([r["popped"] / r["n"] for r in rs]) * 100, sum(r["cleared"] for r in rs),
                     len(rs), np.mean([r["t"] for r in rs if r["cleared"]]) if any(r["cleared"] for r in rs) else float("nan"),
                     sum(r["viol"] == 0 for r in rs))
        print()
    print("OVERALL")
    print(f"{'method':28s} {'survived':>10s} {'avg % popped':>13s} {'cleared all':>12s} {'avg clear s':>12s} {'no viol':>9s}")
    for c, (sv, pp, cl, nn, tt, nv) in totals.items():
        print(f"{c:28s} {sv:>6d}/{nn:<3d} {pp:>12.0f}% {cl:>8d}/{nn:<3d} {tt:>12.1f} {nv:>6d}/{nn}")

if __name__ == "__main__":
    main()
