"""
tests/sweep_scenarios.py - the non-hunter simulator scenarios (chase a patrolling target, start beside the no-go zone,
three calm opponents) run many times with different seeds on the fast simulator.

    python tests/sweep_scenarios.py [--seeds N] [--label NAME]
"""
import json, os, sys, time
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import fastsim as F
import simulate as S

def main():
    n = int(sys.argv[sys.argv.index("--seeds") + 1]) if "--seeds" in sys.argv else 8
    label = sys.argv[sys.argv.index("--label") + 1] if "--label" in sys.argv else "run"
    out = {}
    print(f"{'scenario':20s} {'goal reached':>13s} {'popped':>7s} {'mean time':>10s} {'evades':>7s} {'tape fr':>8s} {'nogo fr':>8s}")
    for name in ("chase_around", "start_beside_nogo", "three_calm"):
        rows = []
        for sd in range(n):
            sim, s = F.run(S.SCENARIOS[name], seed=sd)
            rows.append(dict(seed=sd, goal=s["end"] == "goal reached", popped=s["follower_popped"], t=s["duration"], evades=s["evades"],
                             tape=s["tape_frames"], nogo=s["nogo_frames"]))
        out[name] = rows
        print(f"{name:20s} {sum(r['goal'] for r in rows):>9d}/{n:<3d} {sum(r['popped'] for r in rows):>7d} {np.mean([r['t'] for r in rows]):>9.1f}s "
              f"{np.mean([r['evades'] for r in rows]):>7.1f} {int(np.mean([r['tape'] for r in rows])):>8d} {int(np.mean([r['nogo'] for r in rows])):>8d}", flush=True)
    json.dump(out, open(os.path.join(os.path.dirname(os.path.abspath(__file__)), f"scen_{label}.json"), "w"), indent=1)

if __name__ == "__main__":
    main()
