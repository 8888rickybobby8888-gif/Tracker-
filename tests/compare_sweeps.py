"""
tests/compare_sweeps.py - side-by-side table of two saved sweeps (sweep_<label>.json from sweep_hunter.py).

    python tests/compare_sweeps.py baseline after
"""
import json, os, sys
import numpy as np
here = os.path.dirname(os.path.abspath(__file__))
a, b = (sys.argv[1], sys.argv[2]) if len(sys.argv) >= 3 else ("baseline", "after")
A = json.load(open(os.path.join(here, f"sweep_{a}.json"))); B = json.load(open(os.path.join(here, f"sweep_{b}.json")))
def stats(rows):
    n = len(rows)
    return (sum(r["survived"] for r in rows), sum(r["won"] for r in rows), n, int(np.mean([r["tape"] for r in rows])), int(np.mean([r["nogo"] for r in rows])))
print(f"{'case':44s} | {a:^27s} | {b:^27s}")
print(f"{'':44s} | {'survived  won  tape  nogo':^27s} | {'survived  won  tape  nogo':^27s}")
tot = np.zeros(4)
for k in A:
    if k not in B: continue
    sa, sb = stats(A[k]), stats(B[k])
    print(f"{k:44s} | {sa[0]:>5d}/{sa[2]:<3d} {sa[1]:>3d}  {sa[3]:>5d} {sa[4]:>5d} | {sb[0]:>5d}/{sb[2]:<3d} {sb[1]:>3d}  {sb[3]:>5d} {sb[4]:>5d}")
    tot += [sa[0] / sa[2], sb[0] / sb[2], sa[1] / sa[2], sb[1] / sb[2]]
n = len([k for k in A if k in B])
print(f"{'AVERAGE over all cases':44s} | survival {tot[0]/n*100:4.0f}%  won {tot[2]/n*100:4.0f}%   | survival {tot[1]/n*100:4.0f}%  won {tot[3]/n*100:4.0f}%")
