"""Project the sim car onto the real triangulation frames through the cached cameras and measure the pixel residual.
Cameras: out/cameras_tri.json (python blender/cache_cameras.py). Car pixels: CAR_B / CAR_R1 in triangulate.py.
B frame time -> sim time: t_pc - 61.35. R1 frame time -> sim time: R1_ANCHOR events (69.94, 71.94, 74.69); after 74.69
the last anchor slope (0.327) is used, which is NOT measured (flag "extrapolated").
Writes out/reproj.js (window.REPROJ) for replay.html and prints one line per frame.  Run: python blender/reproj.py"""
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import triangulate as T  # noqa: E402

ROOT = T.ROOT
CAMS = json.loads((ROOT / "out" / "cameras_tri.json").read_text())
A = T.R1_ANCHOR
SLOPE = (A[-1, 1] - A[-2, 1]) / (A[-1, 0] - A[-2, 0])


def r1_time(te):
    return float(np.interp(te, *A.T)) if te <= A[-1, 0] else float(A[-1, 1] + SLOPE * (te - A[-1, 0]))


def frames(path=T.car_sim, car_b=T.CAR_B, car_r1=T.CAR_R1):
    """One row per frame: view, frame time, sim time, image, projected (u, v), measured (u, v) or None, residual px."""
    rows = []
    for k, v in CAMS["B"].items():
        t = float(k)
        ts = t - T.PC_OFFSET
        uv = T.project(path(ts), T.CB, v[:3], v[3])[0]
        m = car_b.get(t)
        rows.append(dict(view="B", t=t, t_sim=round(ts, 3), img=f"frames/tri/b/pc_{t:.1f}.png", edge_rms=round(v[4], 1),
                         sim=[round(float(uv[0]), 1), round(float(uv[1]), 1)], meas=list(m) if m else None,
                         res=round(float(np.hypot(*(uv - m))), 1) if m else None, flag=""))
    C1 = np.array(CAMS["R1"]["C"])
    for k, v in CAMS["R1"].items():
        if k == "C":
            continue
        t = float(k)
        ts = r1_time(t)
        uv = T.project(path(ts), C1, v[:3], v[3], T.CY_R1)[0]
        m = car_r1.get(t)
        flag = "extrapolated time" if t > A[-1, 0] else ("release frame (camera fit used the sim car)" if t == T.R1_REL else "")
        rows.append(dict(view="R1", t=t, t_sim=round(ts, 3), img=f"frames/tri/r1/eq_{t:.2f}.png", edge_rms=round(v[4], 1),
                         sim=[round(float(uv[0]), 1), round(float(uv[1]), 1)], meas=list(m) if m else None,
                         res=round(float(np.hypot(*(uv - m))), 1) if m else None, flag=flag))
    return rows


def summary(rows):
    out = {}
    for view in ("B", "R1"):
        r = np.array([x["res"] for x in rows if x["view"] == view and x["res"] is not None])
        out[view] = dict(n=len(r), rms=round(float(np.sqrt(np.mean(r ** 2))), 1), median=round(float(np.median(r)), 1),
                         max=round(float(r.max()), 1))
    return out


if __name__ == "__main__":
    rows = frames()
    s = summary(rows)
    for x in rows:
        print(f"{x['view']:2s} {x['t']:6.2f} sim t {x['t_sim']:6.2f}  sim px ({x['sim'][0]:7.1f}, {x['sim'][1]:6.1f})  "
              f"car px {str(x['meas']):12s} residual {x['res'] if x['res'] is not None else '-':>6} px  edge rms {x['edge_rms']} {x['flag']}")
    for view, v in s.items():
        print(f"sim car vs by-eye car, {view}: n {v['n']}, rms {v['rms']} px, median {v['median']} px, max {v['max']} px")
    (ROOT / "out" / "reproj.js").write_text("window.REPROJ = " + json.dumps(dict(frames=rows, summary=s)) + ";\n")
    print("wrote out/reproj.js")
