"""Solve the per-frame camera poses for the triangulation frames once and cache them (about 5 min).
B: solve_b() (tripod from calib_cameras.py, 15 frames PCMag 113.6-116.4). R1: solve_r1() with the tripod free (27 frames).
Writes out/cameras_tri.json: {"B": {"113.6": [yaw, pitch, roll, f_px], ...}, "R1": {"C": [x, y, z], "69.94": [...], ...}}.
Run: python blender/cache_cameras.py"""
import json
import time
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import triangulate as T  # noqa: E402

t0 = time.time()
cams_b = T.solve_b()
print(f"solve_b {time.time() - t0:.0f} s", flush=True)
cams_r1 = T.solve_r1(np.array([-125.0, -9.0, 11.0]))
print(f"solve_r1 done {time.time() - t0:.0f} s", flush=True)
out = {"B": {f"{t:.1f}": [float(v) for v in cams_b[t]] for t in T.BT},
       "R1": {"C": [float(v) for v in cams_r1[:3]],
              **{f"{t:.2f}": [float(v) for v in cams_r1[3 + 4 * k:7 + 4 * k]] for k, t in enumerate(T.R1T)}}}
for t in T.BT:
    r = T.edge_res(T.CB, cams_b[t][:3], cams_b[t][3], T.E_B[t], 360.0)
    out["B"][f"{t:.1f}"].append(float(np.sqrt(np.mean(r ** 2))))
for k, t in enumerate(T.R1T):
    r = T.edge_res(cams_r1[:3], cams_r1[3 + 4 * k:6 + 4 * k], cams_r1[6 + 4 * k], T.E_R1[t], T.CY_R1)
    out["R1"][f"{t:.2f}"].append(float(np.sqrt(np.mean(r ** 2))))
out["note"] = "per frame: yaw, pitch, roll (rad), f_px, wall-edge rms px. R1 tripod free, start (-125, -9, 11)."
(T.ROOT / "out" / "cameras_tri.json").write_text(json.dumps(out, indent=1))
print("wrote out/cameras_tri.json", f"{time.time() - t0:.0f} s")
