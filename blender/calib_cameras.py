"""Solve the two broadcast cameras (A and B) from wall-edge and car points read off PCMag frames (pixel coords, 1280x720).
One camera position for all frames (tripod), per-frame yaw/pitch/roll/focal (pan, tilt, zoom).
Edge points are matched point-to-curve against the projected SIM wall edges, so no point correspondence is needed.
The lip corners are in the fit (they fix distance vs focal length); the car point after release is a check only.  Run: python blender/calib_cameras.py"""
import json
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares

ROOT = Path(__file__).resolve().parent.parent
txt = (ROOT / "out" / "replay_data.js").read_text(encoding="utf-8")
SIM = json.loads(txt[txt.index("{"):txt.rindex("}") + 1])
FR = np.array(SIM["frames"])
wx, wz, W = np.array(SIM["wall"]["x"]), np.array(SIM["wall"]["z"]), SIM["wall"]["width"]
EDGE = {s: np.c_[wx, np.full_like(wx, s * W / 2), wz] for s in (+1, -1)}   # +1 = screen-left edge (y = +W/2)

# measured by eye on 2x crops with a 25 px grid (frames/ref/pcmag_<PCMag t>.png); L = y+W/2 edge, R = y-W/2 edge.
# Keys are PCMag times. PCMag = sim + 61.35 s: the ALTITUDE gauge in PCMag 112.2 reads 57.0 (CSV 50.8) and in 112.55
# reads 60.8 (CSV ~51.25); VideoFromSpace at 51.2 reads 60.2 = CSV. The 61.0 s in the CSV header is 0.35 s short.
PC_OFFSET = 61.35
OBS_PC = {
    109.8: dict(L=[(647.5, 555), (675, 525), (685, 500), (692, 450), (696, 400), (697.5, 350), (692.5, 300), (682.5, 250),
                  (664, 200), (650, 175), (632.5, 150), (602.5, 100)],
               R=[(720, 555), (740, 525), (747.5, 500), (752.5, 450), (754, 400), (752.5, 350), (750, 300), (745, 250),
                  (732.5, 200), (720, 175), (707.5, 150), (685, 100)],
               car=(726, 372), lipL=(570, 62), lipR=(650, 50)),
    112.2: dict(L=[(650, 575), (677, 550), (694, 500), (697, 400), (693, 300), (675, 200), (650, 150), (617, 100), (585, 50)],
               R=[(735, 575), (750, 550), (762, 500), (766, 400), (761, 300), (752, 250), (742, 200), (724, 150), (698, 100), (675, 50)],
               car=(675, 155), lipL=(562, 35), lipR=(648, 22)),
    # after release the car point is a check only (w_car = 0): it tests the simulated flight path, not the camera
    113.6: dict(L=[(700, 230), (720, 300), (725, 400), (718, 500), (700, 570)],
               R=[(745, 150), (775, 230), (795, 300), (800, 400), (805, 500)],
               car=(605, 68), lipL=(612, 100), lipR=(685, 75), fit_car=False),
    114.8: dict(L=[(870, 340), (900, 380), (915, 450), (915, 550), (908, 630)],
               R=[(940, 330), (970, 400), (985, 500), (990, 600)],
               car=(685, 408), lipL=(815, 268), lipR=(875, 255), fit_car=False),
    115.8: dict(L=[(1040, 270), (1080, 330), (1105, 420), (1110, 500), (1100, 560)],
               R=[(1100, 240), (1135, 310), (1160, 400), (1170, 500)],
               car=(635, 485), lipL=(975, 188), lipR=(1037, 170), fit_car=False),
}
OBS = {round(k - PC_OFFSET, 2): v for k, v in OBS_PC.items()}       # sim time -> observations
TIMES = sorted(OBS)


def rot(yaw, pitch, roll):
    """World->camera. Camera looks along its +z; yaw about world z from +x, pitch up, roll about view axis."""
    cy, sy, cp, sp, cr, sr = np.cos(yaw), np.sin(yaw), np.cos(pitch), np.sin(pitch), np.cos(roll), np.sin(roll)
    fwd = np.array([cy * cp, sy * cp, sp])
    right = np.array([sy, -cy, 0.0])
    up = np.cross(right, fwd)
    right, up = cr * right + sr * up, -sr * right + cr * up
    return np.vstack([right, -up, fwd])       # image x right, image y down, z forward


def project(X, C, ang, f):
    Xc = (X - C) @ rot(*ang).T
    return np.c_[640 + f * Xc[:, 0] / Xc[:, 2], 360 + f * Xc[:, 1] / Xc[:, 2]]


def car_pos(t):
    i = np.searchsorted(FR[:, 0], t)
    return FR[i, 1:4]


def unpack(p):
    return p[:3], [(p[3 + 4 * k:6 + 4 * k], p[6 + 4 * k]) for k in range(len(TIMES))]


def residuals(p, w_car=1.0):
    C, per = unpack(p)
    res = []
    for t, (ang, f) in zip(TIMES, per):
        o = OBS[t]
        for side, key in ((+1, "L"), (-1, "R")):
            curve = project(EDGE[side], C, ang, f)
            for q in o[key]:
                d = curve - np.array(q)
                res.append(np.sqrt((d ** 2).sum(1)).min())
        # lip corners: clean, well-spread points; they pin distance vs focal length (alone, those trade off 65-116 m)
        for side, key in ((+1, "lipL"), (-1, "lipR")):
            res += list(project(EDGE[side][-1:], C, ang, f)[0] - o[key])
        if o.get("fit_car", True):
            res += list(w_car * (project(car_pos(t)[None], C, ang, f)[0] - o["car"]))
        else:
            res.append(ang[2] * 2000)   # pan frames: pan/tilt head, roll held at 0 (locked-off frames solve |roll| < 1 deg)
    return np.array(res)


# stage 1: tripod position + lens from the locked-off frames only (dense, clean edge readings);
# camera height bounded to 0.3-30 m (distance and focal length trade off; the floor cannot be under the lens)
LOCKED = [t for t in TIMES if OBS[t].get("fit_car", True)]


def solve(times, C_fixed=None):
    global TIMES
    TIMES = times
    p0 = [] if C_fixed is not None else [-80, -15, 3]
    lo = [] if C_fixed is not None else [-300, -150, 0.3]
    hi = [] if C_fixed is not None else [0, 150, 30]
    for t in times:
        p0 += [np.radians(16), np.radians(7), 0.0, 1700.0]
        lo += [-1, -0.5, -0.5, 500]
        hi += [1, 0.5, 0.5, 6000]
    if C_fixed is None:
        fun = residuals
    else:
        fun = lambda q: residuals(np.r_[C_fixed, q])
    sol = least_squares(fun, p0, bounds=(lo, hi))
    return sol.x if C_fixed is None else np.r_[C_fixed, sol.x]


x1 = solve(LOCKED)
C = x1[:3]
# stage 2: every frame's pan/tilt/zoom with the tripod fixed
x2 = solve(sorted(OBS), C_fixed=C)
_, per = unpack(x2)
r = residuals(x2)
print("camera position (m):", np.round(C, 2), " rms px:", round(float(np.sqrt(np.mean(r ** 2))), 2), " max px:", round(float(np.abs(r).max()), 1))
out = dict(position=C.tolist(), frames={})
for t, (ang, f) in zip(TIMES, per):
    o = OBS[t]
    lens = f * 36 / 1280
    lipL = project(EDGE[+1][-1:], C, ang, f)[0]
    lipR = project(EDGE[-1][-1:], C, ang, f)[0]
    carp = project(car_pos(t)[None], C, ang, f)[0]
    print(f"t={t}: yaw {np.degrees(ang[0]):.2f} pitch {np.degrees(ang[1]):.2f} roll {np.degrees(ang[2]):.2f} deg, "
          f"f {f:.0f} px = {lens:.1f} mm (36 mm sensor)")
    print(f"   car  meas {o['car']}  proj {np.round(carp, 1)}")
    print(f"   lip L meas {o['lipL']} proj {np.round(lipL, 1)} | lip R meas {o['lipR']} proj {np.round(lipR, 1)}")
    # where the measured car pixel puts the car if its height is the simulated one (ray / plane z = sim z)
    R = rot(*ang)
    d = R.T @ np.array([(o["car"][0] - 640) / f, (o["car"][1] - 360) / f, 1.0])
    z = car_pos(t)[2]
    X = C + d * (z - C[2]) / d[2]
    print(f"   car from video at sim height {z:.2f} m: x {X[0]:.2f}  y {X[1]:.2f}   (sim x {car_pos(t)[0]:.2f} y {car_pos(t)[1]:.2f})")
    out["frames"][str(t)] = dict(yaw=float(ang[0]), pitch=float(ang[1]), roll=float(ang[2]), f_px=float(f), lens_mm=float(lens),
                                 car_from_video=X.tolist())
# distance from camera to the wall foot, for the report
print("distance to foot (m):", round(float(np.linalg.norm(C)), 1))
(ROOT / "out" / "camera_calib.json").write_text(json.dumps(out, indent=1))
print("wrote out/camera_calib.json")

# ---------------- camera A (behind the car at the wall foot, PCMag 105.0 = sim 43.65; car parked at the foot, so sim 44.0 is used, frames/ref/pcmag_105.0.png) ----------------
A_L = [(410, 460), (440, 400), (460, 330), (478, 250), (490, 170), (500, 100), (507, 0)]
A_R = [(945, 465), (960, 400), (975, 330), (990, 230), (1005, 130), (1015, 30)]
x_rw = FR[0, 1] - SIM["car"]["wheelbase"] / 2                      # rear axle x at t = 44.0
A_PTS = [((480, 495), (x_rw, SIM["car"]["track"] / 2, 0.0)), ((785, 495), (x_rw, -SIM["car"]["track"] / 2, 0.0))]  # rear tyre contacts


def res_A(p):
    C, ang, f = p[:3], p[3:6], p[6]
    out = []
    for side, pts in ((+1, A_L), (-1, A_R)):
        curve = project(EDGE[side], C, ang, f)
        out += [np.sqrt(((curve - np.array(q)) ** 2).sum(1)).min() for q in pts]
    for q, X in A_PTS:
        out += list(project(np.array([X]), C, ang, f)[0] - q)
    return np.array(out)


sA = least_squares(res_A, [-12, 0, 1.5, 0, 0.1, 0, 1300], bounds=([-200, -20, 0.2, -1, -0.5, -0.5, 300], [-3, 20, 10, 1, 0.8, 0.5, 20000]))
rA = res_A(sA.x)
print("camera A position (m):", np.round(sA.x[:3], 2), "yaw/pitch/roll deg:", np.round(np.degrees(sA.x[3:6]), 2),
      f"f {sA.x[6]:.0f} px = {sA.x[6] * 36 / 1280:.1f} mm", " rms px:", round(float(np.sqrt(np.mean(rA ** 2))), 2))
out["camA"] = dict(position=sA.x[:3].tolist(), yaw=float(sA.x[3]), pitch=float(sA.x[4]), roll=float(sA.x[5]), f_px=float(sA.x[6]))
(ROOT / "out" / "camera_calib.json").write_text(json.dumps(out, indent=1))
