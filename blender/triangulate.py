"""Triangulate the car after release from two cameras: broadcast shot B (live, PCMag time) and replay R1 (eqOfmtTJD1A
68.85-79.7, slow motion with a ramped speed, so each R1 frame gets its own real time).
Wall edges are read automatically from frames/tri/*/ (per-row bright run of the wall strip); car centres were read by eye
on 1.5x gridded crops (bounding-box centre of the car silhouette). Car height comes from the HUD altitude (lag HUD_LAG).
Unknowns: R1 tripod + per-frame pan/tilt/roll/zoom, B per-frame pan/tilt/roll/zoom (tripod from calib_cameras.py),
car x(t), y(t) as a cubic B-spline, and the real time of every R1 frame (monotonic).  Run: python blender/triangulate.py"""
import json
from pathlib import Path

import numpy as np
from PIL import Image
from scipy.interpolate import BSpline
from scipy.optimize import least_squares

ROOT = Path(__file__).resolve().parent.parent
txt = (ROOT / "out" / "replay_data.js").read_text(encoding="utf-8")
SIM = json.loads(txt[txt.index("{"):txt.rindex("}") + 1])
FR = np.array(SIM["frames"])
wx, wz, W = np.array(SIM["wall"]["x"]), np.array(SIM["wall"]["z"]), SIM["wall"]["width"]
EDGE = {s: np.c_[wx, np.full_like(wx, s * W / 2), wz] for s in (+1, -1)}   # +1 = screen-left edge in both cameras
CAL = json.loads((ROOT / "out" / "camera_calib.json").read_text())
CB = np.array(CAL["position"])
PC_OFFSET, T_REL, HUD_LAG, H_CG, FT = 61.35, 52.25, 0.30, 0.45, 0.3048
R1_REL = 69.94                      # eq time of the release frame in R1

# car centre (px) by eye; keys = video time of the frame
CAR_R1 = {69.94: (660, 285), 70.19: (650, 312), 70.44: (650, 310), 70.69: (650, 328), 70.94: (648, 332), 71.19: (645, 335),
          71.44: (640, 338), 71.69: (635, 360), 71.94: (625, 375), 72.19: (630, 375), 72.44: (627, 375), 72.69: (628, 372),
          72.94: (630, 375), 73.19: (630, 375), 73.44: (645, 378), 73.69: (675, 375), 73.94: (697, 376), 74.19: (718, 376),
          74.44: (715, 373), 74.69: (705, 347), 74.94: (693, 312), 75.19: (680, 288), 75.44: (665, 273), 75.69: (648, 265),
          75.94: (632, 267), 76.19: (613, 275), 76.44: (600, 313)}
CAR_B = {113.6: (610, 66), 113.8: (628, 122), 114.0: (643, 175), 114.2: (655, 245), 114.4: (670, 305), 114.6: (678, 355),
         114.8: (685, 410), 115.0: (683, 460), 115.2: (668, 503), 115.4: (662, 523), 115.6: (652, 510), 115.8: (638, 488),
         116.0: (623, 470), 116.2: (605, 453), 116.4: (588, 457)}   # 116.6+: wall leaves the frame / car in the cloud
LIP_R1 = {69.94: ((602.5, 259), (763.5, 256))}      # lip corners (L, R), frames/angles/eqOfmtTJD1A_69.936.png


def rot(yaw, pitch, roll):
    cy, sy, cp, sp, cr, sr = np.cos(yaw), np.sin(yaw), np.cos(pitch), np.sin(pitch), np.cos(roll), np.sin(roll)
    fwd = np.array([cy * cp, sy * cp, sp])
    right = np.array([sy, -cy, 0.0])
    up = np.cross(right, fwd)
    right, up = cr * right + sr * up, -sr * right + cr * up
    return np.vstack([right, -up, fwd])


def project(X, C, ang, f, cy=360.0):
    Xc = (np.atleast_2d(X) - C) @ rot(*ang).T
    return np.c_[640 + f * Xc[:, 0] / Xc[:, 2], cy + f * Xc[:, 1] / Xc[:, 2]]


def wall_edges(path, ymax, thr=75, minw=40, maxw=260, gap=8, maxgap=10 ** 6):
    """Per 10th row: the widest bright run = the wall strip (runs split by gaps <= 8 px are merged);
    drop rows off a quintic trend (car, plume, cloud)."""
    g = np.asarray(Image.open(path).convert("L"), float)
    pts = []
    for y in range(0, min(ymax, g.shape[0]), 10):
        b = np.r_[0, (g[y] > thr).astype(int), 0]
        d = np.diff(b)
        rs = []
        for a, e in zip(np.where(d == 1)[0], np.where(d == -1)[0]):
            if rs and a - rs[-1][1] <= gap:
                rs[-1] = (rs[-1][0], e)
            else:
                rs.append((a, e))
        rs = [r for r in rs if minw <= r[1] - r[0] <= maxw]
        if rs:
            a, e = max(rs, key=lambda r: r[1] - r[0])
            pts.append((y, a, e))
    pts = np.array(pts, float)
    for _ in range(3):
        keep = np.ones(len(pts), bool)
        for c in (1, 2):
            r = pts[:, c] - np.polyval(np.polyfit(pts[:, 0], pts[:, c], 5), pts[:, 0])
            keep &= np.abs(r) < max(4, 2.5 * np.median(np.abs(r)))
        pts = pts[keep]
    jump = np.where(np.diff(pts[:, 0]) > maxgap)[0]      # B: cut at the first 40+ row gap (smoke below the wall, 115.8)
    pts = pts[:jump[0] + 1] if len(jump) else pts
    out = {+1: pts[:, [1, 0]], -1: pts[:, [2, 0]], "lip": {}}
    # lip corners: the lip end is a straight top boundary of the strip; fit it per column (first bright row from the top,
    # robust line), then intersect it with each edge extended linearly from its top 4 readings
    y_top, a_top, e_top = pts[0]
    if y_top >= 15:
        cols = np.arange(int(a_top) - 25, int(e_top) + 26)
        cols = cols[(cols >= 0) & (cols < g.shape[1])]
        y0 = int(max(0, y_top - 80))
        sub = g[y0:int(y_top) + 1, cols] > thr
        has = sub.any(0)
        if has.sum() > 20:
            cx, cy = cols[has], y0 + sub[:, has].argmax(0)
            keep = np.ones(len(cx), bool)
            for _ in range(4):
                m, c0 = np.polyfit(cx[keep], cy[keep], 1)
                r = cy - (m * cx + c0)
                keep = np.abs(r) < max(2.0, 2.5 * np.median(np.abs(r[keep])))
            if keep.sum() > 20 and (m * cx[keep] + c0).min() > 3:
                for s, c in ((+1, 1), (-1, 2)):
                    k, x0 = np.polyfit(pts[:4, 0], pts[:4, c], 1)        # edge: x = k y + x0
                    y = (m * x0 + c0) / (1 - m * k)
                    if y < y_top + 5:
                        out["lip"][s] = np.array([k * y + x0, y])
    return out


def edge_res(C, ang, f, edges, cy):
    """Horizontal distance from each edge reading to the projected wall edge at the same image row.
    ponytail: row interpolation assumes the projected edge is monotonic in image y (true for these near-level views);
    it overstates the true distance by 1/cos(edge slope) where the overhang leans. Rows off the curve score a flat 50 px.
    (Nearest-point distance was tried 2026-10-17: R1 edge rms rose from ~2 to 16 px, so rows off the curve must stay flat.)"""
    out = []
    for s in (+1, -1):
        cur = project(EDGE[s], C, ang, f, cy)[::-1]        # wall z rises along the profile, so image y falls
        u = np.interp(edges[s][:, 1], cur[:, 1], cur[:, 0], left=np.nan, right=np.nan)
        out.append(np.nan_to_num(u - edges[s][:, 0], nan=50.0))
        if s in edges["lip"]:
            out.append(cur[0] - edges["lip"][s])              # cur is reversed: cur[0] = profile end = lip corner
    return np.concatenate(out)


R1T = sorted(CAR_R1)
BT = sorted(CAR_B)
E_R1 = {t: wall_edges(ROOT / "frames" / "tri" / "r1" / f"eq_{t:.2f}.png", 700) for t in R1T}
E_B = {t: wall_edges(ROOT / "frames" / "tri" / "b" / f"pc_{t:.1f}.png", 590, maxgap=40) for t in BT}
CY_R1 = 359.0                                        # eqOfmtTJD1A frames are 1280x718
for E, t in ((E_B, 113.6), (E_B, 113.8), (E_R1, 70.19)):   # the car hangs over the lip: auto corners unusable
    E[t]["lip"] = {}
E_R1[69.94]["lip"] = {+1: np.array(LIP_R1[69.94][0]), -1: np.array(LIP_R1[69.94][1])}


def car_sim(t):
    return np.array([np.interp(t, FR[:, 0], FR[:, k]) for k in (1, 2, 3)])


# ---------- stage 1: R1 tripod + per-frame angles from the wall (and the car parked on the lip at release) ----------
def r1_res(p, frames):
    C = p[:3]
    out = []
    for k, t in enumerate(frames):
        ang, f = p[3 + 4 * k:6 + 4 * k], p[6 + 4 * k]
        out.append(edge_res(C, ang, f, E_R1[t], CY_R1))
        if t == R1_REL:
            out.append(0.5 * (project(car_sim(T_REL), C, ang, f, CY_R1)[0] - CAR_R1[t]))
    if len(frames) > 1:   # one tripod head: zoom and roll change smoothly (1% zoom step = 2 px, 0.1 deg roll step = 1.7 px)
        per = p[3:].reshape(-1, 4)
        out.append(200 * np.diff(np.log(per[:, 3])))
        out.append(1000 * np.diff(per[:, 2]))
    return np.concatenate(out)


def fit_frame(C, edges, cy, init):
    """One frame's pan/tilt/roll/zoom with the tripod fixed; several starts."""
    best = None
    for dp in (0.0, -0.1, 0.1):
        p0 = [init[0], init[1] + dp, 0.0, init[2]]
        s = least_squares(lambda q: edge_res(C, q[:3], q[3], edges, cy), p0, loss="soft_l1", f_scale=5.0,
                          bounds=([-np.pi, -1.2, -0.12, 300], [np.pi, 1.2, 0.12, 30000]))
        if best is None or s.cost < best.cost:
            best = s
    return best.x


def solve_r1(C0, fix_x=None):
    per = []
    for t in R1T:
        d = car_sim(T_REL) - C0
        prev = per[-1] if per else [np.arctan2(d[1], d[0]), np.arctan2(d[2], np.hypot(d[0], d[1])), 0, 5600]
        per.append(fit_frame(C0, E_R1[t], CY_R1, [prev[0], prev[1], prev[3]]))
    p0 = np.r_[C0, np.concatenate(per)]
    # x <= -60 m: the two-frame solve puts R1 on a ridge x -90..-144 (rms 2.3 px); its near minima (x -43..-49) fit at 5+ px
    lo = np.r_[[-400, -150, 0.3], np.tile([-np.pi, -1.2, -0.3, 300], len(R1T))]
    hi = np.r_[[-60, 150, 60], np.tile([np.pi, 1.2, 0.3, 30000], len(R1T))]
    if fix_x is not None:
        p0[0], lo[0], hi[0] = fix_x, fix_x - 1e-6, fix_x + 1e-6
    # each frame's residuals depend only on the tripod and that frame's 4 angles
    sizes = [len(r1_res(np.r_[p0[:3], p0[3 + 4 * k:7 + 4 * k]], [t])) for k, t in enumerate(R1T)]
    n_sm = len(R1T) - 1
    sp = np.zeros((sum(sizes) + 2 * n_sm, len(p0)), bool)
    row = 0
    for k, n in enumerate(sizes):
        sp[row:row + n, :3] = True
        sp[row:row + n, 3 + 4 * k:7 + 4 * k] = True
        row += n
    for j, col in ((0, 3), (1, 2)):           # smoothness rows: zoom (index 3), roll (index 2) of frames k, k+1
        for k in range(n_sm):
            sp[row + j * n_sm + k, [3 + 4 * k + col, 3 + 4 * (k + 1) + col]] = True
    s = least_squares(r1_res, p0, args=(R1T,), bounds=(lo, hi), x_scale="jac", jac_sparsity=sp, loss="soft_l1", f_scale=5.0,
                      ftol=1e-12, xtol=1e-12, gtol=1e-12, max_nfev=400)
    print(f"solve_r1: {s.nfev} evals, {s.message}")
    return s.x


# ---------- stage 2: B per-frame angles, tripod fixed ----------
def solve_b():
    known = sorted((float(k) + PC_OFFSET, v) for k, v in CAL["frames"].items())
    kt = np.array([k for k, _ in known])
    out, prev = {}, None
    for t in BT:          # start from the previous frame (B pans right on past the calibrated frames)
        init = [np.interp(t, kt, [v[n] for _, v in known]) for n in ("yaw", "pitch", "f_px")] if prev is None else prev[[0, 1, 3]]
        out[t] = prev = fit_frame(CB, E_B[t], 360.0, init)
    return out


# ---------- stage 3: trajectory x(t), y(t) + R1 frame times; z(t) from the HUD ----------
hud = np.genfromtxt(ROOT / "data" / "hud_wall_flip.csv", delimiter=",", names=True, skip_header=2)
def z_hud(t):
    return np.interp(np.asarray(t) + HUD_LAG, hud["video_t"], hud["altitude_ft"]) * FT + H_CG


T_END = 57.0
KN = np.r_[[T_REL] * 3, np.arange(T_REL, T_END, 0.3), [T_END] * 4]
NC = len(KN) - 4


# R1 time map: fixed through events seen in both R1 and the real-time axis (2026-10-17), one free speed after the last.
# release 69.94 -> 52.25 (G drop); car upright and level 71.94 -> 53.10 (ROLL_T1, frames/roll/);
# condensation cloud first on the floor 74.69 -> 54.00 (B: none at PCMag 115.2, small at 115.4, clear at 115.6)
R1_ANCHOR = np.array([[69.94, 52.25], [71.94, 53.10], [74.69, 54.00]])


Z_SIG = 0.5          # m: HUD altitude is a soft prior. With z hard-wired, B ranges the car from height alone and
                     # dx/dz ~ 5 m/m (B sits 7.7 m up, ~60 m away), so HUD read/offset errors swamped x (2026-10-17).


def unpack3(q):
    cx, cy_, cz, u = q[:NC], q[NC:2 * NC], q[2 * NC:3 * NC], q[3 * NC]
    te = np.array(R1T)
    a = R1_ANCHOR[-1]
    tau = np.where(te <= a[0], np.interp(te, *R1_ANCHOR.T), a[1] + np.exp(u) * (te - a[0]))
    return BSpline(KN, cx, 3), BSpline(KN, cy_, 3), BSpline(KN, cz, 3), np.minimum(tau, T_END)


def traj_res(q, cams_r1, cams_b):
    sx, sy, sz, tau = unpack3(q)
    out = []
    C1 = cams_r1[:3]
    for k, t in enumerate(R1T):
        ang, f = cams_r1[3 + 4 * k:6 + 4 * k], cams_r1[6 + 4 * k]
        tt = tau[k]
        out.append(project(np.array([sx(tt), sy(tt), sz(tt)]), C1, ang, f, CY_R1)[0] - CAR_R1[t])
    for t in BT:
        tt = t - PC_OFFSET
        ang, f = cams_b[t][:3], cams_b[t][3]
        out.append(project(np.array([sx(tt), sy(tt), sz(tt)]), CB, ang, f)[0] - CAR_B[t])
    tg = np.arange(T_REL, 56.41, 0.1)
    out.append((sz(tg) - z_hud(tg)) / Z_SIG)
    for c in (q[:NC], q[NC:2 * NC], q[2 * NC:3 * NC]):
        out.append(0.3 * np.diff(c, 3))
    return np.concatenate(out)


def solve_traj(cams_r1, cams_b):
    tg = np.linspace(T_REL, T_END, NC)
    q0 = np.r_[np.interp(tg, FR[:, 0], FR[:, 1]), np.zeros(NC), z_hud(tg), np.log(0.6)]
    lo = np.r_[[-80] * NC, [-40] * NC, [-5] * NC, np.log(0.2)]
    hi = np.r_[[20] * NC, [40] * NC, [30] * NC, np.log(1.5)]
    return least_squares(traj_res, q0, args=(cams_r1, cams_b), bounds=(lo, hi), x_scale="jac", loss="soft_l1", f_scale=10.0)


if __name__ == "__main__":
    cams_b = solve_b()
    rb = np.concatenate([edge_res(CB, cams_b[t][:3], cams_b[t][3], E_B[t], 360.0) for t in BT])
    print(f"B: {len(BT)} frames, wall-edge rms {np.sqrt(np.mean(rb ** 2)):.2f} px")
    runs = {}
    for fx in (None, -95.0, -110.0, -140.0):        # R1 distance is weak (tele lens): repeat with the tripod x held
        cams_r1 = solve_r1(np.array([-125.0 if fx is None else fx, -9.0, 11.0]), fix_x=fx)
        r = r1_res(cams_r1, R1T)
        s = solve_traj(cams_r1, cams_b)
        sx, sy, sz, tau = unpack3(s.x)
        nr = 2 * (len(R1T) + len(BT))
        car_r = s.fun[:nr].reshape(-1, 2)
        runs[fx] = dict(C1=cams_r1[:3].round(2).tolist(), r1_edge_rms=float(np.sqrt(np.mean(r ** 2))),
                        car_rms_r1=float(np.sqrt(np.mean(car_r[:len(R1T)] ** 2))), car_rms_b=float(np.sqrt(np.mean(car_r[len(R1T):] ** 2))),
                        tau=dict(zip([f"{t:.2f}" for t in R1T], tau.round(3).tolist())),
                        f_mm=[round(cams_r1[6 + 4 * k] * 36 / 1280, 1) for k in range(len(R1T))],
                        path={f"{t:.2f}": [round(float(sx(t)), 2), round(float(sy(t)), 2), round(float(sz(t)), 2)]
                              for t in np.arange(T_REL, 56.31, 0.25)})
        print(f"\nR1 tripod x {'free' if fx is None else 'held'}: position {runs[fx]['C1']}, edge rms {runs[fx]['r1_edge_rms']:.2f} px, "
              f"car rms R1 {runs[fx]['car_rms_r1']:.1f} px, B {runs[fx]['car_rms_b']:.1f} px, R1 lens {min(runs[fx]['f_mm'])}-{max(runs[fx]['f_mm'])} mm")
        print("   R1 frame -> real time:", " ".join(f"{t:.2f}>{v:.2f}" for t, v in zip(R1T, tau)))
        if fx is None:
            print("   car residual px R1:", " ".join(f"{t:.2f}:{np.hypot(*r):.0f}" for t, r in zip(R1T, car_r[:len(R1T)])))
            print("   car residual px B: ", " ".join(f"{t:.1f}:{np.hypot(*r):.0f}" for t, r in zip(BT, car_r[len(R1T):])))
        for t in np.arange(T_REL, 56.31, 0.5):
            P, Ps = runs[fx]["path"][f"{t:.2f}"], car_sim(t)
            print(f"   t {t:.2f}: video x {P[0]:7.2f} y {P[1]:6.2f} z {P[2]:5.2f}   sim x {Ps[0]:7.2f} y {Ps[1]:5.2f} z {Ps[2]:5.2f}")
    (ROOT / "out" / "triangulation.json").write_text(json.dumps({str(k): v for k, v in runs.items()}, indent=1))
    print("wrote out/triangulation.json")
