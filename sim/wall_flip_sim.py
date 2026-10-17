"""Tesla Roadster wall-launch flip (2026-10-15 unveil) - physics reconstruction.

Segment simulated: PCMag quRC56Z0EJw 1:41-2:00 = eqOfmtTJD1A 0:40-0:59 (live, real time).
The car drives up a curved wall, rides the overhang inverted, lets go near the lip,
half-rolls upright in free fall, brakes with the front nozzles and lands on its thrusters.

Two phases:
  1. On the wall: path-constrained. Path = wall profile rebuilt from the HUD of the first, slow climb
     (pitch vs altitude). Motion s(t) = HUD altitude of this run mapped onto that path. Physics gives the
     tyre force, the suction needed to stay on, motor power and the accelerometer reading.
  2. Free flight: 6-DOF rigid body, 10 cold-gas nozzles, blowdown COPV (Abel-Noble gas), valve lag,
     4 spring-damper wheels, a flight controller that tracks the HUD altitude and a half-roll schedule.
Output: out/replay_data.js (for replay.html) and a fit report. Every number marked "est." is an estimate.
Run:  python sim/wall_flip_sim.py [short]   ("short": lip at the release point, see OVERHANG)
"""
import json
import sys
from pathlib import Path

import numpy as np
from scipy.interpolate import PchipInterpolator, make_smoothing_spline
from scipy.optimize import lsq_linear

ROOT = Path(__file__).resolve().parent.parent
FT, G0 = 0.3048, 9.80665
GVEC = np.array([0.0, 0.0, -G0])

# ---------------- parameters (sources in README.html) ----------------
M_DRY = 2100.0       # kg est.: HUD max suction 4700 kg / "up to 2 G" -> ~2350 kg wet, minus 250 kg gas
GAS0 = 250.0         # kg, "about 250 kilos of air" (event)
V_TANK = 0.600       # m^3, two COPVs, 600 L total (event)
T_GAS0 = 293.0       # K est.
R_GAS, GAMMA = 296.8, 1.4   # N2 (Top Gear: "high pressure nitrogen"); air differs by <3 %
B_ABEL = 8.0e-4      # m^3/kg, Abel-Noble co-volume fitted to N2 Z=1.65 at 690 bar
POLY_N = 1.25        # blowdown polytropic exponent est.: 1.4 = no wall heat, 1.0 = isothermal; fast COPV blowdowns ~1.2-1.3
CF = 1.6             # thrust coefficient est. (expanded nozzle, ~500:1 pressure ratio)
INERTIA = np.diag([650.0, 2900.0, 3200.0])  # kg m^2 est. (roll, pitch, yaw)
H_CG = 0.45          # m est., CG height at rest
WHEELBASE, TRACK = 2.825, 1.70              # wheelbase from Tesla (Top Gear); track est.
CAR_L, CAR_W, CAR_H = 4.70, 2.00, 1.15      # m; width 2 m (Top Gear), length/height est.
K_SUSP, C_SUSP = 80e3, 6e3                   # per wheel, est.
MU = 1.1             # tyre friction est.
CDA = np.array([0.7, 2.0, 8.0])             # m^2 body-axis drag areas est.
RHO = 1.2
SUCTION_MAX_KG = 4700.0                      # HUD DOWNFORCE peak on the overhang (first climb)
VALVE_TAU = 0.02     # s, valve actuator lag est.
RAMP_WIDTH = 3.3     # m est. from frame PCMag 1:44 (car width 2.0 m as scale)
T_RELEASE = 52.25    # s video time: HUD G drops 1.0 -> 0.4 between 52.2 and 52.3
ROLL_T0, ROLL_T1 = 52.35, 53.10   # half-roll window, VideoFromSpace 51.9-53.9 s at 10 fps (frames/roll/): leaves the lip 52.3, on its side 52.6-52.7, upright 53.1
BRAKE_T1 = 55.0      # horizontal speed brought to ~0 by this time (car hovers at ~10 ft 54.8-55.2)
HUD_LAG = 0.30       # s, HUD altitude lags truth: free-fall fit of 52.2-53.95 gives 0.34 s (rms 1.26 ft); G gauge taken as unlagged
# Overhang switch. "hud": straight 170 deg ceiling runs 1.5 m past the release point to the lip (est.).
# "short": lip at the release point = shortest wall that still reaches the HUD release altitude on a 170 deg ceiling.
OVERHANG = "short" if "short" in sys.argv[1:] else "hud"
LIP_EXT = {"hud": 1.5, "short": 0.0}[OVERHANG]
T_BURN = 53.95       # s, landing burn starts (HUD G 0.9 -> 1.7); before this the car falls free except roll/brake thrust

# 10 nozzles (Top Gear / event): 4 rear main, 4 corner lift, 2 front (braking + yaw). Body frame x fwd, y left, z up.
C20, S20 = np.cos(np.radians(20)), np.sin(np.radians(20))
NOZZLES = (
    [(f"rear{i}", (-2.30, y, -0.10), (1, 0, 0), 20e3) for i, y in enumerate((0.45, 0.15, -0.15, -0.45))]
    + [(f"lift_{n}", (x, y, -0.20), (0, 0, 1), 35e3)
       for n, x, y in (("FL", 1.40, 0.85), ("FR", 1.40, -0.85), ("RL", -1.40, 0.85), ("RR", -1.40, -0.85))]
    + [("front_L", (2.20, 0.70, -0.10), (-C20, -S20, 0), 15e3),   # exhaust forward, 20 deg outward; thrust aft-inward
       ("front_R", (2.20, -0.70, -0.10), (-C20, S20, 0), 15e3)]
)
N_POS = np.array([n[1] for n in NOZZLES], float)
N_DIR = np.array([n[2] for n in NOZZLES], float)
N_FMAX0 = np.array([n[3] for n in NOZZLES], float)
ALLOC = np.vstack([N_DIR.T, np.cross(N_POS, N_DIR).T])     # 6 x 10 wrench per newton of each nozzle


def load(name):
    return np.genfromtxt(ROOT / "data" / name, delimiter=",", names=True, skip_header=2)


# ---------------- tank: Abel-Noble gas, adiabatic blowdown ----------------
GAMMA_FN = GAMMA * (2 / (GAMMA + 1)) ** ((GAMMA + 1) / (2 * (GAMMA - 1)))


class Tank:
    def __init__(self):
        self.m, self.T = GAS0, T_GAS0
        self.P0 = self.pressure()

    def pressure(self):
        return self.m * R_GAS * self.T / (V_TANK - self.m * B_ABEL)

    def cstar(self):
        return np.sqrt(GAMMA * R_GAS * self.T) / GAMMA_FN

    def draw(self, thrust_total, dt):
        mdot = thrust_total / (CF * self.cstar())
        v0 = V_TANK / self.m
        self.m -= mdot * dt
        self.T *= ((v0 - B_ABEL) / (V_TANK / self.m - B_ABEL)) ** (POLY_N - 1)
        return mdot


# ---------------- wall geometry ----------------
def build_wall():
    """CG path of the car on the wall: x(s), z(s) (z = HUD altitude), pitch theta(s)."""
    d = load("wall_profile_hud.csv")
    th, h = np.radians(d["pitch_deg"]), d["altitude_ft"] * FT
    keep = th > np.radians(20)                     # below 20 deg the 0.1 ft read precision swamps x
    th, h = th[keep], h[keep]
    first170 = np.argmax(th >= np.radians(169.9))
    th_c, h_c = th[:first170 + 1], h[:first170 + 1]
    _, u = np.unique(th_c, return_index=True)
    th_c, h_c = th_c[u], h_c[u]
    # entry: circular arc fitted to the 27.6-76.4 deg points  (z = R(1 - cos th))
    sel = th_c < np.radians(80)
    R = np.sum(h_c[sel] * (1 - np.cos(th_c[sel]))) / np.sum((1 - np.cos(th_c[sel])) ** 2)
    tg = np.radians(np.linspace(0, 170, 1701))
    z_curve = PchipInterpolator(np.r_[0, th_c], np.r_[0, h_c])(tg)
    blend = np.clip((tg - np.radians(20)) / np.radians(20), 0, 1)       # arc -> data over 20-40 deg
    z = (1 - blend) * R * (1 - np.cos(tg)) + blend * z_curve
    dz = np.diff(z)
    tm = (tg[1:] + tg[:-1]) / 2
    x = np.r_[0, np.cumsum(dz / np.tan(tm))]
    # straight 170 deg ceiling up to the release point (+LIP_EXT to the lip)
    z_rel = float(np.interp(T_RELEASE, *_hud_alt_m()))
    L = (z_rel - z[-1]) / np.sin(np.radians(170)) + LIP_EXT
    tl = np.array([np.cos(tg[-1]), np.sin(tg[-1])])
    sl = np.linspace(0, L, 60)[1:]
    x = np.r_[x, x[-1] + sl * tl[0]]
    z = np.r_[z, z[-1] + sl * tl[1]]
    th_all = np.r_[tg, np.full(sl.size, tg[-1])]
    s = np.r_[0, np.cumsum(np.hypot(np.diff(x), np.diff(z)))]
    # flat approach road in front of the foot
    s_app = np.linspace(-40, 0, 81)[:-1]
    return dict(R=R, s=np.r_[s_app, s], x=np.r_[s_app, x], z=np.r_[0 * s_app, z],
                th=np.r_[0 * s_app, th_all], z_rel=z_rel)


def _hud_alt_m():
    d = load("hud_wall_flip.csv")
    return d["video_t"] - HUD_LAG, d["altitude_ft"] * FT


def frame_from_tn(t_hat, n_hat):
    return np.column_stack([t_hat, np.cross(n_hat, t_hat), n_hat])


def quat_from_R(R):
    w = np.sqrt(max(1e-12, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    return np.array([w, (R[2, 1] - R[1, 2]) / (4 * w), (R[0, 2] - R[2, 0]) / (4 * w), (R[1, 0] - R[0, 1]) / (4 * w)])


def R_from_quat(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def Rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def Ry(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def smoothstep(u):  # minimum-jerk 0->1
    u = np.clip(u, 0, 1)
    return u ** 3 * (10 - 15 * u + 6 * u * u)


# ---------------- phase 1: on the wall ----------------
def wall_phase(W, m):
    t, h = _hud_alt_m()
    on = (t <= T_RELEASE) & (h >= 0.1 * FT)          # 0.1 ft is the first readable altitude
    wall = W["s"] >= 0
    s_of_h = lambda hh: np.interp(hh, W["z"][wall], W["s"][wall])
    # car rests until 45.65 s (G 1.0 -> 1.1); CG start found from G=1.1 launch (a~0.46 g over 0.65 s)
    s0 = s_of_h(0.1 * FT) - 0.5 * 0.46 * G0 * (46.3 - 45.65) ** 2
    tt = np.r_[np.arange(44.0, 45.66, 0.1), t[on]]
    ss = np.r_[np.full(np.arange(44.0, 45.66, 0.1).size, s0), s_of_h(h[on])]
    spl = make_smoothing_spline(tt, ss, lam=2e-3)
    tg = np.arange(44.0, T_RELEASE + 1e-9, 0.01)
    s, v, a = spl(tg), spl.derivative()(tg), spl.derivative(2)(tg)
    s = np.maximum(s, s0)
    v[tg < 45.65], a[tg < 45.65] = 0.0, 0.0
    th = np.interp(s, W["s"], W["th"])
    kap = np.gradient(W["th"], W["s"])
    kap = np.interp(s, W["s"], kap)
    t_hat = np.column_stack([np.cos(th), 0 * th, np.sin(th)])
    n_hat = np.column_stack([-np.sin(th), 0 * th, np.cos(th)])
    drag = 0.5 * RHO * CDA[0] * v ** 2
    F_t = m * a + m * G0 * np.sin(th) + drag                          # tyre force along the path
    F_s_min = np.maximum(0, np.abs(F_t) / MU - m * v ** 2 * kap - m * G0 * np.cos(th))
    acc = a[:, None] * t_hat + (v ** 2 * kap)[:, None] * n_hat
    g_sim = np.linalg.norm(acc - GVEC, axis=1) / G0
    pos = np.column_stack([np.interp(s, W["s"], W["x"]), 0 * s, np.interp(s, W["s"], W["z"]) + H_CG])
    return dict(t=tg, s=s, v=v, a=a, th=th, kap=kap, F_t=F_t, F_s_min=F_s_min, g=g_sim, pos=pos,
                t_hat=t_hat, n_hat=n_hat, power=F_t * v)


# ---------------- phase 2: free flight ----------------
def flight_phase(P1, m_dry=M_DRY, dt=1e-3, t_end=58.0):
    t, h = _hud_alt_m()
    sel = t >= T_RELEASE - 0.05
    zref = make_smoothing_spline(t[sel], h[sel] + H_CG, lam=1e-3)
    zr_d, zr_dd = zref.derivative(), zref.derivative(2)

    tank = Tank()
    p = P1["pos"][-1].copy()
    v = P1["v"][-1] * P1["t_hat"][-1]
    R = frame_from_tn(P1["t_hat"][-1], P1["n_hat"][-1])
    q, w = quat_from_R(R), np.zeros(3)
    vx0, x0 = v[0], p[0]
    R_up = np.array([[-1.0, 0, 0], [0, -1, 0], [0, 0, 1]])             # upright, heading -x
    pitch0 = -np.arctan2(R[2, 0], -R[0, 0])                             # release nose-up angle as Ry angle

    def R_ref(tt):
        u = smoothstep((tt - ROLL_T0) / (ROLL_T1 - ROLL_T0))
        return R_up @ Ry(pitch0 * (1 - smoothstep((tt - ROLL_T0) / (ROLL_T1 + 0.3 - ROLL_T0)))) @ Rx(np.pi * (1 - u))

    def x_ref(tt):  # min-jerk braking of the release velocity to zero by BRAKE_T1
        T = BRAKE_T1 - T_RELEASE
        u = np.clip((tt - T_RELEASE) / T, 0, 1)
        # velocity profile vx0*(1 - smoothstep(u)); position = integral
        us = np.linspace(0, u, 50)
        xr = x0 + vx0 * T * np.trapezoid(1 - smoothstep(us), us) + vx0 * max(0, tt - BRAKE_T1) * 0
        return xr, vx0 * (1 - smoothstep(u))

    wheels = np.array([[sx * WHEELBASE / 2, sy * TRACK / 2, 0.0] for sx in (1, -1) for sy in (1, -1)])
    delta0 = (m_dry + GAS0) * G0 / (4 * K_SUSP)
    wheels[:, 2] = -(H_CG + delta0)
    corners = np.array([[sx * CAR_L / 2, sy * CAR_W / 2, sz] for sx in (1, -1) for sy in (1, -1)
                        for sz in (-H_CG + 0.12, CAR_H - H_CG)])
    f_act, f_cmd = np.zeros(len(NOZZLES)), np.zeros(len(NOZZLES))
    landed_since, thrust_cut = None, False
    out = {k: [] for k in ("t", "pos", "quat", "noz", "g", "alt", "gas", "P", "ref_alt", "contact", "min_corner_z")}
    tt, step = T_RELEASE, 0
    while tt < t_end:
        m = m_dry + tank.m
        R = R_from_quat(q)
        # --- contact forces (world) ---
        Fc, Mc, Nsum = np.zeros(3), np.zeros(3), 0.0
        for rw in wheels:
            pw = p + R @ rw
            if pw[2] < 0:
                vw = v + R @ np.cross(w, rw)
                N = max(0.0, -K_SUSP * pw[2] - C_SUSP * vw[2])
                vt = np.array([vw[0], vw[1], 0.0])
                Ft = -vt * min(5e3, MU * N / max(np.linalg.norm(vt), 1e-6))
                f = Ft + np.array([0, 0, N])
                Fc += f
                Mc += R.T @ np.cross(R @ rw, f)
                Nsum += N
        # --- controller at 100 Hz ---
        if step % 10 == 0:
            if Nsum > 0.5 * m * G0:
                landed_since = tt if landed_since is None else landed_since
                thrust_cut = thrust_cut or (tt - landed_since > 0.05)
            else:
                landed_since = None
            if thrust_cut:
                f_cmd[:] = 0
            else:
                xr, vxr = x_ref(tt)
                axr = (x_ref(tt + 0.01)[1] - vxr) / 0.01
                a_des = np.array([axr + 4.0 * (xr - p[0]) + 4.0 * (vxr - v[0]),
                                  4.0 * (0 - p[1]) + 4.0 * (0 - v[1]),
                                  zr_dd(tt) + 9.0 * (zref(tt) - p[2]) + 4.2 * (zr_d(tt) - v[2])
                                  if tt >= T_BURN else -G0])
                F_b = R.T @ (m * (a_des - GVEC))
                Rt = R_ref(tt)
                Wd = (Rt.T @ R_ref(tt + 0.01) - Rt.T @ R_ref(tt - 0.01)) / 0.02
                w_ref = np.array([Wd[2, 1], Wd[0, 2], Wd[1, 0]])
                Re = Rt.T @ R
                e = 0.5 * np.array([Re[2, 1] - Re[1, 2], Re[0, 2] - Re[2, 0], Re[1, 0] - Re[0, 1]])
                M_b = INERTIA @ (-36.0 * e - 8.4 * (w - w_ref)) + np.cross(w, INERTIA @ w)
                fmax = N_FMAX0 * tank.pressure() / tank.P0
                Wt = np.r_[np.full(3, 1e-3), np.full(3, 3e-3)]
                A = np.vstack([ALLOC * Wt[:, None], 1e-2 * np.eye(len(NOZZLES)) * 1e-3])
                b = np.r_[np.r_[F_b, M_b] * Wt, np.zeros(len(NOZZLES))]
                f_cmd = lsq_linear(A, b, bounds=(0, fmax + 1e-9)).x
        # --- valves, tank, forces ---
        f_act += (f_cmd - f_act) * dt / VALVE_TAU
        f_act = np.clip(f_act, 0, N_FMAX0 * tank.pressure() / tank.P0)
        tank.draw(f_act.sum(), dt)
        F_th_b = N_DIR.T @ f_act
        M_th_b = np.cross(N_POS, N_DIR).T @ f_act
        vb = R.T @ v
        F_drag_b = -0.5 * RHO * CDA * np.abs(vb) * vb
        F_ng = R @ (F_th_b + F_drag_b) + Fc                 # all non-gravity forces, world
        # --- integrate (semi-implicit Euler) ---
        v = v + (F_ng / m + GVEC) * dt
        p = p + v * dt
        M_b = M_th_b + Mc
        w = w + np.linalg.solve(INERTIA, M_b - np.cross(w, INERTIA @ w)) * dt
        wq = np.r_[0, w]
        qw, qx, qy, qz = q
        dq = 0.5 * np.array([-qx * wq[1] - qy * wq[2] - qz * wq[3], qw * wq[1] + qy * wq[3] - qz * wq[2],
                             qw * wq[2] - qx * wq[3] + qz * wq[1], qw * wq[3] + qx * wq[2] - qy * wq[1]])
        q = q + dq * dt
        q /= np.linalg.norm(q)
        if step % 10 == 0:
            out["t"].append(tt); out["pos"].append(p.copy()); out["quat"].append(q.copy())
            out["noz"].append(f_act / N_FMAX0); out["g"].append(np.linalg.norm(F_ng) / (m * G0))
            out["alt"].append((p[2] - H_CG) / FT); out["gas"].append(tank.m); out["P"].append(tank.pressure() / 1e5)
            out["ref_alt"].append((zref(tt) - H_CG) / FT); out["contact"].append(Nsum)
            out["min_corner_z"].append(min((p + R @ c)[2] for c in corners))
        tt += dt
        step += 1
    return {k: np.array(val) for k, val in out.items()} | {"tank_P0_bar": tank.P0 / 1e5, "corners": corners}


def clearance_to_wall(W, F, corners):
    """Min distance from car box corners to the wall surface (xz polyline, |y| < width/2) after release."""
    on = W["s"] >= 0
    sx = W["x"][on] - 0 * H_CG
    nx, nz = -np.sin(W["th"][on]), np.cos(W["th"][on])
    surf = np.column_stack([W["x"][on] - H_CG * nx, W["z"][on] + H_CG - H_CG * nz])   # world z incl. CG offset
    best = np.inf
    for tt, p, q in zip(F["t"], F["pos"], F["quat"]):
        if tt < T_RELEASE + 0.3:
            continue
        R = R_from_quat(q)
        for c in corners:
            w = p + R @ c
            if abs(w[1]) < RAMP_WIDTH / 2:
                best = min(best, np.min(np.hypot(surf[:, 0] - w[0], surf[:, 1] - w[2])))
    return best, surf


def main():
    W = build_wall()
    m0 = M_DRY + GAS0
    P1 = wall_phase(W, m0)
    F = flight_phase(P1)
    hud = load("hud_wall_flip.csv")
    clear, surf = clearance_to_wall(W, F, F["corners"])

    # --- fit report ---
    ht, hg, ha = hud["video_t"], hud["g_force"], hud["altitude_ft"]
    wall_sel = (ht >= 46.0) & (ht <= T_RELEASE - 0.05)
    g_wall_rmse = np.sqrt(np.mean((np.interp(ht[wall_sel], P1["t"], P1["g"]) - hg[wall_sel]) ** 2))
    fl_sel = (ht > T_RELEASE + HUD_LAG) & (ht <= 57.2)
    alt_rmse = np.sqrt(np.mean((np.interp(ht[fl_sel] - HUD_LAG, F["t"], F["alt"]) - ha[fl_sel]) ** 2))
    g_fl_rmse = np.sqrt(np.mean((np.interp(ht[fl_sel], F["t"], F["g"]) - hg[fl_sel]) ** 2))
    i_apex = np.argmax(F["alt"])
    td = F["t"][np.argmax(F["contact"] > 0.5 * m0 * G0)]
    Rend = R_from_quat(F["quat"][-1])
    tilt_end = np.degrees(np.arccos(np.clip(Rend[2, 2], -1, 1)))
    i_rel = -1
    rep = {
        "overhang": OVERHANG, "wall_entry_radius_m": W["R"], "wall_height_release_m": W["z_rel"],
        "wall_horizontal_extent_m": float(W["x"][W["s"] >= 0].max() - W["x"][W["s"] >= 0].min()),
        "release_time_s": T_RELEASE, "release_x_m": float(P1["pos"][-1][0]), "release_track_s_m": float(P1["s"][-1]), "release_speed_mps": float(P1["v"][i_rel]),
        "release_pitch_deg": float(np.degrees(P1["th"][i_rel])),
        "wall_peak_speed_mps": float(P1["v"].max()),
        "wall_peak_power_kW": float(P1["power"].max() / 1e3),
        "wall_peak_tyre_force_kN": float(np.abs(P1["F_t"]).max() / 1e3),
        "wall_min_suction_needed_kgf": float(P1["F_s_min"].max() / G0), "suction_available_kgf": SUCTION_MAX_KG,
        "wall_G_rmse_vs_HUD": float(g_wall_rmse),
        "flight_alt_rmse_ft": float(alt_rmse), "flight_G_rmse_vs_HUD": float(g_fl_rmse),
        "apex_sim_ft": float(F["alt"][i_apex]), "apex_sim_t": float(F["t"][i_apex]),
        "apex_hud_ft": float(ha.max()), "apex_hud_t": float(ht[np.argmax(ha)] - HUD_LAG),
        "touchdown_sim_t": float(td), "touchdown_hud_t": 56.6 - HUD_LAG,
        "landing_x_m": float(F["pos"][-1][0]), "landing_y_m": float(F["pos"][-1][1]),
        "landing_tilt_deg": float(tilt_end),
        "gas_used_kg": float(GAS0 - F["gas"][-1]), "gas_left_kg": float(F["gas"][-1]),
        "tank_P_start_bar": float(F["tank_P0_bar"]), "tank_P_end_bar": float(F["P"][-1]),
        "peak_G_sim_flight": float(F["g"].max()), "peak_G_hud_flight": float(hg[fl_sel].max()),
        "min_clearance_to_wall_m": float(clear), "min_body_corner_z_m": float(F["min_corner_z"].min()),
        "mass_wet_kg": m0,
    }
    for k, val in rep.items():
        print(f"{k:32s} {val:10.3f}" if isinstance(val, float) else f"{k:32s} {val}")

    # --- replay data (30 fps) ---
    out = ROOT / "out"
    out.mkdir(exist_ok=True)
    frames = []
    for i in range(0, len(P1["t"]) - 1, 3):
        R = frame_from_tn(P1["t_hat"][i], P1["n_hat"][i])
        frames.append([round(P1["t"][i], 3), *np.round(P1["pos"][i], 3), *np.round(quat_from_R(R), 5),
                       *[0] * len(NOZZLES), round(P1["g"][i], 2), round((P1["pos"][i][2] - H_CG) / FT, 1),
                       GAS0, round(F["tank_P0_bar"], 0)])
    for i in range(0, len(F["t"]), 3):
        frames.append([round(F["t"][i], 3), *np.round(F["pos"][i], 3), *np.round(F["quat"][i], 5),
                       *np.round(F["noz"][i], 3), round(F["g"][i], 2), round(F["alt"][i], 1),
                       round(F["gas"][i], 1), round(F["P"][i], 0)])
    data = dict(report=rep, frames=frames, nozzles=[dict(name=n[0], pos=n[1], dir=n[2]) for n in NOZZLES],
                wall=dict(x=np.round(surf[:, 0], 3).tolist(), z=np.round(surf[:, 1], 3).tolist(), width=RAMP_WIDTH),
                car=dict(L=CAR_L, W=CAR_W, H=CAR_H, h_cg=H_CG, wheelbase=WHEELBASE, track=TRACK),
                hud=dict(t=ht.tolist(), g=hg.tolist(), alt=ha.tolist()),
                wall_profile=dict(s=np.round(P1["s"][::5], 3).tolist(), t=np.round(P1["t"][::5], 2).tolist(),
                                  suction_min_kgf=np.round(P1["F_s_min"][::5] / G0, 0).tolist()))
    (out / "replay_data.js").write_text("window.SIM = " + json.dumps(data) + ";\n", encoding="utf-8")
    print("wrote", out / "replay_data.js")

    # --- self-check: the reconstruction must reproduce the observed flight within these tolerances ---
    assert abs(rep["apex_sim_ft"] - rep["apex_hud_ft"]) < 1.5, "apex"
    assert abs(rep["touchdown_sim_t"] - rep["touchdown_hud_t"]) < 0.4, "touchdown time"
    # 2.5 ft: the pure free-fall fit to the HUD (52.2-53.95 s) already leaves 1.26 ft rms (read precision + lag model)
    assert rep["flight_alt_rmse_ft"] < 2.5, "altitude track"
    assert rep["landing_tilt_deg"] < 10, "lands upright"
    assert 0 < rep["gas_used_kg"] < GAS0, "gas budget"
    assert rep["min_clearance_to_wall_m"] > 0.1, "hits the wall"
    assert rep["min_body_corner_z_m"] > -0.05, "body touches ground"
    print("self-check passed")


if __name__ == "__main__":
    main()
