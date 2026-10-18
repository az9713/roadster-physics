"""Part 4 of PHYSICS.html: the flip (ceiling -> release -> half-roll -> free fall -> burn), 51.31-53.95 s.

Run: python out/physics/flip.py (about 10 s). Imports sim/wall_flip_sim.py in-process and does not change it.
Every number Part 4 quotes is printed here. The terms the sim omits are evaluated on the sim's own trajectory.
Writes flip_film.png, flip_side.png, flip_roll.png, flip_moments.png, flip_forces.png, flip_terms.png.
"""
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon, Rectangle
from scipy.spatial import ConvexHull

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "sim"))
sys.argv = sys.argv[:1]                      # default "hud" overhang
import wall_flip_sim as S                     # noqa: E402

OUT = Path(__file__).resolve().parent
SIM, HUD, C3, C4 = "#0d9488", "#ea580c", "#8b5cf6", "#ec4899"   # same validated palette as make_charts.py
SURF, INK, MUTED, GRID = "#1e293b", "#cbd5e1", "#94a3b8", "#334155"
plt.rcParams.update({"figure.facecolor": SURF, "axes.facecolor": SURF, "savefig.facecolor": SURF,
                     "text.color": INK, "axes.labelcolor": INK, "xtick.color": MUTED, "ytick.color": MUTED,
                     "axes.edgecolor": GRID, "grid.color": GRID, "axes.grid": True, "grid.linewidth": 0.6,
                     "lines.linewidth": 2, "font.size": 10, "legend.frameon": False,
                     "axes.spines.top": False, "axes.spines.right": False})
G0, M0 = S.G0, S.M_DRY + S.GAS0
IXX, IYY, IZZ = np.diag(S.INERTIA)
D_ARM = 0.85                                  # lift nozzle |y|
T0, T1 = S.ROLL_T0, S.ROLL_T1
T_PITCH = T1 + 0.3
T = T1 - T0


def save(fig, name):
    fig.tight_layout()
    fig.savefig(OUT / name, dpi=130)
    plt.close(fig)
    print("wrote", OUT / name)


# ---------------- run the model ----------------
W = S.build_wall()
P1 = S.wall_phase(W, M0)
F = S.flight_phase(P1)
hud = S.load("hud_wall_flip.csv")
ht, hg = hud["video_t"], hud["g_force"]
i170 = np.argmax(P1["th"] >= np.radians(169.9))
T170 = P1["t"][i170]
print(f"window: 170 deg reached {T170:.2f} s, release {S.T_RELEASE}, roll {T0}-{T1}, pitch done {T_PITCH:.2f}, "
      f"burn {S.T_BURN}")

# ---------------- before: riding the ceiling ----------------
th = np.radians(170)
v_rel = P1["v"][-1]
D_rel = 0.5 * S.RHO * S.CDA[0] * v_rel ** 2
Ft0 = M0 * G0 * np.sin(th) + D_rel
Fs0 = Ft0 / S.MU - M0 * G0 * np.cos(th)
print(f"ceiling: weight normal part {-np.cos(th)*M0:.0f} kgf; at a = 0, v = {v_rel:.3f}: drag {D_rel:.1f} N, "
      f"F_t = {Ft0:.0f} N, F_s,min = {Fs0:.0f} N = {Fs0/G0:.0f} kgf ({Fs0/G0/S.SUCTION_MAX_KG*100:.0f}% of 4700)")
cur = (P1["t"] > 50.0) & (P1["t"] < T170)
qrate = P1["v"] * np.gradient(P1["th"], P1["t"]) / np.maximum(P1["v"], 1e-6)
print(f"pitch rate on the overhang 50.0-{T170:.2f} s: max {np.degrees(np.abs(np.gradient(P1['th'], P1['t'])[cur]).max()):.1f} deg/s;"
      f" on the straight ceiling it is 0 (kappa = 0)")
hinge_I = IXX + M0 * D_ARM ** 2
hinge_M = M0 * G0 * (-np.cos(th)) * D_ARM
print(f"hinge release (one side lets go first): gravity moment {hinge_M/1e3:.1f} kN m about the wheel line, "
      f"I_hinge {hinge_I:.0f} kg m2, alpha0 {hinge_M/hinge_I:.2f} rad/s2")

# ---------------- release and separation ----------------
gn, gt = -G0 * np.cos(th), G0 * np.sin(th)     # 0.985 g away from the ceiling, 0.174 g against the motion
ahead = W["s"][-1] - P1["s"][-1]
print(f"hud wall ends at s = {W['s'][-1]:.3f} m; release CG at s = {P1['s'][-1]:.3f} m: {ahead:.3f} m of ceiling ahead "
      f"(the spline puts the CG {S.LIP_EXT - ahead:.3f} m past the nominal release point)")
for name, lip in (("hud", ahead), ("short", 0.0)):
    tau = (v_rel - np.sqrt(max(v_rel ** 2 - 2 * gt * lip, 0))) / gt if lip > 0 else 0.0
    print(f"{name}: CG passes the lip {tau:.3f} s after release (t = {S.T_RELEASE + tau:.3f}); "
          f"free-fall gap then {0.5*gn*tau**2:.3f} m")
    tau_r = (v_rel - np.sqrt(max(v_rel ** 2 - 2 * gt * (lip + S.CAR_L / 2), 0))) / gt
    print(f"   rear bumper (L/2 = {S.CAR_L/2} m behind the CG) passes the lip at t = {S.T_RELEASE + tau_r:.3f}")
print(f"gap after release: 0.5 * {gn/G0:.3f} g * tau^2; reaches 0.5 m at tau = {np.sqrt(2*0.5/gn):.3f} s")

# ---------------- flight record: attitude, rates, moments, forces ----------------
t = F["t"]
Rs = np.array([S.R_from_quat(q) for q in F["quat"]])
phi = np.unwrap(np.arctan2(Rs[:, 2, 1], Rs[:, 2, 2]))
theta = -np.arcsin(np.clip(Rs[:, 2, 0], -1, 1))
q = F["quat"]
qd = np.gradient(q, t, axis=0)
wq = np.array([2 * np.array([a[0], -a[1], -a[2], -a[3]]) for a in q])   # conj, then product below


def qmul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1*w2 - x1*x2 - y1*y2 - z1*z2, w1*x2 + x1*w2 + y1*z2 - z1*y2,
                     w1*y2 - x1*z2 + y1*w2 + z1*x2, w1*z2 + x1*y2 - y1*x2 + z1*w2])


om = np.array([qmul(c, d)[1:] for c, d in zip(wq, qd)])            # body rates p, q, r
pdot = np.gradient(om[:, 0], t)
f = F["noz"] * S.N_FMAX0                                              # N, actual thrust per nozzle
M_jet = (S.ALLOC[3:] @ f.T).T                                         # body moment from all jets
M_lift_x = D_ARM * (f[:, 4] + f[:, 6] - f[:, 5] - f[:, 7])
M_front_x = (S.ALLOC[3, 8:] @ f[:, 8:].T)
gyro = (IYY - IZZ) * om[:, 1] * om[:, 2]
m = S.M_DRY + F["gas"]
v = np.gradient(F["pos"], t, axis=0)
vb = np.einsum("nji,nj->ni", Rs, v)
Fth_w = np.einsum("nij,nj->ni", Rs, (S.N_DIR.T @ f.T).T)
Fdr_b = -0.5 * S.RHO * S.CDA * np.abs(vb) * vb
Fdr_w = np.einsum("nij,nj->ni", Rs, Fdr_b)
sf = (Fth_w + Fdr_w) / m[:, None] / G0                               # specific force (g), no wheels before the burn
win = (t >= S.T_RELEASE) & (t < S.T_BURN)
roll = (t >= T0 - 0.05) & (t <= T1 + 0.1)

i90 = np.argmin(np.abs(phi - np.pi / 2) + 10 * (t < T0))
for tk in np.arange(52.30, 53.61, 0.05):
    k = np.argmin(np.abs(t - tk))
    uk = np.clip((tk - T0) / T, 0, 1)
    print(f"   t {tk:.2f}: phi {np.degrees(phi[k]):7.1f} (ref {180*(1-(10*uk**3-15*uk**4+6*uk**5)):6.1f}), "
          f"p {om[k, 0]:6.2f}, Mx {M_jet[k, 0]/1e3:6.1f} kN m, FR+RR {(f[k, 5]+f[k, 7])/1e3:5.1f}, "
          f"FL+RL {(f[k, 4]+f[k, 6])/1e3:5.1f} kN, theta {np.degrees(theta[k]):5.1f}")
izero = np.argmax((phi < 0) & (t > T0))
imin = np.argmin(phi + 10 * (t < T0) + 10 * (t > 53.6))
print(f"reference phi = 90 deg at {(T0 + T1) / 2:.3f} s; sim crosses 0 at {t[izero]:.2f} s, minimum "
      f"{np.degrees(phi[imin]):.1f} deg at {t[imin]:.2f} s; phi at 53.60 {np.degrees(np.interp(53.6, t, phi)):.1f} deg")
print(f"sim roll: phi = 90 deg at {t[i90]:.3f} s; phi at 53.10 = {np.degrees(np.interp(53.10, t, phi)):.2f} deg; "
      f"pitch at release {np.degrees(theta[0]):.2f}, at {T_PITCH:.2f} {np.degrees(np.interp(T_PITCH, t, theta)):.2f} deg")
print(f"sim roll rate peak {np.abs(om[roll, 0]).max():.2f} rad/s at {t[roll][np.argmax(np.abs(om[roll, 0]))]:.3f} s; "
      f"roll accel peak {np.abs(pdot[roll]).max():.1f} rad/s2")
ipk1 = np.argmax((f[:, 5] + f[:, 7]) * win)
ipk2 = np.argmax((f[:, 4] + f[:, 6]) * win)
print(f"pair FR+RR peak {(f[ipk1, 5] + f[ipk1, 7])/1e3:.1f} kN at {t[ipk1]:.3f} s "
      f"(FR {f[ipk1, 5]/1e3:.1f}, RR {f[ipk1, 7]/1e3:.1f}, limit {35*F['P'][ipk1]/F['P'][0]:.1f} kN); "
      f"phi {np.degrees(phi[ipk1]):.0f} deg")
print(f"pair FL+RL peak {(f[ipk2, 4] + f[ipk2, 6])/1e3:.1f} kN at {t[ipk2]:.3f} s "
      f"(FL {f[ipk2, 4]/1e3:.1f}, RL {f[ipk2, 6]/1e3:.1f}); phi {np.degrees(phi[ipk2]):.0f} deg")
print(f"jet roll moment peak {np.abs(M_jet[win, 0]).max()/1e3:.1f} kN m; I*pdot peak {IXX*np.abs(pdot[roll]).max()/1e3:.1f} kN m; "
      f"lift-only part peak {np.abs(M_lift_x[win]).max()/1e3:.1f}; front-nozzle part peak {np.abs(M_front_x[win]).max():.0f} N m")
print(f"gyroscopic (Iyy-Izz) q r peak {np.abs(gyro[win]).max():.1f} N m; q peak {np.abs(om[win, 1]).max():.3f}, "
      f"r peak {np.abs(om[win, 2]).max():.3f} rad/s")

# ---------------- analytic min-jerk roll ----------------
T = T1 - T0
uu = np.linspace(0, 1, 20001)
Sp = 30 * uu ** 2 - 60 * uu ** 3 + 30 * uu ** 4
Spp = 60 * uu - 180 * uu ** 2 + 120 * uu ** 3
pk_rate, pk_acc = np.pi * Sp.max() / T, np.pi * Spp.max() / T ** 2
M_need = IXX * pk_acc
print(f"min-jerk T = {T:.2f} s: S' max {Sp.max():.4f}, S'' max {Spp.max():.4f}; peak rate {pk_rate:.3f} rad/s, "
      f"peak accel {pk_acc:.2f} rad/s2; moment {M_need/1e3:.2f} kN m; per nozzle {M_need/(2*D_ARM)/1e3:.2f} kN "
      f"= {M_need/(2*D_ARM)/35e3*100:.0f}% of 35 kN; pair force {M_need/D_ARM/1e3:.2f} kN = {M_need/D_ARM/(M0*G0):.2f} g")
L_ang = IXX * pk_rate
print(f"angular impulse per half {L_ang:.0f} N m s; linear impulse per pair >= {L_ang/D_ARM:.0f} N s "
      f"= dv {L_ang/D_ARM/M0:.2f} m/s; both pairs, magnitudes summed: {2*L_ang/D_ARM/M0:.2f} m/s")
phi_r = np.pi * (1 - (10 * uu ** 3 - 15 * uu ** 4 + 6 * uu ** 5))
acc_r = np.pi * Spp / T ** 2
Jy = IXX / D_ARM * np.trapezoid(np.abs(acc_r) * np.sin(phi_r), uu * T)
Jz = IXX / D_ARM * np.trapezoid(np.abs(acc_r) * np.cos(phi_r), uu * T)
print(f"open-loop impulse from the minimum pair forces: J_y = {Jy:.0f} N s (dv_y {Jy/M0:.2f} m/s), J_z = {Jz:.1f} N s")
tt_ol = np.linspace(T0, S.T_BURN, 2000)
u_ol = np.clip((tt_ol - T0) / T, 0, 1)
a_ol = IXX / D_ARM / M0 * np.abs(np.interp(u_ol, uu, acc_r)) * np.sin(np.interp(u_ol, uu, phi_r)) * (tt_ol <= T1)
vy_ol = np.r_[0, np.cumsum((a_ol[1:] + a_ol[:-1]) / 2 * np.diff(tt_ol))]
y_ol = np.r_[0, np.cumsum((vy_ol[1:] + vy_ol[:-1]) / 2 * np.diff(tt_ol))]
print(f"open-loop lateral drift: vy {vy_ol[-1]:.2f} m/s, y at the burn {y_ol[-1]:.2f} m, at roll end "
      f"{np.interp(T1, tt_ol, y_ol):.2f} m")
print(f"sim lateral: max |y| {np.abs(F['pos'][win, 1]).max():.3f} m, max |vy| {np.abs(v[win, 1]).max():.3f} m/s, "
      f"y at burn {np.interp(S.T_BURN, t, F['pos'][:, 1]):.3f} m")
for I in (400, 650, 1042):
    print(f"   I_xx = {I}: per-nozzle peak {I*pk_acc/(2*D_ARM)/1e3:.2f} kN, pair specific force {I*pk_acc/D_ARM/(M0*G0):.2f} g, "
          f"dv_y open loop {I/IXX*Jy/M0:.2f} m/s")

# ---------------- G gauge ----------------
hsel = (ht >= T0) & (ht <= T1)
hsel2 = (ht > S.T_RELEASE) & (ht < S.T_BURN - 0.01)
print(f"HUD G in the roll {T0}-{T1}: {hg[hsel].min():.1f}-{hg[hsel].max():.1f} g; release->burn "
      f"{hg[hsel2].min():.1f}-{hg[hsel2].max():.1f} g; sim |sf| in roll {np.linalg.norm(sf[roll], axis=1).min():.2f}-"
      f"{np.linalg.norm(sf[roll], axis=1).max():.2f} g")
gmax = hg[hsel].max()
for Tx in (0.75, 0.85):
    print(f"any rest-to-rest half-roll in T = {Tx} s needs peak |accel| >= 4 pi / T^2 = {4*np.pi/Tx**2:.2f} rad/s2 "
          f"-> unfiltered gauge <= {gmax:.1f} g implies I_xx <= {gmax*G0*M0*D_ARM/(4*np.pi/Tx**2):.0f} kg m2")
print(f"if the gauge is unfiltered |specific force|: I_xx <= {gmax*G0*M0*D_ARM/pk_acc:.0f} kg m2 "
      f"(k = {np.sqrt(gmax*G0*M0*D_ARM/pk_acc/M0):.3f} m)")

# ---------------- gas cost ----------------
gas_used = F["gas"][np.argmin(np.abs(t - S.T_RELEASE))] - np.interp(S.T_BURN, t, F["gas"])
imp = np.trapezoid(f[win], t[win], axis=0)
share = {"rear": imp[:4].sum(), "lift": imp[4:8].sum(), "front": imp[8:].sum()}
tot = sum(share.values())
print(f"gas release->burn {gas_used:.1f} kg; thrust impulse shares " +
      ", ".join(f"{k} {v/1e3:.1f} kN s ({v/tot*100:.0f}%)" for k, v in share.items()))

# ---------------- aerodynamics on the sim trajectory ----------------
V = np.linalg.norm(v, axis=1)
qdyn = 0.5 * S.RHO * V ** 2
print(f"airspeed: release {V[0]:.2f} m/s, q {qdyn[0]:.1f} Pa; at burn {np.interp(S.T_BURN, t, V):.2f} m/s, "
      f"q {np.interp(S.T_BURN, t, qdyn):.1f} Pa")
Fd = np.linalg.norm(Fdr_w, axis=1)
print(f"drag in window: max {Fd[win].max():.0f} N = {Fd[win].max()/(M0*G0)*100:.2f}% of weight at "
      f"{t[win][np.argmax(Fd[win])]:.2f} s; during roll max {Fd[roll].max():.0f} N")
CDz = S.CDA[2] / (S.CAR_L * S.CAR_W)
CDy = S.CDA[1] / (S.CAR_L * S.CAR_H)
kq = 0.5 * S.RHO * S.CAR_L * (CDz * (S.CAR_W / 2) ** 4 / 2 + CDy * (S.CAR_H / 2) ** 4 / 2)
AR = S.CAR_W / S.CAR_L
CLa = np.pi * AR / 2
kl = 0.5 * S.RHO * CLa * S.CAR_L * 2 * (S.CAR_W / 2) ** 3 / 3
p = om[:, 0]
Mq, Ml = kq * p * np.abs(p), kl * V * p
print(f"roll damping: C_Dz {CDz:.3f}, C_Dy {CDy:.3f}, k_quad {kq:.3f} N m s2; AR {AR:.3f}, C_La {CLa:.3f}, "
      f"k_lin {kl:.3f} N s2; peaks {np.abs(Mq[roll]).max():.0f} and {np.abs(Ml[roll]).max():.0f} N m "
      f"= {np.abs(Mq[roll]).max()/np.abs(M_jet[win, 0]).max()*100:.2f}% and "
      f"{np.abs(Ml[roll]).max()/np.abs(M_jet[win, 0]).max()*100:.2f}% of the jet peak")
# roll angle lost to damping if uncompensated: integrate -M/I twice over the roll window
a_d = -(Mq + Ml)[roll] / IXX
w_d = np.r_[0, np.cumsum((a_d[1:] + a_d[:-1]) / 2 * np.diff(t[roll]))]
ang_d = np.r_[0, np.cumsum((w_d[1:] + w_d[:-1]) / 2 * np.diff(t[roll]))]
print(f"uncompensated aero damping would change the roll by {np.degrees(ang_d[-1]):.2f} deg over the roll")
M_cp = 0.1 * Fd
print(f"aero moment per 0.1 m of centre-of-pressure offset: max {M_cp[win].max():.0f} N m in the window "
      f"(pitch: {M_cp[win].max()/IYY:.3f} rad/s2)")
print(f"jet / flight speed ratio {V[win].max()/(S.CF*430.7):.4f}")

# jets: exit diameter
g_ = S.GAMMA
npr = F["P"][0] * 1e5 / 101325
Me = np.sqrt(2 / (g_ - 1) * (npr ** ((g_ - 1) / g_) - 1))
eps = 1 / Me * (2 / (g_ + 1) * (1 + (g_ - 1) / 2 * Me ** 2)) ** ((g_ + 1) / (2 * (g_ - 1)))
d_t = 2 * np.sqrt(35e3 / (S.CF * F["P"][0] * 1e5) / np.pi)
d_e = d_t * np.sqrt(eps)
print(f"lift nozzle: NPR {npr:.0f}, ideal exit Mach {Me:.2f}, area ratio {eps:.1f}, throat {d_t*1e3:.1f} mm, "
      f"ideal exit {d_e*1e3:.0f} mm")
npr_lo = np.interp(53.05, t, F["P"]) * 1e5 / 101325
print(f"NPR {npr_lo:.0f}-{npr:.0f} against the 1985 suckdown data at NPR 1.45-2.4: "
      f"{npr_lo/2.4:.0f} to {npr/1.45:.0f} times larger")
Spl = S.CAR_L * S.CAR_W
for d in (d_t, d_e):
    Aj = 2 * np.pi * d ** 2 / 4
    for P_ in (F["P"][0], np.interp(53.05, t, F["P"])):
        nprx = P_ * 1e5 / 101325
        dL = -0.0001 * np.sqrt(Spl / Aj) * (2 * np.pi * d / (np.sqrt(2) * d)) ** 1.58 * nprx ** -0.5
        print(f"   OGE lift loss, two jets d = {d*1e3:.0f} mm, NPR {nprx:.0f}: dL/T = {dL*100:.3f}% "
              f"-> {dL*24e3:.0f} N on a 24 kN pair")
D_eff = 2 * np.sqrt(Spl / np.pi)
print(f"weight over plan area {M0*G0/Spl:.0f} Pa")
print(f"planform {Spl:.2f} m2, Wyatt data to h/(D-d) = 1.5 -> h <= {1.5*(2*np.sqrt(Spl/np.pi)-d_e):.2f} m")
print(f"planform {Spl:.2f} m2, equal-area diameter {D_eff:.2f} m; Wyatt valid h/(D-d) 0.17-1.6 -> "
      f"h {0.17*(D_eff-d_e):.2f}-{1.6*(D_eff-d_e):.2f} m")

# ceiling geometry: surface line of the straight 170 deg part, both lips
on = W["s"] >= 0
nx, nz = -np.sin(W["th"][on]), np.cos(W["th"][on])
sx = W["x"][on] - S.H_CG * nx
sz = W["z"][on] + S.H_CG - S.H_CG * nz
ceil = W["th"][on] >= np.radians(169.9)
cx, cz = sx[ceil], sz[ceil]
tc = np.array([np.cos(th), np.sin(th)])
nc = np.array([-np.sin(th), np.cos(th)])                 # from the surface toward the car
x_lip = {"hud": cx[-1], "short": P1["pos"][-1][0] + S.H_CG * np.sin(th)}   # short: lip at the release CG
print(f"ceiling surface from x = {cx[0]:.2f} to lip x = {x_lip['hud']:.2f} (hud), {x_lip['short']:.2f} (short)")
corners = F["corners"]
gap = np.full(t.size, np.nan)
for k in range(t.size):
    w_ = F["pos"][k][:, None] + Rs[k] @ corners.T
    gap[k] = np.min((w_[0] - cx[0]) * nc[0] + (w_[2] - cz[0]) * nc[1])  # distance below the ceiling plane
early = (t >= S.T_RELEASE) & (t <= S.T_RELEASE + 0.3)
print(f"gap minimum in the first 0.3 s (the self-check skips it): {gap[early].min():.3f} m at {t[early][np.argmin(gap[early])]:.3f} s")
print(f"sign check: world y of body z at phi = 90 deg (t {t[i90]:.2f}) = {Rs[i90][1, 2]:+.3f}; sim v_y peak "
      f"{v[win, 1].max():+.3f} m/s at {t[win][np.argmax(v[win, 1])]:.2f} s; min {v[win, 1].min():+.3f} at {t[win][np.argmin(v[win, 1])]:.2f}")
print(f"body-to-ceiling-plane gap: release {gap[0]:.3f} m, 52.35 {np.interp(52.35, t, gap):.3f}, "
      f"52.45 {np.interp(52.45, t, gap):.3f}, 52.55 {np.interp(52.55, t, gap):.3f} m")
nozzle_hits = []
for k in np.where(win & (t <= T1 + 0.05))[0]:
    for i in range(4, 8):
        if f[k, i] < 500:
            continue
        pn = F["pos"][k] + Rs[k] @ S.N_POS[i]
        ex = -Rs[k] @ S.N_DIR[i]                                  # exhaust direction, world
        den = ex[0] * nc[0] + ex[2] * nc[1]
        if den >= 0:
            continue                                              # exhaust points away from the ceiling plane
        dist = -((pn[0] - cx[0]) * nc[0] + (pn[2] - cz[0]) * nc[1]) / den
        hit = pn + dist * ex
        for name in ("hud", "short"):
            if hit[0] >= x_lip[name] and abs(hit[1]) < S.RAMP_WIDTH / 2:
                nozzle_hits.append((name, t[k], S.NOZZLES[i][0], f[k, i], dist))
for name in ("hud", "short"):
    h = [r for r in nozzle_hits if r[0] == name]
    if h:
        print(f"{name}: lift exhaust axis hits the ceiling from {h[0][1]:.2f} to {h[-1][1]:.2f} s, "
              f"{len(h)} nozzle-samples, max thrust {max(r[3] for r in h)/1e3:.1f} kN, distances "
              f"{min(r[4] for r in h):.2f}-{max(r[4] for r in h):.2f} m, nozzles {sorted(set(r[2] for r in h))}")
    else:
        print(f"{name}: no lift exhaust axis meets the ceiling after release")
hk = np.array(sorted({round(r[1], 3) for r in nozzle_hits if r[0] == "hud"}))
under = (t >= hk.min() - 1e-9) & (t <= hk.max() + 1e-9)
J_rr = np.trapezoid(f[under, 7], t[under])
print(f"RR thrust impulse while its axis meets the ceiling (hud): {J_rr:.0f} N s = dv {J_rr/M0:.3f} m/s; "
      f"roll part {D_ARM*J_rr:.0f} N m s = {D_ARM*J_rr/(IXX*pk_rate)*100:.1f}% of one half-roll's angular impulse")
for hh in (0.57, 0.12, 0.49):
    print(f"   Wyatt dL/T = -0.012 (h/(D-d))^-2.3 at h = {hh} m: {-0.012*(hh/(D_eff-d_e))**-2.3:.2f}"
          + ("" if hh >= 0.57 else " (outside the fitted range)"))
k40 = np.argmin(np.abs(t - 52.40))
x_rr = (F["pos"][k40] + Rs[k40] @ S.N_POS[7])[0]
print(f"RR nozzle at 52.40 s: x = {x_rr:.2f} m, {x_rr - x_lip['hud']:.2f} m inside the hud lip")
fire = win & ((f[:, 4:8] > 500).any(axis=1))
print(f"first lift-nozzle thrust > 0.5 kN after release at {t[fire][0]:.3f} s (phi {np.degrees(phi[fire][0]):.1f} deg), "
      f"CG x {F['pos'][fire][0][0]:.2f} m")

# ---------------- term sizes ----------------
terms = [
    ("aero drag, peak (in sim)", Fd[win].max() / (M0 * G0) * 100, "force"),
    ("jet-induced lift loss, free air (not in sim)", 0.0001 * np.sqrt(Spl / (2 * np.pi * d_e ** 2 / 4)) * np.pi ** 1.58 * 2 ** (1.58 / 2) /
     np.sqrt(npr) * M_need / D_ARM / (M0 * G0) * 100, "force"),
    ("gyroscopic (Iyy - Izz) q r (in sim)", np.abs(gyro[win]).max() / np.abs(M_jet[win, 0]).max() * 100, "moment"),
    ("aero roll damping, cross-flow (not in sim)", np.abs(Mq[roll]).max() / np.abs(M_jet[win, 0]).max() * 100, "moment"),
    ("aero roll damping, slender wing (not in sim)", np.abs(Ml[roll]).max() / np.abs(M_jet[win, 0]).max() * 100, "moment"),
    ("aero moment per 0.1 m CP offset (not in sim)", M_cp[win].max() / np.abs(M_jet[win, 0]).max() * 100, "moment"),
    ("front-nozzle roll arm (in sim)", np.abs(M_front_x[win]).max() / np.abs(M_jet[win, 0]).max() * 100, "moment"),
]
for n_, v_, k_ in terms:
    print(f"term {n_}: {v_:.3f}% of {'weight' if k_ == 'force' else 'peak jet roll moment'}")

# ---------------- charts ----------------
EV = ((T170, "170°"), (S.T_RELEASE, "release"), (T0, "roll"), (T1, "upright"), (S.T_BURN, "burn"))


def events(ax, labels=True):
    lo, hi = ax.get_xlim()
    y0, y1 = ax.get_ylim()
    for j, (x_, s_) in enumerate(EV):
        if not lo <= x_ <= hi:
            continue
        ax.axvline(x_, color=MUTED, lw=1, ls=":")
        if labels:
            ax.text(x_ + 0.02, y1 - (0.09 * (y1 - y0) if s_ == "roll" else 0), s_, color=MUTED, va="top", fontsize=8)


tw = P1["t"] >= T170 - 0.05
XL = (T170 - 0.05, S.T_BURN + 0.05)
XR = (S.T_RELEASE - 0.05, S.T_BURN + 0.05)
# 1. roll kinematics
fig, axs = plt.subplots(3, 1, figsize=(8, 7), sharex=True)
tr = np.linspace(T170, S.T_BURN, 800)
ur = np.clip((tr - T0) / T, 0, 1)
axs[0].plot(t, np.degrees(phi), color=SIM, label="sim")
axs[0].plot(tr, 180 * (1 - (10 * ur ** 3 - 15 * ur ** 4 + 6 * ur ** 5)), color=C3, ls="--", lw=1.5,
            label="min-jerk reference")
axs[0].plot(P1["t"][tw], np.full(tw.sum(), 180), color=SIM)
axs[0].plot([52.6, 52.7], [90, 90], color=HUD, lw=6, solid_capstyle="butt", label="video: on its side")
axs[0].plot([53.1], [0], "o", ms=8, color=HUD, mec=SURF, mew=2, label="video: upright")
axs[0].set(ylabel="roll φ (deg), 180 = inverted", ylim=(-45, 200), yticks=[-45, 0, 45, 90, 135, 180],
           title="The half-roll: angle, rate, acceleration")
axs[0].legend(loc="lower left", fontsize=8)
axs[1].plot(t, om[:, 0], color=SIM)
axs[1].plot(tr, -np.pi * np.interp(ur, uu, Sp) / T, color=C3, ls="--", lw=1.5)
axs[1].set(ylabel="roll rate p (rad/s)")
axs[2].plot(t, pdot, color=SIM)
axs[2].plot(tr, -np.pi * np.interp(ur, uu, Spp) / T ** 2 * ((tr >= T0) & (tr <= T1)), color=C3, ls="--", lw=1.5)
axs[2].set(ylabel="roll accel ṗ (rad/s²)", xlabel="time (s)", xlim=XL)
for a in axs:
    events(a, a is axs[0])
save(fig, "flip_roll.png")

# 2. moment budget
fig, (a1, a2) = plt.subplots(2, 1, figsize=(8, 6.4), sharex=True)
a1.plot(t, M_jet[:, 0] / 1e3, color=SIM, label="jet roll moment Mₓ")
a1.plot(t, IXX * pdot / 1e3, color=C3, ls="--", lw=1.5, label="Iₓₓ ṗ (from the attitude record)")
a1.set(ylabel="roll moment (kN·m)", title="Roll moment budget", xlim=XR)
a1.legend(loc="lower right", fontsize=8)
a2.semilogy(t[win], np.abs(M_jet[win, 0]) + 1e-3, color=SIM, label="jets (in sim)")
a2.semilogy(t[win], np.abs(Mq[win] + Ml[win]) + 1e-3, color=HUD, label="aero roll damping (not in sim)")
a2.semilogy(t[win], np.abs(gyro[win]) + 1e-3, color=C3, label="gyroscopic (Iyy−Izz)qr (in sim)")
a2.semilogy(t[win], M_cp[win], color=C4, label="aero, per 0.1 m CP offset (not in sim)")
a2.set(ylabel="|moment| (N·m), log", ylim=(0.1, 3e6), xlabel="time (s)", xlim=XR)
a2.legend(loc="upper center", fontsize=8, ncol=2)
for a in (a1, a2):
    events(a, a is a1)
save(fig, "flip_moments.png")

# 3. forces
fig, axs = plt.subplots(3, 1, figsize=(8, 7.6), sharex=True)
for c, i in zip((SIM, HUD, C3, C4), (4, 5, 6, 7)):
    axs[0].plot(t, f[:, i] / 1e3, color=c, lw=1.5, label=S.NOZZLES[i][0])
axs[0].plot(t, 35 * F["P"] / F["P"][0], color=MUTED, lw=1, ls="--", label="limit P/P₀ × 35 kN")
axs[0].set(ylabel="lift thrust (kN)", title="Forces in the flip: lift jets, G gauge, sideways speed", ylim=(-1, 52))
axs[0].legend(loc="upper center", ncol=5, fontsize=8, bbox_to_anchor=(0.55, 0.80))
axs[1].plot(t, np.linalg.norm(sf, axis=1), color=SIM, label="sim |specific force|")
hs = (ht >= T170 - 0.05) & (ht <= S.T_BURN + 0.05)
axs[1].plot(ht[hs], hg[hs], "o", ms=4, color=HUD, label="HUD G-FORCE")
axs[1].plot(P1["t"][tw], P1["g"][tw], color=SIM)
axs[1].set(ylabel="G", ylim=(-0.1, 3.4))
axs[1].legend(loc="upper left", fontsize=8)
axs[2].plot(tt_ol, vy_ol, color=C3, ls="--", lw=1.5, label="open loop: roll pairs only, no sideways control")
axs[2].plot(t, v[:, 1], color=SIM, label="sim (controller holds y = 0)")
axs[2].set(ylabel="sideways speed v_y (m/s)", xlabel="time (s)", xlim=XL)
axs[2].legend(loc="upper left", fontsize=8)
for a in axs:
    events(a, a is axs[0])
save(fig, "flip_forces.png")

# 4. term sizes
fig, ax = plt.subplots(figsize=(8, 3.8))
names = [n_ for n_, _, _ in terms][::-1]
vals = [v_ for _, v_, _ in terms][::-1]
cols = [SIM if k_ == "force" else C3 for _, _, k_ in terms][::-1]
ax.barh(names, vals, color=cols, height=0.6)
for y_, v_ in enumerate(vals):
    ax.text(v_ * 1.15, y_, f"{v_:.2g}%", va="center", color=INK, fontsize=9)
ax.set_xscale("log")
ax.set(xlim=(1e-3, 30), xlabel="peak size (log). Teal: % of weight. Purple: % of the peak jet "
       "roll moment", title="Small terms in the flip window, on the sim's own path")
ax.xaxis.label.set_size(8.5)
ax.grid(axis="y", visible=False)
save(fig, "flip_terms.png")

# 5. rear-view filmstrip: body (y, z) -> world (y, z) through the sim's own rotation matrix
HB, HT = -S.H_CG + 0.12, S.CAR_H - S.H_CG
BODY = np.array([[-1.0, HB], [1.0, HB], [1.0, 0.25], [0.85, 0.27], [0.55, HT], [-0.55, HT], [-0.85, 0.27],
                 [-1.0, 0.25]])
TYRES = [np.array([[y0 - 0.15, -(S.H_CG + 0.072)], [y0 + 0.15, -(S.H_CG + 0.072)], [y0 + 0.15, 0.14],
                   [y0 - 0.15, 0.14]]) for y0 in (-S.TRACK / 2, S.TRACK / 2)]
times = [52.20, T0, t[ipk1], t[i90], t[ipk2], T1, T_PITCH, 53.90]
fig, axs = plt.subplots(2, 4, figsize=(11, 6.4))
for ax, tk in zip(axs.flat, times):
    ax.set(xlim=(-2.3, 2.3), ylim=(-2.0, 2.6), aspect="equal", xticks=[], yticks=[])
    ax.grid(False)
    if tk < S.T_RELEASE:
        k, ph, rate, gp = None, np.pi, 0.0, 0.12
        Rk = S.frame_from_tn(P1["t_hat"][-1], P1["n_hat"][-1])
        pc = np.interp(tk, P1["t"], P1["pos"][:, 0])
    else:
        k = np.argmin(np.abs(t - tk))
        ph, rate, gp, Rk, pc = phi[k], om[k, 0], gap[k], Rs[k], F["pos"][k][0]

    def to_w(pts, R_=Rk):
        return (R_ @ np.column_stack([np.zeros(len(pts)), pts]).T).T[:, 1:]
    if pc >= x_lip["hud"]:
        top = to_w(BODY)[:, 1].max()
        ax.add_patch(Rectangle((-S.RAMP_WIDTH / 2, top + gp), S.RAMP_WIDTH, 0.25, color="#64748b"))
        ax.text(0, top + gp + 0.32, "ceiling" if pc >= x_lip["short"] else "ceiling (hud only)", color=MUTED,
                ha="center", fontsize=8)
    else:
        ax.text(0, 2.35, "CG past the lip", color=MUTED, ha="center", fontsize=8)
    for ty in TYRES:
        ax.add_patch(Polygon(to_w(ty), closed=True, fc="#0b1220", ec=MUTED, lw=0.8))
    ax.add_patch(Polygon(to_w(BODY), closed=True, fc="#475569", ec=INK, lw=1))
    ax.plot(*to_w(np.array([[0, HT]]))[0], "o", ms=4, color=INK)
    ax.plot(0, 0, "+", ms=9, color=INK, mew=1.5)
    if k is not None:
        dirw = to_w(np.array([[0, 1.0]]))[0]
        for side, (ia, ib), c in ((-1, (5, 7), HUD), (1, (4, 6), C3)):
            fsum = f[k, ia] + f[k, ib]
            if fsum < 300:
                continue
            base = to_w(np.array([[side * D_ARM, -0.20]]))[0]
            Lr = fsum / 24e3 * 1.3
            ax.plot(*np.array([base, base - dirw * Lr * 0.9]).T, color="#38bdf8", lw=5, alpha=0.35,
                    solid_capstyle="round")
            ax.annotate("", xy=base + dirw * Lr, xytext=base,
                        arrowprops=dict(arrowstyle="-|>", color=c, lw=2.2, mutation_scale=14))
            ax.text(*(base + dirw * (Lr + 0.3)), f"{fsum/1e3:.0f} kN", color=INK, fontsize=8, ha="center",
                    va="center")
    ax.annotate("", xy=(-1.95, -1.85), xytext=(-1.95, -1.15), arrowprops=dict(arrowstyle="-|>", color=MUTED, lw=1.5))
    ax.text(-1.83, -1.55, "g", color=MUTED, fontsize=9)
    if k == i90:
        a_, b_ = to_w(np.array([[0, -0.20], [-D_ARM, -0.20]]))
        ax.plot(*np.array([a_, b_]).T, color=INK, lw=1.2)
        ax.text(-1.2, 1.75, "d = 0.85 m", color=INK, fontsize=8, ha="center")
        ax.plot([-1.2, (a_[0] + b_[0]) / 2], [1.68, (a_[1] + b_[1]) / 2], color=MUTED, lw=0.6)
    ax.set_title(f"{tk:.2f} s   φ = {np.degrees(ph):.0f}°\np = {rate:.1f} rad/s", fontsize=9, color=INK)
fig.suptitle("Rear view of the half-roll (screen right = +y). Arrows: lift-jet thrust per side; blue: exhaust; "
             "dot: roof", fontsize=10, color=INK)
save(fig, "flip_film.png")

# 6. side view under the ceiling
fig, ax = plt.subplots(figsize=(9, 4.4))
seg = sx < x_lip["hud"] + 9
ax.plot(sx[seg], sz[seg], color=MUTED, lw=4, solid_capstyle="butt")
ax.text(-5.2, 21.35, "ceiling surface, 170°", color=MUTED, fontsize=8, rotation=-9)
for name, mk, c in (("hud", "hud lip", HUD), ("short", "short lip", C4)):
    zl = np.interp(x_lip[name], cx[::-1], cz[::-1])
    ax.plot(x_lip[name], zl, "v", ms=9, color=c, mec=SURF, mew=1.5)
    ax.annotate(mk, (x_lip[name], zl), xytext=(0, 10), textcoords="offset points", ha="center", color=INK, fontsize=8)
for j, tk in enumerate((52.25, 52.40, 52.55, 52.70)):
    k = np.argmin(np.abs(t - tk))
    w_ = (F["pos"][k][:, None] + Rs[k] @ corners.T)[[0, 2]].T
    hull = ConvexHull(w_)
    ax.add_patch(Polygon(w_[hull.vertices], closed=True, fc="none", ec=SIM, lw=1.4))
    ax.plot(F["pos"][k][0], F["pos"][k][2], "o", ms=4, color=SIM)
    ax.text(F["pos"][k][0], F["pos"][k][2] - 0.12, f"{tk:.2f} s", color=INK, fontsize=8, ha="center", va="top")
    for i in range(4, 8):
        if f[k, i] > 2e3:
            pn = F["pos"][k] + Rs[k] @ S.N_POS[i]
            ex = -Rs[k] @ S.N_DIR[i]
            Lx = 0.4 + 1.2 * f[k, i] / 15e3
            ax.plot([pn[0], pn[0] + Lx * ex[0]], [pn[2], pn[2] + Lx * ex[2]], color="#38bdf8", lw=3, alpha=0.6,
                    solid_capstyle="round")
sel_ = win & (t < 52.9)
ax.plot(F["pos"][sel_, 0], F["pos"][sel_, 2], color=SIM, lw=1, ls=":")
ax.set(xlabel="x (m)", ylabel="z (m)", aspect="equal", xlim=(-16.2, -3.0), ylim=(18.9, 22.5),
       title="Side view: leaving the ceiling. Outlines every 0.15 s, dot = CG; blue: lift exhaust over 2 kN")
save(fig, "flip_side.png")
