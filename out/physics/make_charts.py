"""Charts and derived numbers for PHYSICS.html. Run: python out/physics/make_charts.py (about 30 s).

Imports sim/wall_flip_sim.py in-process (same code and data that write out/replay_data.js), so the wall-phase
speed/suction/power are available. Prints every derived number the report quotes.
"""
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import least_squares

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "sim"))
sys.argv = sys.argv[:1]                      # default "hud" overhang
import wall_flip_sim as S                     # noqa: E402

OUT = Path(__file__).resolve().parent
SIM, HUD, C3, C4 = "#0d9488", "#ea580c", "#8b5cf6", "#ec4899"   # validated: dark, surface #1e293b
SURF, INK, MUTED, GRID = "#1e293b", "#cbd5e1", "#94a3b8", "#334155"
plt.rcParams.update({"figure.facecolor": SURF, "axes.facecolor": SURF, "savefig.facecolor": SURF,
                     "text.color": INK, "axes.labelcolor": INK, "xtick.color": MUTED, "ytick.color": MUTED,
                     "axes.edgecolor": GRID, "grid.color": GRID, "axes.grid": True, "grid.linewidth": 0.6,
                     "lines.linewidth": 2, "font.size": 10, "legend.frameon": False,
                     "axes.spines.top": False, "axes.spines.right": False})


def save(fig, name):
    fig.tight_layout()
    fig.savefig(OUT / name, dpi=130)
    plt.close(fig)
    print("wrote", OUT / name)


def vlines(ax, labels=True):
    for t, txt in ((S.T_RELEASE, "release"), (S.T_BURN, "burn")):
        ax.axvline(t, color=MUTED, lw=1, ls=":")
        if labels:
            ax.text(t + 0.05, ax.get_ylim()[1], txt, color=MUTED, va="top", fontsize=8)


# ---------------- run the model ----------------
W = S.build_wall()
m0 = S.M_DRY + S.GAS0
P1 = S.wall_phase(W, m0)
F = S.flight_phase(P1)
_, surf = S.clearance_to_wall(W, F, F["corners"])
hud = S.load("hud_wall_flip.csv")
ht, hg, ha = hud["video_t"], hud["g_force"], hud["altitude_ft"]
prof = S.load("wall_profile_hud.csv")

# ---------------- derived numbers ----------------
tank = S.Tank()
P0 = tank.P0
v690 = S.R_GAS * S.T_GAS0 / 690e5
print(f"P0 = {P0/1e5:.1f} bar; Z(690 bar, 293 K) = {(v690 + S.B_ABEL)/v690:.3f}; rho0 = {S.GAS0/S.V_TANK:.1f} kg/m3")
print(f"Gamma = {S.GAMMA_FN:.4f}; c* = {tank.cstar():.1f} m/s; c_eff = Cf c* = {S.CF*tank.cstar():.1f} m/s")
for name, f in (("rear", 20e3), ("lift", 35e3), ("front", 15e3)):
    A = f / (S.CF * P0)
    print(f"throat {name}: A* = {A*1e6:.0f} mm2, d = {2*np.sqrt(A/np.pi)*1e3:.1f} mm; mdot at full = {f/(S.CF*tank.cstar()):.1f} kg/s")
print(f"sum Fmax0 = {S.N_FMAX0.sum()/1e3:.0f} kN; lift sum = {S.N_FMAX0[4:8].sum()/1e3:.0f} kN = {S.N_FMAX0[4:8].sum()/(m0*S.G0):.2f} g at full tank")
vrel = P1["v"][-1] * P1["t_hat"][-1]
print(f"release v = {P1['v'][-1]:.3f} m/s -> vx {vrel[0]:.3f}, vz {vrel[2]:.3f}")
box = m0 / 12 * np.array([S.CAR_W**2 + S.CAR_H**2, S.CAR_L**2 + S.CAR_H**2, S.CAR_L**2 + S.CAR_W**2])
print("uniform box inertia", np.round(box), "ratio", np.round(np.diag(S.INERTIA) / box, 2))
print(f"entry radius R = {W['R']:.3f} m; launch a = {np.sqrt(1.1**2-1):.3f} g; static deflection = {m0*S.G0/(4*S.K_SUSP)*1e3:.1f} mm")
print(f"suspension f_n = {np.sqrt(S.K_SUSP/(m0/4))/(2*np.pi):.2f} Hz, zeta = {S.C_SUSP/(2*np.sqrt(S.K_SUSP*m0/4)):.2f}")
i170 = np.argmax(P1["th"] >= np.radians(169.9))
t170 = P1["t"][i170]
print(f"wall reaches 170 deg at t = {t170:.2f} s, s = {P1['s'][i170]:.2f} m")
pre = P1["t"] < t170
print(f"before 170: peak speed {P1['v'][pre].max():.2f} m/s, peak power {P1['power'][pre].max()/1e3:.0f} kW, "
      f"peak suction {P1['F_s_min'][pre].max()/S.G0:.0f} kgf at {P1['t'][pre][np.argmax(P1['F_s_min'][pre])]:.2f} s, "
      f"peak tyre {np.abs(P1['F_t'][pre]).max()/1e3:.1f} kN")
ipk = np.argmax(P1["F_s_min"])
print(f"overall peak suction {P1['F_s_min'][ipk]/S.G0:.0f} kgf at {P1['t'][ipk]:.2f} s (v {P1['v'][ipk]:.2f}, a {P1['a'][ipk]:.2f})")
print(f"0.1 ft step on 170 deg = {0.1*S.FT/np.sin(np.radians(10)):.3f} m of track")

# HUD lag: free-fall fit (piecewise: straight 170 deg climb before release, parabola after)
sel = (ht >= 52.2 - 1e-9) & (ht <= 53.95 + 1e-9)
tf, hf = ht[sel], ha[sel] * S.FT


def ff(p, t):
    z0, vz, lag = p
    tau = t - lag - S.T_RELEASE
    return z0 + vz * tau - 0.5 * S.G0 * np.maximum(tau, 0) ** 2


fit = least_squares(lambda p: ff(p, tf) - hf, [20, 1, 0.3])
print(f"free-fall fit: z0 {fit.x[0]:.3f} m, vz0 {fit.x[1]:.3f} m/s, lag {fit.x[2]:.3f} s, "
      f"rms {np.sqrt(np.mean(fit.fun**2))/S.FT:.3f} ft, n = {sel.sum()}")


# ---------------- sensitivity (current code) ----------------
def metrics(P1x, Wx=None):
    Fx = S.flight_phase(P1x)
    s2 = (ht > S.T_RELEASE + S.HUD_LAG) & (ht <= 57.2)
    rmse = np.sqrt(np.mean((np.interp(ht[s2] - S.HUD_LAG, Fx["t"], Fx["alt"]) - ha[s2]) ** 2))
    i = np.argmax(Fx["contact"] > 0.5 * m0 * S.G0)
    out = (f"touchdown {Fx['t'][i]:.2f} s, alt rmse {rmse:.2f} ft, apex {Fx['alt'].max():.2f} ft, "
           f"gas {S.GAS0 - Fx['gas'][-1]:.1f} kg, P_end {Fx['P'][-1]:.1f} bar, peak G {Fx['g'].max():.2f}")
    if Wx is not None:
        out += f", clearance {S.clearance_to_wall(Wx, Fx, Fx['corners'])[0]:.2f} m"
    return out


for n in (1.4, 1.25, 1.1):
    S.POLY_N = n
    print(f"n = {n}: {metrics(P1)}")
S.POLY_N = 1.25
f0 = S.N_FMAX0.copy()
S.N_FMAX0[4:8], S.N_FMAX0[8:] = 20e3, 8e3
print(f"first-guess nozzles (lift 20 kN, front 8 kN): {metrics(P1)}")
S.N_FMAX0[:] = f0
S.LIP_EXT = 0.0
Ws = S.build_wall()
P1s = S.wall_phase(Ws, m0)
ext = Ws["x"][Ws["s"] >= 0].max() - Ws["x"][Ws["s"] >= 0].min()
print(f"short overhang: extent {ext:.2f} m, release x {P1s['pos'][-1][0]:.2f} m, {metrics(P1s, Ws)}")
S.LIP_EXT = 1.5

# ---------------- charts ----------------
# 1. wall profile
fig, ax = plt.subplots(figsize=(9, 4.4))
on = W["s"] >= 0
ax.plot(W["x"][W["s"] <= 0], W["z"][W["s"] <= 0] + S.H_CG, color=MUTED, lw=1.5)
ax.plot(W["x"][on], W["z"][on] + S.H_CG, color=SIM, label="CG path (rebuilt from the HUD)")
ax.plot(surf[:, 0], surf[:, 1], color=HUD, lw=1.5, label="wall surface (CG path offset by 0.45 m)")
for deg in (30, 60, 90, 120, 150, 170):
    k = np.argmax(W["th"] >= np.radians(deg) - 1e-9)
    ax.plot(W["x"][k], W["z"][k] + S.H_CG, "o", ms=8, color=SIM, mec=SURF, mew=2)
    ax.annotate(f"{deg}°", (W["x"][k], W["z"][k] + S.H_CG), xytext=(8, -4), textcoords="offset points",
                color=INK, fontsize=9)
xr, zr = P1["pos"][-1][0], P1["pos"][-1][2]
ax.plot(xr, zr, "o", ms=9, color=C4, mec=SURF, mew=2)
ax.annotate("release 52.25 s", (xr, zr), xytext=(-10, -16), textcoords="offset points", color=INK, ha="right")
ax.set(xlabel="x (m), 0 = foot of the curve", ylabel="z (m) above the road", aspect="equal",
       title="Wall profile: CG path and surface, pitch marked")
ax.legend(loc="center left")
save(fig, "wall_profile.png")

# 2. altitude
fig, ax = plt.subplots(figsize=(8, 4.6))
ax.plot(ht, ha, "o", ms=3.5, color=HUD, label="HUD altitude as read (lags)")
ax.plot(ht - S.HUD_LAG, ha, color=HUD, lw=1, alpha=0.6, label="HUD shifted −0.30 s")
tw = P1["t"]
ax.plot(tw, (P1["pos"][:, 2] - S.H_CG) / S.FT, color=SIM, label="sim (wall + flight)")
ax.plot(F["t"], F["alt"], color=SIM)
tt = np.linspace(52.2, 53.95, 100)
ax.plot(tt - fit.x[2], ff(fit.x, tt) / S.FT, color=C3, ls="--", label="free-fall fit, lag removed")
ax.set(xlim=(44, 58), xlabel="time (s, VideoFromSpace clock)", ylabel="altitude (ft)",
       title="CG altitude: sim against the HUD")
vlines(ax)
ax.legend(loc="upper left")
save(fig, "altitude.png")

# 3. G
fig, ax = plt.subplots(figsize=(8, 4.2))
ax.plot(ht, hg, "o-", ms=3, lw=1, color=HUD, label="HUD G-FORCE")
ax.plot(P1["t"], P1["g"], color=SIM, label="sim specific force |a − g| / g₀")
ax.plot(F["t"], F["g"], color=SIM)
ax.set(xlim=(44, 58), xlabel="time (s)", ylabel="G", title="Accelerometer reading: sim against the HUD")
vlines(ax)
ax.legend(loc="upper left")
save(fig, "gforce.png")

# 4. tank: two panels, one axis each
fig, (a1, a2) = plt.subplots(2, 1, figsize=(8, 5.2), sharex=True)
a1.plot(F["t"], F["P"], color=SIM)
a1.set(ylabel="tank pressure (bar)", title="Tank blowdown (Abel-Noble N₂, polytropic n = 1.25)")
a2.plot(F["t"], F["gas"], color=C3)
a2.set(xlabel="time (s)", ylabel="gas left (kg)", xlim=(52.2, 58))
for a in (a1, a2):
    vlines(a)
save(fig, "tank.png")

# 5. nozzles: small multiples by group
noz = F["noz"]
groups = (("rear main (4 × 20 kN)", range(0, 4)), ("corner lift (4 × 35 kN)", range(4, 8)),
          ("front brake/yaw (2 × 15 kN)", range(8, 10)))
fig, axs = plt.subplots(3, 1, figsize=(8, 7), sharex=True)
for a, (title, idx) in zip(axs, groups):
    for c, i in zip((SIM, HUD, C3, C4), idx):
        a.plot(F["t"], noz[:, i], color=c, lw=1.5, label=S.NOZZLES[i][0])
    a.plot(F["t"], F["P"] / F["P"][0], color=MUTED, lw=1, ls="--", label="limit P/P₀")
    a.set(ylabel="thrust / full-tank max", ylim=(-0.03, 1.05), title=title)
    a.legend(loc="upper right", ncol=5, fontsize=8)
    vlines(a, labels=False)
axs[-1].set(xlabel="time (s)", xlim=(52.2, 57))
save(fig, "nozzles.png")

# 6. wall loads
fig, axs = plt.subplots(3, 1, figsize=(8, 7), sharex=True)
axs[0].plot(P1["t"], P1["v"], color=SIM)
axs[0].set(ylabel="speed (m/s)", title="On the wall: speed, minimum suction, motor power")
axs[1].plot(P1["t"], P1["F_s_min"] / S.G0, color=SIM, label="minimum suction to stay on")
axs[1].axhline(S.SUCTION_MAX_KG, color=HUD, lw=1.5, ls="--", label="HUD DOWNFORCE peak 4,700 kgf")
axs[1].set(ylabel="suction (kgf)")
axs[1].legend(loc="lower right")
axs[2].plot(P1["t"], P1["power"] / 1e3, color=SIM)
axs[2].set(ylabel="power F·v (kW)", xlabel="time (s)", xlim=(45.5, S.T_RELEASE))
for a in axs:
    a.axvspan(t170, S.T_RELEASE, color=GRID, alpha=0.6, lw=0)
axs[0].text(t170 + 0.03, axs[0].get_ylim()[1], "170° ceiling:\nread noise", color=MUTED, va="top", fontsize=8)
save(fig, "wall_loads.png")
