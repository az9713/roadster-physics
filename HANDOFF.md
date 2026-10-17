# HANDOFF — Tesla Roadster wall-flip simulation

**Read this first each new session.** There is no project CLAUDE.md; the global rules in `~/.claude/CLAUDE.md` apply.
Updated 2026-10-17. Git repo, pushed to github.com/az9713/roadster-simulation (GitHub Pages on). `PROJECT.html` is the living project document:
**update it (§9 change log) after every step.** All measurements, findings and sources are there.

## Goal
High-fidelity simulation of the stunt in PCMag `quRC56Z0EJw` 1:41–2:00 (= VideoFromSpace `eqOfmtTJD1A` 0:40–0:59):
car climbs a curved wall, rides the 170° overhang inverted, lets go at 52.25 s, half-rolls upright, thrust-lands ~56.3 s.
The owner insists every dimension is grounded in measurements of the real setup.

## Current state (verified 2026-10-17)
- `sim/wall_flip_sim.py` — done. Wall rebuild + on-wall physics + planar 6-DOF flight. `python sim/wall_flip_sim.py`
  prints a report, writes `out/replay_data.js`, self-check passes. ROLL_T0/T1 = 52.35/53.10 (re-measured, approved).
- `replay.html` — done (Three.js replay).
- `blender/calib_cameras.py` → `out/camera_calib.json`: camera A (295 mm, 62 m behind the foot, rms 4.1 px),
  camera B (tripod (-76.9, -18.4, 7.7) m, 43-51 mm, per-frame pan/tilt/zoom, rms 5.6 px). Run takes ~5 min.
- `blender/build_scene.py` + `blender/roadster.py` — scene, car (8,761 faces), plumes, 22-puff cloud, cameras A/B +
  coverage C/D/E/F/G, EDLs `broadcast`/`coverage`. Run Blender with `--factory-startup`. No MP4 rendered yet.
- Time axes: PCMag = VideoFromSpace + 61.35 s (CSV header still says 61.0). PCMag = sim + 61.35. HUD alt lags 0.30 s.
- Data: `data/wall_profile_hud.csv` (wall), `data/hud_wall_flip.csv` (flip run, 10 Hz), `data/hud_telemetry.csv` (other stunt).

### Item 3 — triangulation, IN PROGRESS (`blender/triangulate.py`, 275 lines)
Purpose: measure the real car path after release from two views (shot B + slow-motion replay R1), then refit the sim.
- Frames: `frames/tri/r1/eq_<t>.png` (27, R1, eq 69.94-76.44 every 0.25 s, 1280x718) and `frames/tri/b/pc_<t>.png`
  (21, PCMag 113.6-117.6 every 0.2 s). Car centres read by eye: dicts `CAR_R1`, `CAR_B` in the script.
- Stage 1 (R1 camera, `solve_r1`, jac_sparsity + soft_l1): tripod ~(-126.5, -7.5, 10.5) m, lens 156-177 mm, wall-edge
  rms 1-3 px (8-16 px in frames with car/cloud on an edge). Tripod x from -90 to -144 fits about equally (tele lens).
- Stage 2 (B per-frame refit, `solve_b`, tripod fixed): rms 2-4 px; frame 115.8 had a wrong minimum (roll 16°).
  Roll now limited to ±0.12 rad in `fit_frame`; not rerun since.
- Conditioning: B and R1 sight lines meet at ~13° → ~0.8 m depth error for 5 px in B, against a 51-303 px miss. Usable.
- Side finding: in B the model lip projects 5-11 px left of the detected lip in every frame (overhang maybe short).
- Stage 3 (`solve_traj`: cubic B-spline x(t), y(t), knots every 0.3 s over 52.25-56.6; R1 frame times as monotonic
  exp-increments; z from HUD) — NOT DONE. It has no jac_sparsity, so it is slow (>10 min for the 4 runs at
  fx in (None, -95, -110, -140)). Earlier runs gave non-physical paths, but they used camera solutions broken by a
  lip-index bug (fixed: `edge_res` uses `cur[0]`). **`out/triangulation.json` is invalid — do not use it.**

### Progress 2026-10-17 (autonomous session)
- Overhang switch DONE: `python sim/wall_flip_sim.py short` (lip at release, LIP_EXT 0, extent 18.70 m, self-check
  passes) vs default `hud` (LIP_EXT 1.5, 20.18 m). Run without `short` to restore before Blender. Evidence: B wall-edge
  median rms 3.09/3.23/3.61/3.81/4.93 px for ceiling +1.5/0/-1/-2/-3 m -> B favours HUD length; a 170 deg gauge clamp
  would make the ceiling LONGER (true pitch > 170 needs more run per ft), not shorter.
- Timing measured: solve_b 30 s, solve_r1 254 s (the slow part), solve_traj 9 s. No jac_sparsity needed.
- B 115.8 fixed: smoke rows below the wall were read as edges -> `wall_edges(maxgap=40)` for B only. B rms now 2-6 px.
- CAR_B[116.4] = (588, 457) added; 116.6+ unusable (wall leaves frame, car in cloud).
- R1 time map now FIXED by events (`R1_ANCHOR`): 69.94->52.25, 71.94->53.10 (upright), 74.69->54.00 (cloud on
  floor; B none at 115.2, small 115.4). One free speed after 74.69. soft_l1 loss.
- Side finding: B shows lift jets already at PCMag 114.8 (= 53.45 real), short; long at 115.0. Sim T_BURN = 53.95
  (HUD G). Either the G gauge also lags ~0.3-0.5 s or the jets start before G rises.
- Look pass (build_scene.py): per-camera exposure (`EXPOSURE = {"camA": -1.0}`, `use_cam`), floor Specular IOR 0
  (`--floorspec`), haze 0.00015 (`--haze`), volume tile "2" (`--tile`), puffs bigger (r 4-7 m, reach 4-14 m), density 30
  (`--puffd`), billow scale 0.2 (`--billow`). Stills in `out/look/`. Floods unchanged (moving them darkened B's wall).

- Stage 3 status (z free, HUD prior 0.5 m, anchored R1 times): car rms R1 76 px, B 50 px -> gate FAILS so far.
  Causes found: wrong-minimum camera poses (R1 70.19 = 502 px, R1 73.94-74.44 ~138 px, B 116.4 edge rms 22.5 px ->
  drop 116.4); soft_l1 also applied to the HUD prior rows, so z ran 12 sigma off (55.0: 9.2 vs 2.9 m) and x ran to
  -72 m. `solve_r1` stops after 25 evals with the tripod unmoved (x_scale="jac" on the tripod-x ridge).
  B-only with z = HUD: x is ill-conditioned (B is 7.7 m up, ~60 m away: dx/dz ~ 5 m/m; degenerate at z ~ 7.7 m).
- Image evidence (stands alone): at PCMag 113.6 (52.25) the car is AT/PAST the lip, not 1.5 m before it
  (scratch b_rel.png); car drifts to +y (screen-left in B and R1); lift jets visible at PCMag 114.8 (53.45).
- NEXT CHECK (discriminating): project the SIM path into the cached cameras with a time shift d in {0, .2, .3, .4}
  (B: car_sim(t_pc - 61.35 + d); R1 anchors shifted by -d). If d ~ 0.3 collapses B 113.6-114.6 residuals, the HUD G
  gauge lags like altitude: T_RELEASE ~ 51.95, T_BURN ~ 53.65 (timing-only refit, authorized). Then check the
  shorter ceiling (~2.3 m) against the B lip-dx zero crossing (-2.2 m) for the short variant's value.
  If car rms still > 30 px after fixing outliers + d test: stop, report "path not reliable", keep planar model.

## Outcome of the 2026-10-17 run
- Triangulation gate FAILED (best car rms R1 76 px, B 50 px; time-shift test 0-0.5 s did not help, B rms 168-184 px).
  Per the owner's rule: NO sim refit, planar y=0 model kept. Recorded in PROJECT.html §9. Do not spend more on it unless
  the owner asks; the next step would be robust per-frame camera smoothing (R1 70.19, 73.94-74.44 wrong minima).
- Renders: `out/coverage_draft.mp4` (EEVEE), `out/broadcast_draft.mp4` (EEVEE), `out/broadcast_draft_hud.mp4`
  (HUD + sound, 13.97 s). Sound: `out/sfx/` (8 ElevenLabs takes, 400.8 credits, flow "Roadster wall flip SFX"),
  mix `python out/sfx/mix.py` -> `out/sfx/broadcast_mix.wav`. HUD: `python blender/hud_mux.py <in.mp4> <out.mp4>`.
- CYCLES FINAL RUNNING (started 2026-10-17 ~05:45, est. 15-20 h): chain `out/logs/render_all.sh`, bash PID in
  `out/logs/chain.pid` (13984). Frames -> `out/edl_broadcast_final/segNN/`, log `out/logs/edl_broadcast_final.log`,
  done when `out/logs/chain.log` has "final exit 0" and `out/broadcast_final.mp4` exists. Resumable: rerun
  `render_all.sh` (it deletes 0-byte placeholders; finished drafts are skipped frame by frame).

## NEXT TASK FIRST: physics report (asked 2026-10-17, decisions taken)
The owner's request (verbatim): "Can you write a HTML report about the physics of the simulation? You first present the
theory, the complete theory and the mathematics behind this simulation, and then present the actual numbers. For example,
the parameters for the car, for the roadster, and the ramp, everything else. So the goal is to demystify how you come up
with this simulation based on your earlier research about the setting up and also based on the video."
The owner's answers (2026-10-17):
- Build it in a fresh session (this one).
- Math with **KaTeX** from a CDN (cdnjs, exact version; auto-render), like replay.html needs internet.
- **Static PNG charts** made with matplotlib from `out/replay_data.js` and `data/*.csv`, committed next to the report
  (suggested folder `out/physics/`): wall profile x-z with pitch, altitude sim vs HUD (with the 0.30 s lag),
  G sim vs HUD, tank pressure + gas mass, the 10 nozzle commands, wall-phase speed/suction/power.
- **Full derivations + plain words**: derive each equation from first principles (Newton, rigid-body rotation and
  quaternions, wall-phase tyre/suction/G, Abel-Noble real gas, polytropic blowdown, nozzle thrust and mass flow, the
  controller and the bounded least-squares valve allocator, HUD lag free-fall fit), then a plain-language reading, then
  the code location (`sim/wall_flip_sim.py:<line>`).
Structure: Part 1 theory (complete), Part 2 the numbers (car, ramp, nozzles, tank, controller, estimates marked est. vs
measured vs derived, with sources: PROJECT.html §3-§8, README source table), Part 3 results vs HUD and the sensitivity
findings (n = 1.4/1.25/1.1, lag, nozzle sizing, tolerance 2.0 -> 2.5 ft and why), and known limits (planar y = 0,
suction-noise artefact, gate failed in triangulation, overhang hud vs short).
Rules: read `sim/wall_flip_sim.py` fully first; every number from the code or PROJECT.html, none from memory; dark mode
(CLAUDE.md colours); ASD-STE100 style; no personal info (write "the owner"). File: `PHYSICS.html` at the repo root; link it
from `index.html`, `README.md` and `DEVELOPMENT-JOURNEY.html`; commit, push, wait for the Pages build, check HTTP 200 and
render it once in Chrome. Add a PROJECT.html §9 line.

## Next task
0. The Cycles final STOPPED at about 08:12 on 2026-10-17: last saved frame `seg01/0341.png` (341 of 420), no error in
   `out/logs/edl_broadcast_final.log`, no crash file, no "final exit" line in `out/logs/chain.log`, PID 13984 gone. Cause
   unknown. Resume: rerun `out/logs/render_all.sh` detached (it deletes the 0-byte `0342.png` and skips done frames).
   Ask the owner first: the stop may have been on purpose. Story of the whole project: `DEVELOPMENT-JOURNEY.html`.
1. When `out/broadcast_final.mp4` exists: `python blender/hud_mux.py out/broadcast_final.mp4 out/broadcast_final_hud.mp4`.
   Check 3 frames (44.0, 54.45, 55.95) against `frames/ref/`. Update PROJECT.html (status table row "Blender
   photoreal scene" -> done, §9 entry with render time and file).
2. Tell the owner: deliverables, the failed gate, the image findings (car at/past lip at 52.25; +y drift; jets at 53.45
   vs T_BURN 53.95), credits used. Offer: short variant as the render default? (B wall edges favour HUD length; the
   car-at-lip image favours short.)

## Decisions from the owner (2026-10-17)
- Short-overhang variant as a switch (done). ElevenLabs SFX allowed, cap ~10 generations (8 used).
- Sim refit only if car rms <= 5 px in R1 and B and no frame > 15 px (failed -> not refitted).
- One Cycles final of `broadcast` (running); `coverage` as EEVEE only (done). Agent judges the look itself.

## Session-transient scratch (gone after clear; regenerate only if needed)
The session scratchpad (temporary, not in the repo):
`edges.py` (edge overlay sheets edges2_*.png), `cars.py`/`crops.py` (gridded crops used to read car centres by eye),
`r1_rough.py` (first R1 camera guess), `lips.png`. The durable record is the dicts and functions in `blender/triangulate.py`.

## Workflow provenance
- Skills used this project: `blender-headless` (load before Blender runs), `handoff-after-clear`.
- Advisor guidance (followed): report measured vs sim to the owner before editing the sim.

## Tools and rules (unchanged)
- Blender 5.2.1 CLI: `%LOCALAPPDATA%\Programs\Blender\blender-5.2.1-windows-x64\blender.exe --factory-startup`.
- EEVEE drafts; one Cycles final at 1280x720 only after the owner approves the look (1-3 min/frame, background).
- Car modelled in Python from reference frames. No AI image-to-3D, no downloaded model. Textures CC0 or procedural.
- Paid AI order: free/local → fal.ai or kie.ai → Higgsfield. Ask the owner before any paid call. GPU: RTX 3050 Laptop, 4 GB.
- Reference look: `frames/wall/pc_full_104.png`, `frames/wall/flip_01.jpg`, `frames/pcmag_03.jpg`, `frames/ref/`.
- Never kill a process by name; stop only your own task IDs/PIDs.
