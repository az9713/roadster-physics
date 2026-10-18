# HANDOFF — Tesla Roadster wall-flip simulation

**Read this first each new session.** There is no project CLAUDE.md; the global rules in `~/.claude/CLAUDE.md` apply.
Updated 2026-10-18 09:20. Git repo, pushed to github.com/az9713/roadster-physics (GitHub Pages on). `PROJECT.html` is the living project document:
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
- CYCLES FINAL DONE 2026-10-18 07:58 (3 runs, about 8 h of render): `out/broadcast_final.mp4`, 1280x720, 30 fps,
  419 frames (seg00 21 + seg01 398, same as the EEVEE draft; "420" in older notes was off by one), 13.97 s, 1.38 MB.
  Untracked in git (the drafts are tracked). Frames in `out/edl_broadcast_final/segNN/`. Runs 1 and 2 stopped at frames
  342 and 410 with no crash file and no Windows event (cause not proven; PROJECT.html §9 2026-10-18). Logs:
  `out/logs/edl_broadcast_final.run1.log`, `.run2.log`, current `.log`. Chain: `out/logs/render_all.sh` (resumable).

## Physics report: DONE 2026-10-17
`PHYSICS.html` at the repo root (KaTeX 0.16.9 from cdnjs with SRI), linked from `index.html`, `README.md`,
`DEVELOPMENT-JOURNEY.html`; PROJECT.html §9 line added. Charts + derived numbers: `python out/physics/make_charts.py`
(imports the sim module, ~30 s; prints the free-fall lag fit, throat sizes, sensitivity table, short-variant run).
Known stale: PROJECT.html §8 table still shows the first passing run (pre roll-window change); PHYSICS.html uses current.
Commits: e36a9b3 (report), 69be99f (legend fix), c4d50e7 (card borders: impeccable side-tab finding fixed, no ignores).

## Part 4 "the flip": DONE 2026-10-17 (commit 5f338ad)
`PHYSICS.html#p4` (sections 4.0-4.10, Figs. 7-12, Propositions 1-4 with proofs). `python out/physics/flip.py` (~5 s)
prints every number and writes `out/physics/flip_*.png`. Sim unchanged. PROJECT.html §9 line added.
Findings to tell the owner: sim roll lags its reference ~0.1 s and overshoots upright by 34.3 deg at 53.18 s (no
feed-forward; a fix would be `M_b += INERTIA @ dw_ref`, needs the owner's OK since it changes the sim); the jet roll
pushes the car +y (open loop 1.72 m/s, 2.11 m by the burn; same sign as the observed drift); HUD G 0.2-0.4 g in the roll
implies I_xx <= 351 kg m2 or a filtered gauge; aero < 0.5 % of the jet moment; RR exhaust hits the hud ceiling 52.40-52.56.
Pages 200 verified (page + flip_film.png). Not done: interactive Chrome console check (headless DOM check passed: 507 KaTeX, 0 errors).
Sources used: NASA TM 86825 (Kuhn & Eshleman 1985, Fig. 2, Wyatt), NASA TM 102268 (Kuhn et al. 1991, p. 5).

## Next task
1. DONE 2026-10-18: `out/broadcast_final_hud.mp4` (1.81 MB, 419 frames, 13.97 s, sound). Gauges vs `frames/ref/`:
   44.0 s 1.0 G/0.0 ft = real; 54.45 s 3.2 G/19.2 ft vs 3.2/18.7; 55.95 s 0.8 G/3.8 ft vs 1.4/3.6. PROJECT.html status row
   -> done, §9 line written. Both final MP4s committed 2026-10-18 at the owner's request: `out/broadcast_final.mp4` (1.38 MB),
   `out/broadcast_final_hud.mp4` (1.81 MB).
   NEW FINDING: shot B at 54.45 s matches the fitted wall points (y 170-560 px) but the floor/wall foot is ~140 px low
   (y ~715 vs ~578). Calib points for late B frames have no floor rows; `car_from_video` at 115.8 = x -177 m. Foot
   matches within ~10 px at 48.45/50.85 s, goes low at 53.45/54.45 s (`out/compare/stills_vs_pcmag.jpg`). Possible
   cause of the B residuals in the failed gate. Fix = floor points in `calib_cameras.py` + shot B re-render: owner's call.
   FINDING (2026-10-18, owner saw "no difference"): final vs `out/broadcast_draft.mp4` luma PSNR is 39.6-42.0 dB per
   second for 1-9 s (near identical: same scene, lamps, cameras; only edge and noise differences) and 22.2-22.6 dB for
   11-13 s. There the EEVEE draft cloud is thin and dithered (the car shows through); the Cycles cloud (64 samples,
   denoised, volume_step_rate 2.0, `build_scene.py:646-648`) is dense, smooth and hides the car. Against the real
   frame `frames/ref/pcmag_117.3.png` (= frame 360) the Cycles cloud is closer (dense white billows, car mostly hidden).
   DECIDED by the owner 2026-10-18: keep the more photoreal one -> the Cycles final as rendered, no re-render.
   Remaining gaps vs the real cloud: ours is greyer and has no pink/red tint from the venue lights.
2. Tell the owner: deliverables, the failed gate, the image findings (car at/past lip at 52.25; +y drift; jets at 53.45
   vs T_BURN 53.95), credits used. Offer: short variant as the render default? (B wall edges favour HUD length; the
   car-at-lip image favours short.)
3. Repo improvement plan (agreed with the owner 2026-10-18, from a `/be-ambitious` pass). Goal: every published number
   is checked by a tool against the footage or HUD, and one command rebuilds and rechecks everything. Order:
   a. ALTITUDE (does not need tracking; can run any time). Build `claims.csv` (every number in PROJECT.html,
      PHYSICS.html, README.md + producing script/output line + evidence class HUD/footage/model-only/none). Write
      `check.py`: reruns `sim/wall_flip_sim.py`, `out/physics/make_charts.py`, `out/physics/flip.py`, fails on any
      doc/output mismatch beyond a stated tolerance. Prove it on the git history: it must flag the stale PROJECT.html
      §8 table. Install as pre-commit hook (`setup-pre-commit` skill). In `replay.html`, project the sim car onto the
      real B and R1 frames and show the pixel residual per frame.
   b. FIRST PROBE (max 20 tool calls). Track the car in the 21 B frames (`frames/tri/b/`) with Blender's built-in
      motion tracker or OpenCV; compare to `CAR_B`. Pass = tracks to PCMag 116.4 and median diff <= ~10 px.
      Not verified: `import cv2`, `yt-dlp` on PATH.
   c. COMBINED, trimmed (only if b passes; reuse a's files, do not rebuild them): replace `CAR_R1`/`CAR_B` with tracked
      centres from the highest-resolution source; fit T_RELEASE, T_BURN, ROLL_T0/T1, LIP_EXT and a y-drift term directly
      to tracked pixels through the cached cameras (no triangulate-first; smooth camera poses across frames to remove
      the wrong minima at R1 70.19 and 73.94-74.44; robust loss on pixel rows only, not HUD prior rows). Target = the
      owner's gate on held-out frames (every 4th held out). Write `out/fit.json`; change sim defaults only after the
      owner approves. Add the reprojection check to `check.py`.

## Decisions from the owner (2026-10-17)
- Short-overhang variant as a switch (done). ElevenLabs SFX allowed, cap ~10 generations (8 used).
- Sim refit only if car rms <= 5 px in R1 and B and no frame > 15 px (failed -> not refitted).
- One Cycles final of `broadcast` (done 2026-10-18, see Outcome); `coverage` as EEVEE only (done). Agent judges the look itself.

## Session-transient scratch (gone after clear; regenerate only if needed)
The session scratchpad (temporary, not in the repo):
`edges.py` (edge overlay sheets edges2_*.png), `cars.py`/`crops.py` (gridded crops used to read car centres by eye),
`r1_rough.py` (first R1 camera guess), `lips.png`. The durable record is the dicts and functions in `blender/triangulate.py`.
2026-10-17 report session: `fitlag.py`, `sens.py` (lag fit, sensitivity) — superseded by `out/physics/make_charts.py`.

## Workflow provenance
- Skills used this project: `blender-headless` (load before Blender runs), `handoff-after-clear`; for PHYSICS.html
  `rigorous-explainer` (checktex/checklt/check_links/check_prose run; MathJax-only checks skipped) and `dataviz`
  (palette validator); `claude-in-chrome` (opened the repo for the owner).
- Advisor guidance (followed): report measured vs sim to the owner before editing the sim.
- 2026-10-18: render-stop diagnosis used Windows event logs, CLI transcripts in
  `~/.claude/projects/C--Users-simon-Downloads-tesla-roadster-simulation/*.jsonl`, and `search_session_transcripts`.
  `handoff-after-clear` refreshed this file.
- 2026-10-18: `be-ambitious` skill used to write the plan in Next task 3 (prompts only; none run yet).
- Local `origin` URL still says `roadster-simulation`; the GitHub repo is now `roadster-physics` (commit 7e281bc).
  Push works through GitHub's redirect.

## Tools and rules (unchanged)
- Blender 5.2.1 CLI: `%LOCALAPPDATA%\Programs\Blender\blender-5.2.1-windows-x64\blender.exe --factory-startup`.
- EEVEE drafts; one Cycles final at 1280x720 only after the owner approves the look (1-3 min/frame, background).
- Car modelled in Python from reference frames. No AI image-to-3D, no downloaded model. Textures CC0 or procedural.
- Paid AI order: free/local → fal.ai or kie.ai → Higgsfield. Ask the owner before any paid call. GPU: RTX 3050 Laptop, 4 GB.
- Reference look: `frames/wall/pc_full_104.png`, `frames/wall/flip_01.jpg`, `frames/pcmag_03.jpg`, `frames/ref/`.
- Never kill a process by name; stop only your own task IDs/PIDs.
