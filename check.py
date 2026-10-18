"""One command to recheck the published numbers.

    python check.py              full: rerun sim/wall_flip_sim.py (default "hud" variant), out/physics/make_charts.py,
                                 out/physics/flip.py; save their text to out/check_outputs.json; compare (about 30 s)
    python check.py --fast       rerun only the sim (about 6 s); take the other two outputs from out/check_outputs.json.
                                 Stops with an error if an input script or data file changed since the last full run.
                                 The pre-commit hook uses this mode.
    python check.py --rev REV    compare the docs as they are in git revision REV (outputs are always fresh)
    python check.py --inventory  rebuild claims.csv from the docs and the fresh outputs (rules below)
    python check.py --install-hook   write .git/hooks/pre-commit (runs "python check.py --fast")

claims.csv holds one row per number in PROJECT.html, PHYSICS.html and README.md: where it is (doc, section, the text
before it), the value shown, the evidence class (HUD, footage, model-only, none), and, when a tool prints it, the
script, the output line and an expression that reads the fresh value. A fourth output, "par", is the sim's input
constants (PARAMS below), read by importing the sim module. A row with check=yes must agree with the fresh
value within tol (default: half a unit of the last digit shown). The expression pins the doc number to one output
line; the pin is set when doc and output agree (--inventory) and stays when the doc later drifts.
Exit code: 0 = all checked rows agree, 1 = a mismatch or a missing anchor, 2 = the tool itself failed.
Edit the RULES in this file, not claims.csv: --inventory regenerates the CSV.
"""
import csv
import hashlib
import json
import math
import os
import re
import subprocess
import sys
from html.parser import HTMLParser
from pathlib import Path

ROOT = Path(__file__).resolve().parent
DOCS = ["PROJECT.html", "PHYSICS.html", "README.md"]
SCRIPTS = {"sim": "sim/wall_flip_sim.py", "mc": "out/physics/make_charts.py", "flip": "out/physics/flip.py"}
INPUTS = ["sim/wall_flip_sim.py", "out/physics/make_charts.py", "out/physics/flip.py", "data/wall_profile_hud.csv",
          "data/hud_wall_flip.csv"]
CACHE = ROOT / "out" / "check_outputs.json"
CLAIMS = ROOT / "claims.csv"
FIELDS = ["id", "doc", "section", "context", "value", "after", "evidence", "check", "script", "output_line", "expr", "tol",
          "note"]

# ---------------------------------------------------------------- rules (curation lives here) --------------------------
# Curated pins: (doc, text that ends right before the number, expression, note). They win over the automatic match.
# S("key") = sim report value; L("mc"|"flip", "text in the line", n) = n-th number (1-based) on the first output line
# that contains the text.
PINS = [
    # PROJECT.html section 8 table and 8.1 (the table the proof must flag)
    ("PROJECT.html", "Apex", 'S("apex_sim_ft")', ""),
    ("PROJECT.html", "Apex 68.5 ft at", 'S("apex_sim_t")', ""),
    ("PROJECT.html", "68.0 ft at 52.30 s (lag-corrected) Touchdown", 'S("touchdown_sim_t")', ""),
    ("PROJECT.html", "Altitude RMSE in flight", 'S("flight_alt_rmse_ft")', ""),
    ("PROJECT.html", "G RMSE: wall / flight", 'S("wall_G_rmse_vs_HUD")', ""),
    ("PROJECT.html", "G RMSE: wall / flight 0.26 g /", 'S("flight_G_rmse_vs_HUD")', ""),
    ("PROJECT.html", "Peak G in flight", 'S("peak_G_sim_flight")', ""),
    ("PROJECT.html", "Release speed, pitch", 'S("release_speed_mps")', ""),
    ("PROJECT.html", "Gas used / left", 'S("gas_used_kg")', ""),
    ("PROJECT.html", "kg /", 'S("gas_left_kg")', "section 8 gas left"),
    ("PROJECT.html", "Tank pressure", 'S("tank_P_start_bar")', ""),
    ("PROJECT.html", "Tank pressure 544 →", 'S("tank_P_end_bar")', ""),
    ("PROJECT.html", "Landing point x =", 'S("landing_x_m")', ""),
    ("PROJECT.html", "m behind the foot line,", 'S("release_x_m") - S("landing_x_m")', "landing past release"),
    ("PROJECT.html", "past the release point at x =", 'S("release_x_m")', ""),
    ("PROJECT.html", "Closest approach to the wall after release", 'S("min_clearance_to_wall_m")', ""),
    ("PROJECT.html", "The observed burns need about", 'S("gas_used_kg")', ""),
    ("PROJECT.html", "Pressure falls from", 'S("tank_P_start_bar")', ""),
    ("PROJECT.html", "Pressure falls from 544 to", 'S("tank_P_end_bar")', ""),
    ("PROJECT.html", "Polytropic exponent n = 1.4 (no wall heat): touchdown", 'L("mc", "n = 1.4:", 2)', ""),
    ("PROJECT.html", "check fails. n = 1.25:", 'L("mc", "n = 1.25:", 2)', ""),
    ("PROJECT.html", "n = 1.1:", 'L("mc", "n = 1.1:", 2)', ""),
    ("PROJECT.html", "The result is", 'S("flight_alt_rmse_ft")', ""),
    ("PROJECT.html", "A free-fall fit to 52.2–53.95 s gives a lag of", 'L("mc", "free-fall fit", 3)', ""),
    ("PROJECT.html", "and a release vertical speed of", 'L("mc", "free-fall fit", 2)', ""),
    ("PROJECT.html", "m/s (rms", 'L("mc", "free-fall fit", 4)', ""),
    ("PROJECT.html", "The release CG altitude becomes", 'S("wall_height_release_m")', ""),
    ("PROJECT.html", "and the rebuilt wall extends", 'S("wall_horizontal_extent_m")', ""),
    ("PROJECT.html", "The model's highest suction need is", 'S("wall_min_suction_needed_kgf")', ""),
    ("PROJECT.html", "so the computed speed (", 'L("mc", "overall peak suction", 3)', ""),
    ("PROJECT.html", "and acceleration (", 'L("mc", "overall peak suction", 4)', ""),
    # PROJECT.html section 3 and 7.1 (model values printed by a tool)
    ("PROJECT.html", "Entry curve radius", 'S("wall_entry_radius_m")', ""),
    ("PROJECT.html", "About 21.6 m (≈71 ft) at the release point", 'S("wall_height_release_m")', ""),
    ("PROJECT.html", "Track length, foot to release", 'S("release_track_s_m")', ""),
    ("PROJECT.html", "Horizontal size About", 'S("wall_horizontal_extent_m")', ""),
    ("PROJECT.html", "m³/kg → about", 'S("tank_P_start_bar")', ""),
    ("PROJECT.html", "exhaust speed ≈", 'L("mc", "c_eff", 3)', "rounded to 10 m/s", 5),
    # PHYSICS.html (automatic match took a line with the same value but another quantity)
    ("PHYSICS.html", "x = −10.45 m — Gas used", 'L("flip", "gas release->burn", 1)', ""),
    ("PHYSICS.html", "338.9 bar at the burn →", 'S("tank_P_end_bar")', ""),
    # PHYSICS.html 4.8 aerodynamics (2-digit values: the automatic match needs 3 digits or a shared word)
    ("PHYSICS.html", "the car's weight spread over its", 'L("flip", "planform", 1)', ""),
    ("PHYSICS.html", r"the sim's own drag areas: \(C_{Dz} =", 'L("par", "CDA_Z =", 1)', ""),
    ("PHYSICS.html", r"= 0.851\) and \(C_{Dy} =", 'L("par", "CDA_Y =", 1)', ""),
    ("PHYSICS.html", r"with aspect ratio \(A = W/L =", 'L("flip", "roll damping:", 4)', ""),
    ("PHYSICS.html", r"A = W/L = 0.426\), so \(C_{L\alpha} =", 'L("flip", "roll damping:", 5)', ""),
    ("PHYSICS.html", "102 N·m and 119 N·m:", 'L("flip", "roll damping:", 9)', ""),
    ("PHYSICS.html", "102 N·m and 119 N·m: 0.39% and", 'L("flip", "roll damping:", 10)', ""),
    ("PHYSICS.html", "the aerodynamic force at the CG. Each", 'L("flip", "aero moment per 0.1 m of centre", 1)', ""),
    ("PHYSICS.html", r"of offset adds a moment of \(", 'L("flip", "aero moment per 0.1 m of centre", 1)', ""),
    ("PHYSICS.html", r"F_{aero}|\): at most", 'L("flip", "aero moment per 0.1 m of centre", 2)', ""),
    ("PHYSICS.html", "at most 93 N·m in the window,", 'L("flip", "term aero moment per 0.1", 2)', ""),
    ("PHYSICS.html", r"The result is \(\Delta L_\infty/T =", 'L("flip", "d = 114 mm, NPR 536", 3)', ""),
    ("PHYSICS.html", r"\(\Delta L_\infty/T = -0.10\%\) to \(", 'L("flip", "d = 23 mm, NPR 435", 3)', ""),
    ("PHYSICS.html", r"to \(-0.55\%\), at most", 'abs(L("flip", "d = 23 mm, NPR 435", 4))', ""),
    ("PHYSICS.html", r"\(-0.55\%\), at most 131 N on a", 'L("flip", "d = 23 mm, NPR 435", 5)', ""),
    ("PHYSICS.html", "here the NPR is 435–536,", 'L("flip", "NPR 435-536", 6)', ""),
    ("PHYSICS.html", "here the NPR is 435–536, 181 to", 'L("flip", "NPR 435-536", 7)', ""),
    ("PHYSICS.html", "the term. In plain words. At", 'L("flip", "airspeed:", 1)', ""),
    ("PHYSICS.html", "the term. In plain words. At 8–", 'L("flip", "airspeed:", 3)', ""),
    ("PHYSICS.html", "air pushes on the car with about", 'L("flip", "drag in window", 2)', ""),
    ("PHYSICS.html", "also resists the spin, but with about", 'L("flip", "roll damping:", 10)', ""),
    ("PHYSICS.html", "they would change the roll by about", 'L("flip", "uncompensated aero", 1)', ""),
    # PHYSICS.html "The wall" (Figs. 1 and 2 and the paragraphs under them)
    ("PHYSICS.html", "Orange: the surface,", 'L("par", "H_CG =", 1)', ""),
    ("PHYSICS.html", "The release point (pink) is", 'abs(S("release_x_m"))', ""),
    ("PHYSICS.html", "m behind the foot, with the CG", 'L("mc", "release CG z", 1)', ""),
    ("PHYSICS.html", "The grey band from 51.31 s is the", 'S("release_pitch_deg")', ""),
    ("PHYSICS.html", "Reading Fig. 2. The speed is about", 'L("mc", "speed: entry curve", 2)', ""),
    ("PHYSICS.html", "m/s in the entry curve and falls to", 'L("mc", "speed: entry curve", 5)', ""),
    ("PHYSICS.html", "4.6 m/s on the overhang (", 'L("mc", "speed: entry curve", 3)', ""),
    ("PHYSICS.html", "4.6 m/s on the overhang (49.0–", 'L("mc", "speed: entry curve", 4)', ""),
    ("PHYSICS.html", "(49.0–50.5 s). Up to", 'L("mc", "up to 51.0 s:", 1)', ""),
    ("PHYSICS.html", "s the minimum suction peaks at", 'L("mc", "up to 51.0 s:", 2)', ""),
    ("PHYSICS.html", "against 4,700 kgf available:", 'L("mc", "up to 51.0 s:", 3)', ""),
    ("PHYSICS.html", "80% of it. The median from", 'L("mc", "up to 51.0 s:", 6)', ""),
    ("PHYSICS.html", "80% of it. The median from 48.5 to", 'L("mc", "up to 51.0 s:", 7)', ""),
    ("PHYSICS.html", "The median from 48.5 to 51.0 s is", 'L("mc", "up to 51.0 s:", 8)', ""),
    ("PHYSICS.html", "kgf. Wheel power up to", 'L("mc", "up to 51.0 s:", 1)', ""),
    ("PHYSICS.html", "Wheel power up to 51.0 s peaks at", 'L("mc", "up to 51.0 s:", 5)', ""),
    ("PHYSICS.html", "Example: the HUD altitude jumps from", 'L("mc", "HUD altitude jump", 2)', ""),
    ("PHYSICS.html", "the HUD altitude jumps from 48.1 to", 'L("mc", "HUD altitude jump", 4)', ""),
    ("PHYSICS.html", "jumps from 48.1 to 50.6 ft between", 'L("mc", "HUD altitude jump", 1)', ""),
    ("PHYSICS.html", "to 50.6 ft between 50.2 and", 'L("mc", "HUD altitude jump", 3)', ""),
    ("PHYSICS.html", r"s lag shift, \(a\) rises to", 'L("mc", "saw sample 49.8", 2)', ""),
    ("PHYSICS.html", "rises to 6.4 m/s² at", 'L("mc", "saw sample 49.8", 1)', ""),
    ("PHYSICS.html", "m/s² at 49.8 s and falls to", 'L("mc", "saw sample 50.0", 2)', ""),
    ("PHYSICS.html", "and falls to −3.2 m/s² at", 'L("mc", "saw sample 50.0", 1)', ""),
    ("PHYSICS.html", "s. The suction need follows:", 'L("mc", "saw sample 49.8", 3)', ""),
    ("PHYSICS.html", "The suction need follows: 3,779 kgf, then", 'L("mc", "saw sample 50.0", 3)', ""),
    ("PHYSICS.html", "The ceiling values are noise. On a", 'S("release_pitch_deg")', ""),
    ("PHYSICS.html", "noise. On a 170° ceiling, one", 'L("mc", "0.1 ft step", 1)', ""),
    ("PHYSICS.html", "acceleration 10.62 m/s²),", 'S("wall_peak_power_kW")', ""),
    # README.md (status line)
    ("README.md", "rides the 170° overhang upside down, lets go at about", 'S("release_time_s")', ""),
]
# Contexts where a same-value output line is a coincidence (video readings, angles in formulas): no automatic match.
NOAUTO = [("PHYSICS.html", "on its side 52.6–"), ("PHYSICS.html", r"axis. \(\varphi =")]
# Numbers that describe the past on purpose (change logs, quoted old runs). Never compared to fresh output.
HISTORICAL = [
    ("PROJECT.html", "section", "9. Change log"),
    ("PROJECT.html", "section", "4.1 Speed and loads on the wall (computed)"),     # "computed before the lag correction"
    ("PHYSICS.html", "text", "PROJECT.html §8 listed the first passing run"),
    ("PHYSICS.html", "text", "The change log recorded"),
    ("PHYSICS.html", "text", "At n = 1.25 the run gave"),
    ("PHYSICS.html", "text", "The first full run touched down at"),
    # "end" = the text that ends right before the number (a "text" entry covers the next 400 characters)
    ("PHYSICS.html", "end", "gives the typical overhang need as about"), ("PHYSICS.html", "end", "kgf and the peak power as about"),
    ("PHYSICS.html", "end", "kW; those values were sampled every"),
]
# Sections that list facts quoted from the press and the videos (spec table, stunt and venue facts). A number there that
# no tool prints is noted "cited source", not "no tool prints it".
# An entry is (doc, section name) or (doc, text that ends right before the number).
CITED = [("PROJECT.html", "2. Which stunt this is"), ("PROJECT.html", "5. The car"), ("PROJECT.html", "6. Venue and press"),
         ("README.md", "Secondary sources"),
         # PHYSICS.html 4.8: constants of the NASA TM 102268 free-air fit and the NPR range of the NASA TM 86825 data
         ("PHYSICS.html", r"\right)^{"), ("PHYSICS.html", r"\mathrm{NPR}^{"), ("PHYSICS.html", r"\qquad K_\infty ="),
         ("PHYSICS.html", "survey were taken at NPR"), ("PHYSICS.html", "survey were taken at NPR 1.45–"),
         # PHYSICS.html "The wall": the published motor power
         ("PHYSICS.html", "kW, against more than")]
LABEL_BEFORE = re.compile(r"(§|Fig\.|Figs\.|Fig|Part|Parts|Section|section|Proposition|Propositions|Prop\.|step|Step|steps|"
                          r"line|lines|item|Item|Items|items|Table|Eq\.|eq\.|No\.|#|r|v|Opus|Blender|KaTeX|Three\.js|"
                          r"Python|ffmpeg|stage|Stage|Stages|task|Task|commit|Contents|\bTM|\bp\.|Figs\. \d+ and)\s*\(?$")
# Sim inputs that check.py reads by importing the sim module (main() does not run): (name, words the docs use near the
# value, expression on the module, unit). The "par" output prints "<words> <name> = <value> <unit>" per entry.
# auto_match pins a doc number to it when the value is equal (not rounded), a word is shared, and the unit appears in
# the 30 characters after the doc number ("" = no unit test). Nozzle rows: 0 = rear (y 0.45), 4 = lift FL, 8 = front L.
PARAMS = [("M_DRY", "dry mass", "M_DRY", "kg"), ("GAS0", "gas mass air", "GAS0", "kg"), ("V_TANK", "tank volume copv", "V_TANK", "m³"),
          ("T_GAS0", "start temperature", "T_GAS0", "K"), ("R_GAS", "gas constant", "R_GAS", "J"),
          ("GAMMA", "gamma ratio heats", "GAMMA", ""), ("POLY_N", "polytropic exponent n", "POLY_N", ""),
          ("CF", "thrust coefficient cf", "CF", ""), ("INERTIA_XX", "inertia roll ixx", "INERTIA[0, 0]", "kg"),
          ("INERTIA_YY", "inertia pitch iyy", "INERTIA[1, 1]", "kg"), ("INERTIA_ZZ", "inertia yaw izz", "INERTIA[2, 2]", "kg"),
          ("H_CG", "cg height hcg rest", "H_CG", "m"), ("WHEELBASE", "wheelbase", "WHEELBASE", "m"), ("TRACK", "track", "TRACK", "m"),
          ("CAR_L", "length", "CAR_L", "m"), ("CAR_W", "width", "CAR_W", "m"), ("CAR_H", "height", "CAR_H", "m"),
          ("K_SUSP_KN", "suspension spring wheel", "K_SUSP / 1e3", "kN"), ("C_SUSP_KN", "suspension damper wheel", "C_SUSP / 1e3", "kN"),
          ("MU", "friction tyre", "MU", ""), ("RHO", "air density rho", "RHO", "kg"),
          ("CDA_X", "drag area cda x", "CDA[0]", "m"), ("CDA_Y", "drag area cda y", "CDA[1]", "m"), ("CDA_Z", "drag area cda z", "CDA[2]", "m"),
          ("SUCTION_MAX_KG", "hud downforce suction", "SUCTION_MAX_KG", "kg"), ("VALVE_TAU_MS", "valve lag tau", "VALVE_TAU * 1e3", "ms"),
          ("RAMP_WIDTH", "ramp wall width", "RAMP_WIDTH", "m"), ("T_RELEASE", "release", "T_RELEASE", "s"),
          ("ROLL_T0", "roll starts window", "ROLL_T0", "s"), ("ROLL_T1", "roll ends upright window", "ROLL_T1", "s"),
          ("BRAKE_T1", "brake braking", "BRAKE_T1", "s"), ("HUD_LAG", "hud lag", "HUD_LAG", "s"), ("LIP_EXT", "lip", "LIP_EXT", "m"),
          ("T_BURN", "burn landing", "T_BURN", "s"), ("G0", "gravity g0 standard", "G0", "m/s"), ("FT", "ft foot", "FT", "m"),
          ("REAR_X", "rear main x", "N_POS[0, 0]", ""), ("REAR_Y1", "rear main y", "N_POS[0, 1]", ""),
          ("REAR_Y2", "rear main y", "N_POS[1, 1]", ""), ("REAR_Z", "rear main z", "N_POS[0, 2]", ""),
          ("REAR_KN", "rear main forward each", "N_FMAX0[0] / 1e3", "kN"),
          ("LIFT_X", "corner lift x", "N_POS[4, 0]", ""), ("LIFT_Y", "corner lift y", "N_POS[4, 1]", ""),
          ("LIFT_Z", "corner lift z", "N_POS[4, 2]", ""), ("LIFT_KN", "corner lift up each", "N_FMAX0[4] / 1e3", "kN"),
          ("FRONT_X", "front x", "N_POS[8, 0]", ""), ("FRONT_Y", "front y", "N_POS[8, 1]", ""), ("FRONT_Z", "front z", "N_POS[8, 2]", ""),
          ("FRONT_KN", "front aft inward each", "N_FMAX0[8] / 1e3", "kN"),
          ("FRONT_DEG", "front angle sin cos outward", "np.degrees(np.arctan2(-N_DIR[8, 1], -N_DIR[8, 0]))", "°")]
EVID_WORDS = [("footage", r"\bpx\b|frame|PCMag|shot B|camera|R1\b|replay|footage|video|image|lens|tripod|pixel"),
              ("HUD", r"HUD|gauge|G-FORCE|ALTITUDE|PITCH|DOWNFORCE")]
STOP = set("the and of at in to a an is from with for by on as it this that are be was its or not than then so".split())

# ---------------------------------------------------------------- text extraction -------------------------------------
NUM = re.compile(r"(?<![\w.])([-−]?)(\d{1,3}(?:(?:,|\{,\})\d{3})+|\d+)(\.\d+)?([eE][-−]?\d+)?")   # {,} = KaTeX comma
SPAN_LABEL = re.compile(r"\b\d{4}-\d{2}-\d{2}\b|\b\d{1,2}:\d{2}(?:\.\d+)?(?:[–-]\d{1,2}:\d{2}(?:\.\d+)?)?")


class _Text(HTMLParser):
    def __init__(self):
        super().__init__(convert_charrefs=True)
        self.skip, self.parts, self.sections, self.head, self.n = 0, [], [], None, 0

    def handle_starttag(self, tag, attrs):
        if tag in ("script", "style", "head"):
            self.skip += 1
        elif tag in ("h1", "h2", "h3"):
            self.head = []
        elif tag in ("br", "p", "li", "tr", "td", "th", "div", "figcaption", "dd", "dt", "ol", "ul", "table"):
            self._add(" ")

    def handle_endtag(self, tag):
        if tag in ("script", "style", "head"):
            self.skip -= 1
        elif tag in ("h1", "h2", "h3") and self.head is not None:
            self.sections.append((self.n, " ".join("".join(self.head).split())))
            self.head = None
        if tag in ("td", "th", "li", "p", "tr"):
            self._add(" ")

    def handle_data(self, d):
        if self.skip:
            return
        if self.head is not None:
            self.head.append(d)
        self._add(d)

    def _add(self, d):
        d = re.sub(r"\s+", " ", d)
        if self.parts and self.parts[-1].endswith(" ") and d.startswith(" "):
            d = d[1:]
        if d:
            self.parts.append(d)
            self.n += len(d)


def read_doc(doc, rev=None):
    if rev:
        return subprocess.run(["git", "show", f"{rev}:{doc}"], cwd=ROOT, capture_output=True, encoding="utf-8",
                              errors="replace", check=True).stdout
    return (ROOT / doc).read_text(encoding="utf-8")


def doc_text(doc, src):
    """Visible text with whitespace collapsed, and (start, heading) section marks. HUD table dumps are cut out
    (they are checked cell by cell against the CSV files)."""
    if doc.endswith(".md"):
        src = re.sub(r"\]\([^)]*\)", "]", src)                    # drop link targets
        text, secs, n = [], [], 0
        for line in src.splitlines():
            line = " ".join(line.split())
            if line.startswith("#"):
                secs.append((n, line.lstrip("# ")))
            text.append(line + " ")
            n += len(line) + 1
        return "".join(text), secs
    src = re.sub(r"<!--TABLE:.*?</table>", " ", src, flags=re.S)
    p = _Text()
    p.feed(src)
    return "".join(p.parts), p.sections


def section_at(secs, pos):
    s = ""
    for start, name in secs:
        if start <= pos:
            s = name
    return s


def parse_num(m):
    sign, intp, frac, exp = m.groups()
    v = float((("-" if sign else "") + intp.replace("{,}", "").replace(",", "") + (frac or "") + (exp or "").replace("−", "-")))
    dec = len(frac) - 1 if frac else 0
    return v, dec, bool(exp)


def tokens(text):
    """All numbers in the text: (start, end, shown, value, decimals, is_label_reason)."""
    out, spans = [], [(m.start(), m.end()) for m in SPAN_LABEL.finditer(text)]
    for a, b in spans:
        out.append((a, b, text[a:b], None, 0, "date or video timestamp"))
    for m in NUM.finditer(text):
        if any(a <= m.start() < b for a, b in spans):
            continue
        v, dec, exp = parse_num(m)
        before, after = text[max(0, m.start() - 14):m.start()], text[m.end():m.end() + 2]
        label = ""
        if LABEL_BEFORE.search(before):
            label = "label (section, figure, line, version)"
        elif after.startswith(".") and after[1:2].isdigit():
            label = "label (version or file name)"
        elif dec == 0 and 1900 <= v <= 2100 and "," not in m.group(0):
            label = "label (year)"
        out.append((m.start(), m.end(), m.group(0), v, dec, label))
    return sorted(out)


# ---------------------------------------------------------------- tool outputs ----------------------------------------
def inputs_hash():
    h = hashlib.sha256()
    for p in INPUTS:
        h.update((ROOT / p).read_bytes().replace(b"\r\n", b"\n"))
    return h.hexdigest()[:16]


def run_script(name):
    env = dict(os.environ, MPLBACKEND="Agg", PYTHONIOENCODING="utf-8")
    r = subprocess.run([sys.executable, SCRIPTS[name]], cwd=ROOT, capture_output=True, encoding="utf-8", errors="replace",
                       env=env)
    if r.returncode != 0:
        print(f"check.py: {SCRIPTS[name]} failed (exit {r.returncode}):\n{r.stdout[-2000:]}\n{r.stderr[-2000:]}")
        sys.exit(2)
    return r.stdout


def params_text():
    """One line per PARAMS entry, from the sim module as imported (the overhang switch reads sys.argv: "hud" here)."""
    import importlib.util
    spec = importlib.util.spec_from_file_location("sim_params", ROOT / SCRIPTS["sim"])
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    # eval is safe enough here: the expressions are the PARAMS strings above, evaluated on the sim module's names
    return "".join(f"{w} {n} = {float(eval(e, vars(mod))):.10g} {u}".rstrip() + "\n" for n, w, e, u in PARAMS)


def outputs(fast):
    out = _outputs(fast)
    out["par"] = params_text()
    return out


def _outputs(fast):
    if fast:
        if not CACHE.exists():
            print("check.py: out/check_outputs.json is missing; run: python check.py")
            sys.exit(2)
        cache = json.loads(CACHE.read_text(encoding="utf-8"))
        if cache["inputs"] != inputs_hash():
            print("check.py --fast: a script or data file changed since the last full run; run: python check.py")
            sys.exit(2)
        out = dict(cache["outputs"])
        sim = run_script("sim")
        if sim != out["sim"]:
            print("check.py --fast: the sim output differs from the cached run; run: python check.py")
            sys.exit(2)
        return out
    out = {k: run_script(k) for k in SCRIPTS}
    CACHE.write_text(json.dumps({"inputs": inputs_hash(), "outputs": out}, indent=1), encoding="utf-8")
    return out


def lines_of(out):
    """(script key, line text, [(number, words near it)]) for every output line."""
    res = []
    for k, txt in out.items():
        for line in txt.splitlines():
            if line.startswith("wrote "):
                continue
            ln = line.replace("_", " ")
            nums = [(parse_num(m)[0], words(ln if k == "par" else ln[max(0, m.start() - 30):m.end() + 12]))
                    for m in NUM.finditer(ln)]
            if nums:
                res.append((k, line.strip(), nums))
    return res


UNITS = set("ft kg kgf kn bar mm px deg hz kw rad mps nm".split())


def words(s):
    return {w for w in re.findall(r"[a-z][a-z0-9]*", s.lower().replace("_", " "))
            if (len(w) > 1 or w in "xyz") and w not in STOP}


def evaluator(out):
    sim = {}
    for line in out["sim"].splitlines():
        p = line.split()
        if len(p) == 2:
            try:
                sim[p[0]] = float(p[1])
            except ValueError:
                pass

    def S(key):
        return sim[key]

    def L(script, text, n):
        for line in out[script].splitlines():
            if text in line:
                return [parse_num(m)[0] for m in NUM.finditer(line.replace("_", " "))][n - 1]
        raise KeyError(f"no line with {text!r} in {script}")

    def line_of(expr):
        m = re.match(r'S\("(\w+)"\)', expr)
        if m:
            return next((ln.strip() for ln in out["sim"].splitlines() if ln.startswith(m.group(1) + " ")), "")
        m = re.match(r'L\("(\w+)", "([^"]+)"', expr)
        if m:
            return next((ln.strip() for ln in out[m.group(1)].splitlines() if m.group(2) in ln), "")
        return ""

    # eval is safe enough here: expressions come only from this repo's PINS / claims.csv, no builtins, only S and L
    return (lambda expr: eval(expr,{"__builtins__": {}, "abs": abs}, {"S": S, "L": L})), line_of


def tol_of(dec, shown):
    return 0.5 * 10 ** (-dec) + 1e-9


# ---------------------------------------------------------------- inventory -------------------------------------------
def inventory(out):
    ev, line_of = evaluator(out)
    olines = lines_of(out)
    rows, used_pins = [], set()
    for doc in DOCS:
        text, secs = doc_text(doc, read_doc(doc))
        hist_text = [h[2] for h in HISTORICAL if h[0] == doc and h[1] == "text"]
        hist_secs = [h[2] for h in HISTORICAL if h[0] == doc and h[1] == "section"]
        hist_end = [h[2] for h in HISTORICAL if h[0] == doc and h[1] == "end"]
        for a, b, shown, v, dec, label in tokens(text):
            label = label or label_of(text, secs, a, b, shown, v, dec)
            sec = section_at(secs, a)
            ctx = text[max(0, a - 60):a]
            after = text[b:b + 25]
            row = dict(id="", doc=doc, section=sec, context=ctx, value=shown, after=after, evidence="none", check="no",
                       script="", output_line="", expr="", tol="", note="")
            sentence = text[max(0, text.rfind(". ", 0, a) + 1 if text.rfind(". ", 0, a) > a - 400 else a - 400):b + 200]
            if label:
                row["note"] = label
            elif (sec in hist_secs or any(h in text[max(0, a - 400):a] for h in hist_text)
                  or any(ctx.rstrip().endswith(h) for h in hist_end)):
                row["note"] = "historical (change log or quoted old run); not compared"
                row["evidence"] = classify(sentence)
            else:
                pin = next((p for p in PINS if p[0] == doc and ctx.rstrip().endswith(p[1])), None)
                if pin:
                    used_pins.add(pin)
                    row.update(expr=pin[2], match="pin", note=pin[3] or "pinned")
                    if len(pin) > 4:
                        row["tol"] = str(pin[4])
                elif not any(n[0] == doc and ctx.rstrip().endswith(n[1]) for n in NOAUTO):
                    m = auto_match(v, dec, text[max(0, a - 35):b + 12], olines, text[b:b + 30])
                    if m:
                        row.update(expr=m, note="auto match (value + words)")
                if row["expr"]:
                    row["output_line"] = line_of(row["expr"]) if row["expr"].count("(") == 1 else row["expr"]
                    key = m.group(1) if (m := re.search(r'L\("(\w+)"', row["expr"])) else "sim"
                    row["script"] = SCRIPTS["sim"] + " (constants)" if key == "par" else SCRIPTS[key]
                    row["check"] = "yes"
                    row["tol"] = row["tol"] or f"{tol_of(dec, shown):.3g}"
                    row["evidence"] = "HUD" if re.search(r"hud|free-fall fit", row["output_line"], re.I) else "model-only"
                else:
                    row["evidence"] = classify(sentence)
                    cited = (doc, sec) in CITED or any(c[0] == doc and ctx.rstrip().endswith(c[1]) for c in CITED)
                    row["note"] = "cited source (press or video); no tool" if cited else "no tool prints it"
            rows.append(row)
        for name in re.findall(r"<!--TABLE:([\w.]+)-->", read_doc(doc)):
            rows.append(dict(id="", doc=doc, section="table", context=f"<!--TABLE:{name}-->", value="(table)", after="",
                             evidence="HUD", check="yes", script=f"data/{name}", output_line="every cell",
                             expr=f"TABLE:{name}", tol="0", note="HTML table must equal the CSV cell by cell"))
    for p in PINS:
        if p not in used_pins:
            print(f"check.py --inventory: pin not found in {p[0]}: {p[1]!r}")
    for k, r in enumerate(rows, 1):
        r["id"] = k
        r.pop("match", None)
    with CLAIMS.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, FIELDS)
        w.writeheader()
        w.writerows(rows)
    n = len(rows)
    by = {}
    for r in rows:
        by.setdefault((r["evidence"], r["check"]), 0)
        by[(r["evidence"], r["check"])] += 1
    print(f"wrote claims.csv: {n} rows; checked {sum(r['check'] == 'yes' for r in rows)}; "
          f"labels {sum(r['note'].startswith('label') or r['note'].startswith('date') for r in rows)}; "
          f"historical {sum(r['note'].startswith('historical') for r in rows)}; "
          f"no tool {sum(r['note'] == 'no tool prints it' for r in rows)}")
    for k in sorted(by):
        print(f"   evidence {k[0]:10s} check {k[1]:3s}: {by[k]}")


def label_of(text, secs, a, b, shown, v, dec):
    """Numbers that name something instead of measuring it: section numbers (heading text and the contents list),
    code line numbers after "file.py:", and small integers inside a formula (exponents, indices, 1/2 factors)."""
    if any(text.startswith(name, a) for _, name in secs):
        return "label (section number)"
    py = text.rfind(".py:", max(0, a - 250), a)
    if dec == 0 and py >= 0 and ". " not in text[py + 4:a]:
        return "label (code line number)"
    if dec == 0 and abs(v) <= 10 and in_math(text, a):
        return "label (formula constant or index)"
    return ""


def in_math(text, a):
    pre = text[:a]                                   # a window can cut off the "$$" that opens a display formula
    return pre.count("$$") % 2 == 1 or pre.rfind("\\(") > pre.rfind("\\)")


def classify(s):
    for name, pat in EVID_WORDS:
        if re.search(pat, s):
            return name
    return "none"


def auto_match(v, dec, ctx, olines, after=""):
    """Pin a doc number to an output number with the same value at the shown precision and at least one shared
    non-unit word near both numbers (doc: 35 chars before, 12 after; output: 30 before, 12 after; a "par" line: all of
    it, and the value must be equal, not rounded, with the unit after the doc number). Fewer than 3 significant
    digits: only a "par" line (sim constant) can match. 4 or more: the value alone is enough (score 0.5), except the
    time stamps that start the flip.py grid lines ("t 53.45: ..."), which are not results."""
    sig = len(f"{abs(v):.{dec}f}".replace(".", "").lstrip("0"))
    cw = words(ctx) - UNITS
    best, best_score = "", 0
    for k, line, nums in olines:
        if sig < 3 and k != "par":
            continue
        for n, (x, lw) in enumerate(nums, 1):
            if abs(round(x, dec) - v) > 1e-9 * max(1, abs(v)):
                continue
            if k == "par":
                unit = line.split(" = ")[1].partition(" ")[2]
                tail = re.sub(r"^[\s\\(){}/,.\d+−–→-]*", "",after.replace("\\text{", ""))   # "/ 2,900 / 3,200 kg"
                if abs(x - v) > 1e-9 * max(1, abs(v)) or not tail.startswith(unit):
                    continue
            if k == "flip" and n == 1 and re.match(r"t \d", line):
                continue
            score = len(cw & lw) or (0.5 if sig >= 4 else 0)
            if score > best_score:
                if k == "sim":
                    best = f'S("{line.split()[0]}")'
                else:
                    key = line
                    for cut in range(12, len(line) + 1, 4):     # shortest line prefix that is unique in that output
                        key = line[:cut]
                        if sum(key in ln for kk, ln, _ in olines if kk == k) == 1:
                            break
                    best = f'L("{k}", "{key.replace(chr(34), chr(39))}", {n})'
                best_score = score
    return best


# ---------------------------------------------------------------- check -----------------------------------------------
def check(out, rev=None):
    ev, _ = evaluator(out)
    rows = list(csv.DictReader(CLAIMS.open(encoding="utf-8")))
    texts = {d: doc_text(d, read_doc(d, rev))[0] for d in DOCS}
    raw = {d: read_doc(d, rev) for d in DOCS}
    bad, ok = [], 0
    for r in rows:
        if r["check"] != "yes":
            continue
        if r["expr"].startswith("TABLE:"):
            err = check_table(raw[r["doc"]], r["expr"][6:])
            if err:
                bad.append(f"{r['doc']} table {r['expr'][6:]}: {err}")
            else:
                ok += 1
            continue
        found = find_value(texts[r["doc"]], r["context"], r["value"])
        if found is None:
            bad.append(f"{r['doc']} [{r['section']}] anchor missing: ...{r['context'][-40:]!r} (doc edited? run --inventory)")
            continue
        shown, v, dec = found
        try:
            fresh = ev(r["expr"])
        except Exception as e:                                   # noqa: BLE001
            bad.append(f"{r['doc']} [{r['section']}] {r['expr']}: cannot read the output ({e})")
            continue
        tol = max(float(r["tol"] or 0), tol_of(dec, shown))
        if abs(fresh - v) > tol:
            bad.append(f"{r['doc']} [{r['section']}] ...{r['context'][-45:]} {shown} | fresh {fresh:g} "
                       f"({r['expr']}) | tol {tol:g}")
        else:
            ok += 1
    return ok, bad


def find_value(text, context, value):
    """Locate the number that follows the stored context; numbers inside the context match any number, so a change of a
    neighbour does not lose the anchor. Prefer the occurrence that shows the stored value."""
    tail = context[-30:]
    if len(context) > 30 and 0 <= tail.find(" ") < 15:              # do not start inside a word or a number
        tail = tail[tail.find(" "):]
    pat, k = "", 0
    for m in NUM.finditer(tail):
        pat += re.escape(tail[k:m.start()]) + r"[-−]?(?:[\d,]|\{,\})+(?:\.\d+)?(?:[eE][-−]?\d+)?"
        k = m.end()
    pat += re.escape(tail[k:])
    hits = []
    for h in re.finditer(pat, text):
        m = NUM.match(text, h.end())
        if m:
            v, dec, _ = parse_num(m)
            hits.append((m.group(0), v, dec))
    if not hits:
        return None
    return next((h for h in hits if h[0] == value), hits[0])


def check_table(src, name):
    m = re.search(rf"<!--TABLE:{re.escape(name)}-->(.*?)</table>", src, re.S)
    cells = [re.findall(r"<t[dh]>(.*?)</t[dh]>", tr) for tr in re.findall(r"<tr>(.*?)</tr>", m.group(1))]
    with (ROOT / "data" / name).open(encoding="utf-8") as f:
        data = [r for r in csv.reader(line for line in f if not line.startswith("#")) if r]
    if [[c.strip() for c in r] for r in data] != cells:
        diff = next((k for k, (x, y) in enumerate(zip(data, cells)) if [c.strip() for c in x] != y), min(len(data), len(cells)))
        return f"differs from data/{name} at row {diff} ({len(cells)} vs {len(data)} rows)"
    return ""


def git_artifact_stale():
    r = subprocess.run(["git", "diff", "--quiet", "--", "out/replay_data.js"], cwd=ROOT)
    return r.returncode != 0


def install_hook():
    hook = ROOT / ".git" / "hooks" / "pre-commit"
    hook.write_text("#!/bin/sh\n# installed by: python check.py --install-hook\nexec python check.py --fast\n", newline="\n")
    hook.chmod(0o755)
    print(f"wrote {hook}")


def main(argv):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    if "--install-hook" in argv:
        install_hook()
        return 0
    fast = "--fast" in argv
    rev = argv[argv.index("--rev") + 1] if "--rev" in argv else None
    out = outputs(fast)
    if "--inventory" in argv:
        inventory(out)
        return 0
    ok, bad = check(out, rev)
    stale = git_artifact_stale() if rev is None else False
    print(f"check.py ({'fast' if fast else 'full'}{', docs at ' + rev if rev else ''}): {ok} claims agree, {len(bad)} do not")
    for b in bad:
        print("  MISMATCH", b)
    if stale:
        print("  STALE out/replay_data.js differs from the git index after the sim rerun: stage the fresh file")
    return 1 if bad or stale else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
