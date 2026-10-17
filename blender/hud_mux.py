"""Broadcast-style HUD (G-FORCE left, ALTITUDE right, as in PCMag 1:41-2:00) + sound bed, muxed onto a rendered cut.
Values are the sim's, shown with the measured HUD lag (0.30 s) so the gauges behave like the broadcast ones.
Run: python blender/hud_mux.py out/broadcast_draft.mp4 out/broadcast_draft_hud.mp4
     (input must be the broadcast EDL: sim t 44.0 -> frame 1, 30 fps; audio from out/sfx/broadcast_mix.wav)"""
import json
import subprocess
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parent.parent
FPS, T0, LAG = 30, 44.0, 0.30
txt = (ROOT / "out" / "replay_data.js").read_text(encoding="utf-8")
SIM = json.loads(txt[txt.index("{"):txt.rindex("}") + 1])
F = np.array(SIM["frames"])
N = int(np.floor((F[-1, 0] - T0) * FPS)) + 1
OD = ROOT / "out" / "hud"
OD.mkdir(parents=True, exist_ok=True)


def font(size):
    for name in ("bahnschrift.ttf", "segoeui.ttf", "arial.ttf"):
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            continue
    return ImageFont.load_default()


BIG, SMALL = font(30), font(11)


def gauge(d, cx, cy, label, value, unit, frac):
    r = 34
    d.ellipse([cx - r, cy - r, cx + r, cy + r], outline=(255, 255, 255, 90), width=2)
    d.arc([cx - r, cy - r, cx + r, cy + r], 135, 135 + 270 * float(np.clip(frac, 0, 1)), fill=(255, 255, 255, 230), width=3)
    d.text((cx, cy - 17), label, font=SMALL, fill=(255, 255, 255, 200), anchor="mm")
    d.text((cx, cy + 3), value, font=BIG, fill=(255, 255, 255, 255), anchor="mm")
    d.text((cx, cy + 22), unit, font=SMALL, fill=(255, 255, 255, 170), anchor="mm")


for f in range(1, N + 1):
    t = T0 + (f - 1) / FPS - LAG
    g = float(np.interp(t, F[:, 0], F[:, 18]))
    alt = max(0.0, float(np.interp(t, F[:, 0], F[:, 19])))
    im = Image.new("RGBA", (1280, 720), (0, 0, 0, 0))
    d = ImageDraw.Draw(im)
    gauge(d, 72, 646, "G-FORCE", f"{g:.1f}", "G", g / 4.0)
    gauge(d, 1208, 646, "ALTITUDE", f"{alt:.1f}", "FT", alt / 70.0)
    d.text((24, 700), "SIMULATION", font=SMALL, fill=(255, 255, 255, 150))
    im.save(OD / f"{f:04d}.png")

src, dst = sys.argv[1], sys.argv[2]
subprocess.run(["ffmpeg", "-v", "error", "-y", "-i", src, "-framerate", str(FPS), "-i", str(OD / "%04d.png"),
                "-i", str(ROOT / "out" / "sfx" / "broadcast_mix.wav"),
                "-filter_complex", "[0:v][1:v]overlay=0:0:shortest=1[v]", "-map", "[v]", "-map", "2:a",
                "-c:v", "libx264", "-crf", "18", "-pix_fmt", "yuv420p", "-c:a", "aac", "-b:a", "192k", "-shortest", dst],
               check=True)
print("wrote", dst, N, "HUD frames")
