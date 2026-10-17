"""Sound bed for the broadcast cut (sim t 44.0-57.98 -> 0-13.98 s). ElevenLabs SFX takes in out/sfx/ (2026-10-17).
Jet loudness follows the sim's total nozzle command; the climb bed follows the sim's wall speed. Writes out/sfx/broadcast_mix.wav."""
import json, subprocess, wave
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
SR, T0, T1 = 44100, 44.0, 57.98
D = ROOT / "out" / "sfx"


def load(name):
    raw = subprocess.run(["ffmpeg", "-v", "error", "-i", str(D / name), "-f", "s16le", "-ac", "1", "-ar", str(SR), "-"],
                         capture_output=True, check=True).stdout
    return np.frombuffer(raw, np.int16).astype(float) / 32768


def loop_to(x, n, fade=2205):                       # crossfaded loop
    out = np.zeros(n)
    pos, w = 0, np.linspace(0, 1, fade)
    while pos < n:
        seg = x[:min(len(x), n - pos)].copy()
        if pos:
            k = min(fade, len(seg))
            seg[:k] = seg[:k] * w[:k] + out[pos:pos + k] * (1 - w[:k])
            out[pos:pos + len(seg)] = 0
        out[pos:pos + len(seg)] += seg
        if len(seg) <= fade:                         # tail shorter than the crossfade: done
            break
        pos += len(seg) - fade
    return out


txt = (ROOT / "out" / "replay_data.js").read_text(encoding="utf-8")
SIM = json.loads(txt[txt.index("{"):txt.rindex("}") + 1])
F = np.array(SIM["frames"])
n = int((T1 - T0) * SR)
t = T0 + np.arange(n) / SR
v = np.r_[0, np.linalg.norm(np.diff(F[:, 1:4], axis=0), axis=1) / np.maximum(np.diff(F[:, 0]), 1e-3)]
speed = np.interp(t, F[:, 0], v) * (t <= 52.25)
noz = np.interp(t, F[:, 0], F[:, 8:18].sum(1))      # columns 8-17: nozzle commands (0-1 each)
td = SIM["report"]["touchdown_sim_t"]

mix = np.zeros(n)
mix += 0.6 * loop_to(load("climb_a.mp3"), n) * np.clip(speed / 9.0, 0, 1)          # motor + tyres follow wall speed
jets = loop_to(load("jets_b.mp3")[:int(2.2 * SR)], n, fade=8820)   # sustained first 2.2 s only; the take decays after
mix += 0.9 * jets * np.clip(noz / 2.0, 0, 1) ** 0.7                               # cold-gas roar follows nozzle command
land = load("land_a.mp3")
i = int((td - T0) * SR)
mix[i:i + len(land)] += 0.9 * land[:n - i]
crowd = load("crowd_a.mp3")
i = int((td + 0.2 - T0) * SR)
c = crowd[:n - i] * np.clip(np.arange(n - i) / (0.8 * SR), 0, 1)                  # cheer swells 0.8 s after touchdown
mix[i:] += 0.7 * c
mix *= 0.89 / max(1e-9, np.abs(mix).max())                                         # peak -1 dBFS
with wave.open(str(D / "broadcast_mix.wav"), "wb") as w:
    w.setnchannels(1); w.setsampwidth(2); w.setframerate(SR)
    w.writeframes((mix * 32767).astype(np.int16).tobytes())
print("wrote", D / "broadcast_mix.wav", f"{n / SR:.2f} s, touchdown {td:.2f}")
