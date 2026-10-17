"""Photoreal Blender rebuild of the PCMag wall-flip run (sim t 44.0-57.98 s; PCMag t = sim t + 61.35 s, see calib_cameras.py).

Builds wall + truss + venue + Roadster from out/replay_data.js, keyframes the car and the 10 plumes
(30 fps, t = 44.0 -> frame 1), sets the two broadcast cameras, renders.
Run (headless):
  blender -b --factory-startup -P blender/build_scene.py -- stills 44.0 48.45 50.85  # -> out/stills/t_<t>.png
  blender -b --factory-startup -P blender/build_scene.py -- anim [first last]         # -> out/frames_eevee/####.png
  blender -b --factory-startup -P blender/build_scene.py -- edl coverage|broadcast  # -> out/<name>_draft.mp4
  blender -b --factory-startup -P blender/build_scene.py -- cams 53.0              # one still per camera
  options: --standard (view transform Standard, default AgX), --save (writes out/roadster_wall.blend),
           --res 50 (resolution %), --samples 32
Camera shots (PCMag 1:41-2:00, scene cut found by ffmpeg at PCMag 106.04 s = sim 44.69 s):
  A  sim -44.69      low camera behind the car on the approach road, looking +x at the wall foot
  B  sim 44.69-end   distant long-lens camera on the -x/-y side, one continuous shot: locked off,
                     tilts up for the release, pans left and widens for the fall, tightens at the landing
"""
import json
import math
import sys
from pathlib import Path

import bpy
import numpy as np
from mathutils import Matrix, Quaternion, Vector

ROOT = Path(__file__).resolve().parent.parent
FPS, T0 = 30, 44.0
ARGS = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []


def opt(name, default):
    return type(default)(ARGS[ARGS.index(name) + 1]) if name in ARGS else default


def t_of(f):
    return T0 + (f - 1) / FPS


def f_of(t):
    return int(round((t - T0) * FPS)) + 1


# ---------------- data ----------------
txt = (ROOT / "out" / "replay_data.js").read_text(encoding="utf-8")
SIM = json.loads(txt[txt.index("{"):txt.rindex("}") + 1])
FR = np.array(SIM["frames"], float)
CAR = SIM["car"]
NZ = SIM["nozzles"]
N_FRAMES = int(math.floor((FR[-1, 0] - T0) * FPS)) + 1          # 420


def resample():
    """Sim rows are 0.03 s apart with a non-uniform join at release -> exact 30 fps (lerp + slerp)."""
    t_s = FR[:, 0]
    keep = np.r_[True, np.diff(t_s) > 1e-6]                        # drop duplicate times at the join
    t_s, rows = t_s[keep], FR[keep]
    out = []
    for f in range(1, N_FRAMES + 1):
        t = t_of(f)
        i = int(np.clip(np.searchsorted(t_s, t) - 1, 0, len(t_s) - 2))
        u = float(np.clip((t - t_s[i]) / (t_s[i + 1] - t_s[i]), 0, 1))
        r = rows[i] + (rows[i + 1] - rows[i]) * u
        qa, qb = Quaternion(rows[i][4:8]), Quaternion(rows[i + 1][4:8])   # sim (w,x,y,z) = Blender order
        if qa.dot(qb) < 0:
            qb.negate()
        out.append((t, Vector(r[1:4]), qa.slerp(qb, u), r[8:18].copy(), r))
    # quaternion sign continuity across frames (keyframe interpolation would spin the long way)
    for k in range(1, len(out)):
        if out[k][2].dot(out[k - 1][2]) < 0:
            out[k][2].negate()
    return out


# ---------------- helpers ----------------
def new_obj(name, mesh, parent=None):
    o = bpy.data.objects.new(name, mesh)
    bpy.context.scene.collection.objects.link(o)
    if parent:
        o.parent = parent
    return o


def mesh_from(name, verts, faces, smooth=True):
    me = bpy.data.meshes.new(name)
    me.from_pydata([tuple(v) for v in verts], [], faces)
    me.update()
    if smooth:
        for p in me.polygons:
            p.use_smooth = True
    return me


def principled(name, rgb, rough=0.5, metal=0.0, coat=0.0, emit=None, emit_str=0.0):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    b = m.node_tree.nodes["Principled BSDF"]
    b.inputs["Base Color"].default_value = (*rgb, 1)
    b.inputs["Roughness"].default_value = rough
    b.inputs["Metallic"].default_value = metal
    b.inputs["Coat Weight"].default_value = coat
    if emit:
        b.inputs["Emission Color"].default_value = (*emit, 1)
        b.inputs["Emission Strength"].default_value = emit_str
    return m


def emissive(name, rgb, strength):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    nt = m.node_tree
    nt.nodes.remove(nt.nodes["Principled BSDF"])
    e = nt.nodes.new("ShaderNodeEmission")
    e.inputs["Color"].default_value = (*rgb, 1)
    e.inputs["Strength"].default_value = strength
    nt.links.new(e.outputs[0], nt.nodes["Material Output"].inputs["Surface"])
    return m


def box(name, size, loc, mat, parent=None, bevel=0.0):
    sx, sy, sz = (s / 2 for s in size)
    v = [(x, y, z) for x in (-sx, sx) for y in (-sy, sy) for z in (-sz, sz)]
    f = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    o = new_obj(name, mesh_from(name, v, f, smooth=False), parent)
    o.location = loc
    o.data.materials.append(mat)
    if bevel:
        md = o.modifiers.new("bev", "BEVEL")
        md.width, md.segments = bevel, 2
    return o


def cylinder(name, r, depth, loc, mat, parent=None, n=32, axis="Y"):
    v, f = [], []
    for i in range(n):
        a = 2 * math.pi * i / n
        v += [(r * math.cos(a), r * math.sin(a), -depth / 2), (r * math.cos(a), r * math.sin(a), depth / 2)]
        f.append((2 * i, 2 * ((i + 1) % n), 2 * ((i + 1) % n) + 1, 2 * i + 1))
    f += [tuple(range(0, 2 * n, 2))[::-1], tuple(range(1, 2 * n, 2))]
    o = new_obj(name, mesh_from(name, v, f), parent)
    if axis == "Y":
        o.rotation_euler = (math.pi / 2, 0, 0)
    elif axis == "X":
        o.rotation_euler = (0, math.pi / 2, 0)
    o.location = loc
    o.data.materials.append(mat)
    return o


def loft(name, stations, n_exp, ring=32):
    """Closed body from superellipse cross sections. stations: (x, half_width, z_bottom, z_top)."""
    v, f = [], []
    for x, hw, zb, zt in stations:
        zc, hh = (zb + zt) / 2, (zt - zb) / 2
        for j in range(ring):
            a = 2 * math.pi * j / ring
            c, s = math.cos(a), math.sin(a)
            v.append((x, hw * math.copysign(abs(c) ** (2 / n_exp), c), zc + hh * math.copysign(abs(s) ** (2 / n_exp), s)))
    for i in range(len(stations) - 1):
        for j in range(ring):
            a, b = i * ring + j, i * ring + (j + 1) % ring
            f.append((a, b, b + ring, a + ring))
    f.append(tuple(range(ring)))
    last = (len(stations) - 1) * ring
    f.append(tuple(range(last, last + ring))[::-1])
    o = new_obj(name, mesh_from(name, v, f))
    return o


# ---------------- scene ----------------
bpy.ops.wm.read_factory_settings(use_empty=True)
sc = bpy.context.scene
sc.render.fps = FPS
sc.frame_start, sc.frame_end = 1, N_FRAMES
bpy.context.preferences.edit.keyframe_new_interpolation_type = "LINEAR"

# ---- world: black venue with thin haze ----
world = bpy.data.worlds.new("venue")
sc.world = world
world.use_nodes = True
wn = world.node_tree
wn.nodes["Background"].inputs["Color"].default_value = (0.0015, 0.0018, 0.0025, 1)
wn.nodes["Background"].inputs["Strength"].default_value = 1.0
vol = wn.nodes.new("ShaderNodeVolumePrincipled")
vol.inputs["Density"].default_value = float(opt("--haze", 0.00015))   # look pass: thinner veil (shot A read grey)
vol.inputs["Anisotropy"].default_value = 0.55
vol.inputs["Color"].default_value = (0.8, 0.85, 0.9, 1)
wn.links.new(vol.outputs[0], wn.nodes["World Output"].inputs["Volume"])

# ---- ground ----
asphalt = bpy.data.materials.new("asphalt")
asphalt.use_nodes = True
nt = asphalt.node_tree
b = nt.nodes["Principled BSDF"]
noise = nt.nodes.new("ShaderNodeTexNoise")
noise.inputs["Scale"].default_value = 3.0
noise.inputs["Detail"].default_value = 8.0
ramp = nt.nodes.new("ShaderNodeValToRGB")
ramp.color_ramp.elements[0].color = (0.005, 0.005, 0.006, 1)   # look pass: venue floor reads near-black (pcmag_105.0, 117.3)
ramp.color_ramp.elements[1].color = (0.014, 0.014, 0.015, 1)
nt.links.new(noise.outputs["Fac"], ramp.inputs["Fac"])
nt.links.new(ramp.outputs["Color"], b.inputs["Base Color"])
rr = nt.nodes.new("ShaderNodeMapRange")
rr.inputs["To Min"].default_value, rr.inputs["To Max"].default_value = 0.75, 0.95   # matte venue floor: no grazing sheen
nt.links.new(noise.outputs["Fac"], rr.inputs["Value"])
nt.links.new(rr.outputs["Result"], b.inputs["Roughness"])
b.inputs["Specular IOR Level"].default_value = float(opt("--floorspec", 0.0))   # 0.5 = default dielectric; the floor read grey from its sheen (pcmag_105.0 floor ~10/255)
G = 300
ground = new_obj("ground", mesh_from("ground", [(-G, -G, 0), (G, -G, 0), (G, G, 0), (-G, G, 0)], [(0, 1, 2, 3)], False))
ground.data.materials.append(asphalt)

# ground LED strips along the approach road (pc_23, pc_29: red line on the floor left of the wall foot)
red_led = emissive("red_led", (1.0, 0.03, 0.02), 8.0)
# one strip across the view (it reads horizontal in camera B), on the screen-left (+y) side of the foot
box("floor_led", (0.08, 54, 0.03), (-8, 33, 0.015), red_led)

# ---- wall: ribbon from the measured profile (SIM.wall) ----
wx, wz, W = np.array(SIM["wall"]["x"]), np.array(SIM["wall"]["z"]), SIM["wall"]["width"]
step = 3
P = np.c_[wx[::step], wz[::step]]
if not np.allclose(P[-1], [wx[-1], wz[-1]]):
    P = np.vstack([P, [wx[-1], wz[-1]]])
P = np.vstack([[-1.5, 0.0], P])                                   # short apron in front of the foot
tang = np.gradient(P, axis=0)
tang /= np.linalg.norm(tang, axis=1)[:, None]
nrm = np.c_[-tang[:, 1], tang[:, 0]]                              # left normal = toward the car (concave side)
THK = 0.35
verts, faces = [], []
for (x, z), (nx_, nz_) in zip(P, nrm):
    for y in (-W / 2, W / 2):
        verts.append((x, y, z))
    for y in (-W / 2, W / 2):
        verts.append((x - THK * nx_, y, z - THK * nz_))
n = len(P)
for i in range(n - 1):
    a, b_ = 4 * i, 4 * (i + 1)
    faces += [(a, a + 1, b_ + 1, b_), (a + 2, b_ + 2, b_ + 3, a + 3), (a, b_, b_ + 2, a + 2), (a + 1, a + 3, b_ + 3, b_ + 1)]
faces += [(4 * (n - 1), 4 * (n - 1) + 2, 4 * (n - 1) + 3, 4 * (n - 1) + 1)]
wall = new_obj("wall", mesh_from("wall", verts, faces, smooth=False))   # dense enough; smooth shading banded

concrete = bpy.data.materials.new("concrete")
concrete.use_nodes = True
nt = concrete.node_tree
b = nt.nodes["Principled BSDF"]
tc = nt.nodes.new("ShaderNodeTexCoord")
n1 = nt.nodes.new("ShaderNodeTexNoise")
n1.inputs["Scale"].default_value = 0.35
n1.inputs["Detail"].default_value = 6.0
n1.inputs["Roughness"].default_value = 0.65
n2 = nt.nodes.new("ShaderNodeTexNoise")
n2.inputs["Scale"].default_value = 9.0
n2.inputs["Detail"].default_value = 10.0
nt.links.new(tc.outputs["Object"], n1.inputs["Vector"])
nt.links.new(tc.outputs["Object"], n2.inputs["Vector"])
stain = nt.nodes.new("ShaderNodeValToRGB")
stain.color_ramp.elements[0].position, stain.color_ramp.elements[1].position = 0.35, 0.7
stain.color_ramp.elements[0].color = (0.22, 0.22, 0.21, 1)
stain.color_ramp.elements[1].color = (0.48, 0.48, 0.46, 1)
nt.links.new(n1.outputs["Fac"], stain.inputs["Fac"])
grain = nt.nodes.new("ShaderNodeMixRGB")
grain.blend_type = "MULTIPLY"
grain.inputs["Fac"].default_value = 0.25
nt.links.new(stain.outputs["Color"], grain.inputs["Color1"])
nt.links.new(n2.outputs["Color"], grain.inputs["Color2"])
# white centre line: |object y| < 0.06 (PCMag 1:44: one painted line on the centre of the track)
sep = nt.nodes.new("ShaderNodeSeparateXYZ")
nt.links.new(tc.outputs["Object"], sep.inputs[0])
ab = nt.nodes.new("ShaderNodeMath")
ab.operation = "ABSOLUTE"
nt.links.new(sep.outputs["Y"], ab.inputs[0])
lt = nt.nodes.new("ShaderNodeMath")
lt.operation = "LESS_THAN"
lt.inputs[1].default_value = 0.06
nt.links.new(ab.outputs[0], lt.inputs[0])
line = nt.nodes.new("ShaderNodeMixRGB")
line.inputs["Color2"].default_value = (0.85, 0.85, 0.83, 1)
nt.links.new(lt.outputs[0], line.inputs["Fac"])
nt.links.new(grain.outputs["Color"], line.inputs["Color1"])
nt.links.new(line.outputs["Color"], b.inputs["Base Color"])
b.inputs["Roughness"].default_value = 0.85
bump = nt.nodes.new("ShaderNodeBump")
bump.inputs["Strength"].default_value = 0.15
nt.links.new(n2.outputs["Fac"], bump.inputs["Height"])
nt.links.new(bump.outputs["Normal"], b.inputs["Normal"])
wall.data.materials.append(concrete)


def tube_curve(name, polylines, radius, mat):
    cu = bpy.data.curves.new(name, "CURVE")
    cu.dimensions = "3D"
    cu.bevel_depth, cu.bevel_resolution = radius, 1
    for pts in polylines:
        sp = cu.splines.new("POLY")
        sp.points.add(len(pts) - 1)
        for p, q in zip(sp.points, pts):
            p.co = (*q, 1)
    o = new_obj(name, cu)
    o.data.materials.append(mat)
    return o


# red LED strips on both track edges, 2 cm above the surface
edge = [[(x + 0.02 * nx_, y, z + 0.02 * nz_) for (x, z), (nx_, nz_) in zip(P[1:], nrm[1:])] for y in (W / 2 - 0.03, -W / 2 + 0.03)]
tube_curve("wall_led", edge, 0.025, red_led)

# steel truss on the convex side (pc_full_104, pc_23, flip_01): two lattice planes at the track edges, 1.6 m deep
steel = principled("steel", (0.12, 0.12, 0.13), rough=0.45, metal=0.9)
s_arc = np.r_[0, np.cumsum(np.hypot(*np.diff(P, axis=0).T))]
st = np.interp(np.arange(0, s_arc[-1], 1.6), s_arc, np.arange(len(P))).astype(int)
DEPTH = 1.6
segs = []
for y in (-W / 2 - 0.1, W / 2 + 0.1):
    inner = [(P[i][0] - THK * nrm[i][0], y, P[i][1] - THK * nrm[i][1]) for i in st]
    outer = [(P[i][0] - (THK + DEPTH) * nrm[i][0], y, max(0.0, P[i][1] - (THK + DEPTH) * nrm[i][1])) for i in st]
    segs += [[a, b_] for a, b_ in zip(inner, outer)]
    segs += [outer]
    segs += [[inner[k], outer[k + 1]] for k in range(len(st) - 1)]
for k in range(0, len(st), 2):                                    # cross members between the two planes
    i = st[k]
    for d in (THK, THK + DEPTH):
        segs.append([(P[i][0] - d * nrm[i][0], y0, max(0.0, P[i][1] - d * nrm[i][1])) for y0 in (-W / 2 - 0.1, W / 2 + 0.1)])
tube_curve("truss", segs, 0.06, steel)

# ---- venue lights ----
def spot(name, loc, target, energy, size_deg, rgb=(1, 1, 1), blend=0.3, radius=0.3):
    L = bpy.data.lights.new(name, "SPOT")
    L.energy, L.color = energy, rgb
    L.spot_size, L.spot_blend = math.radians(size_deg), blend
    L.shadow_soft_size = radius
    o = new_obj(name, L)
    o.location = loc
    o.rotation_euler = (Vector(target) - Vector(loc)).to_track_quat("-Z", "Y").to_euler()
    return o


spot("flood_L", (-55, -15, 30), (2, 0, 10), 3.0e5, 26, (0.95, 0.97, 1.0))
spot("flood_R", (-55, 10, 30), (2, 0, 10), 2.0e5, 26, (0.95, 0.97, 1.0))
spot("foot_key", (4, 0, 16), (0, 0, 0.5), 6.0e3, 35, (1.0, 0.98, 0.95), blend=0.6, radius=1.0)   # from the wall side: car rear stays dark (pc_t44.0)
spot("landing_flood", (-40, -12, 25), (-21, 0, 2), 3.0e5, 50, (0.95, 0.97, 1.0), blend=1.0, radius=0.5)   # lights the condensation cloud
for k, (x, y) in enumerate([(-45, 35), (-20, 45), (25, 40), (40, -30)]):   # teal show beams in the haze
    spot(f"beam{k}", (x, y, 0.5), (x + 3, y - 4, 40), 4.0e4, 6, (0.35, 0.85, 0.9), blend=0.15, radius=0.05)
lamp = emissive("lamp", (1.0, 0.92, 0.8), 25.0)
# ground-level white lamps along the floor strip, left of the landing (PCMag 114.8 / 115.8)
for k, y in enumerate((10.0, 12.5, 15.0, 17.5)):
    o = cylinder(f"floorlamp{k}", 0.25, 0.1, (-8.6, y, 0.6), emissive("floorlamp", (0.9, 0.95, 1.0), 150.0), axis="X")
rng = np.random.default_rng(7)
for k in range(6):                                                # distant venue lamps (dots in the black)
    a = rng.uniform(-0.6, 2.2)
    r = rng.uniform(70, 140)
    o = new_obj(f"farlamp{k}", mesh_from(f"farlamp{k}", [(0, 0, 0.25), (0.25, 0, 0), (0, 0.25, 0), (-0.25, 0, 0), (0, -0.25, 0), (0, 0, -0.25)],
                                         [(0, 1, 2), (0, 2, 3), (0, 3, 4), (0, 4, 1), (5, 2, 1), (5, 3, 2), (5, 4, 3), (5, 1, 4)]))
    o.location = (r * math.cos(a), r * math.sin(a), rng.uniform(6, 22))
    o.data.materials.append(lamp)

# ---------------- the Roadster (body frame: nose +x, left +y, origin at CG, ground at z = -h_cg) ----------------
car = new_obj("roadster", None)
car.rotation_mode = "QUATERNION"
HC = CAR["h_cg"]
sys.path.insert(0, str(Path(__file__).resolve().parent))
import roadster  # noqa: E402  (car model lives in its own file)

wheels, WR = roadster.build(car, CAR, NZ, sys.modules[__name__])

# ---- plumes: emissive core cone + scattering cloud cone, apex at the nozzle, along -dir ----
def cone_mesh(name, r_end, n=24):
    v = [(0, 0, 0)] + [(r_end * math.cos(2 * math.pi * k / n), r_end * math.sin(2 * math.pi * k / n), 1.0) for k in range(n)]
    f = [(0, k + 1, (k + 1) % n + 1) for k in range(n)] + [tuple(range(n, 0, -1))]
    return mesh_from(name, v, f)


jet = bpy.data.materials.new("jet_core")
jet.use_nodes = True
jet.surface_render_method = "BLENDED"
nt = jet.node_tree
nt.nodes.remove(nt.nodes["Principled BSDF"])
tc = nt.nodes.new("ShaderNodeTexCoord")
sep = nt.nodes.new("ShaderNodeSeparateXYZ")
nt.links.new(tc.outputs["Generated"], sep.inputs[0])
fade = nt.nodes.new("ShaderNodeMath")                              # bright at the nozzle, gone at the tip
fade.operation = "POWER"
inv = nt.nodes.new("ShaderNodeMath")
inv.operation = "SUBTRACT"
inv.inputs[0].default_value = 1.0
nt.links.new(sep.outputs["Z"], inv.inputs[1])
nt.links.new(inv.outputs[0], fade.inputs[0])
fade.inputs[1].default_value = 1.6
lw = nt.nodes.new("ShaderNodeLayerWeight")
lw.inputs["Blend"].default_value = 0.6
soft = nt.nodes.new("ShaderNodeMath")
soft.operation = "SUBTRACT"
soft.inputs[0].default_value = 1.0
nt.links.new(lw.outputs["Facing"], soft.inputs[1])
mask = nt.nodes.new("ShaderNodeMath")
mask.operation = "MULTIPLY"
nt.links.new(fade.outputs[0], mask.inputs[0])
nt.links.new(soft.outputs[0], mask.inputs[1])
em = nt.nodes.new("ShaderNodeEmission")
em.inputs["Color"].default_value = (0.15, 0.45, 1.0, 1)
em.inputs["Strength"].default_value = 2.0
tr = nt.nodes.new("ShaderNodeBsdfTransparent")
mix = nt.nodes.new("ShaderNodeMixShader")
nt.links.new(mask.outputs[0], mix.inputs["Fac"])
nt.links.new(tr.outputs[0], mix.inputs[1])
nt.links.new(em.outputs[0], mix.inputs[2])
nt.links.new(mix.outputs[0], nt.nodes["Material Output"].inputs["Surface"])


def cloud_material(name, density, world_noise=False):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    nt = m.node_tree
    nt.nodes.remove(nt.nodes["Principled BSDF"])
    pv = nt.nodes.new("ShaderNodeVolumePrincipled")
    pv.inputs["Color"].default_value = (0.97, 0.98, 1.0, 1)
    pv.inputs["Anisotropy"].default_value = 0.3
    tcn = nt.nodes.new("ShaderNodeTexCoord")
    nz = nt.nodes.new("ShaderNodeTexNoise")
    nz.inputs["Scale"].default_value = 3.0
    nz.inputs["Detail"].default_value = 10.0
    nz.inputs["Distortion"].default_value = 0.5
    if world_noise:                                               # puffs share the material; world noise keeps each one different
        geo = nt.nodes.new("ShaderNodeNewGeometry")
        sc_ = nt.nodes.new("ShaderNodeVectorMath")
        sc_.operation = "SCALE"
        sc_.inputs["Scale"].default_value = float(opt("--billow", 0.2))   # noise 3.0 x 0.2 -> ~1.7 m billows
        nt.links.new(geo.outputs["Position"], sc_.inputs[0])
        nt.links.new(sc_.outputs[0], nz.inputs["Vector"])
    else:
        nt.links.new(tcn.outputs["Object"], nz.inputs["Vector"])
    # radial falloff from the object centre so the cloud has no hard mesh edge
    ln = nt.nodes.new("ShaderNodeVectorMath")
    ln.operation = "LENGTH"
    nt.links.new(tcn.outputs["Object"], ln.inputs[0])
    fall = nt.nodes.new("ShaderNodeMapRange")
    fall.inputs["From Min"].default_value, fall.inputs["From Max"].default_value = 0.3, 1.0
    fall.inputs["To Min"].default_value, fall.inputs["To Max"].default_value = 1.0, 0.0
    nt.links.new(ln.outputs["Value"], fall.inputs["Value"])
    shape = nt.nodes.new("ShaderNodeMapRange")
    shape.inputs["From Min"].default_value, shape.inputs["From Max"].default_value = 0.42, 0.62   # billows with soft fringes
    nt.links.new(nz.outputs["Fac"], shape.inputs["Value"])
    mul = nt.nodes.new("ShaderNodeMath")
    mul.operation = "MULTIPLY"
    nt.links.new(shape.outputs["Result"], mul.inputs[0])
    nt.links.new(fall.outputs["Result"], mul.inputs[1])
    val = nt.nodes.new("ShaderNodeValue")                          # keyframed strength
    val.name = "amount"
    val.outputs[0].default_value = density
    mul2 = nt.nodes.new("ShaderNodeMath")
    mul2.operation = "MULTIPLY"
    nt.links.new(mul.outputs[0], mul2.inputs[0])
    nt.links.new(val.outputs[0], mul2.inputs[1])
    nt.links.new(mul2.outputs[0], pv.inputs["Density"])
    nt.links.new(pv.outputs[0], nt.nodes["Material Output"].inputs["Volume"])
    return m


plumes = []
jet_cloud = cloud_material("jet_cloud", 4.0)
for n_ in NZ:
    d = Vector(n_["dir"]).normalized()
    q = Vector((0, 0, 1)).rotation_difference(-d)
    core = new_obj("jet_" + n_["name"], cone_mesh("jetcone", 0.06), car)
    core.location = n_["pos"]
    core.rotation_mode = "QUATERNION"
    core.rotation_quaternion = q
    core.data.materials.append(jet)
    cl = new_obj("jetcloud_" + n_["name"], cone_mesh("cloudcone", 0.45), car)
    cl.location = n_["pos"]
    cl.rotation_mode = "QUATERNION"
    cl.rotation_quaternion = q
    cl.data.materials.append(jet_cloud)
    plumes.append((core, cl))

# ground condensation cloud (dominant look of the landing, PCMag 115.5-118; not in the sim, driven from it):
# puffs are born where the lift jets reach the floor, roll outward and grow
puff_mat = cloud_material("puff", float(opt("--puffd", 30.0 if "--final" not in ARGS else 6.0)), world_noise=True)   # dense: crisp billow edges
N_PUFF = 22
puffs = [box(f"puff{k}", (2, 2, 2), (0, 0, -50), puff_mat) for k in range(N_PUFF)]

# ---------------- animation ----------------
S = resample()
lift_idx = [i for i, n_ in enumerate(NZ) if n_["name"].startswith("lift")]
spin = 0.0
src_log = []
for f, (t, pos, q, noz, row) in enumerate(S, start=1):
    car.location = pos
    car.rotation_quaternion = q
    car.keyframe_insert("location", frame=f)
    car.keyframe_insert("rotation_quaternion", frame=f)
    # wheels roll with the car while on the wall, then spin down
    if f > 1:
        v = (pos - S[f - 2][1]).length * FPS
        spin += (v if t <= 52.25 else v * math.exp(-(t - 52.25) / 1.5)) / FPS / WR
    for hub in wheels:
        hub.rotation_euler = (0, spin, 0)
        hub.keyframe_insert("rotation_euler", frame=f)
    for k, (core, cl) in enumerate(plumes):
        fr = float(noz[k])
        on = fr > 0.005
        L = 3.2 * min(1.0, fr / 0.6) ** 0.7 if on else 0.001
        w = 1.0 if on else 0.001
        core.scale = (w, w, L)
        cl.scale = (w * 1.2, w * 1.2, L * 1.1)
        core.keyframe_insert("scale", frame=f)
        cl.keyframe_insert("scale", frame=f)
        cl.hide_render = not on                                  # Cycles crashes (access violation) on a 0.001-scale volume
        cl.keyframe_insert("hide_render", frame=f)
    # lift-jet flow that reaches the floor (jets ~14 m long); drives the ground cloud below
    clear = pos.z - HC
    src_log.append((sum(float(noz[i]) for i in lift_idx) * float(np.clip(1 - clear / 14.0, 0, 1)), pos.copy()))

# puff k is born when the cumulative floor flow passes k / N_PUFF of its total; it rolls outward 3-9 m and grows to 2-4 m radius
cum = np.cumsum([a for a, _ in src_log])
rng_p = np.random.default_rng(11)
for k, pf in enumerate(puffs):
    fb = int(np.searchsorted(cum, cum[-1] * (k + 0.5) / N_PUFF)) + 1
    base = src_log[fb - 1][1]
    ang, reach, rmax = rng_p.uniform(0, 2 * math.pi), rng_p.uniform(4, 14), rng_p.uniform(4.0, 7.0)   # pcmag_117.3: cloud ~40 m wide, ~15 m tall
    tall = rng_p.uniform(0.6, 1.5)                                # some puffs tower (pcmag_117.3), most stay low
    pf.scale = (0.001, 0.001, 0.001)
    pf.keyframe_insert("scale", frame=max(1, fb - 1))
    for f in range(fb, N_FRAMES + 1, 3):
        a = (f - fb) / FPS
        d = reach * (1 - math.exp(-a / 1.0))
        rad = 1.5 + (rmax - 1.5) * (1 - math.exp(-a / 0.7))
        pf.location = (base.x + d * math.cos(ang), base.y + d * math.sin(ang), 0.8 * tall * rad)
        pf.scale = (rad * 1.3, rad * 1.3, rad * tall)              # flat, wide ground cloud (pcmag_115.8)
        pf.keyframe_insert("location", frame=f)
        pf.keyframe_insert("scale", frame=f)

# ---------------- cameras ----------------
def camera(name, loc, lens):
    c = bpy.data.cameras.new(name)
    c.lens, c.sensor_width = lens, 36.0
    c.clip_start, c.clip_end = 0.1, 600
    o = new_obj(name, c)
    o.location = loc
    return o


CAL = json.loads((ROOT / "out" / "camera_calib.json").read_text())   # from blender/calib_cameras.py


def cam_rot(yaw, pitch, roll):
    """Calibration angles (yaw from +x about z, pitch up, roll about the view axis) -> Blender camera quaternion."""
    cy, sy, cp, sp = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch)
    fwd = Vector((cy * cp, sy * cp, sp))
    right = Vector((sy, -cy, 0.0))
    up = right.cross(fwd)
    right, up = math.cos(roll) * right + math.sin(roll) * up, -math.sin(roll) * right + math.cos(roll) * up
    return Matrix((right, up, -fwd)).transposed().to_quaternion()


def px_to_mm(f_px):
    return f_px * 36.0 / 1280


# A: 295 mm tele 62 m behind the wall foot (solved from wall edges + rear tyres in PCMag 105.0, rms 4 px)
cA = CAL["camA"]
camA = camera("camA", cA["position"], px_to_mm(cA["f_px"]))
camA.rotation_mode = "QUATERNION"
camA.rotation_quaternion = cam_rot(cA["yaw"], cA["pitch"], cA["roll"])
# B: one tripod position, pan/tilt/zoom solved per reference frame (48.8 ... 54.8); after 54.8 no wall is in
# frame, so the move is held toward the landing (aim at the simulated car, lens kept)
camB = camera("camB", CAL["position"], 40)
camB.rotation_mode = "QUATERNION"
keys = sorted((float(t), v) for t, v in CAL["frames"].items())
last = keys[-1][1]
Cb = Vector(CAL["position"])
for t_land in (56.3, 57.98):
    tgt = S[min(f_of(t_land), len(S)) - 1][1]
    d = (tgt - Cb).normalized()
    keys.append((t_land, dict(yaw=math.atan2(d.y, d.x), pitch=math.asin(d.z) - math.radians(2), roll=0.0, f_px=last["f_px"])))
keys.insert(0, (44.7, keys[0][1]))
qprev = None
for t, v in keys:
    q = cam_rot(v["yaw"], v["pitch"], v["roll"])
    if qprev is not None and q.dot(qprev) < 0:
        q.negate()
    qprev = q
    camB.rotation_quaternion = q
    camB.keyframe_insert("rotation_quaternion", frame=f_of(t))
    camB.data.lens = px_to_mm(v["f_px"])
    camB.data.keyframe_insert("lens", frame=f_of(t))
CUT = 44.69                                                       # PCMag 106.04 - 61.35
sc.camera = camA
mA = sc.timeline_markers.new("A", frame=1)
mA.camera = camA
mB = sc.timeline_markers.new("B", frame=f_of(CUT))
mB.camera = camB

# ---- coverage: virtual cameras a film crew would add around the two broadcast ones ----
def fixed_cam(name, loc, target, lens):
    o = camera(name, loc, lens)
    o.rotation_mode = "QUATERNION"
    o.rotation_quaternion = (Vector(target) - Vector(loc)).to_track_quat("-Z", "Y")
    return o


cams = {"camA": camA, "camB": camB,
        # C: low wide at the foot, beside the track; the car rushes past and up the curve
        "camC_foot": fixed_cam("camC_foot", (2.5, -6.0, 0.35), (6.5, 0.0, 8.0), 16),
        # D: rigged on the truss above the lip, looking out and down over the drop
        "camD_lip": fixed_cam("camD_lip", (-12.8, -3.2, 24.0), (-18.0, 0.5, 3.0), 20),
        # G: low ground camera across the landing zone, wall behind
        "camG_land": fixed_cam("camG_land", (-31.0, -19.0, 0.8), (-21.0, 0.0, 2.5), 40)}
# E: car-mounted, on the right flank looking forward (wall rushing past, world turning over)
camE = camera("camE_onboard", (0, 0, 0), 14)
camE.parent = car
camE.location = (-0.4, -1.12, 0.30)
camE.rotation_mode = "QUATERNION"
camE.rotation_quaternion = Matrix(((-0.17, 0, -0.985), (-0.985, 0, 0.17), (0, 1, 0))).to_quaternion()   # cols: right, up, back; 10 deg out
cams["camE_onboard"] = camE
# F: chase drone, lags the car by ~0.6 s from behind-left, always looking at it
camF = camera("camF_chase", (0, 0, 0), 32)
con = camF.constraints.new("TRACK_TO")
con.target, con.track_axis, con.up_axis = car, "TRACK_NEGATIVE_Z", "UP_Y"
cams["camF_chase"] = camF
sm = None
for f, (t, pos, *_rest) in enumerate(S, start=1):
    goal = pos + Vector((7.0, -11.0, 2.5))
    sm = goal.copy() if sm is None else sm.lerp(goal, 1 - math.exp(-1 / FPS / 0.6))
    camF.location = (sm.x, sm.y, max(1.2, sm.z))
    camF.keyframe_insert("location", frame=f)

# Cut lists: (t_start, t_end, camera, speed). speed < 1 = slow motion (frame_map stretches the 30 fps keys).
EDL = {
    "broadcast": [(44.0, CUT, "camA", 1.0), (CUT, 57.98, "camB", 1.0)],
    "coverage": [(44.0, 45.6, "camA", 1.0),            # establishing: the car waits at the foot
                 (45.6, 48.4, "camC_foot", 1.0),       # it launches past the low camera
                 (48.4, 50.4, "camB", 1.0),            # wide: the whole wall, car climbing
                 (50.4, 52.3, "camE_onboard", 1.0),    # onboard: upside down under the overhang, let go
                 (52.3, 54.0, "camD_lip", 1.0),        # from the lip: the half-roll and the drop
                 (54.0, 56.4, "camF_chase", 1.0),      # chase: landing burn into the cloud
                 (56.4, 57.98, "camG_land", 1.0),      # ground: touchdown, the cloud rolls out
                 (52.0, 54.6, "camF_chase", 0.4)],     # replay: release and roll at 0.4x
}

# ---------------- render settings ----------------
r = sc.render
r.engine = "BLENDER_EEVEE"
if "--final" in ARGS:                                             # Cycles final: GPU path tracing, denoised ("--cycles" clashes with Blender's own args)
    r.engine = "CYCLES"
    # GPU: pass "--cycles-device OPTIX" after "--" (Blender's own flag). Setting the device from this script after the
    # scene was built crashed Blender 5.2.1 (exit 127, no log), though the same calls work in an empty file.
    sc.cycles.device = "GPU"
    sc.cycles.samples = opt("--samples", 128)
    sc.cycles.use_denoising = True
    sc.cycles.volume_step_rate = 2.0                              # coarser volume stepping: 4 GB VRAM, cloud dominates cost
r.resolution_x, r.resolution_y = 1280, 720
r.resolution_percentage = opt("--res", 100)
r.use_motion_blur = True
r.motion_blur_shutter = 0.5
ee = sc.eevee
ee.taa_render_samples = opt("--samples", 32)
ee.use_raytracing = True
ee.volumetric_tile_size = opt("--tile", "2")      # "8" read as blocks in the cloud
ee.volumetric_samples = 64
ee.volumetric_start, ee.volumetric_end = 0.5, 220
ee.use_volumetric_shadows = True
sc.view_settings.view_transform = "Standard" if "--standard" in ARGS else "AgX"
sc.view_settings.look = "None"
EXPOSURE = {"camA": -1.0}      # broadcast camera A ran 1 stop darker than B: wall top and floor match pcmag_105.0 at -1.0


def use_cam(c):
    sc.camera = c
    sc.view_settings.exposure = EXPOSURE.get(c.name, 0.0) + float(opt("--exposure", 0.0))

# compositor: bloom on LEDs and jets (EEVEE Next has no built-in bloom). "--nocomp" skips it.
try:
    assert "--nocomp" not in ARGS, "--nocomp"
    ng = bpy.data.node_groups.new("comp", "CompositorNodeTree")
    ng.interface.new_socket("Image", in_out="OUTPUT", socket_type="NodeSocketColor")
    rl = ng.nodes.new("CompositorNodeRLayers")
    gl = ng.nodes.new("CompositorNodeGlare")
    for key, val in (("Type", "Bloom"), ("Quality", "High"), ("Threshold", 1.0), ("Strength", 0.35), ("Size", 0.5)):
        try:
            gl.inputs[key].default_value = val
        except (TypeError, KeyError, ValueError):
            print("glare input not set:", key)
    out = ng.nodes.new("NodeGroupOutput")
    ng.links.new(rl.outputs["Image"], gl.inputs["Image"])
    ng.links.new(gl.outputs["Image"], out.inputs[0])
    sc.compositing_node_group = ng
    r.use_compositing = True
except Exception as e:                                            # bloom is cosmetic: render without it
    print("COMPOSITOR SKIPPED:", e)

if "--save" in ARGS:
    bpy.ops.wm.save_as_mainfile(filepath=str(ROOT / "out" / "roadster_wall.blend"))

# ---------------- render ----------------
mode = ARGS[0] if ARGS else ""
if mode == "stills":
    od = ROOT / "out" / "stills"
    od.mkdir(parents=True, exist_ok=True)
    for a in ARGS[1:]:
        if a.startswith("--"):
            break
        f = f_of(float(a))
        sc.frame_set(f)
        use_cam(camA if float(a) < CUT else camB)
        r.filepath = str(od / f"t_{float(a):.2f}.png")
        bpy.ops.render.render(write_still=True)
        print("STILL", a, f, r.filepath)
elif mode == "edl":
    import subprocess
    name = ARGS[1] if len(ARGS) > 1 and not ARGS[1].startswith("--") else "coverage"
    tag = "_final" if "--final" in ARGS else ""
    od = ROOT / "out" / f"edl_{name}{tag}"
    r.image_settings.file_format = "PNG"
    r.use_overwrite, r.use_placeholder = False, True              # resumable: a rerun skips frames already on disk
    parts = []
    for k, (t0, t1, cam, speed) in enumerate(EDL[name]):
        new = round(100 / speed)
        r.frame_map_old, r.frame_map_new = 100, new
        sc.frame_start, sc.frame_end = round(f_of(t0) * new / 100), round(f_of(t1) * new / 100) - 1
        sc.timeline_markers.clear()                               # markers would override the shot camera
        use_cam(cams[cam])
        seg = od / f"seg{k:02d}"
        r.filepath = str(seg / "####")
        bpy.ops.render.render(animation=True)
        part = od / f"seg{k:02d}.mp4"
        subprocess.run(["ffmpeg", "-loglevel", "error", "-y", "-framerate", str(FPS), "-start_number", str(sc.frame_start),
                        "-i", str(seg / "%04d.png"), "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "18", str(part)], check=True)
        parts.append(part)
        print("SEG", k, cam, t0, t1, speed, sc.frame_start, sc.frame_end)
    (od / "list.txt").write_text("".join(f"file '{p.name}'" + chr(10) for p in parts))
    subprocess.run(["ffmpeg", "-loglevel", "error", "-y", "-f", "concat", "-safe", "0", "-i", str(od / "list.txt"),
                    "-c", "copy", str(ROOT / "out" / f"{name}{tag or '_draft'}.mp4")], check=True)
    print("MP4", ROOT / "out" / f"{name}{tag or '_draft'}.mp4")
elif mode == "cams":                                              # one still per camera at a given time (framing check)
    od = ROOT / "out" / "stills"
    od.mkdir(parents=True, exist_ok=True)
    for a in ARGS[1:]:
        if a.startswith("--"):
            break
        sc.frame_set(f_of(float(a)))
        sc.timeline_markers.clear()
        for nm, c in cams.items():
            use_cam(c)
            r.filepath = str(od / f"cam_{nm}_{float(a):.2f}.png")
            bpy.ops.render.render(write_still=True)
            print("CAM", nm, a)
elif mode == "anim":
    nums = [int(a) for a in ARGS[1:3] if a.isdigit()]
    if nums:
        sc.frame_start, sc.frame_end = nums[0], nums[-1]
    r.filepath = str(ROOT / "out" / "frames_eevee" / "####")
    r.image_settings.file_format = "PNG"
    bpy.ops.render.render(animation=True)
print("OK build_scene", N_FRAMES, "frames")
