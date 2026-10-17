"""The Roadster model, built from Python primitives (no downloaded model, no AI image-to-3D).
Body frame: nose +x, left +y, origin at the CG, ground at z = -h_cg.
Reference look: frames/wall/pc_full_104.png (rear), frames/pcmag_03.jpg (stage), frames/ref/pcmag_105.0.png.
Called from build_scene.py: wheels, WR = roadster.build(car_empty, SIM["car"], SIM["nozzles"], helpers)
where helpers is the build_scene module (box, cylinder, loft, principled, emissive, mesh_from, new_obj)."""
import math

import bmesh
import bpy


def section_loft(h, name, rows, mats, mat_of, crease=None):
    """Closed body from half cross sections. rows: (x, [(y, z) ... bottom centre -> top centre], y >= 0), mirrored to -y.
    Ends are closed with horizontal strips. mat_of(x, k) -> material index of a face on half-profile segment k.
    crease(i, j, i2, j2) -> subsurf crease of the edge between ring point j of row i and ring point j2 of row i2."""
    n = len(rows[0][1])
    ring = 2 * n - 2
    v, f, fm = [], [], []
    for x, half in rows:
        v += [(x, y, z) for y, z in half + [(-y, z) for y, z in half[-2:0:-1]]]
    for i in range(len(rows) - 1):
        xc = (rows[i][0] + rows[i + 1][0]) / 2
        for j in range(ring):
            a, b = i * ring + j, i * ring + (j + 1) % ring
            f.append((a, b, b + ring, a + ring))
            fm.append(mat_of(xc, j if j < n - 1 else ring - 1 - j))
    for i in (0, len(rows) - 1):                                      # end caps as strips -> per-height materials
        o = i * ring
        for k in range(n - 1):
            f.append(tuple(dict.fromkeys((o + k, o + k + 1, o + (ring - k - 1) % ring, o + (ring - k) % ring))))
            fm.append(mat_of(rows[i][0], k))
    me = h.mesh_from(name, v, f)
    for m in mats:
        me.materials.append(m)
    for p, k in zip(me.polygons, fm):
        p.material_index = k
    bm = bmesh.new()
    bm.from_mesh(me)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    bm.to_mesh(me)
    bm.free()
    if crease:
        ce = me.attributes.new("crease_edge", "FLOAT", "EDGE")
        for e in me.edges:
            (ia, ja), (ib, jb) = (divmod(u, ring) for u in e.vertices)
            ce.data[e.index].value = crease(ia, ja, ib, jb)
    return h.new_obj(name, me)


def flat_caps(o):
    """Flat-shade the n-gon end caps of a helper cylinder (smooth caps shade like domes)."""
    for p in o.data.polygons:
        p.use_smooth = len(p.vertices) == 4
    return o


def build(car, CAR, NZ, h):
    box, cylinder, principled, emissive, new_obj = h.box, h.cylinder, h.principled, h.emissive, h.new_obj
    HC = CAR["h_cg"]
    paint = principled("paint", (0.80, 0.81, 0.83), rough=0.22, metal=0.15, coat=1.0)
    glass = principled("glass", (0.01, 0.01, 0.012), rough=0.04, coat=1.0)
    black = principled("trim", (0.005, 0.005, 0.006), rough=0.7)                # matte black lower body / diffuser
    black.node_tree.nodes["Principled BSDF"].inputs["Specular IOR Level"].default_value = 0.1   # floods sit near camera A
    gloss = principled("gloss_black", (0.01, 0.01, 0.01), rough=0.12, coat=1.0)
    rubber = principled("rubber", (0.02, 0.02, 0.02), rough=0.85)
    rim_m = principled("rim", (0.55, 0.56, 0.58), rough=0.3, metal=0.8)
    spoke_m = principled("spoke", (0.02, 0.02, 0.022), rough=0.3, metal=0.5)
    void = principled("void", (0.0, 0.0, 0.0), rough=1.0)
    tail = emissive("taillight", (1.0, 0.02, 0.02), 40.0)
    head = emissive("headlight", (0.9, 0.95, 1.0), 60.0)

    zg = -HC                                                          # ground in body frame; heights below are above ground
    # x, half width, bottom, shoulder (fender crown), buttress/deck edge (y, z), channel edge (y, z), black line
    # side view: very low sharp nose, front fender crowns, flat waist, wide rear haunches, cut-off tail (pcmag_03)
    # rear view: white buttresses either side of a dark glass channel (pc_full_104)
    ST = [
        (2.35, 0.78, 0.18, 0.29, 0.36, 0.285, 0.18, 0.280, 0.23),   # nose tip: wide flat blade
        (2.28, 0.88, 0.14, 0.36, 0.46, 0.335, 0.22, 0.325, 0.23),
        (2.10, 0.93, 0.13, 0.49, 0.52, 0.420, 0.25, 0.400, 0.24),
        (1.85, 0.98, 0.12, 0.65, 0.58, 0.520, 0.30, 0.490, 0.24),
        (1.55, 0.995, 0.12, 0.79, 0.64, 0.745, 0.30, 0.580, 0.24),  # front fender crown above a low hood
        (1.25, 0.99, 0.12, 0.81, 0.64, 0.775, 0.30, 0.640, 0.25),
        (0.95, 0.97, 0.12, 0.81, 0.62, 0.730, 0.40, 0.710, 0.25),   # cowl
        (0.50, 0.94, 0.12, 0.81, 0.72, 0.810, 0.62, 0.780, 0.26),   # door
        (0.00, 0.93, 0.12, 0.82, 0.72, 0.820, 0.62, 0.780, 0.27),
        (-0.50, 0.94, 0.12, 0.83, 0.70, 0.840, 0.60, 0.800, 0.28),
        (-0.85, 0.97, 0.12, 0.85, 0.56, 1.080, 0.48, 1.030, 0.30),  # buttress root at the roof end
        (-1.20, 0.995, 0.13, 0.86, 0.55, 1.020, 0.46, 0.960, 0.33),  # rear haunch
        (-1.60, 1.00, 0.13, 0.86, 0.56, 0.960, 0.46, 0.900, 0.40),
        (-1.95, 0.98, 0.14, 0.86, 0.58, 0.910, 0.47, 0.865, 0.50),
        (-2.20, 0.93, 0.15, 0.86, 0.60, 0.875, 0.50, 0.855, 0.62),  # tail narrower than the haunches
        (-2.32, 0.89, 0.16, 0.86, 0.60, 0.865, 0.50, 0.855, 0.72),
        (-2.36, 0.86, 0.19, 0.84, 0.60, 0.850, 0.50, 0.840, 0.72),  # cut-off tail
    ]
    rows = []
    for x, hw, ub, us, wb, ubt, wc, uc, ul in ST:
        p2 = ub + min(0.07, 0.3 * (us - ub))
        ul = max(p2 + 0.03, min(ul, us - 0.06))
        half = [(0, ub), (hw - 0.10, ub), (hw - 0.015, p2), (hw, max(p2 + 0.01, ul - 0.05)), (hw, ul),   # support point keeps
                (hw, ul + 0.55 * (us - 0.05 - ul)), (hw - 0.03, us - 0.035), (hw - 0.11, us),             # the black line sharp
                (wb, ubt), (wc, uc), (0, uc)]
        rows.append((x, [(y, zg + z) for y, z in half]))
    # segments 0-3 (below the black line) matte black; channel floor between the buttresses = rear glass
    body = section_loft(h, "body", rows, (paint, black, glass),
                        lambda x, k: 1 if k < 4 else 2 if k == 9 and -1.98 < x < -0.70 else 0,
                        lambda i, j, i2, j2: 0.8 if i == i2 == len(rows) - 1 and (j - j2) % 20 in (1, 19)    # sharp cut-off tail
                        else 0.5 if i != i2 and j == j2 in (7, 13) else 0.6 if i != i2 and j == j2 in (8, 9, 11, 12)  # shoulder,
                        else 0.0)                                                                             # buttress + glass edges
    body.parent = car
    canopy_rows = []                                                  # black glass cabin: x, belt half width, roof half width, bottom, roof
    for x, wb, wt, ub, ut in [(0.95, 0.50, 0.30, 0.64, 0.73), (0.75, 0.62, 0.42, 0.68, 0.88), (0.35, 0.70, 0.50, 0.70, 1.05),
                              (-0.05, 0.71, 0.53, 0.70, 1.13), (-0.40, 0.70, 0.53, 0.72, 1.15), (-0.75, 0.66, 0.51, 0.76, 1.11),
                              (-1.00, 0.56, 0.46, 0.85, 1.04)]:
        half = [(0, ub), (wb, ub), (wb, ub + 0.10), (wt + 0.05, ut - 0.06), (wt, ut - 0.015), (wt * 0.5, ut), (0, ut + 0.005)]
        canopy_rows.append((x, [(y, zg + z) for y, z in half]))
    canopy = section_loft(h, "canopy", canopy_rows, (glass,), lambda x, k: 0)
    canopy.parent = car
    for o in (body, canopy):
        md = o.modifiers.new("sub", "SUBSURF")
        md.levels = md.render_levels = 2

    WR = 0.36
    WX, WY = CAR["wheelbase"] / 2, CAR["track"] / 2
    for sx in (1, -1):                                                # wheel arches cut out of the sides only
        for sy in (1, -1):
            cut = cylinder(f"arch{sx:+d}{sy:+d}", WR + 0.035, 0.36, (sx * WX, sy * (WY + 0.07), zg + WR), black, car, n=40)
            cut.hide_render = cut.hide_viewport = True
            md = body.modifiers.new(f"arch{sx:+d}{sy:+d}", "BOOLEAN")
            md.operation, md.object, md.solver = "DIFFERENCE", cut, "FLOAT"     # re-evaluated every frame: EXACT cost 1.4 s/frame
    wheels = []
    for sx in (1, -1):
        for sy in (1, -1):
            hub = new_obj(f"wheel{sx:+d}{sy:+d}", None, car)
            hub.location = (sx * WX, sy * WY, zg + WR)
            hub.rotation_mode = "XYZ"
            tyre = flat_caps(cylinder("tyre", WR, 0.28, (0, 0, 0), rubber, hub, n=48))   # thin tyre, large rim (pcmag_03)
            bm = tyre.modifiers.new("bev", "BEVEL")
            bm.width, bm.segments = 0.04, 3
            flat_caps(cylinder("rim", WR - 0.065, 0.285, (0, 0, 0), rim_m, hub, n=48))
            for k in range(10):                                       # 5 black twin spokes on a silver face
                a = 2 * math.pi * (k // 2) / 5 + (0.13 if k % 2 else -0.13)
                sp = box("spoke", (0.045, 0.02, WR - 0.10), (0, 0, 0), spoke_m, hub)
                sp.rotation_euler = (0, a, 0)
                sp.location = (math.sin(a) * (WR - 0.08) / 2, sy * 0.146, math.cos(a) * (WR - 0.08) / 2)
            flat_caps(cylinder("hubcap", 0.05, 0.30, (0, 0, 0), spoke_m, hub, n=24))
            wheels.append(hub)

    # rear (pc_full_104, pcmag_03 6:52): red light bar in a full-width black groove, plate, black diffuser with 4 round nozzles
    box("taillip", (0.09, 1.76, 0.035), (-2.33, 0, zg + 0.835), paint, car, bevel=0.012)   # white lip overhangs the groove
    box("tailband", (0.07, 1.76, 0.075), (-2.345, 0, zg + 0.78), gloss, car)
    box("lightbar", (0.02, 1.68, 0.022), (-2.375, 0, zg + 0.785), tail, car)
    box("plate", (0.012, 0.30, 0.15), (-2.37, 0, zg + 0.60), principled("plate", (0.85, 0.85, 0.85), 0.4), car)
    cu = new_obj("plate_text", bpy.data.curves.new("plate_text", "FONT"), car)
    cu.data.body, cu.data.size, cu.data.align_x, cu.data.align_y = "RDSTR", 0.075, "CENTER", "CENTER"
    cu.data.materials.append(black)
    cu.location, cu.rotation_euler = (-2.377, 0, zg + 0.595), (math.pi / 2, 0, -math.pi / 2)
    for y in (0.06, -0.06):                                           # plate lamps
        box("platelamp", (0.008, 0.025, 0.008), (-2.36, y, zg + 0.69), head, car)
    box("diff_top", (0.10, 1.25, 0.05), (-2.34, 0, zg + 0.44), black, car, bevel=0.015)
    for y in (0.60, -0.60):                                           # diffuser frame legs
        box("diff_leg", (0.16, 0.06, 0.34), (-2.30, y, zg + 0.28), black, car, bevel=0.015)
    for n_ in NZ:
        p = n_["pos"]
        if n_["name"].startswith("rear"):                             # ring bell with a dark throat
            flat_caps(cylinder(n_["name"] + "_bell", 0.05, 0.10, (p[0] - 0.06, p[1], p[2]), principled("bell", (0.10, 0.10, 0.10), 0.45, 1.0), car, n=24, axis="X"))
            flat_caps(cylinder(n_["name"] + "_throat", 0.038, 0.10, (p[0] - 0.065, p[1], p[2]), void, car, n=24, axis="X"))
    # front (pcmag_03 5:48): full-width light strip on the nose, lamp points at its ends, side intakes
    box("headlight", (0.02, 1.40, 0.018), (2.335, 0, zg + 0.30), head, car)
    for y in (0.66, -0.66):
        box("headlamp", (0.03, 0.10, 0.03), (2.32, y, zg + 0.305), head, car)
        box("side_intake", (0.42, 0.03, 0.12), (-0.72, math.copysign(0.94, y), zg + 0.50), black, car, bevel=0.01)
    return wheels, WR
