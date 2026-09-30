"""Parametric sketch-region face (Shapr3D extrude/revolve profile).

Shapr3D profiles are cells of the planar arrangement of a sketch's curves,
optionally bounded by edges of bodies the sketch lies on. The feature stores
which sketch curves bound each region and recomputes the face from the live
sketch, so editing the sketch updates everything built on it.
"""

import FreeCAD as App
import Part

LIN_TOL = 1e-5


class ShaprProfile:
    """Part::FeaturePython proxy.

    Regions: IntegerList of sketch geometry indices, regions separated by -1.
    BodyBounded: IntegerList, 1 per region if it is bounded by body edges too.
    Bodies: bodies whose faces coplanar with the sketch bound the regions.
    Topology: expected (wires, edges) per region, to pick between equal cells.
    Senses: parallel to Regions; 1 region left of curve direction, 0 right, -1 unknown.
    """

    def __init__(self, obj):
        obj.addProperty("App::PropertyLink", "Sketch", "Profile", "Sketch the regions come from")
        obj.addProperty("App::PropertyIntegerList", "Regions", "Profile",
                        "Sketch geometry indices per region, -1 separated")
        obj.addProperty("App::PropertyIntegerList", "BodyBounded", "Profile",
                        "1 if the region is also bounded by body edges")
        obj.addProperty("App::PropertyLinkList", "Bodies", "Profile",
                        "Bodies whose coplanar faces bound the regions")
        obj.addProperty("App::PropertyIntegerList", "Topology", "Profile",
                        "Expected wire and edge count per region (pairs)")
        obj.addProperty("App::PropertyIntegerList", "Senses", "Profile",
                        "Per region curve: 1 region left of curve, 0 right, -1 unknown")
        obj.addProperty("App::PropertyString", "Status", "Profile", "Result of last region match")
        obj.setEditorMode("Status", 1)
        obj.Proxy = self

    def dumps(self):
        return None

    def loads(self, state):
        return None

    def execute(self, obj):
        regions = split_regions(obj.Regions)
        sense_lists = split_regions(obj.Senses, sep=-2) if obj.Senses else [[]] * len(regions)
        bounded = list(obj.BodyBounded) + [0] * (len(regions) - len(obj.BodyBounded))
        topo = list(obj.Topology)
        faces, notes = [], []
        for i, geo in enumerate(regions):
            t = tuple(topo[2 * i:2 * i + 2]) if len(topo) >= 2 * i + 2 else None
            sense = dict(zip(geo, sense_lists[i])) if i < len(sense_lists) else {}
            face, note = region_face(obj.Sketch, geo, obj.Bodies if bounded[i] else [], t, sense)
            if face is not None:
                faces.append(_drop_slits(face))
            if note:
                notes.append("region %d: %s" % (i + 1, note))
        obj.Status = "; ".join(notes) or "ok"
        if not faces:
            raise RuntimeError("ShaprProfile: no region matched (%s)" % obj.Status)
        face = faces[0] if len(faces) == 1 else faces[0].fuse(faces[1:]).removeSplitter()
        obj.Shape = Part.Compound([_merge_collinear(f) for f in face.Faces]) \
            if len(face.Faces) > 1 else _merge_collinear(face.Faces[0])
        # Shape is already global; keep Placement neutral.
        obj.Placement = App.Placement()


def _drop_slits(face):
    """Remove zero-area inner wires (dangling edges inside a cell), which
    make extrusions of the face invalid."""
    holes = [w for w in face.Wires if not w.isSame(face.OuterWire)]
    keep = [w for w in holes if w.isClosed() and Part.Face(w).Area > LIN_TOL]
    if len(keep) == len(holes):
        return face
    out = Part.Face(face.OuterWire)
    return out.cut([Part.Face(w) for w in keep]).Faces[0] if keep else out


def _merge_collinear(face):
    """Join runs of collinear line edges, so that a run of collinear sketch
    curves extrudes to one side face, as in Shapr3D."""
    wires, changed = [], False
    for wire in face.Wires:
        pts = [v.Point for v in wire.OrderedVertexes]
        edges = wire.OrderedEdges
        if not all(isinstance(e.Curve, Part.Line) for e in edges) or len(pts) < 3:
            wires.append(wire)
            continue
        keep = [p for i, p in enumerate(pts)
                if (pts[i] - pts[i - 1]).cross(pts[(i + 1) % len(pts)] - pts[i]).Length
                > LIN_TOL * (pts[i] - pts[i - 1]).Length]
        changed |= len(keep) < len(pts)
        wires.append(Part.makePolygon(keep + [keep[0]]) if len(keep) < len(pts) else wire)
    if not changed:
        return face
    outer = [w for w, old in zip(wires, face.Wires) if old.isSame(face.OuterWire)][0]
    holes = [Part.Face(w) for w, old in zip(wires, face.Wires) if not old.isSame(face.OuterWire)]
    out = Part.Face(outer)
    return out.cut(holes).Faces[0] if holes else out


def split_regions(values, sep=-1):
    regions, cur = [], []
    for v in values:
        if v == sep:
            regions.append(cur)
            cur = []
        else:
            cur.append(v)
    if cur:
        regions.append(cur)
    return regions


def join_regions(regions, sep=-1):
    out = []
    for i, r in enumerate(regions):
        if i:
            out.append(sep)
        out.extend(r)
    return out


def _sketch_edges(sketch):
    """(geo index, global edge) for all non-construction curves."""
    pl = sketch.getGlobalPlacement().toMatrix()
    out = []
    for idx, g in enumerate(sketch.Geometry):
        if sketch.getConstruction(idx) or isinstance(g, Part.Point):
            continue
        out.append((idx, g.toShape().transformed(pl).Edges[0]))
    return out


def _body_edges(bodies, plane_pt, normal):
    """Edges of body faces lying in the sketch plane (fallback: plane section)."""
    edges = []
    for b in bodies:
        shape = Part.getShape(b)
        if shape.isNull():
            continue
        coplanar = []
        for f in shape.Faces:
            s = f.Surface
            if not isinstance(s, Part.Plane):
                continue
            n = s.Axis
            if n.cross(normal).Length < 1e-7 and abs((f.Vertexes[0].Point - plane_pt).dot(normal)) < LIN_TOL:
                coplanar.append(f)
        if coplanar:
            for f in coplanar:
                edges.extend(f.Edges)
        else:
            for w in shape.slice(normal, plane_pt.dot(normal)):
                edges.extend(w.Edges)
    return edges


def region_face(sketch, geo, bodies, topo=None, sense=None):
    """Find the arrangement cell bounded by exactly the given sketch curves.

    Ties between cells bounded by the same curves are broken by sense (which
    side of each curve the region lies on), then topo (wire/edge counts).
    """
    pl = sketch.getGlobalPlacement()
    normal = pl.Rotation.multVec(App.Vector(0, 0, 1))
    want = set(geo)
    # Only the region's own curves: the sketch holds its final state, and
    # curves added after this feature may cut through the region.
    sk_edges = [(i, e) for i, e in _sketch_edges(sketch) if i in want]
    if not sk_edges:
        return None, "region curves missing from sketch"
    b_edges = _body_edges(bodies, pl.Base, normal) if bodies else []

    sense = {i: s for i, s in (sense or {}).items() if s in (0, 1)}
    curve_of = dict(sk_edges)

    def rank(c):
        cell = c[0]
        t = abs(len(cell.Wires) - topo[0]) + abs(len(cell.Edges) - topo[1]) if topo else 0
        return (_sense_mismatch(cell, curve_of, sense, normal), t)

    attempts = [b_edges, []] if b_edges else [[]]
    for extra in attempts:
        candidates = _cells(pl, sk_edges, extra)
        exact = [c for c in candidates if c[1] == want and (bodies or not c[2])]
        if not exact:
            continue
        exact.sort(key=rank)
        best = [c for c in exact if rank(c) == rank(exact[0])]
        if len(best) > 1 and bodies:
            best = [c for c in best if _inside_bodies(c[0], bodies)] or best
        note = "matched without body edges" if b_edges and extra is not b_edges else ""
        if len(best) > 1:
            note = "ambiguous (%d cells), took first" % len(best)
        return best[0][0], note
    # Best effort: most overlap with the wanted curves, fewest extra ones.
    candidates = _cells(pl, sk_edges, b_edges)
    scored = sorted(candidates, key=lambda c: (-(len(c[1] & want) - len(c[1] - want)), rank(c)))
    if scored and scored[0][1] & want:
        return scored[0][0], "approximate match"
    return None, "no matching region"


def _sense_mismatch(cell, curve_of, sense, normal):
    """Count cell boundary edges lying on the wrong side of their sketch curve."""
    bad = 0
    for e in cell.Edges:
        m = e.valueAt((e.FirstParameter + e.LastParameter) / 2)
        for i, s in sense.items():
            se = curve_of[i]
            if se.distToShape(Part.Vertex(m))[0] > LIN_TOL:
                continue
            tan = se.tangentAt(se.Curve.parameter(m))
            left = normal.cross(tan)
            if left.Length < 1e-12:
                continue
            step = max(e.Length * 1e-3, 1e-4)
            probe = m + left.normalize() * step
            inside_left = cell.isInside(probe, LIN_TOL / 10, True)
            if inside_left != bool(s):
                bad += 1
            break
    return bad


def _cells(pl, sk_edges, b_edges):
    """Cells of the planar arrangement: (face, touched geo indices, touches body edge)."""
    normal = pl.Rotation.multVec(App.Vector(0, 0, 1))
    all_edges = [e for _i, e in sk_edges] + b_edges
    bb = Part.Compound(all_edges).BoundBox
    size = max(bb.DiagonalLength, 1.0) * 2
    xdir = pl.Rotation.multVec(App.Vector(1, 0, 0))
    ydir = pl.Rotation.multVec(App.Vector(0, 1, 0))
    c = bb.Center
    c = c - normal * (c - pl.Base).dot(normal)
    corner = c - xdir * size / 2 - ydir * size / 2
    frame = Part.makePlane(size, size, corner, normal, xdir)
    pieces, _map = frame.generalFuse(all_edges, LIN_TOL)
    frame_edges = frame.Edges

    body_comp = Part.Compound(b_edges) if b_edges else None
    sk_boxes = []
    for i, se in sk_edges:
        box = se.BoundBox
        box.enlarge(LIN_TOL * 10)
        sk_boxes.append((i, se, box))
    out = []
    for cell in pieces.Faces:
        touched, has_body, rejected = set(), False, False
        for e in cell.Edges:
            p = e.valueAt((e.FirstParameter + e.LastParameter) / 2)
            mid = Part.Vertex(p)
            hit = [i for i, se, box in sk_boxes if box.isInside(p) and se.distToShape(mid)[0] < LIN_TOL]
            if hit:
                touched.update(hit)
            elif body_comp is not None and body_comp.distToShape(mid)[0] < LIN_TOL:
                has_body = True
            else:  # on the helper frame (unbounded outside cell) or unknown
                rejected = True
                break
        if not rejected:
            out.append((cell, touched, has_body))
    return out


def _inside_bodies(face, bodies):
    p = face.CenterOfMass if face.isInside(face.CenterOfMass, LIN_TOL, True) else None
    if p is None:
        return False
    return any(Part.getShape(b).isInside(p, LIN_TOL, True) for b in bodies)


def make_profile(doc, sketch, regions, senses, bounded, bodies, topology, label):
    obj = doc.addObject("Part::FeaturePython", "Profile")
    ShaprProfile(obj)
    obj.Label = label
    obj.Sketch = sketch
    obj.Regions = join_regions(regions)
    obj.Senses = join_regions(senses, sep=-2)
    obj.BodyBounded = [1 if b else 0 for b in bounded]
    obj.Bodies = bodies
    obj.Topology = [n for pair in topology for n in pair]
    return obj
