"""Build parametric Sketcher objects from Shapr3D sketches."""

import math

import FreeCAD as App
import Part
import Sketcher

M2MM = 1000.0
TOL = 1e-6  # mm: only constraints the source geometry satisfies exactly


def plane_placement(sk):
    """Placement of a Shapr3D sketch plane (meters -> mm)."""
    z = App.Vector(*sk.normal).normalize()
    x = App.Vector(*sk.udir).normalize()
    y = z.cross(x).normalize()
    m = App.Matrix(x.x, y.x, z.x, 0,
                   x.y, y.y, z.y, 0,
                   x.z, y.z, z.z, 0,
                   0, 0, 0, 1)
    return App.Placement(App.Vector(*sk.center) * M2MM, App.Rotation(m))


class SketchBuild:
    """Result of building one sketch: the object plus ID maps."""

    def __init__(self, obj, src):
        self.obj = obj
        self.src = src
        self.geo = {}     # curveID -> geo index
        self.points = {}  # pointID -> (geo index, pos)
        self.coords = {}  # pointID -> (x, y) mm
        self.curves = {}  # curveID -> Curve (source)


def _vec(p):
    return App.Vector(p.x * M2MM, p.y * M2MM, 0)


def _add_geometry(b):
    sk = b.obj
    for c in b.src.curves:
        if c.kind == "line":
            g = Part.LineSegment(_vec(c.start), _vec(c.end))
        elif c.kind == "circle":
            g = Part.Circle(_vec(c.center), App.Vector(0, 0, 1), c.radius * M2MM)
        else:
            ctr = _vec(c.center)
            r = (_vec(c.start) - ctr).Length
            a0 = math.atan2(c.start.y - c.center.y, c.start.x - c.center.x)
            a1 = math.atan2(c.end.y - c.center.y, c.end.x - c.center.x)
            while a1 <= a0:
                a1 += 2 * math.pi
            g = Part.ArcOfCircle(Part.Circle(ctr, App.Vector(0, 0, 1), r), a0, a1)
        idx = sk.addGeometry(g, c.construction)
        b.geo[c.id] = idx
        b.curves[c.id] = c
        for pid, pos in c.points():
            b.points.setdefault(pid, (idx, pos))
        for p in (c.start, c.end, c.center):
            if p is not None:
                b.coords[p.id] = (p.x * M2MM, p.y * M2MM)
    for p in b.src.extra_points:
        if p.id in b.points:
            continue
        idx = sk.addGeometry(Part.Point(_vec(p)), True)
        b.points[p.id] = (idx, 1)
        b.coords[p.id] = (p.x * M2MM, p.y * M2MM)


# ---- geometric checks: only add a constraint the source geometry satisfies ----

def _line_dir(c):
    dx, dy = c.end.x - c.start.x, c.end.y - c.start.y
    n = math.hypot(dx, dy)
    return (dx / n, dy / n) if n else None


def _radius(c):
    if c.kind == "circle":
        return c.radius * M2MM
    if c.kind == "arc":
        return math.hypot(c.start.x - c.center.x, c.start.y - c.center.y) * M2MM
    return None


def _on_curve(b, pid, cid):
    c = b.curves[cid]
    px, py = b.coords[pid]
    if c.kind == "line":
        d = _line_dir(c)
        if d is None:
            return False
        sx, sy = c.start.x * M2MM, c.start.y * M2MM
        return abs((px - sx) * d[1] - (py - sy) * d[0]) < TOL
    cx, cy = c.center.x * M2MM, c.center.y * M2MM
    return abs(math.hypot(px - cx, py - cy) - _radius(c)) < TOL


def _translate(b, j):
    """Shapr3D constraint JSON -> (list of Sketcher.Constraint, reason-if-skipped)."""
    t = j["type"]
    G, P, C = b.geo, b.points, b.curves

    def need(*ids, kind):
        table = P if kind == "p" else G
        missing = [i for i in ids if i not in table]
        return "references missing %s %s" % ("point" if kind == "p" else "curve", missing) if missing else None

    if t == 3:  # point-point distance
        r = need(j["point1"], j["point2"], kind="p")
        if r:
            return [], r
        (x1, y1), (x2, y2) = b.coords[j["point1"]], b.coords[j["point2"]]
        d = j["distance"] * M2MM
        dt = j.get("distanceType", 2)
        if dt == 2 and abs(math.hypot(x2 - x1, y2 - y1) - d) < TOL:
            g1, p1 = P[j["point1"]]
            g2, p2 = P[j["point2"]]
            return [Sketcher.Constraint("Distance", g1, p1, g2, p2, d)], None
        return [], "distance (type %s) does not match geometry" % dt
    if t == 5:  # point on curve
        r = need(j["point"], kind="p") or need(j["curve"], kind="g")
        if r:
            return [], r
        g, p = P[j["point"]]
        if g == G[j["curve"]]:
            return [], None  # a curve's own endpoint, implicit
        if not _on_curve(b, j["point"], j["curve"]):
            return [], "point-on-curve does not match geometry"
        return [Sketcher.Constraint("PointOnObject", g, p, G[j["curve"]])], None
    if t in (7, 9):  # 7 perpendicular, 9 parallel
        r = need(j["line1"], j["line2"], kind="g")
        if r:
            return [], r
        c1, c2 = C[j["line1"]], C[j["line2"]]
        if c1.kind != "line" or c2.kind != "line":
            return [], "angle constraint on non-line"
        d1, d2 = _line_dir(c1), _line_dir(c2)
        cross = abs(d1[0] * d2[1] - d1[1] * d2[0])
        ok = cross < 1e-7 if t == 9 else abs(d1[0] * d2[0] + d1[1] * d2[1]) < 1e-7
        if not ok:
            return [], "angle constraint does not match geometry"
        name = "Parallel" if t == 9 else "Perpendicular"
        return [Sketcher.Constraint(name, G[j["line1"]], G[j["line2"]])], None
    if t == 8:  # horizontal / vertical
        r = need(j["line"], kind="g")
        if r:
            return [], r
        d = _line_dir(C[j["line"]]) if C[j["line"]].kind == "line" else None
        if d is None:
            return [], "axis constraint on non-line"
        if j["axis"] == 1 and abs(d[0]) < 1e-7:
            return [Sketcher.Constraint("Vertical", G[j["line"]])], None
        if j["axis"] == 0 and abs(d[1]) < 1e-7:
            return [Sketcher.Constraint("Horizontal", G[j["line"]])], None
        return [], "axis constraint does not match geometry"
    if t == 12:  # midpoint of line
        r = need(j["point"], kind="p") or need(j["line"], kind="g")
        if r:
            return [], r
        c = C[j["line"]]
        if c.kind != "line":
            return [], "midpoint on non-line"
        mx, my = (c.start.x + c.end.x) / 2 * M2MM, (c.start.y + c.end.y) / 2 * M2MM
        px, py = b.coords[j["point"]]
        if math.hypot(px - mx, py - my) > TOL:
            return [], "midpoint does not match geometry"
        g, p = P[j["point"]]
        gl = G[j["line"]]
        return [Sketcher.Constraint("Symmetric", gl, 1, gl, 2, g, p)], None
    if t == 21:  # radius
        r = need(j["curve"], kind="g")
        if r:
            return [], r
        rad = _radius(C[j["curve"]])
        if rad is None or abs(rad - j["radius"] * M2MM) > TOL:
            return [], "radius does not match geometry"
        return [Sketcher.Constraint("Radius", G[j["curve"]], rad)], None
    names = {23: "circular pattern", 30: "offset"}
    return [], "unsupported constraint type %s (%s)" % (t, names.get(t, "unknown"))


def _try_add(sk, cons):
    """Add constraint; roll back if the sketch no longer solves cleanly."""
    n = sk.addConstraint(cons)
    if sk.solve() != 0:
        sk.delConstraint(n)
        sk.solve()
        return False
    return True


def build_sketch(doc, src, label, report):
    obj = doc.addObject("Sketcher::SketchObject", "Sketch")
    obj.Label = label
    obj.Placement = plane_placement(src)
    b = SketchBuild(obj, src)
    _add_geometry(b)

    candidates = []  # (constraint, description)
    for group in src.coincident_groups:
        pts = [p for p in group if p in b.points]
        for other in pts[1:]:
            g1, p1 = b.points[pts[0]]
            g2, p2 = b.points[other]
            if g1 == g2 and p1 == p2:
                continue
            (x1, y1), (x2, y2) = b.coords[pts[0]], b.coords[other]
            if math.hypot(x2 - x1, y2 - y1) > TOL:
                report.warn("%s: coincident group points not coincident, skipped" % label)
                continue
            candidates.append((Sketcher.Constraint("Coincident", g1, p1, g2, p2), "coincident"))
    for center, lines in src.symbols:
        # Keep a symbol's center point attached: symmetric between two opposite corners.
        corners = [(pid, b.coords[pid]) for cid in lines if cid in b.curves
                   for pid, _pos in b.curves[cid].points()]
        cx, cy = b.coords[center.id]
        pair = next(((a, q) for a, (ax, ay) in corners for q, (qx, qy) in corners
                     if math.hypot(ax - qx, ay - qy) > TOL
                     and math.hypot((ax + qx) / 2 - cx, (ay + qy) / 2 - cy) < TOL), None)
        if pair:
            (g1, p1), (g2, p2) = b.points[pair[0]], b.points[pair[1]]
            gc, pc = b.points[center.id]
            candidates.append((Sketcher.Constraint("Symmetric", g1, p1, g2, p2, gc, pc), "symbol"))
    skipped = {}
    for j in src.constraints:
        cons, why = _translate(b, j)
        if why:
            skipped[why] = skipped.get(why, 0) + 1
        candidates.extend((c, "type %s" % j["type"]) for c in cons)
    for pid in src.locked_points:
        if pid in b.points:
            g, p = b.points[pid]
            x, y = b.coords[pid]
            candidates.append((Sketcher.Constraint("DistanceX", g, p, x), "lock"))
            candidates.append((Sketcher.Constraint("DistanceY", g, p, y), "lock"))

    rejected = 0
    for cons, _desc in candidates:
        if not _try_add(obj, cons):
            rejected += 1
    for why, n in sorted(skipped.items()):
        report.info("%s: %d constraint(s) not converted: %s" % (label, n, why))
    if rejected:
        report.info("%s: %d constraint(s) dropped as redundant/conflicting in FreeCAD" % (label, rejected))

    obj.Visibility = not src.hidden
    return b

