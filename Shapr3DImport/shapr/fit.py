"""Curve fitting helpers for XT geometry STEP cannot express directly."""

import math

import FreeCAD as App
import Part

from .xt_step import knot_vector

M2MM = 1000.0


def interpolate(pts):
    """Points (mm) -> (degree, poles, mults, knots) of an interpolating B-spline."""
    vecs = [App.Vector(*p) for p in pts]
    closed = len(vecs) > 3 and (vecs[0] - vecs[-1]).Length < 1e-6
    if closed:
        vecs = vecs[:-1]
    if len(vecs) == 2:
        return 1, [tuple(v) for v in vecs], [2, 2], [0.0, 1.0]
    c = Part.BSplineCurve()
    c.interpolate(vecs, PeriodicFlag=closed)
    if closed:
        c.setNotPeriodic()
    return c.Degree, [tuple(p) for p in c.getPoles()], c.getMultiplicities(), c.getKnots()


def _vec(x):
    return App.Vector(*x)


def _frame(z, x):
    z = _vec(z).normalize()
    x = _vec(x)
    x = (x - z * x.dot(z)).normalize()
    return z, x, z.cross(x)


class SurfaceEval:
    """Evaluate XT surfaces in their own parametrisation (meters)."""

    def __init__(self, data):
        self.x = data
        self._bs = {}

    def value(self, idx, u, v):
        n = self.x[idx]
        t = n.name
        if t == "PLANE":
            z, x, y = _frame(n["normal"], n["x_axis"])
            return _vec(n["pvec"]) + x * u + y * v
        if t == "CYLINDER":
            a, x, y = _frame(n["axis"], n["x_axis"])
            r = n["radius"]
            return _vec(n["pvec"]) + (x * math.cos(u) + y * math.sin(u)) * r + a * v
        if t == "CONE":
            a, x, y = _frame(n["axis"], n["x_axis"])
            tan = n["sin_half_angle"] / n["cos_half_angle"]
            return _vec(n["pvec"]) - a * v + (x * math.cos(u) + y * math.sin(u)) * (n["radius"] + v * tan)
        if t == "SPHERE":
            a, x, y = _frame(n["axis"], n["x_axis"])
            r = n["radius"]
            return _vec(n["centre"]) + (x * math.cos(u) + y * math.sin(u)) * (r * math.cos(v)) + a * (r * math.sin(v))
        if t == "TORUS":
            a, x, y = _frame(n["axis"], n["x_axis"])
            R, r = n["major_radius"], n["minor_radius"]
            return _vec(n["centre"]) + (x * math.cos(u) + y * math.sin(u)) * (R + r * math.cos(v)) + a * (r * math.sin(v))
        if t == "B_SURFACE":
            return self._bsurface(idx).value(u, v)
        if t == "OFFSET_SURF":
            base = n["surface"]
            p = self.value(base, u, v)
            e = 1e-7
            du = self.value(base, u + e, v) - self.value(base, u - e, v)
            dv = self.value(base, u, v + e) - self.value(base, u, v - e)
            nrm = du.cross(dv).normalize()
            if self.x[base]["sense"] == "-":
                nrm = -nrm
            return p + nrm * n["offset"]
        raise ValueError("cannot evaluate %s" % t)

    def _bsurface(self, idx):
        if idx not in self._bs:
            nb = self.x[self.x[idx]["nurbs"]]
            dim = nb["vertex_dim"]
            verts = self.x[nb["bspline_vertices"]]["vertices"]
            nu, nv = nb["n_u_vertices"], nb["n_v_vertices"]
            poles = [[None] * nv for _ in range(nu)]
            weights = [[1.0] * nv for _ in range(nu)]
            for i in range(nu):
                for j in range(nv):
                    k = (i * nv + j) * dim
                    p = verts[k:k + dim]
                    if nb["rational"]:
                        weights[i][j] = p[-1]
                        p = [c / p[-1] for c in p[:-1]]
                    poles[i][j] = App.Vector(*p[:3])
            uk, um = knot_vector(self.x, nb["u_knots"], nb["u_knot_mult"], nb["n_u_knots"])
            vk, vm = knot_vector(self.x, nb["v_knots"], nb["v_knot_mult"], nb["n_v_knots"])
            s = Part.BSplineSurface()
            s.buildFromPolesMultsKnots(
                poles, um, vm, uk, vk,
                False, False, nb["u_degree"], nb["v_degree"],
                weights if nb["rational"] else None)
            self._bs[idx] = s
        return self._bs[idx]


def sp_curve_points(writer, sp, n=64):
    """Sample an SP-curve (2D B-spline in a surface's parameter space), in mm."""
    ev = getattr(writer, "_surface_eval", None)
    if ev is None:
        ev = writer._surface_eval = SurfaceEval(writer.x)
    nb = writer.x[writer.x[sp["b_curve"]]["nurbs"]]
    dim = nb["vertex_dim"]
    verts = writer.x[nb["bspline_vertices"]]["vertices"]
    poles, weights = [], []
    for i in range(nb["n_vertices"]):
        p = verts[i * dim:(i + 1) * dim]
        if nb["rational"]:
            weights.append(p[-1])
            p = [c / p[-1] for c in p[:-1]]
        poles.append(App.Base.Vector2d(p[0], p[1]))
    knots, mults = knot_vector(writer.x, nb["knots"], nb["knot_mult"], nb["n_knots"])
    c2 = Part.Geom2d.BSplineCurve2d()
    c2.buildFromPolesMultsKnots(poles, mults, knots, False, nb["degree"], weights or None)
    t0, t1 = c2.FirstParameter, c2.LastParameter
    out = []
    for i in range(n + 1):
        uv = c2.value(t0 + (t1 - t0) * i / n)
        out.append(tuple(ev.value(sp["surface"], uv.x, uv.y) * M2MM))
    return out


def occ_curve(writer, idx):
    """XT curve -> FreeCAD geometry in mm (used for blend spines)."""
    n = writer.x[idx]
    t = n.name
    if t == "LINE":
        p = _vec(n["pvec"]) * M2MM
        return Part.Line(p, p + _vec(n["direction"]))
    if t == "CIRCLE":
        c = Part.Circle(_vec(n["centre"]) * M2MM, _vec(n["normal"]), n["radius"] * M2MM)
        c.XAxis = _vec(n["x_axis"])
        return c
    if t == "ELLIPSE":
        c = Part.Ellipse(App.Vector(), n["major_radius"] * M2MM, n["minor_radius"] * M2MM)
        c.Center = _vec(n["centre"]) * M2MM
        c.Axis = _vec(n["normal"])
        c.XAxis = _vec(n["x_axis"])
        return c
    if t == "TRIMMED_CURVE":
        return occ_curve(writer, n["basis_curve"])
    pts = [App.Vector(*p) for p in writer.curve_points(idx)] if t != "B_CURVE" else None
    if t == "B_CURVE":
        nb = writer.x[n["nurbs"]]
        dim = nb["vertex_dim"]
        verts = writer.x[nb["bspline_vertices"]]["vertices"]
        poles, weights = [], []
        for i in range(nb["n_vertices"]):
            p = verts[i * dim:(i + 1) * dim]
            if nb["rational"]:
                weights.append(p[-1])
                p = [c / p[-1] for c in p[:-1]]
            poles.append(App.Vector(*p[:3]) * M2MM)
        knots, mults = knot_vector(writer.x, nb["knots"], nb["knot_mult"], nb["n_knots"])
        c = Part.BSplineCurve()
        c.buildFromPolesMultsKnots(poles, mults, knots, False, nb["degree"], weights or None)
        return c
    c = Part.BSplineCurve()
    c.interpolate(pts)
    return c


def rolling_ball_surface(writer, n):
    """Rolling-ball blend as the tube of its radius around the spine (mm)."""
    spine = occ_curve(writer, n["spine"])
    r = abs(n["range"][0]) * M2MM
    if r <= 0:
        raise ValueError("cliff-edge blends are not supported")
    t0, t1 = spine.FirstParameter, spine.LastParameter
    if spine.isPeriodic():
        t1 = t0 + spine.getPeriod() if hasattr(spine, "getPeriod") else t0 + 2 * math.pi
    elif abs(t0) > 1e50 or abs(t1) > 1e50:  # infinite line: span the blend face
        t0, t1 = -1e4, 1e4
    path = spine.toShape(t0, t1)
    p0 = path.valueAt(path.FirstParameter)
    d0 = path.tangentAt(path.FirstParameter)
    profile = Part.Wire(Part.Circle(p0, d0, r).toShape())
    pipe = Part.Wire(path).makePipeShell([profile], False, True)
    surf = pipe.Faces[0].Surface
    if not isinstance(surf, Part.BSplineSurface):
        surf = pipe.Faces[0].toNurbs().Faces[0].Surface
    surf = surf.copy()
    if surf.isUPeriodic():
        surf.setUNotPeriodic()
    if surf.isVPeriodic():
        surf.setVNotPeriodic()
    return surf
