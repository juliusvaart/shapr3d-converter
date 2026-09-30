"""Translate Parasolid XT bodies into STEP AP214 B-rep text.

XT topology maps one-to-one onto STEP's advanced B-rep entities, and OCC's
STEP reader builds robust shapes from them (pcurves, periodic seams), so
bodies are handed to FreeCAD through an in-memory STEP file.

Lengths are written in millimeters.

XT conventions used (Parasolid XT Format Reference):
- edge end vertex = edge.halfedge.vertex, start = edge.halfedge.other.vertex
- curve.sense '-': the edge runs against the curve parameter
- fin.sense '-': the loop runs against its edge
- face normal = natural surface normal iff face and surface senses agree;
  loops keep the face on their left (same rule as STEP face bounds)
- spun surfaces swap u/v against STEP surfaces of revolution, so their
  normals flip
"""

import math

M2MM = 1000.0


class StepError(ValueError):
    pass


def _f(x):
    s = repr(float(x))
    return s if ("." in s or "e" in s or "n" in s) else s + "."


def _list(items):
    return "(" + ",".join(items) + ")"


class StepWriter:
    def __init__(self, data):
        self.x = data
        self.lines = []
        self.n = 0
        self.cache = {}
        self.warnings = []

    def add(self, text):
        self.n += 1
        self.lines.append("#%d=%s;" % (self.n, text))
        return "#%d" % self.n

    # ---- primitives ----

    def point(self, v, scale=M2MM):
        return self.add("CARTESIAN_POINT('',(%s,%s,%s))" % tuple(_f(c * scale) for c in v))

    def direction(self, v):
        n = math.sqrt(sum(c * c for c in v)) or 1.0
        return self.add("DIRECTION('',(%s,%s,%s))" % tuple(_f(c / n) for c in v))

    def axis2(self, origin, z, x):
        x = _orthogonal(x, z)
        return self.add("AXIS2_PLACEMENT_3D('',%s,%s,%s)" % (
            self.point(origin), self.direction(z), self.direction(x)))

    # ---- curves ----

    def curve(self, idx):
        key = ("c", idx)
        if key in self.cache:
            return self.cache[key]
        n = self.x[idx]
        t = n.name
        if t == "LINE":
            r = self.add("LINE('',%s,%s)" % (
                self.point(n["pvec"]),
                self.add("VECTOR('',%s,1.)" % self.direction(n["direction"]))))
        elif t == "CIRCLE":
            r = self.add("CIRCLE('',%s,%s)" % (
                self.axis2(n["centre"], n["normal"], n["x_axis"]), _f(n["radius"] * M2MM)))
        elif t == "ELLIPSE":
            r = self.add("ELLIPSE('',%s,%s,%s)" % (
                self.axis2(n["centre"], n["normal"], n["x_axis"]),
                _f(n["major_radius"] * M2MM), _f(n["minor_radius"] * M2MM)))
        elif t == "B_CURVE":
            r = self.bcurve(self.x[n["nurbs"]])
        elif t == "TRIMMED_CURVE":
            r = self.curve(n["basis_curve"])
        elif t in ("INTERSECTION", "SP_CURVE"):
            r = self.sampled_curve(idx)
        else:
            raise StepError("unsupported curve %s" % t)
        self.cache[key] = r
        return r

    def bcurve(self, nb):
        dim = nb["vertex_dim"]
        verts = self.x[nb["bspline_vertices"]]["vertices"]
        knots, mults = knot_vector(self.x, nb["knots"], nb["knot_mult"], nb["n_knots"])
        pts, weights = [], []
        for i in range(nb["n_vertices"]):
            p = verts[i * dim:(i + 1) * dim]
            if nb["rational"]:
                weights.append(p[-1])
                p = [c / p[-1] for c in p[:-1]]
            pts.append(self.point(p))
        return self._bspline_curve(nb["degree"], pts, mults, knots, weights)

    def _bspline_curve(self, degree, pts, mults, knots, weights):
        body = "%d,%s,.UNSPECIFIED.,.F.,.F.,%s,%s,.UNSPECIFIED." % (
            degree, _list(pts), _list(str(int(m)) for m in mults), _list(_f(k) for k in knots))
        if not weights:
            return self.add("B_SPLINE_CURVE_WITH_KNOTS(''," + body + ")")
        return self.add(
            "(BOUNDED_CURVE()B_SPLINE_CURVE(%d,%s,.UNSPECIFIED.,.F.,.F.)"
            "B_SPLINE_CURVE_WITH_KNOTS(%s,%s,.UNSPECIFIED.)CURVE()GEOMETRIC_REPRESENTATION_ITEM()"
            "RATIONAL_B_SPLINE_CURVE(%s)REPRESENTATION_ITEM(''))" % (
                degree, _list(pts), _list(str(int(m)) for m in mults),
                _list(_f(k) for k in knots), _list(_f(w) for w in weights)))

    def sampled_curve(self, idx):
        """Intersection / SP-curves: cubic B-spline through sample points."""
        pts = self.curve_points(idx)
        if len(pts) < 2:
            raise StepError("degenerate curve #%d" % idx)
        return self._interpolate(pts)

    def _interpolate(self, pts):
        from .fit import interpolate
        degree, poles, mults, knots = interpolate(pts)
        return self._bspline_curve(degree, [self.point(p, 1.0) for p in poles], mults, knots, None)

    def curve_points(self, idx):
        n = self.x[idx]
        if n.name == "INTERSECTION":
            pts = [tuple(c * M2MM for c in h) for h in self.x[n["chart"]]["hvec"]]
            for lim, at_start in ((n["start"], True), (n["end"], False)):
                ln = self.x[lim]
                if ln and ln["type"] == "T" and ln["hvec"]:
                    p = tuple(c * M2MM for c in ln["hvec"][0])
                    pts.insert(0, p) if at_start else pts.append(p)
            return _dedupe(pts)
        if n.name == "SP_CURVE":
            from .fit import sp_curve_points
            return _dedupe(sp_curve_points(self, n))
        if n.name == "TRIMMED_CURVE":
            return self.curve_points(n["basis_curve"])
        raise StepError("cannot sample %s" % n.name)

    # ---- surfaces ----

    def surface(self, idx):
        """(entity, flip) where flip means STEP normal opposes the XT one."""
        key = ("s", idx)
        if key in self.cache:
            return self.cache[key]
        n = self.x[idx]
        t = n.name
        flip = False
        if t == "PLANE":
            r = self.add("PLANE('',%s)" % self.axis2(n["pvec"], n["normal"], n["x_axis"]))
        elif t == "CYLINDER":
            r = self.add("CYLINDRICAL_SURFACE('',%s,%s)" % (
                self.axis2(n["pvec"], n["axis"], n["x_axis"]), _f(n["radius"] * M2MM)))
        elif t == "CONE":
            # Matches Shapr3D's own STEP export: the stored axis is used as is.
            angle = math.atan2(n["sin_half_angle"], n["cos_half_angle"])
            r = self.add("CONICAL_SURFACE('',%s,%s,%s)" % (
                self.axis2(n["pvec"], n["axis"], n["x_axis"]), _f(n["radius"] * M2MM), _f(angle)))
        elif t == "SPHERE":
            r = self.add("SPHERICAL_SURFACE('',%s,%s)" % (
                self.axis2(n["centre"], n["axis"], n["x_axis"]), _f(n["radius"] * M2MM)))
        elif t == "TORUS":
            a, b = n["major_radius"], n["minor_radius"]
            if a > b:
                r = self.add("TOROIDAL_SURFACE('',%s,%s,%s)" % (
                    self.axis2(n["centre"], n["axis"], n["x_axis"]), _f(a * M2MM), _f(b * M2MM)))
            else:
                r = self._apple_torus(n)
        elif t == "B_SURFACE":
            r = self.bsurface(self.x[n["nurbs"]])
        elif t == "OFFSET_SURF":
            base, bflip = self.surface(n["surface"])
            dist = n["offset"] * M2MM
            if self.x[n["surface"]]["sense"] == "-":
                dist = -dist
            if bflip:
                dist = -dist
            r = self.add("OFFSET_SURFACE('',%s,%s,.F.)" % (base, _f(dist)))
            flip = bflip
        elif t == "SWEPT_SURF":
            r = self.add("SURFACE_OF_LINEAR_EXTRUSION('',%s,%s)" % (
                self.curve(n["section"]),
                self.add("VECTOR('',%s,1.)" % self.direction(n["sweep"]))))
        elif t == "SPUN_SURF":
            r = self.add("SURFACE_OF_REVOLUTION('',%s,%s)" % (
                self.curve(n["profile"]),
                self.add("AXIS1_PLACEMENT('',%s,%s)" % (self.point(n["base"]),
                                                       self.direction(n["axis"])))))
            flip = True
        elif t == "BLENDED_EDGE":
            from .fit import rolling_ball_surface
            try:
                r = self.occ_bsurface(rolling_ball_surface(self, n))
            except Exception as e:
                raise StepError("blend surface: %s" % e)
        else:
            raise StepError("unsupported surface %s" % t)
        self.cache[key] = (r, flip)
        return r, flip

    def _apple_torus(self, n):
        """Self-intersecting torus as a revolved circle (same parametrisation)."""
        c, axis, x = n["centre"], n["axis"], _orthogonal(n["x_axis"], n["axis"])
        a, b = n["major_radius"], n["minor_radius"]
        tube = tuple(c[i] + x[i] * a for i in range(3))
        cn = _cross(x, axis)  # circle normal so its parameter matches XT v
        circle = self.add("CIRCLE('',%s,%s)" % (self.axis2(tube, cn, x), _f(b * M2MM)))
        return self.add("SURFACE_OF_REVOLUTION('',%s,%s)" % (
            circle, self.add("AXIS1_PLACEMENT('',%s,%s)" % (self.point(c), self.direction(axis)))))

    def bsurface(self, nb):
        dim = nb["vertex_dim"]
        verts = self.x[nb["bspline_vertices"]]["vertices"]
        nu, nv = nb["n_u_vertices"], nb["n_v_vertices"]
        rows, wrows = [], []
        for i in range(nu):
            row, wrow = [], []
            for j in range(nv):
                k = (i * nv + j) * dim
                p = verts[k:k + dim]
                if nb["rational"]:
                    wrow.append(_f(p[-1]))
                    p = [c / p[-1] for c in p[:-1]]
                row.append(self.point(p))
            rows.append(_list(row))
            wrows.append(_list(wrow))
        uk, um = knot_vector(self.x, nb["u_knots"], nb["u_knot_mult"], nb["n_u_knots"])
        vk, vm = knot_vector(self.x, nb["v_knots"], nb["v_knot_mult"], nb["n_v_knots"])
        knots = "%s,%s,%s,%s,.UNSPECIFIED." % (
            _list(str(int(m)) for m in um), _list(str(int(m)) for m in vm),
            _list(_f(k) for k in uk), _list(_f(k) for k in vk))
        head = "%d,%d,%s,.UNSPECIFIED.,.F.,.F.,.F." % (nb["u_degree"], nb["v_degree"], _list(rows))
        if not nb["rational"]:
            return self.add("B_SPLINE_SURFACE_WITH_KNOTS(''," + head + "," + knots + ")")
        return self.add(
            "(BOUNDED_SURFACE()B_SPLINE_SURFACE(%s)B_SPLINE_SURFACE_WITH_KNOTS(%s)"
            "GEOMETRIC_REPRESENTATION_ITEM()RATIONAL_B_SPLINE_SURFACE(%s)"
            "REPRESENTATION_ITEM('')SURFACE())" % (head, knots, _list(wrows)))

    def occ_bsurface(self, s):
        """FreeCAD BSplineSurface (already in mm) -> STEP entity."""
        poles = s.getPoles()
        weights = s.getWeights()
        rational = any(abs(w - 1.0) > 1e-12 for row in weights for w in row)
        rows = [_list(self.point(tuple(p), 1.0) for p in row) for row in poles]
        head = "%d,%d,%s,.UNSPECIFIED.,.F.,.F.,.F." % (s.UDegree, s.VDegree, _list(rows))
        knots = "%s,%s,%s,%s,.UNSPECIFIED." % (
            _list(str(m) for m in s.getUMultiplicities()), _list(str(m) for m in s.getVMultiplicities()),
            _list(_f(k) for k in s.getUKnots()), _list(_f(k) for k in s.getVKnots()))
        if not rational:
            return self.add("B_SPLINE_SURFACE_WITH_KNOTS(''," + head + "," + knots + ")")
        return self.add(
            "(BOUNDED_SURFACE()B_SPLINE_SURFACE(%s)B_SPLINE_SURFACE_WITH_KNOTS(%s)"
            "GEOMETRIC_REPRESENTATION_ITEM()RATIONAL_B_SPLINE_SURFACE(%s)"
            "REPRESENTATION_ITEM('')SURFACE())" % (
                head, knots, _list(_list(_f(w) for w in row) for row in weights)))

    # ---- topology ----

    def vertex(self, idx):
        key = ("v", idx)
        if key not in self.cache:
            p = self.x[self.x[idx]["point"]]["pvec"]
            self.cache[key] = self.add("VERTEX_POINT('',%s)" % self.point(p))
        return self.cache[key]

    def edge(self, idx):
        key = ("e", idx)
        if key in self.cache:
            return self.cache[key]
        e = self.x[idx]
        fin = self.x[e["halfedge"]]
        other = self.x[fin["other"]] if fin else None
        cidx = e["curve"]
        if not cidx:  # tolerant edge: use the SP-curve of a real fin
            cidx = next((self.x[f]["curve"] for f in self._fins_of(e) if self.x[f]["curve"]), 0)
            if not cidx:
                raise StepError("edge #%d has no curve" % idx)
        cn = self.x[cidx]
        curve = self.curve(cidx)
        same = cn["sense"] == "+"
        sampled = cn.name in ("INTERSECTION", "SP_CURVE") or (
            cn.name == "TRIMMED_CURVE"
            and self.x[cn["basis_curve"]].name in ("INTERSECTION", "SP_CURVE"))
        if sampled and fin["vertex"] and other and other["vertex"]:
            # Resample between the vertices so the curve runs start -> end.
            curve = self._oriented_sample(cidx, other["vertex"], fin["vertex"])
            same = True
        if fin["vertex"] and other and other["vertex"]:
            vs, ve = self.vertex(other["vertex"]), self.vertex(fin["vertex"])
        else:  # ring edge: STEP needs a vertex on the closed curve
            p = self._ring_point(cn)
            vs = ve = self.add("VERTEX_POINT('',%s)" % self.point(p, 1.0))
        r = self.add("EDGE_CURVE('',%s,%s,%s,%s)" % (vs, ve, curve, ".T." if same else ".F."))
        self.cache[key] = r
        return r

    def _oriented_sample(self, cidx, v_start, v_end):
        pts = self.curve_points(cidx)
        ps = [c * M2MM for c in self.x[self.x[v_start]["point"]]["pvec"]]
        pe = [c * M2MM for c in self.x[self.x[v_end]["point"]]["pvec"]]
        if _dist(pts[0], pe) + _dist(pts[-1], ps) < _dist(pts[0], ps) + _dist(pts[-1], pe):
            pts = pts[::-1]
        pts = _trim_polyline(pts, ps, pe)
        return self._interpolate(pts)

    def _ring_point(self, cn):
        t = cn.name
        if t == "CIRCLE":
            x = _orthogonal(cn["x_axis"], cn["normal"])
            return tuple((cn["centre"][i] + x[i] * cn["radius"]) * M2MM for i in range(3))
        if t == "ELLIPSE":
            x = _orthogonal(cn["x_axis"], cn["normal"])
            return tuple((cn["centre"][i] + x[i] * cn["major_radius"]) * M2MM for i in range(3))
        if t == "TRIMMED_CURVE":
            return tuple(c * M2MM for c in cn["point_1"])
        if t == "B_CURVE":
            nb = self.x[cn["nurbs"]]
            v = self.x[nb["bspline_vertices"]]["vertices"]
            d = nb["vertex_dim"]
            p = v[:d]
            if nb["rational"]:
                p = [c / p[-1] for c in p[:-1]]
            return tuple(c * M2MM for c in p[:3])
        pts = self.curve_points(cn.index)
        return pts[0]

    def _fins_of(self, e):
        out, f, seen = [], e["halfedge"], set()
        while f and f not in seen:
            seen.add(f)
            out.append(f)
            f = self.x[f]["other"]
        return out

    def loop(self, idx):
        oriented, f, seen = [], self.x[idx]["halfedge"], set()
        while f and f not in seen:
            seen.add(f)
            fin = self.x[f]
            if fin["edge"]:
                oriented.append(self.add("ORIENTED_EDGE('',*,*,%s,%s)" % (
                    self.edge(fin["edge"]), ".T." if fin["sense"] == "+" else ".F.")))
            f = fin["forward"]
        if not oriented:
            return None
        return self.add("FACE_BOUND('',%s,.T.)" % self.add("EDGE_LOOP('',%s)" % _list(oriented)))

    def face(self, idx):
        fn = self.x[idx]
        surf, flip = self.surface(fn["surface"])
        same = (fn["sense"] == self.x[fn["surface"]]["sense"]) != flip
        bounds, lp = [], fn["loop"]
        while lp:
            b = self.loop(lp)
            if b:
                bounds.append(b)
            lp = self.x[lp]["next"]
        return self.add("ADVANCED_FACE('',%s,%s,%s)" % (_list(bounds), surf, ".T." if same else ".F."))

    # ---- bodies ----

    def body_shells(self, bidx):
        """[(shell node index, [face indices])] for the solid shells of a body."""
        b = self.x[bidx]
        regions, r = [], b["region"]
        while r:
            regions.append(self.x[r])
            r = self.x[r]["next"]
        solid = [s for reg in regions if reg["type"] == "S" for s in self._chain(reg["shell"])]
        shells = solid or [s for reg in regions for s in self._chain(reg["shell"])]
        faces = {s: [] for s in shells}
        for fn in self.x.of_type("FACE"):
            for key in ("shell", "front_shell"):
                if fn[key] in faces:
                    faces[fn[key]].append(fn.index)
        return [(s, faces[s]) for s in shells if faces[s]], bool(solid)

    def _chain(self, start):
        out, i = [], start
        while i and i not in out:
            out.append(i)
            i = self.x[i]["next"]
        return out

    def body(self, bidx, skip_faces=()):
        """STEP representation items for a body; returns item refs."""
        shells, solid = self.body_shells(bidx)
        items = []
        for _s, faces in shells:
            refs = []
            for f in faces:
                if f in skip_faces:
                    continue
                try:
                    refs.append(self.face(f))
                except StepError as e:
                    self.warnings.append("face #%d: %s" % (f, e))
            if not refs:
                continue
            if solid and not skip_faces and len(refs) == len(faces):
                shell = self.add("CLOSED_SHELL('',%s)" % _list(refs))
                items.append(self.add("MANIFOLD_SOLID_BREP('',%s)" % shell))
            else:
                shell = self.add("OPEN_SHELL('',%s)" % _list(refs))
                items.append(self.add("SHELL_BASED_SURFACE_MODEL('',(%s))" % shell))
        return items

    def document(self, items, name="body"):
        unit_len = self.add("(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.))")
        unit_ang = self.add("(NAMED_UNIT(*)PLANE_ANGLE_UNIT()SI_UNIT($,.RADIAN.))")
        unit_sa = self.add("(NAMED_UNIT(*)SI_UNIT($,.STERADIAN.)SOLID_ANGLE_UNIT())")
        unc = self.add("UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-05),%s,"
                       "'DISTANCE_ACCURACY_VALUE','')" % unit_len)
        ctx = self.add("(GEOMETRIC_REPRESENTATION_CONTEXT(3)GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((%s))"
                       "GLOBAL_UNIT_ASSIGNED_CONTEXT((%s,%s,%s))REPRESENTATION_CONTEXT('',''))"
                       % (unc, unit_len, unit_ang, unit_sa))
        origin = self.add("AXIS2_PLACEMENT_3D('',%s,%s,%s)" % (
            self.point((0, 0, 0)), self.direction((0, 0, 1)), self.direction((1, 0, 0))))
        rep = self.add("ADVANCED_BREP_SHAPE_REPRESENTATION('%s',%s,%s)" % (
            name, _list(items + [origin]), ctx))
        app = self.add("APPLICATION_CONTEXT('automotive design')")
        pc = self.add("PRODUCT_CONTEXT('',%s,'mechanical')" % app)
        prod = self.add("PRODUCT('%s','%s','',(%s))" % (name, name, pc))
        pdf = self.add("PRODUCT_DEFINITION_FORMATION('','',%s)" % prod)
        pdc = self.add("PRODUCT_DEFINITION_CONTEXT('part definition',%s,'design')" % app)
        pd = self.add("PRODUCT_DEFINITION('design','',%s,%s)" % (pdf, pdc))
        pds = self.add("PRODUCT_DEFINITION_SHAPE('','',%s)" % pd)
        self.add("SHAPE_DEFINITION_REPRESENTATION(%s,%s)" % (pds, rep))
        return ("ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION((''),'2;1');\n"
                "FILE_NAME('','',(''),(''),'','','');\n"
                "FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }'));\nENDSEC;\nDATA;\n"
                + "\n".join(self.lines) + "\nENDSEC;\nEND-ISO-10303-21;\n")


def knot_vector(data, knots_ptr, mult_ptr, n):
    """Distinct knots and multiplicities; stored arrays may carry padding."""
    return list(data[knots_ptr]["knots"][:n]), [int(m) for m in data[mult_ptr]["mult"][:n]]


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _orthogonal(x, z):
    """x made exactly perpendicular to z (unit)."""
    zn = math.sqrt(sum(c * c for c in z)) or 1.0
    z = [c / zn for c in z]
    d = sum(x[i] * z[i] for i in range(3))
    v = [x[i] - d * z[i] for i in range(3)]
    n = math.sqrt(sum(c * c for c in v))
    if n < 1e-12:
        v = [1.0, 0.0, 0.0] if abs(z[0]) < 0.9 else [0.0, 1.0, 0.0]
        return _orthogonal(v, z)
    return tuple(c / n for c in v)


def _dist(a, b):
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def _dedupe(pts):
    out = [pts[0]] if pts else []
    for p in pts[1:]:
        if _dist(p, out[-1]) > 1e-7:
            out.append(p)
    return out


def _trim_polyline(pts, ps, pe):
    """Replace polyline ends with the exact vertex positions."""
    pts = list(pts)
    pts[0] = tuple(ps)
    pts[-1] = tuple(pe)
    return _dedupe(pts) if len(pts) > 2 else [tuple(ps), tuple(pe)]


def read_bodies(data):
    """Parsed XT data -> [(BODY node, FreeCAD shape)], warnings."""
    import os
    import tempfile

    import Part

    out, warnings = [], []
    for bn in data.of_type("BODY"):
        owner = data[bn["owner"]]
        if owner is not None and owner.name == "BODY":
            continue  # child of a compound body
        w = StepWriter(data)
        items = w.body(bn.index)
        warnings.extend(w.warnings)
        if not items:
            continue
        fd, path = tempfile.mkstemp(suffix=".step")
        try:
            with os.fdopen(fd, "w") as fh:
                fh.write(w.document(items))
            shape = Part.Shape()
            shape.read(path)
        finally:
            os.remove(path)
        if shape.isNull():
            continue
        if not shape.Solids and shape.Shells:
            closed = _close_holes(shape)
            if closed is not None:
                warnings.append("body %d: unsupported faces approximated by surface filling"
                                % bn.index)
                shape = closed
        out.append((bn, shape))
    return out, warnings


def _close_holes(shape):
    """Fill the holes of an open shell (faces we could not translate) and sew."""
    import Part

    faces = list(shape.Faces)
    count = {}
    for f in faces:
        for e in f.Edges:
            count[e.hashCode()] = count.get(e.hashCode(), 0) + 1
    free = [e for f in faces for e in f.Edges if count[e.hashCode()] == 1]
    if not free:
        return None
    fills = []
    for group in Part.sortEdges(free):
        try:
            fills.append(Part.makeFilledFace(group))
        except Exception:
            return None
    sewed = Part.Shape(Part.Compound(faces + fills))
    sewed.sewShape(1e-3)
    shells = sewed.Shells
    if len(shells) != 1 or not shells[0].isClosed():
        return None
    solid = Part.Solid(shells[0])
    if solid.Volume < 0:
        solid.reverse()
    return solid
