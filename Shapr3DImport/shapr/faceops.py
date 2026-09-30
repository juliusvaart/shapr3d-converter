"""Parametric Shapr3D face tools: face offset (push/pull) and chamfer.

Face offset: the face's plane moves along its normal and its neighbours
extend or shrink with it, as in Shapr3D. The volume between the old and the
new face position (side walls in the neighbouring faces' planes) is fused to
the body when the face moves out and cut from it when it moves in. Faces
with curved edges or curved neighbours are swept straight along the normal.

Face move: a face's plane is moved (rotated and/or translated) and the
neighbouring faces extend or shrink to meet it: material between the old and
the new plane is added where the new plane lies outside and removed where it
lies inside, within the column bounded by the neighbouring faces' planes.

Chamfer: a straight edge between two planar faces is cut away by a wedge,
which, unlike OCC's chamfer, also allows a chamfer as wide as a face.

Each tool also stores a geometric hint per referenced face (point, outward
normal) or edge (midpoint, direction), refreshed on every recompute. Sketch
edits renumber the body's faces and edges, and FreeCAD's own element map
does not follow them through these features reliably, so elements are found
from their hints: the face facing the same way nearest the hint's plane and
point, or the parallel edge nearest the hint's midpoint. The stored names
are only used when there are no hints.
"""

import FreeCAD as App
import Part

MODES = ["Distance", "ToReference"]
_OUTPUT_HIDDEN = 8 | 4  # App::Prop_Output | App::Prop_Hidden: no recompute on change


def _add_hints(obj, name, doc):
    obj.addProperty("App::PropertyVectorList", name, "Hints", doc, _OUTPUT_HIDDEN)


def _element(shape, sub):
    try:
        return shape.getElement(sub.split(".")[-1].lstrip("?"))
    except Exception:
        return None


def resolve_faces(obj, link, hint_prop):
    """Faces of a LinkSub property, falling back to the stored hints."""
    base, subs = getattr(obj, link)
    shape = Part.getShape(base)
    hints = list(getattr(obj, hint_prop))
    faces = []
    for i, sub in enumerate(subs):
        if len(hints) >= 2 * i + 2:
            f = _nearest_face(shape, hints[2 * i], hints[2 * i + 1])
        else:
            f = _element(shape, sub)
        if f is None:
            raise ValueError("%s: face %s not found" % (obj.Label, sub))
        faces.append(f)
    setattr(obj, hint_prop, face_hints(faces))
    return shape, faces


def face_hints(faces):
    return [v for f in faces for v in (f.CenterOfMass, outward_normal(f))]


def _nearest_face(shape, point, normal):
    cands = [f for f in shape.Faces if plane_of(f) is not None and outward_normal(f).dot(normal) > 1 - 1e-6]
    if not cands:
        return None
    return min(cands, key=lambda f: (round(abs(normal.dot(f.CenterOfMass - point)), 3),
                                     (f.CenterOfMass - point).Length))


class ShaprFaceOffset:
    """Part::FeaturePython proxy.

    Faces: planar faces of one body to move.
    Mode Distance: move them by Distance along their outward normal.
    Mode ToReference: move them until they lie Distance away from Reference.
    """

    def __init__(self, obj):
        obj.addProperty("App::PropertyLinkSub", "Faces", "FaceOffset", "Faces to move")
        obj.addProperty("App::PropertyEnumeration", "Mode", "FaceOffset", "How Distance is measured")
        obj.Mode = MODES
        obj.addProperty("App::PropertyDistance", "Distance", "FaceOffset",
                        "Offset, or gap to the reference face")
        obj.addProperty("App::PropertyLinkSub", "Reference", "FaceOffset",
                        "Face the gap is measured from (ToReference)")
        _add_hints(obj, "FaceHints", "Point and outward normal per face")
        _add_hints(obj, "ReferenceHints", "Point and outward normal of the reference face")
        obj.Proxy = self

    def dumps(self):
        return None

    def loads(self, state):
        return None

    def execute(self, obj):
        shape, faces = resolve_faces(obj, "Faces", "FaceHints")
        ref = None
        if obj.Mode == "ToReference":
            ref = resolve_faces(obj, "Reference", "ReferenceHints")[1][0]
        obj.Shape = offset_faces(shape, faces, obj.Distance.Value, ref)
        obj.Placement = App.Placement()


class ShaprFaceMove:
    """Part::FeaturePython proxy: move the planes of Faces by Move."""

    def __init__(self, obj):
        obj.addProperty("App::PropertyLinkSub", "Faces", "FaceMove", "Faces to move")
        obj.addProperty("App::PropertyPlacement", "Move", "FaceMove",
                        "Rigid motion applied to the faces' planes")
        _add_hints(obj, "FaceHints", "Point and outward normal per face")
        obj.Proxy = self

    def dumps(self):
        return None

    def loads(self, state):
        return None

    def execute(self, obj):
        shape, faces = resolve_faces(obj, "Faces", "FaceHints")
        obj.Shape = move_faces(shape, faces, obj.Move)
        obj.Placement = App.Placement()


def move_faces(shape, faces, move):
    targets = [(f.CenterOfMass, plane_of(f)) for f in faces]
    for center, plane in targets:
        if plane is None:
            raise ValueError("face move supports planar faces only")
        # Faces are re-found by plane after each move changed the topology.
        cands = [f for f in shape.Faces if plane_of(f) is not None
                 and abs(abs(plane_of(f).Axis.dot(plane.Axis)) - 1) < 1e-6
                 and abs(plane.Axis.dot(f.CenterOfMass - plane.Position)) < 1e-6]
        if not cands:
            raise ValueError("face to move no longer exists")
        face = min(cands, key=lambda f: (f.CenterOfMass - center).Length)
        shape = _tweak(shape, face, move)
    return shape.removeSplitter()


def _tweak(shape, face, move):
    n = outward_normal(face)
    q = face.CenterOfMass
    q2, n2 = move.multVec(q), move.Rotation.multVec(n)
    size = shape.BoundBox.DiagonalLength + (q2 - q).Length
    column = _swept_region(shape, face, size).fuse(_swept_region(shape, face, -size))
    outside_old = _half_space(q, n, size * 4)
    inside_new = _half_space(q2, -n2, size * 4)
    add = column.common(outside_old).common(inside_new)
    cut = column.cut(outside_old).cut(inside_new)
    if add.Volume > 1e-9:
        shape = shape.fuse(add)
    if cut.Volume > 1e-9:
        shape = shape.cut(cut)
    return shape


def _half_space(point, normal, size):
    """Large box on the side of the plane (point, normal) normal points to."""
    rot = App.Rotation(App.Vector(0, 0, 1), normal)
    box = Part.makeBox(size, size, size)
    box.Placement = App.Placement(point + rot.multVec(App.Vector(-size / 2, -size / 2, 0)), rot)
    return box


def make_face_move(doc, base, subs, move, label):
    obj = doc.addObject("Part::FeaturePython", "FaceMove")
    ShaprFaceMove(obj)
    obj.Label = label
    obj.Faces = (base, subs)
    obj.FaceHints = face_hints([Part.getShape(base).getElement(s) for s in subs])
    obj.Move = move
    return obj


class ShaprChamfer:
    """Part::FeaturePython proxy: chamfer Edges by Size1 (and Size2)."""

    def __init__(self, obj):
        obj.addProperty("App::PropertyLinkSub", "Edges", "Chamfer", "Edges to chamfer")
        obj.addProperty("App::PropertyLength", "Size1", "Chamfer", "Setback on the first face")
        obj.addProperty("App::PropertyLength", "Size2", "Chamfer", "Setback on the second face")
        _add_hints(obj, "EdgeHints", "Midpoint and direction per edge")
        obj.Proxy = self

    def dumps(self):
        return None

    def loads(self, state):
        return None

    def execute(self, obj):
        base, subs = obj.Edges
        shape = Part.getShape(base)
        hints = list(obj.EdgeHints)
        edges = []
        for i, sub in enumerate(subs):
            if len(hints) >= 2 * i + 2:
                hp, hd = hints[2 * i], hints[2 * i + 1]
                e = min((x for x in shape.Edges if _along(x, hd)),
                        key=lambda x: (_mid(x) - hp).Length, default=None)
            else:
                e = _element(shape, sub)
            if e is None:
                raise ValueError("%s: edge %s not found" % (obj.Label, sub))
            edges.append(e)
        obj.EdgeHints = edge_hints(edges)
        obj.Shape = chamfer_edges(shape, edges, obj.Size1.Value, obj.Size2.Value)
        obj.Placement = App.Placement()


def _mid(e):
    return e.valueAt((e.FirstParameter + e.LastParameter) / 2)


def _along(e, direction):
    return isinstance(e.Curve, Part.Line) and abs(e.Curve.Direction.dot(direction)) > 1 - 1e-6


def edge_hints(edges):
    return [v for e in edges for v in (_mid(e), e.Curve.Direction if isinstance(e.Curve, Part.Line)
                                       else App.Vector())]


def chamfer_edges(shape, edges, d1, d2):
    wedges, other = [], []
    for e in edges:
        w = _wedge(shape, e, d1, d2)
        (wedges if w is not None else other).append(w or e)
    if other:
        shape = shape.makeChamfer(d1, d2, other)
    if wedges:
        shape = shape.cut(wedges)
    return shape.removeSplitter()


def _wedge(shape, edge, d1, d2):
    """Triangular prism removing a straight edge between two planar faces."""
    if not isinstance(edge.Curve, Part.Line):
        return None
    faces = shape.ancestorsOfType(edge, Part.Face)
    if len(faces) != 2 or any(plane_of(f) is None for f in faces):
        return None
    a, b = edge.Vertexes[0].Point, edge.Vertexes[-1].Point
    t = (b - a).normalize()
    ext = 0.01
    base = a - t * ext
    pts = [base]
    for f, d in zip(faces, (d1, d2)):
        u = plane_of(f).Axis.cross(t).normalize()
        mid = (a + b) * 0.5
        eps = min(d, edge.Length) * 1e-3
        if f.distToShape(Part.Vertex(mid + u * eps))[0] > eps * 1e-3:
            u = -u
        pts.append(base + u * d)
    tri = Part.Face(Part.makePolygon(pts + [base]))
    return tri.extrude(t * (edge.Length + 2 * ext))


def make_chamfer(doc, base, subs, d1, d2, label):
    obj = doc.addObject("Part::FeaturePython", "Chamfer")
    ShaprChamfer(obj)
    obj.Label = label
    obj.Edges = (base, subs)
    obj.EdgeHints = edge_hints([Part.getShape(base).getElement(s) for s in subs])
    obj.Size1 = d1
    obj.Size2 = d2
    return obj


def plane_of(face):
    """Plane of a planar face (whatever its surface type), else None."""
    if isinstance(face.Surface, Part.Plane):
        return face.Surface
    return face.findPlane(1e-7)


def outward_normal(face):
    u0, u1, v0, v1 = face.ParameterRange
    return face.normalAt((u0 + u1) / 2, (v0 + v1) / 2)


def offset_delta(face, distance, ref=None):
    """Signed move of face along its outward normal."""
    if ref is None:
        return distance
    n = outward_normal(face)
    s = n.dot(face.CenterOfMass - ref.CenterOfMass)
    return (distance if s >= 0 else -distance) - s


def offset_faces(shape, faces, distance, ref=None):
    add, cut = [], []
    for f in faces:
        if plane_of(f) is None:
            raise ValueError("face offset supports planar faces only")
        d = offset_delta(f, distance, ref)
        if abs(d) < 1e-9:
            continue
        (add if d > 0 else cut).append(_swept_region(shape, f, d))
    result = shape
    if add:
        result = result.fuse(add)
    if cut:
        result = result.cut(cut)
    return result.removeSplitter()


def _swept_region(shape, face, d):
    """Solid between face and face moved by d along its outward normal, with
    side walls in the planes of the neighbouring faces."""
    n = outward_normal(face)
    walls = []
    for wire in face.Wires:
        edges = wire.OrderedEdges
        planes = []
        for e in edges:
            nb = [g for g in shape.ancestorsOfType(e, Part.Face) if not g.isSame(face)]
            plane = plane_of(nb[0]) if len(nb) == 1 and isinstance(e.Curve, Part.Line) else None
            if plane is None:
                return face.extrude(n * d)
            planes.append(plane)
        old = [e.Vertexes[0].Point if _starts(e, edges[i - 1]) else e.Vertexes[-1].Point
               for i, e in enumerate(edges)]
        # old[i] is the vertex between edge i-1 and edge i.
        new = [_slide(old[i], planes[i - 1], planes[i], n, d) for i in range(len(edges))]
        walls.append((old, new))
    faces = [face.copy(), face.translated(n * d) if all(
        (b - a - n * d).Length < 1e-9 for old, new in walls for a, b in zip(old, new))
        else _polygon_face(walls, face)]
    for old, new in walls:
        for i in range(len(old)):
            j = (i + 1) % len(old)
            quad = [old[i], old[j], new[j], new[i]]
            faces.append(Part.Face(Part.makePolygon(quad + [quad[0]])))
    solid = Part.Solid(Part.Shell(faces))
    if solid.Volume < 0:
        solid.reverse()
    return solid


def _starts(edge, prev):
    """True if edge starts at the vertex it shares with prev."""
    p = edge.Vertexes[0].Point
    return any((p - v.Point).Length < 1e-7 for v in prev.Vertexes)


def _slide(p, pa, pb, n, d):
    """Vertex p (on neighbour planes pa and pb) after its face moves by d*n."""
    na, nb = pa.Axis, pb.Axis
    target = n.dot(p) + d
    if na.cross(nb).Length > 1e-9:
        # Intersection of the two neighbour planes and the moved face plane.
        line = na.cross(nb).normalize()
        if abs(line.dot(n)) > 1e-9:
            return p + line * ((target - n.dot(p)) / line.dot(n))
    # Coplanar neighbours: slide within their plane.
    slide = n - na * n.dot(na)
    if slide.Length < 1e-9:
        return p + n * d
    return p + slide * (d / slide.dot(n))


def _polygon_face(walls, face):
    outer = [w for w, wire in zip(walls, face.Wires) if wire.isSame(face.OuterWire)][0]
    f = Part.Face(Part.makePolygon(outer[1] + [outer[1][0]]))
    holes = [Part.Face(Part.makePolygon(new + [new[0]]))
             for (old, new), wire in zip(walls, face.Wires) if not wire.isSame(face.OuterWire)]
    return f.cut(holes).Faces[0] if holes else f


def make_face_offset(doc, base, subs, distance, ref, label):
    obj = doc.addObject("Part::FeaturePython", "FaceOffset")
    ShaprFaceOffset(obj)
    obj.Label = label
    obj.Faces = (base, subs)
    obj.FaceHints = face_hints([Part.getShape(base).getElement(s) for s in subs])
    obj.Distance = distance
    if ref is not None:
        obj.Mode = "ToReference"
        obj.Reference = ref
        obj.ReferenceHints = face_hints([Part.getShape(ref[0]).getElement(ref[1][0])])
    return obj
