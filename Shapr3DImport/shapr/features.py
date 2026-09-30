"""Replay Shapr3D history as FreeCAD Part features (best effort).

Shapr3D is a multi-body direct modeller, so bodies are tracked by the
history node that created them; every feature that consumes or modifies a
body replaces the tracked FreeCAD object for it.

Face-based tools (face offset, face move, chamfer, fillet, shell) name faces
by the feature that created them (see NameResolver.face_keys). The tracker
keeps, per body, the plane and a point of every face it can name, moved
along with the body by transforms and offsets, and finds a named face in the
current FreeCAD shape by that plane and point.
"""

import math

import FreeCAD as App
import Part

from .faceops import (make_chamfer, make_face_move, make_face_offset, offset_delta,
                      outward_normal, plane_of)
from .profile import make_profile
from .sketches import M2MM

# Extrude/Revolve operation enum (param 5 / 4).
OP_NEW, OP_ADD, OP_INTERSECT, OP_CUT = 0, 1, 2, 3

# Not rebuilt yet.
TOPOLOGY_OPS = {"Align", "Split", "Mirror"}
FACE_TOL = 1e-3  # mm
IGNORED_OPS = {"MaterializeSketchPlane", "CreateCGPlane"}


class Bodies:
    """Shapr body identity -> current FreeCAD object."""

    def __init__(self, resolver, order):
        self.r = resolver
        self.order = order   # history node -> position in history
        self.node_body = {}  # history node -> body key
        self.node_outputs = {}  # history node -> {input body key: output body key}
        self.node_new = {}  # history node -> bodies it created
        self.obj = {}        # body key -> FreeCAD object
        self.parent = {}     # merged bodies: key -> surviving key
        self.dead = set()
        self.next_key = 0
        # body key -> {face key: (point, normal or None, radius)}, global mm.
        # Face keys are (node, qualifier, source name[, index]), see
        # NameResolver.face_keys.
        self.faces = {}

    def snapshot(self):
        return (dict(self.node_body), {n: dict(m) for n, m in self.node_outputs.items()},
                dict(self.obj), dict(self.parent), set(self.dead), self.next_key,
                {k: dict(f) for k, f in self.faces.items()},
                {n: list(v) for n, v in self.node_new.items()})

    def restore(self, state):
        nb, no, obj, parent, dead, nk, faces, first = state
        self.node_body, self.obj, self.parent, self.dead, self.next_key = (
            dict(nb), dict(obj), dict(parent), set(dead), nk)
        self.node_new = {n: list(v) for n, v in first.items()}
        self.node_outputs = {n: dict(m) for n, m in no.items()}
        self.faces = {k: dict(f) for k, f in faces.items()}

    def find(self, k):
        while k in self.parent:
            k = self.parent[k]
        return k

    def new(self, node, obj):
        k = self.next_key
        self.next_key += 1
        self.obj[k] = obj
        self.node_body[node] = k
        self.node_new.setdefault(node, []).append(k)
        return k

    def set(self, k, obj, node=None):
        self.obj[k] = obj
        if node is not None:
            self.node_body[node] = k

    def merge(self, keys, obj, node):
        keys = [self.find(k) for k in keys]
        keep = keys[0]
        for k in keys[1:]:
            if k != keep:
                self.parent[k] = keep
                self.faces.setdefault(keep, {}).update(self.faces.pop(k, {}))
        self.set(keep, obj, node)
        return keep

    def resolve(self, ref):
        """Topology name (body, face or edge) -> live body key, or None."""
        k = self._resolve_raw(ref, 0)
        if k is None:
            return None
        k = self.find(k)
        return None if k in self.dead else k

    def _resolve_raw(self, ref, depth):
        node, sources = self.r.creator_sources(ref)
        outputs = self.node_outputs.get(node)
        if outputs and depth < 20:
            for src in sources:
                inp = self._resolve_raw(src, depth + 1)
                if inp in outputs:
                    return outputs[inp]
        k = self.node_body.get(node)
        if k is None:
            # Fall back to the latest known node in the name's ancestry.
            known = [n for n in self.r.origins(ref) if n in self.node_body]
            if not known:
                return None
            k = self.node_body[max(known, key=lambda n: self.order.get(n, -1))]
        return k

    def resolve_all(self, refs):
        out = []
        for ref in refs or []:
            if isinstance(ref, tuple) and ref[0] == "name":
                for k in self._prefix_bodies(ref[1]) or [self.resolve(ref[1])]:
                    if k is not None and k not in out:
                        out.append(k)
        return out

    def _prefix_bodies(self, nid):
        """A callKeyPrefix body name stands for every body its node created."""
        t, d = self.r.names.get(self.r.unwrap(nid), (None, None))
        if t != 1 or d.get("qualifier", 4) != 4:
            return []
        node = d["callKeyPrefix"].get("nodeID")
        keys = [self.find(k) for k in self.node_new.get(node, [])]
        return [k for k in dict.fromkeys(keys) if k not in self.dead]

    def move_faces(self, src, dst, placement):
        """Faces of body src, moved by placement, become the faces of dst."""
        rot = placement.Rotation
        self.faces[dst] = {fk: (placement.multVec(p), n and rot.multVec(n), r)
                           for fk, (p, n, r) in self.faces.get(src, {}).items()}

    def alive(self):
        return {k: o for k, o in self.obj.items() if k not in self.parent and k not in self.dead}


def _names(v):
    if isinstance(v, tuple) and v[0] == "name":
        return [v]
    if isinstance(v, list):
        return [x for x in v if isinstance(x, tuple) and x[0] == "name"]
    return []


def _num(v, report, what):
    """Quantity parameter; evaluates simple expression nodes."""
    if isinstance(v, (int, float)) and not isinstance(v, bool):
        return float(v)
    if isinstance(v, list) and len(v) == 5 and isinstance(v[3], str):
        report.warn("%s uses an expression (%s); value not evaluated, using 0" % (what, v[3]))
    return 0.0


class Replayer:
    def __init__(self, doc, shapr, resolver, sketch_builds, report):
        self.doc = doc
        self.f = shapr
        self.r = resolver
        self.builds = sketch_builds  # sketchID -> SketchBuild
        self.curve_sketch = {cid: b for b in sketch_builds.values() for cid in b.geo}
        self.report = report
        self.bodies = Bodies(resolver, {ft.node_id: i for i, ft in enumerate(shapr.features)})
        # Faces named by these nodes are copies of their source faces.
        self.renaming = {ft.node_id for ft in shapr.features if ft.op == "Transform"}
        self.skipped = {}

    # ---- helpers ----

    def _add(self, typ, name, label):
        o = self.doc.addObject(typ, name)
        o.Label = label
        return o

    def _profile(self, ft, refs):
        """Profile regions -> ShaprProfile object (all regions must share a sketch)."""
        regions, senses, bounded, topology, sketch = [], [], [], [], None
        body_keys = []
        for ref in refs:
            reg = self.r.region(ref[1])
            if reg is None:
                raise ValueError("profile is not a sketch region (face push-pull)")
            curves, sense, body_edges, topo = reg
            present = [c for c in curves if c in self.curve_sketch]
            if len(present) < len(curves):
                self.report.warn("%s: profile references %d deleted sketch curve(s)"
                                 % (ft.name, len(curves) - len(present)))
            if not present:
                raise ValueError("profile has no sketch curves left")
            b = self.curve_sketch[present[0]]
            if sketch is not None and b is not sketch:
                raise ValueError("profile regions span several sketches")
            sketch = b
            regions.append([b.geo[c] for c in present])
            senses.append([s for c, s in zip(curves, sense) if c in self.curve_sketch])
            bounded.append(bool(body_edges))
            topology.append(topo)
            for e in body_edges:
                k = self.bodies.resolve(e)
                if k is not None and k not in body_keys:
                    body_keys.append(k)
        if any(bounded) and not body_keys:
            raise ValueError("profile is bounded by edges of an unknown body")
        bodies = [self.bodies.obj[k] for k in body_keys]
        prof = make_profile(self.doc, sketch.obj, regions, senses, bounded, bodies, topology,
                            ft.name + " profile")
        error = ""
        try:
            self.doc.recompute([prof])
        except Exception as e:
            error = str(e)
        if prof.Shape.isNull():
            raise ValueError("no sketch region matched (%s)"
                             % (prof.Status or error or prof.getStatusString()))
        if prof.Status != "ok":
            self.report.info("%s: profile %s" % (ft.name, prof.Status))
        return prof, sketch

    def _combine(self, ft, tool, op, targets):
        """Apply a new solid to target bodies according to the operation enum."""
        b = self.bodies
        if op == OP_NEW or not targets:
            if op not in (OP_NEW,):
                self.report.warn("%s: operation %s without target bodies, created new body" % (ft.name, op))
            return [b.new(ft.node_id, tool)]
        if op == OP_ADD:
            fuse = self._add("Part::MultiFuse", "Fusion", ft.name)
            fuse.Shapes = [b.obj[k] for k in targets] + [tool]
            fuse.Refine = True
            return [b.merge(targets, fuse, ft.node_id)]
        if op in (OP_CUT, OP_INTERSECT):
            for k in targets:
                if op == OP_CUT:
                    o = self._add("Part::Cut", "Cut", ft.name)
                    o.Base, o.Tool = b.obj[k], tool
                    o.Refine = True
                else:
                    o = self._add("Part::MultiCommon", "Common", ft.name)
                    o.Shapes = [b.obj[k], tool]
                b.set(k, o, ft.node_id)
            return targets
        raise ValueError("unknown extrude operation %s" % op)

    def _extrude_faces(self, ft, ext, prof, direction, length, keep=None):
        """Face keys of a new extrusion -> registry entries.

        keep(face) filters the extrusion's faces (e.g. to those a cut leaves).
        """
        ext.recompute()
        shape = ext.Shape
        entries = {}
        for _nid, q, src in self.r.node_names(ft.node_id):
            if q == 0:
                mid = self._fin_midpoint(src) if src is not None else None
                if mid is None:
                    continue
                pt = mid + direction * (length / 2)
                found = _nearest_faces(shape, [pt])
                if not found:
                    # The curve runs past the region edge: take the side face
                    # whose plane holds it.
                    sides = [(f.distToShape(Part.Vertex(pt))[0], i, f) for i, f in enumerate(shape.Faces)
                             if plane_of(f) is not None and abs(plane_of(f).Axis.dot(direction)) < 1e-6
                             and abs(plane_of(f).Axis.dot(pt - plane_of(f).Position)) < FACE_TOL]
                    found = [min(sides, key=lambda x: x[0])[1:]] if sides else []
            elif q in (1, 2):
                base = prof.Shape.Faces[0].CenterOfMass + (direction * length if q == 1 else App.Vector())
                found = [(i, f) for i, f in enumerate(shape.Faces)
                         if _on_plane(f, base, direction)]
            else:
                continue
            for j, (_i, f) in enumerate(found):
                if keep is None or keep(f):
                    entries[(ft.node_id, q, src, j)] = _entry(f)
        return entries

    def _fin_midpoint(self, fin):
        """Midpoint of the sketch curve or body edge a region edge runs along."""
        cid = self.r.fin_curve(fin)
        if cid in self.curve_sketch:
            b = self.curve_sketch[cid]
            edge = b.obj.Geometry[b.geo[cid]].toShape().transformed(
                b.obj.getGlobalPlacement().toMatrix()).Edges[0]
            return _mid(edge)
        edge_name = self.r.fin_body_edge(fin)
        if edge_name is None:
            return None
        try:
            _k, edges, shape = self._edges([edge_name])
        except ValueError:
            return None
        return _centroid([shape.Edges[i] for i in edges[edge_name]])

    def _register(self, keys, entries):
        for k in keys:
            self.bodies.faces.setdefault(self.bodies.find(k), {}).update(entries)

    def _find_faces(self, k, name):
        """Face name on body k -> [(registry key, face index)]."""
        keys = self.r.face_keys(name, self.renaming)
        shape = Part.getShape(self.bodies.obj[k])
        out = []
        for fk, (p, n, radius) in self.bodies.faces.get(k, {}).items():
            if not any(fk[:3] == key or (key[2] is None and fk[:2] == key[:2]) for key in keys):
                continue
            best = None
            for i, f in enumerate(shape.Faces):
                if n is not None and not _on_plane(f, p, n):
                    continue
                d = f.distToShape(Part.Vertex(p))[0]
                if d <= radius + FACE_TOL and (best is None or d < best[0]):
                    best = (d, i)
            if best is not None:
                out.append((fk, best[1]))
        return out, shape

    def _face_subs(self, name):
        """Face name -> (body key, [(registry key, face index)], body shape)."""
        k = self.bodies.resolve(name)
        if k is None:
            raise ValueError("face of an unknown body")
        found, shape = self._find_faces(k, name)
        if not found:
            raise ValueError("face not found on its body")
        return k, found, shape

    # ---- features ----

    def extrude(self, ft):
        p = ft.params
        prof, build = self._profile(ft, _names(p[0]))
        dist = _num(p[1], self.report, ft.name) * M2MM
        draft = _num(p[2], self.report, ft.name)
        normal = build.obj.getGlobalPlacement().Rotation.multVec(App.Vector(0, 0, 1))
        ext = self._add("Part::Extrusion", "Extrude", ft.name)
        ext.Base = prof
        ext.DirMode = "Custom"
        ext.Dir = normal
        ext.LengthFwd = abs(dist)
        ext.Reversed = dist < 0
        ext.Solid = True
        ext.TaperAngle = math.degrees(draft)
        prof.Visibility = False
        op = p[5] if len(p) > 5 and isinstance(p[5], int) else OP_NEW
        targets = self.bodies.resolve_all(p[6] if len(p) > 6 else [])
        keep = None
        if op == OP_CUT and targets:
            # A tool face lying on a face the body already has does not
            # become a face of its own; the body's face keeps its name.
            old = [f for k in targets for f in Part.getShape(self.bodies.obj[k]).Faces]
            keep = lambda f: not any(_overlap(f, g) for g in old)  # noqa: E731
        keys = self._combine(ft, ext, op, targets)
        direction = normal * (-1 if dist < 0 else 1)
        self._register(keys, self._extrude_faces(ft, ext, prof, direction, abs(dist), keep))

    def revolve(self, ft):
        p = ft.params
        prof, build = self._profile(ft, _names(p[0]))
        axis_ref = _names(p[1])
        cid = self.r.curve(axis_ref[0][1]) if axis_ref else None
        if cid not in self.curve_sketch:
            raise ValueError("revolve axis is not a sketch line")
        ab = self.curve_sketch[cid]
        c = ab.curves[cid]
        if c.kind != "line":
            raise ValueError("revolve axis is not a line")
        pl = ab.obj.getGlobalPlacement()
        a = pl.multVec(App.Vector(c.start.x, c.start.y, 0) * M2MM)
        e = pl.multVec(App.Vector(c.end.x, c.end.y, 0) * M2MM)
        rev = self._add("Part::Revolution", "Revolve", ft.name)
        rev.Source = prof
        rev.Base = a
        rev.Axis = (e - a).normalize()
        rev.Angle = math.degrees(_num(p[2], self.report, ft.name))
        rev.Solid = True
        prof.Visibility = False
        op = p[4] if len(p) > 4 and isinstance(p[4], int) else OP_NEW
        self._combine(ft, rev, op, self.bodies.resolve_all(p[5] if len(p) > 5 else []))

    def union(self, ft):
        keys = []
        for i in (0, 1, 6):
            for k in self.bodies.resolve_all(ft.params[i] if len(ft.params) > i else []):
                if k not in keys:
                    keys.append(k)
        if len(keys) < 2:
            raise ValueError("union needs two known bodies, found %d" % len(keys))
        fuse = self._add("Part::MultiFuse", "Fusion", ft.name)
        fuse.Shapes = [self.bodies.obj[k] for k in keys]
        fuse.Refine = True
        self.bodies.merge(keys, fuse, ft.node_id)

    def subtract(self, ft):
        b = self.bodies
        targets = b.resolve_all(ft.params[0])
        tools = [k for k in b.resolve_all(ft.params[1]) if k not in targets]
        if not targets or not tools:
            raise ValueError("subtract needs known target and tool bodies")
        if len(tools) == 1:
            tool = b.obj[tools[0]]
        else:
            tool = self._add("Part::MultiFuse", "Fusion", ft.name + " tools")
            tool.Shapes = [b.obj[k] for k in tools]
        tool_faces = {}
        for k in tools:
            tool_faces.update(b.faces.get(k, {}))
        for k in targets:
            cut = self._add("Part::Cut", "Cut", ft.name)
            cut.Base, cut.Tool = b.obj[k], tool
            cut.Refine = True
            b.set(k, cut, ft.node_id)
        self._register(targets, tool_faces)
        b.dead.update(tools)

    def delete(self, ft):
        refs = _names(ft.params[0])
        body_refs = [x for x in refs if self.r.is_body_name(x[1])]
        for k in self.bodies.resolve_all(body_refs):
            self.bodies.dead.add(k)
        if len(body_refs) < len(refs):
            raise ValueError("deleting faces/edges is not supported")

    def transform(self, ft):
        p = ft.params
        b = self.bodies
        # Bodies are selected in slot 0, or as whole-body names among the
        # selected faces (slot 1) and edges (slot 2).
        picked = _names(p[0]) + [n for n in _names(p[1]) + _names(p[2]) if self.r.is_body_name(n[1])]
        keys = b.resolve_all(picked)
        # The destination frame (rotation and translation) is expressed in
        # the source frame, which sits on the pivot.
        src, dst = p[3], p[4]
        fs = _frame(src)
        rot = fs.multiply(_frame(dst)).multiply(fs.inverted())
        move = fs.multVec(_position(dst) or App.Vector())
        if not keys:
            if _names(p[2]):
                raise ValueError("moving edges is not supported")
            if not _names(p[1]):
                raise ValueError("transform of unknown body")
            return self._move_faces(ft, _names(p[1]), rot, move, _position(src))
        pivot = _position(src)
        if pivot is None:
            objs = [b.obj[k] for k in keys]
            for o in objs:
                o.recompute(True)
            bb = Part.getShape(objs[0]).BoundBox
            for o in objs[1:]:
                bb.add(Part.getShape(o).BoundBox)
            pivot = bb.Center
            if not rot.isNull() and abs(rot.Angle) > 1e-9:
                self.report.info("%s: rotation pivot taken from body bounds (approximate)" % ft.name)
        placement = App.Placement(pivot + move - rot.multVec(pivot), rot)
        copy = bool(p[5]) if len(p) > 5 else False
        for k in keys:
            link = self._add("App::Link", "Link", ft.name)
            link.LinkedObject = b.obj[k]
            link.LinkTransform = True
            link.Placement = placement
            if copy:
                new = b.new(ft.node_id, link)
                b.node_outputs.setdefault(ft.node_id, {})[k] = new
                b.move_faces(k, new, placement)
            else:
                b.set(k, link, ft.node_id)
                b.move_faces(k, k, placement)

    def _move_faces(self, ft, refs, rot, move, pivot):
        """Transform of faces: their planes move, neighbours follow."""
        b = self.bodies
        per_body = {}
        for ref in refs:
            k, found, shape = self._face_subs(ref[1])
            per_body.setdefault(k, (shape, []))[1].extend(found)
        for k, (shape, found) in per_body.items():
            index = sorted({i for _fk, i in found})
            if pivot is None:  # gizmo on the faces: their centre
                pts = [shape.Faces[i].CenterOfMass for i in index]
                pivot = sum(pts, App.Vector()) * (1.0 / len(pts))
            placement = App.Placement(pivot + move - rot.multVec(pivot), rot)
            obj = make_face_move(self.doc, b.obj[k], ["Face%d" % (i + 1) for i in index],
                                 placement, ft.name)
            faces = b.faces[k]
            for fk, _i in found:
                pt, n, r = faces[fk]
                faces[fk] = (placement.multVec(pt), n and rot.multVec(n), r)
            b.set(k, obj, ft.node_id)

    def offset_face(self, ft):
        p = ft.params
        dist = _num(p[1], self.report, ft.name) * M2MM
        ref = None
        refs = _names(p[4]) if len(p) > 4 else []
        if refs:  # distance measured from a reference face (mode 1, 3)
            rk, rfound, rshape = self._face_subs(refs[0][1])
            ref = (self.bodies.obj[rk], ["Face%d" % (rfound[0][1] + 1)])
            ref_face = rshape.Faces[rfound[0][1]]
        per_body = {}
        for ref_name in _names(p[0]):
            k, found, shape = self._face_subs(ref_name[1])
            per_body.setdefault(k, (shape, []))[1].extend(found)
        b = self.bodies
        for k, (shape, found) in per_body.items():
            index = sorted({i for _fk, i in found})
            obj = make_face_offset(self.doc, b.obj[k], ["Face%d" % (i + 1) for i in index],
                                   dist, ref, ft.name)
            moved = {}
            for i in index:
                f = shape.Faces[i]
                moved[i] = outward_normal(f) * offset_delta(f, dist, ref_face if ref else None)
            faces = b.faces[k]
            for fk, i in found:
                pt, n, r = faces[fk]
                faces[fk] = (pt + moved[i], n, r)
            b.set(k, obj, ft.node_id)

    def _edges(self, edge_names, side_faces=None):
        """Body edge names -> (body key, {edge name: [edge index]}, body shape).

        An edge is found as the edges shared by its two neighbouring faces.
        """
        k, shape, out = None, None, {}
        for nid in edge_names:
            sides = side_faces or self.r.edge_faces(nid, self.renaming)
            if not sides:
                raise ValueError("edge %s has no known neighbouring faces" % nid)
            found = []
            for names in sides:
                idx = set()
                for n in names:
                    kk, faces, shape = self._face_subs(n)
                    if k is not None and kk != k:
                        raise ValueError("edges span several bodies")
                    k = kk
                    idx.update(i for _fk, i in faces)
                found.append(idx)
            edges_a = [e for i in found[0] for e in shape.Faces[i].Edges]
            edges_b = [e for i in found[1] for e in shape.Faces[i].Edges]
            common = [e for e in edges_a if any(e.isSame(x) for x in edges_b)]
            idx = [i for i, e in enumerate(shape.Edges) if any(e.isSame(c) for c in common)]
            if not idx:
                raise ValueError("edge %s: neighbouring faces do not meet" % nid)
            out[nid] = idx
        return k, out, shape

    def _blend(self, ft, make, edge_names, side_faces=None):
        """Fillet / chamfer on named edges; registers the new faces.

        make(base object, sorted edge indices) creates the feature.
        """
        k, edges, shape = self._edges(edge_names, side_faces)
        b = self.bodies
        obj = make(b.obj[k], sorted({i for idx in edges.values() for i in idx}))
        obj.recompute()
        result = Part.getShape(obj)
        mids = {nid: _centroid([shape.Edges[i] for i in idx]) for nid, idx in edges.items()}
        entries = {}
        for _nid, q, src in self.r.node_names(ft.node_id):
            src = self.r.unwrap(src) if src is not None else None
            mid = mids.get(src) or mids.get(next(iter(mids)) if len(mids) == 1 else None)
            if mid is None or not result.Faces:
                continue
            f = min(result.Faces, key=lambda f: (f.CenterOfMass - mid).Length)
            entries[(ft.node_id, q, src, 0)] = _entry(f)
        b.set(k, obj, ft.node_id)
        b.faces.setdefault(k, {}).update(entries)

    def chamfer(self, ft):
        p = ft.params
        d1 = _num(p[3], self.report, ft.name) * M2MM
        d2 = _num(p[4], self.report, ft.name) * M2MM if p[4] is not None else d1
        sides = None
        if _names(p[1]) and _names(p[2]):
            sides = ([p[1][1]], [p[2][1]])
        self._blend(ft, lambda base, idx: make_chamfer(
            self.doc, base, ["Edge%d" % (i + 1) for i in idx], d1, d2, ft.name),
            [n[1] for n in _names(p[0])], sides)

    def fillet(self, ft):
        r = _num(ft.params[3], self.report, ft.name) * M2MM

        def make(base, idx):
            obj = self._add("Part::Fillet", "Fillet", ft.name)
            obj.Base = base
            obj.Edges = [(i + 1, r, r) for i in idx]
            return obj
        self._blend(ft, make, [n[1] for n in _names(ft.params[0])])

    def shell(self, ft):
        p = ft.params
        thickness = _num(p[1], self.report, ft.name) * M2MM
        refs = _names(p[0])
        if not refs:
            raise ValueError("shell without open faces")
        k, found, shape = self._face_subs(refs[0][1])
        for ref_name in refs[1:]:
            k2, more, _s = self._face_subs(ref_name[1])
            if k2 != k:
                raise ValueError("shell faces span several bodies")
            found += more
        b = self.bodies
        obj = self._add("Part::Thickness", "Thickness", ft.name)
        obj.Faces = (b.obj[k], sorted({"Face%d" % (i + 1) for _fk, i in found}))
        obj.Value = -thickness
        obj.Join = "Intersection"
        # Inner walls are named after the outer face they are offset from.
        entries = {}
        for _nid, q, src in self.r.node_names(ft.node_id):
            if src is None:
                continue
            for _fk, i in self._find_faces(k, src)[0][:1]:
                f = shape.Faces[i]
                pt, n, r = _entry(f)
                entries[(ft.node_id, q, src, 0)] = (pt - outward_normal(f) * thickness, n, r)
        b.set(k, obj, ft.node_id)
        b.faces[k].update(entries)

    def imported(self, ft):
        raise ValueError("imported bodies are stored as Parasolid, which FreeCAD cannot read")

    # ---- driver ----

    def run(self):
        handlers = {"Extrude": self.extrude, "Revolve": self.revolve, "Union": self.union,
                    "Subtract": self.subtract, "Delete": self.delete, "Transform": self.transform,
                    "OffsetFace": self.offset_face, "Shell": self.shell,
                    "Chamfer": self.chamfer, "Fillet": self.fillet,
                    "MaterializeImportedBodies": self.imported}
        for ft in self.f.features:
            if ft.op in IGNORED_OPS:
                continue
            h = handlers.get(ft.op)
            before = set(o.Name for o in self.doc.Objects)
            state = self.bodies.snapshot()
            try:
                if h is None:
                    why = ("is not rebuilt" if ft.op in TOPOLOGY_OPS
                           else "unsupported operation")
                    raise ValueError("%s %s" % (ft.op, why))
                h(ft)
                self._check_new(before)
            except Exception as e:  # one failing feature never aborts the import
                self._rollback(before)
                self.bodies.restore(state)
                self._passthrough(ft)
                self.report.warn("%s (%s) skipped: %s" % (ft.name or ft.op, ft.op, e))
                self.skipped[ft.op] = self.skipped.get(ft.op, 0) + 1
        if self.skipped:
            self.report.info("skipped features: " + ", ".join(
                "%s x%d" % kv for kv in sorted(self.skipped.items(), key=lambda kv: -kv[1])))
        self.doc.recompute()
        return self.bodies.alive()

    def _check_new(self, before):
        """Recompute objects created by one feature; raise if any failed."""
        new = [o for o in self.doc.Objects if o.Name not in before]
        if not new:
            return
        self.doc.recompute(new)
        for o in new:
            shape = Part.getShape(o)
            if "Invalid" in o.State or "Error" in o.State or shape.isNull() or not shape.isValid():
                raise ValueError("%s did not compute: %s" % (o.Label, o.getStatusString()))

    def _rollback(self, before):
        new = [o for o in self.doc.Objects if o.Name not in before]
        for o in reversed(new):
            self.doc.removeObject(o.Name)

    def _passthrough(self, ft):
        """Skipped feature: later references to its output resolve to the input body."""
        for v in ft.params:
            for ref in _names(v):
                k = self.bodies.resolve(ref[1])
                if k is not None:
                    self.bodies.node_body.setdefault(ft.node_id, k)
                    return


def _get(d, *path):
    for p in path:
        if not isinstance(d, dict):
            return None
        d = d.get(p)
    return d


def _frame(fr):
    z = _get(fr, "axis", "coordinates")
    x = _get(fr, "referenceDirection", "coordinates")
    if not z or not x:
        return App.Rotation()
    z = App.Vector(*z).normalize()
    x = App.Vector(*x)
    x = (x - z * x.dot(z)).normalize()
    y = z.cross(x)
    return App.Rotation(App.Matrix(x.x, y.x, z.x, 0, x.y, y.y, z.y, 0, x.z, y.z, z.z, 0, 0, 0, 0, 1))


def _position(fr):
    pos = _get(fr, "center", "position")
    return App.Vector(*pos) * M2MM if pos else None


def _entry(face):
    """Registry entry for a face: (point, plane normal or None, radius)."""
    plane = plane_of(face)
    return face.CenterOfMass, plane and plane.Axis, face.BoundBox.DiagonalLength / 2


def _on_plane(face, point, normal):
    plane = plane_of(face)
    if plane is None:
        return False
    n = plane.Axis
    return abs(abs(n.dot(normal)) - 1) < 1e-6 and abs(n.dot(point - plane.Position)) < FACE_TOL


def _overlap(a, b):
    """True if two faces share a plane and some area."""
    pa = plane_of(a)
    if pa is None or not _on_plane(b, pa.Position, pa.Axis) or not a.BoundBox.intersect(b.BoundBox):
        return False
    return a.common(b).Area > 1e-6


def _mid(edge):
    return edge.valueAt((edge.FirstParameter + edge.LastParameter) / 2)


def _centroid(edges):
    pts = [_mid(e) for e in edges]
    return sum(pts, App.Vector()) * (1.0 / len(pts))


def _nearest_faces(shape, points):
    """Faces touching each point (within FACE_TOL): [(index, face)]."""
    out = []
    for pt in points:
        v = Part.Vertex(pt)
        for i, f in enumerate(shape.Faces):
            if f.distToShape(v)[0] < FACE_TOL:
                out.append((i, f))
                break
    return out
