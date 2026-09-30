"""Read a Shapr3D .shapr file (zip containing a SQLite 'workspace').

Only plain Python here (no FreeCAD imports) so it can be tested standalone.
Lengths are stored in meters, angles in radians.
"""

import json
import os
import shutil
import sqlite3
import tempfile
import zipfile
from dataclasses import dataclass, field

import msgpack


@dataclass
class Point:
    id: int
    x: float
    y: float


@dataclass
class Curve:
    id: int
    kind: str  # 'line' | 'arc' | 'circle'
    start: Point = None
    end: Point = None
    center: Point = None
    radius: float = None
    construction: bool = False

    def points(self):
        """(pointID, pos) pairs; pos follows Sketcher: 1=start, 2=end, 3=center."""
        out = []
        if self.start:
            out.append((self.start.id, 1))
        if self.end:
            out.append((self.end.id, 2))
        if self.center:
            out.append((self.center.id, 3))
        return out


@dataclass
class Sketch:
    id: int
    name: str
    hidden: bool
    center: tuple
    normal: tuple
    udir: tuple
    curves: list = field(default_factory=list)
    constraints: list = field(default_factory=list)  # raw JSON dicts
    locked_points: list = field(default_factory=list)
    coincident_groups: list = field(default_factory=list)  # lists of point IDs
    extra_points: list = field(default_factory=list)  # standalone Points (e.g. symbol centers)
    symbols: list = field(default_factory=list)  # (center Point, [curveIDs]) e.g. center rectangles


@dataclass
class ShaprBody:
    name: str
    xt: bytes  # Parasolid XT (neutral binary) data
    matrix: list = None  # 4x4 placement, translation in meters
    hidden: bool = False
    folder: tuple = ()  # browser folder path
    column_major: bool = False


@dataclass
class BodyLabel:
    name: str
    hidden: bool
    folder: tuple
    node: int  # history node that created the named body
    sources: tuple  # names of the topology it was made from


_UNNAMED = BodyLabel("Body", False, (), None, ())


@dataclass
class Feature:
    node_id: int
    name: str
    op: str
    params: list  # decoded parameter values, see decode_value()


def _point(d):
    return Point(d["id"], d["x"], d["y"])


def _curve(curve_id, d):
    t = d["type"]
    if t == 0:
        return Curve(curve_id, "line", start=_point(d["start"]), end=_point(d["end"]))
    if t == 1:
        # Arcs run counter-clockwise from start to end.
        return Curve(curve_id, "arc", start=_point(d["start"]), end=_point(d["end"]),
                     center=_point(d["center"]))
    if t == 2:
        return Curve(curve_id, "circle", center=_point(d["center"]), radius=d["radius"])
    return None


def decode_value(v):
    """Turn a tagged history value into plain Python.

    Values are wrapped as [[[tag, payload]]] (or [[tag, payload]] for
    references). Tags: 0 empty, 1 bool, 2 enum, 4 dict (flat k/v list),
    5 vector, 6 plane, 7 topology name, 9 list, 10 sketch ref, 11 imported
    body, 12 quantity [[unitKind, dim], value].
    """
    while isinstance(v, list) and len(v) == 1 and isinstance(v[0], list):
        v = v[0]
    if not isinstance(v, list) or not v:
        return None
    tag = v[0]
    payload = v[1] if len(v) > 1 else None
    if tag == 0:
        return None
    if tag in (1, 2, 5, 6):
        return payload
    if tag == 12:
        return payload[1]
    if tag == 7:
        return ("name", payload)
    if tag == 10:
        return ("sketch", payload)
    if tag == 11:
        return ("imported", payload)
    if tag == 9:
        return [decode_value(x) for x in payload]
    if tag == 4:
        return {payload[i]: decode_value(payload[i + 1]) for i in range(0, len(payload), 2)}
    return v


class ShaprFile:
    """Parsed contents of one .shapr file."""

    def __init__(self, path):
        self.path = path
        self._tmp = tempfile.mkdtemp(prefix="shapr_")
        try:
            with zipfile.ZipFile(path) as z:
                if "workspace" not in z.namelist():
                    raise ValueError("%s: no 'workspace' entry, not a Shapr3D file" % path)
                z.extract("workspace", self._tmp)
            db = sqlite3.connect(os.path.join(self._tmp, "workspace"))
            try:
                self._load(db)
            finally:
                db.close()
        finally:
            shutil.rmtree(self._tmp, ignore_errors=True)

    def _load_bodies(self, db, nodes):
        """Final solid bodies: [ShaprBody]."""
        root = next((v for v in nodes.values() if v and v[0] == "<root>"), None)
        order = {nid: i for i, nid in enumerate(root[1] if root else [])}
        labels = self._body_labels(db)
        self.bodies = []
        blocks = dict(db.execute("select PartitionID, Block from BodyRevisionBlocks "
                                 "where IsDeleted = 0 and ChunkIndex = 0"))
        if blocks:
            states, created, touches = _replay_bodies(db, order)
            by_body = _name_bodies(labels, created)
            by_pid = _name_partitions(labels, touches)
            for bid, (pid, matrix) in sorted(states.items()):
                if pid in blocks:
                    label = by_body.get(bid) or by_pid.get(pid) or _UNNAMED
                    self.bodies.append(ShaprBody(label.name, bytes(blocks[pid]), matrix,
                                                 hidden=label.hidden, folder=label.folder,
                                                 column_major=True))
            return
        protos = dict(db.execute("select ImportedPrototypeID, BodyData from HistoryImportedPrototypes"))
        imported = {nid: d.get("bodyID") for nid, (t, d) in self.names.items() if t == 11}
        by_body = {}
        for lab in labels:
            for src in lab.sources:
                if src in imported:
                    by_body.setdefault(imported[src], lab)
        for bid, proto, tf in db.execute(
                "select ImportedBodyID, ImportedPrototypeID, Transform from HistoryImportedBodies "
                "order by ImportedBodyID"):
            if proto in protos:
                m = msgpack.unpackb(tf, raw=False) if tf else None
                label = by_body.get(bid, _UNNAMED)
                self.bodies.append(ShaprBody(label.name, bytes(protos[proto]), m,
                                             hidden=label.hidden, folder=label.folder))

    def _body_labels(self, db):
        """Named bodies from metadata: [BodyLabel], ordered by name ID."""
        from .names import NameResolver
        meta, name_meta = {}, {}
        for nid, mid, mtype, data in db.execute(
                "select a.NameID, m.MetadataID, m.MetadataTypeID, m.Data from MetadataAssignments a "
                "join Metadata m on a.MetadataID = m.MetadataID"):
            if isinstance(data, bytes):
                data = data.decode("utf-8", "replace")
            meta.setdefault(nid, {})[mtype] = data
            if mtype == 3:  # identity entry; browser folders list these IDs
                name_meta[nid] = mid
        folders = _folders(db)
        resolver = NameResolver(self.names)
        out = []
        for nid in sorted(meta):
            m = meta[nid]
            if 0 not in m:
                continue
            path, folder_hidden = folders.get(name_meta.get(nid), ((), False))
            anchor = self.names.get(nid, (None, {}))[1].get("nameToAnchor", nid)
            node, sources = resolver.creator_sources(anchor)
            # metadata 0: name, 1: hidden flag ("1")
            out.append(BodyLabel(m[0], m.get(1) == "1" or folder_hidden, path, node, tuple(sources)))
        return out

    def _load(self, db):
        self.names = {nid: (t, json.loads(d)) for nid, t, d in
                      db.execute("select NameID, Type, Data from HistoryNames")}

        self.sketches = {}
        for row in db.execute(
                "select SketchID, Name, IsHidden, PlaneCenterX, PlaneCenterY, PlaneCenterZ, "
                "PlaneNormX, PlaneNormY, PlaneNormZ, PlaneUDirX, PlaneUDirY, PlaneUDirZ "
                "from SketchControllers order by OrderInList, SketchID"):
            name = row[1].decode() if isinstance(row[1], bytes) else row[1]
            self.sketches[row[0]] = Sketch(row[0], name, bool(row[2]), row[3:6], row[6:9], row[9:12])

        construction = set()
        for sid, data in db.execute("select SketchID, ConstructionCurves from ConstructionSketches"):
            if data:
                construction.update(json.loads(data).get("elements", []))

        for cid, sid, data in db.execute("select CurveID, SketchID, Data from SketchCurves order by CurveID"):
            sk = self.sketches.get(sid)
            c = _curve(cid, json.loads(data))
            if sk and c:
                c.construction = cid in construction
                sk.curves.append(c)

        for sid, data in db.execute("select SketchID, Data from Constraints where IsBroken = 0 order by RowID"):
            if sid in self.sketches:
                self.sketches[sid].constraints.append(json.loads(data))

        for sid, data in db.execute("select SketchID, Data from ConstraintPointProperties"):
            if sid in self.sketches and data:
                d = json.loads(data)
                self.sketches[sid].locked_points = d.get("locked", [])
                self.sketches[sid].coincident_groups = [
                    g["points"] for g in d.get("groups", []) if not g.get("isBroken")]

        for sid, data in db.execute("select SketchID, Data from Symbols"):
            d = json.loads(data) if data else {}
            if sid in self.sketches and "center" in d:
                self.sketches[sid].extra_points.append(_point(d["center"]))
                self.sketches[sid].symbols.append((_point(d["center"]), d.get("lines", [])))
        for sid, data in db.execute("select SketchID, Data from StandaloneConstraintPoints"):
            d = json.loads(data) if data else {}
            if sid in self.sketches and {"id", "x", "y"} <= set(d):
                self.sketches[sid].extra_points.append(_point(d))

        nodes = {nid: msgpack.unpackb(p, raw=False, strict_map_key=False)
                 for nid, _t, p in db.execute(
                     "select HistoryTreeNodeID, HistoryTreeNodeType, Properties from HistoryTreeNodes")}
        suppressed = set()
        row = db.execute("select SettingValue from Settings where SettingName = 'HistorySuppressedNodes'").fetchone()
        if row and row[0]:
            suppressed = set(json.loads(row[0]).get("nodes", []))

        self._load_bodies(db, nodes)

        self.features = []
        root = next((v for v in nodes.values() if v and v[0] == "<root>"), None)
        for nid in (root[1] if root else []):
            v = nodes.get(nid)
            if not (isinstance(v, list) and len(v) == 5) or nid in suppressed:
                continue
            params = []
            for pid in v[4]:
                pv = nodes.get(pid)
                # A parameter may itself be an expression node (e.g. ApplyUnit); keep raw.
                params.append(decode_value(pv) if pv is not None and not _is_feature_node(pv) else pv)
            self.features.append(Feature(nid, v[2], v[3], params))


def _replay_bodies(db, order):
    """Replay persisted history calls at body level.

    Shapr3D bodies (own IDs) live in Parasolid partitions (BodyRevisionBlocks
    hold each partition's latest state). PersistedCalls records per call:
    - v[2]: partitions changed [pid, [ok, [before mark, after mark|[false]]]]
      and a tail [body, output index, ...] binding bodies to those partitions;
      a missing after-mark means the body was consumed or deleted;
    - v[3]: lazy moves/copies/deletes: matrices plus [body, source, matrix]
      where source [True, other body] makes a copy and [False], [False] deletes;
    - v[4]: copy-on-write [body, old partition, new partition, ...].
    Returns ({body: (partition, pending 4x4 column-major matrix or None)},
    {node: bodies first seen in its call}, {partition: [(position, node)]}).
    """
    calls = []
    for key, data in db.execute("select CallKey, CallData from PersistedCalls"):
        try:
            k = msgpack.unpackb(key, raw=False, strict_map_key=False)
            v = msgpack.unpackb(data, raw=False, strict_map_key=False)
        except Exception:
            continue
        calls.append(((order.get(k[0], -1), k[1] if len(k) > 1 else []), k[0], v))
    calls.sort(key=lambda c: c[0])

    part, mat, touches, created, seen = {}, {}, {}, {}, set()

    def born(bid, node):
        if bid not in seen:
            seen.add(bid)
            created.setdefault(node, []).append(bid)

    for pos, node, v in calls:
        section = lambda i: v[i][1] if len(v) > i and isinstance(v[i], list) and v[i] and v[i][0] else None
        cow = section(4)
        for entry in (cow[0] if cow else []):
            if isinstance(entry, list) and len(entry) >= 3:
                part[entry[0]] = entry[2]
                mat.pop(entry[0], None)
        geo = section(2)
        if geo and len(geo) > 1:
            outs = geo[1]
            for o in outs:
                touches.setdefault(o[0], []).append((pos, node))
            for t in (geo[2] if len(geo) > 2 else []):
                bid, idx = t[0], t[1]
                if not isinstance(idx, int) or idx >= len(outs):
                    continue
                pid, info = outs[idx][0], outs[idx][1]
                try:
                    gone = info[0] and info[1][1] == [False]
                except (TypeError, IndexError):
                    gone = False
                if gone:
                    part.pop(bid, None)
                    mat.pop(bid, None)
                else:
                    part[bid] = pid
                    mat.pop(bid, None)
                    born(bid, node)
        lazy = section(3)
        if lazy and len(lazy) > 1:
            mats = lazy[0]
            for entry in lazy[1]:
                bid, src, m = entry[0], entry[1], entry[2]
                moved = mats[m[1]] if m and m[0] and m[1] < len(mats) else None
                if src and src[0]:  # copy of another body
                    if src[1] not in part:
                        continue
                    part[bid] = part[src[1]]
                    base = mat.get(src[1])
                    born(bid, node)
                elif not moved:  # neither copy nor move: deleted
                    part.pop(bid, None)
                    mat.pop(bid, None)
                    continue
                else:
                    base = mat.get(bid)
                if moved:
                    mat[bid] = _compose(moved, base)
                elif base:
                    mat[bid] = base
    return {b: (p, mat.get(b)) for b, p in part.items()}, created, touches


def _compose(a, b):
    """a after b, both column-major 4x4 lists (b may be None = identity)."""
    if b is None:
        return list(a)
    out = [0.0] * 16
    for col in range(4):
        for row in range(4):
            out[col * 4 + row] = sum(a[k * 4 + row] * b[col * 4 + k] for k in range(4))
    return out


def _folders(db):
    """{metadata ID of a body name: (folder path, hidden)} from HistoryFolders."""
    rows = {}
    for fid, name, hidden, children in db.execute(
            "select FolderID, FolderName, Hidden, NamedChildren from HistoryFolders"):
        if isinstance(name, bytes):
            name = name.decode("utf-8", "replace")
        try:
            elements = json.loads(children).get("elements", []) if children else []
        except ValueError:
            elements = []
        rows[fid if isinstance(fid, str) else (fid.hex() if isinstance(fid, bytes) else str(fid))] = (
            name, bool(hidden), elements)
    parent = {}
    for fid, (_n, _h, elements) in rows.items():
        for e in elements:
            if e.get("type") == 0:
                parent[str(e.get("id"))] = fid

    def path(fid):
        out, hidden, seen = [], False, set()
        while fid in rows and fid not in seen:
            seen.add(fid)
            name, h, _e = rows[fid]
            hidden = hidden or h
            if name != "root":
                out.insert(0, name)
            fid = parent.get(fid)
        return tuple(out), hidden

    result = {}
    for fid, (_n, _h, elements) in rows.items():
        p, hidden = path(fid)
        for e in elements:
            if e.get("type") == 6:
                try:
                    result[int(e["id"])] = (p, hidden)
                except (KeyError, ValueError):
                    pass
    return result


def _name_bodies(labels, created):
    """Names created at a call go to the bodies that call created, in order."""
    by_node = {}
    for lab in labels:
        if lab.node is not None:
            by_node.setdefault(lab.node, []).append(lab)
    out = {}
    for node, bodies in created.items():
        labs = by_node.get(node)
        if not labs:
            continue
        for i, bid in enumerate(bodies):
            out[bid] = labs[min(i, len(labs) - 1)]
    return out


def _name_partitions(labels, touches):
    """Give each partition the name created by the latest history call on it.

    A call touching several partitions (e.g. a move-copy) names the ones it
    created, in creation order; partitions it merely modified keep looking
    at earlier calls.
    """
    first = {pid: min(t)[1] for pid, t in touches.items() if t}
    created = {}
    for pid, node in sorted(first.items()):
        created.setdefault(node, []).append(pid)
    names_at = {}
    for label in labels:
        if label.node is not None:
            names_at.setdefault(label.node, []).append(label)
    out = {}
    for pid, t in touches.items():
        for _pos, node in sorted(t, reverse=True):
            labs = names_at.get(node)
            if not labs:
                continue
            made = created.get(node, [])
            if pid in made:
                out[pid] = labs[min(made.index(pid), len(labs) - 1)]
                break
            if not made and len(labs) == 1:
                out[pid] = labs[0]
                break
    return out


def _is_feature_node(v):
    return isinstance(v, list) and len(v) == 5 and isinstance(v[3], str)
