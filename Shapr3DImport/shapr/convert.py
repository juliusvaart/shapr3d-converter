"""Convert a .shapr file into an open FreeCAD document.

The document gets three groups:
- Sketches: parametric Sketcher objects, one per Shapr3D sketch.
- Bodies: the final solids, read exactly from the Parasolid data stored in
  the file.
- Parametric rebuild: the modelling history replayed as Part features on
  the sketches (hidden; features it cannot rebuild are skipped and listed
  in the import report).
"""

import os

import FreeCAD as App
import Part

from . import xt, xt_step
from .features import Replayer
from .names import NameResolver
from .reader import ShaprFile
from .report import Report
from .sketches import build_sketch

_IDENTITY = [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]


def convert(path, doc):
    report = Report()
    shapr = ShaprFile(path)
    labels = {}
    for ft in shapr.features:
        if ft.op == "MaterializeSketchPlane" and ft.params and isinstance(ft.params[0], tuple):
            labels.setdefault(ft.params[0][1], ft.name)

    sketch_group = doc.addObject("App::DocumentObjectGroup", "Sketches")
    builds = {}
    for sid, src in shapr.sketches.items():
        b = build_sketch(doc, src, labels.get(sid) or src.name, report)
        sketch_group.addObject(b.obj)
        builds[sid] = b

    body_group = doc.addObject("App::DocumentObjectGroup", "Bodies")
    n_bodies = _import_bodies(doc, shapr, body_group, report)

    before = {o.Name for o in doc.Objects}
    rebuilt = Replayer(doc, shapr, NameResolver(shapr.names), builds, report).run()
    new = [o for o in doc.Objects if o.Name not in before]
    if new:
        rebuild_group = doc.addObject("App::DocumentObjectGroup", "ParametricRebuild")
        rebuild_group.Label = "Parametric rebuild"
        for o in new:
            if o.TypeId.startswith("Part::") or o.TypeId == "App::Link":
                o.Visibility = False
        for o in rebuilt.values():
            rebuild_group.addObject(o)
        rebuild_group.Visibility = False

    doc.recompute()
    report.info("%d sketches, %d exact bodies, %d rebuilt parametric bodies"
                % (len(builds), n_bodies, len(rebuilt)))
    report.publish(doc, os.path.basename(path))
    doc.recompute()
    return report


def _import_bodies(doc, shapr, group, report):
    """Final solids from the stored Parasolid data -> Part::Feature objects,
    grouped like Shapr3D's browser folders."""
    count = 0
    folders = {(): group}

    def folder(path):
        if path not in folders:
            parent = folder(path[:-1])
            g = doc.addObject("App::DocumentObjectGroup", "Folder")
            g.Label = path[-1]
            parent.addObject(g)
            folders[path] = g
        return folders[path]

    for body in shapr.bodies:
        try:
            shapes, warnings = xt_step.read_bodies(xt.parse(body.xt))
        except Exception as e:
            report.warn("body %s could not be read: %s" % (body.name, e))
            continue
        for w in warnings:
            report.info("body %s: %s" % (body.name, w))
        for _node, shape in shapes:
            if body.matrix and any(abs(a - b) > 1e-12 for a, b in zip(body.matrix, _IDENTITY)):
                shape = shape.transformed(body_matrix(body))
                if not body.column_major:
                    report.info("body %s: import placement applied (layout not verified)" % body.name)
            obj = doc.addObject("Part::Feature", "Body")
            obj.Label = body.name
            obj.Shape = shape
            obj.Visibility = not body.hidden
            folder(tuple(body.folder)).addObject(obj)
            if not shape.isValid():
                report.warn("body %s: shape is not valid" % body.name)
            count += 1
    return count


def body_matrix(body):
    """Body placement (4x4, translation in meters) -> FreeCAD matrix in mm."""
    m = body.matrix
    if body.column_major:
        m = [m[c * 4 + r] for r in range(4) for c in range(4)]
    mat = App.Matrix(*m)
    mat.A14 *= 1000.0
    mat.A24 *= 1000.0
    mat.A34 *= 1000.0
    return mat
