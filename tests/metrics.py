"""Shape comparison helpers shared by the test scripts (FreeCAD required)."""

import glob
import os
import random
import zipfile

import FreeCAD as App
import Part


def iou(a, b, samples=600):
    """Volume intersection over union of two solids.

    Booleans on coincident geometry are fragile, so a point-sampling estimate
    is used when the boolean result looks wrong.
    """
    va, vb = abs(a.Volume), b.Volume
    try:
        c = a.common(b).Volume
        result = c / (va + vb - c)
        if result > 0.5 or abs(va - vb) > 0.05 * vb:
            return result
    except Exception:
        pass
    box = a.BoundBox
    box.add(b.BoundBox)
    rnd = random.Random(1)
    both = either = 0
    for _ in range(samples):
        p = App.Vector(rnd.uniform(box.XMin, box.XMax), rnd.uniform(box.YMin, box.YMax),
                       rnd.uniform(box.ZMin, box.ZMax))
        ia, ib = a.isInside(p, 1e-6, True), b.isInside(p, 1e-6, True)
        both += ia and ib
        either += ia or ib
    return both / either if either else 0.0


def match_solids(got, ref, threshold=0.999):
    """Number of reference solids matched one-to-one by a got solid."""
    used, matched = set(), 0
    for r in ref:
        cands = [i for i, g in enumerate(got) if i not in used
                 and g.BoundBox.intersect(r.BoundBox)
                 and abs(abs(g.Volume) - r.Volume) < 0.02 * r.Volume + 1e-6]
        best = max(((iou(got[i], r), i) for i in cands), default=(0.0, None))
        if best[0] >= threshold:
            matched += 1
            used.add(best[1])
    return matched


def unzip(zip_path, dest):
    with zipfile.ZipFile(zip_path) as z:
        z.extractall(dest)
    return dest


def read_step(path):
    shape = Part.Shape()
    shape.read(path)
    return shape


def files(folder, pattern):
    return sorted(glob.glob(os.path.join(folder, "**", pattern), recursive=True))
