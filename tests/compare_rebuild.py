"""Compare the parametric rebuild against the exact bodies of each model.

For every testfiles/<name>.shapr with modelling history, convert it and match
each exact body solid (Bodies group) to a rebuilt solid (Parametric rebuild
group) by volume overlap. The rebuild is approximate, so this reports a
score rather than asserting.

Run:
    /Applications/FreeCAD.app/Contents/Resources/bin/freecadcmd -c \
        "exec(open('tests/compare_rebuild.py').read())"
Set SHAPR_ONLY=<substring> to compare a single file.
"""

import glob
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(
    __file__ if "__file__" in globals() else "tests/compare_rebuild.py")))
sys.path.insert(0, os.path.join(ROOT, "Shapr3DImport"))
sys.path.insert(0, os.path.join(ROOT, "tests"))

import FreeCAD as App  # noqa: E402
import Part  # noqa: E402

from metrics import match_solids  # noqa: E402
from shapr.convert import convert  # noqa: E402


def solids(group, types):
    objs = [o for o in (group.OutListRecursive if group else []) if o.TypeId in types]
    return objs, [s for o in objs for s in Part.getShape(o).Solids]


def main():
    only = os.environ.get("SHAPR_ONLY", "")
    total = total_ok = 0
    for path in sorted(glob.glob(os.path.join(ROOT, "testfiles", "*.shapr"))):
        name = os.path.basename(path)[:-len(".shapr")]
        if only not in name:
            continue
        doc = App.newDocument()
        try:
            convert(path, doc)
            group = doc.getObject("ParametricRebuild")
            if group is None:
                continue
            _objs, exact = solids(doc.getObject("Bodies"), ("Part::Feature",))
            rebuilt = [s for o in group.Group for s in Part.getShape(o).Solids]
            ok = match_solids(rebuilt, exact)
            total += len(exact)
            total_ok += ok
            print("RESULT %-22s exact solids %3d  rebuilt %3d  matched %3d"
                  % (name, len(exact), len(rebuilt), ok))
        finally:
            App.closeDocument(doc.Name)
    print("TOTAL rebuilt matches %d / %d exact solids" % (total_ok, total))


main()
