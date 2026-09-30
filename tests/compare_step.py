"""Compare converted .shapr models against Shapr3D STEP exports.

For every testfiles/<name>.shapr with a testfiles/<name>_STEP.zip (STEP files
per body or folder), convert the model and match every STEP solid to a
converted body solid by volume overlap (intersection over union).

Run:
    /Applications/FreeCAD.app/Contents/Resources/bin/freecadcmd -c \
        "exec(open('tests/compare_step.py').read())"
Set SHAPR_ONLY=<substring> to compare a single file.
"""

import glob
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(
    __file__ if "__file__" in globals() else "tests/compare_step.py")))
sys.path.insert(0, os.path.join(ROOT, "Shapr3DImport"))
sys.path.insert(0, os.path.join(ROOT, "tests"))

import FreeCAD as App  # noqa: E402

from metrics import files, match_solids, read_step, unzip  # noqa: E402
from shapr.convert import convert  # noqa: E402


def converted_solids(doc):
    group = doc.getObject("Bodies")
    objs = [o for o in (group.OutListRecursive if group else []) if o.TypeId == "Part::Feature"]
    return [s for o in objs for s in o.Shape.Solids]


def compare(shapr_path, zip_path):
    with tempfile.TemporaryDirectory() as tmp:
        ref = [s for f in files(unzip(zip_path, tmp), "*.st*p") for s in read_step(f).Solids]
    doc = App.newDocument()
    try:
        convert(shapr_path, doc)
        got = converted_solids(doc)
        return len(ref), len(got), match_solids(got, ref)
    finally:
        App.closeDocument(doc.Name)


def main():
    only = os.environ.get("SHAPR_ONLY", "")
    total = total_ok = 0
    for zip_path in sorted(glob.glob(os.path.join(ROOT, "testfiles", "*_STEP.zip"))):
        name = os.path.basename(zip_path)[:-len("_STEP.zip")]
        shapr_path = os.path.join(ROOT, "testfiles", name + ".shapr")
        if only not in name or not os.path.exists(shapr_path):
            continue
        n_ref, n_got, ok = compare(shapr_path, zip_path)
        total += n_ref
        total_ok += ok
        print("RESULT %-22s STEP solids %3d  converted %3d  matched %3d" % (name, n_ref, n_got, ok))
    print("TOTAL matched %d / %d STEP solids" % (total_ok, total))


main()
