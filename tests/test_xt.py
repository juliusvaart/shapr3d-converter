"""Parasolid XT reader tests against Shapr3D's own exports. Run with:

    /Applications/FreeCAD.app/Contents/Resources/bin/freecadcmd -c \
        "exec(open('tests/test_xt.py').read())"

- every text .x_t export (testfiles/<model>_XT.zip) must build into the same
  solid as the STEP export of the same name (testfiles/<model>_STEP.zip);
- every binary Parasolid blob stored in the .shapr files must parse.
"""

import glob
import os
import shutil
import sqlite3
import sys
import tempfile
import unittest
import zipfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(
    __file__ if "__file__" in globals() else "tests/test_xt.py")))
sys.path.insert(0, os.path.join(ROOT, "Shapr3DImport"))
sys.path.insert(0, os.path.join(ROOT, "tests"))

import Part  # noqa: E402

from metrics import files, iou, read_step, unzip  # noqa: E402
from shapr import xt, xt_step  # noqa: E402

TESTFILES = os.path.join(ROOT, "testfiles")


class XTTest(unittest.TestCase):
    def test_text_exports_match_step(self):
        checked, bad = 0, []
        for xt_zip in sorted(glob.glob(os.path.join(TESTFILES, "*_XT.zip"))):
            model = os.path.basename(xt_zip)[:-len("_XT.zip")]
            step_zip = os.path.join(TESTFILES, model + "_STEP.zip")
            if not os.path.exists(step_zip):
                continue
            tmp = tempfile.mkdtemp()
            try:
                xt_dir = unzip(xt_zip, os.path.join(tmp, "xt"))
                steps = {os.path.splitext(os.path.basename(f))[0]: f
                         for f in files(unzip(step_zip, os.path.join(tmp, "step")), "*.st*p")}
                for f in files(xt_dir, "*.x_t"):
                    name = os.path.splitext(os.path.basename(f))[0]
                    if name not in steps:
                        continue
                    with open(f, "rb") as fh:
                        shapes, _w = xt_step.read_bodies(xt.parse(fh.read()))
                    ref = read_step(steps[name])
                    got = [s for _b, sh in shapes for s in sh.Solids]
                    score = _score(got, ref.Solids)
                    checked += 1
                    if score < 0.999:
                        bad.append((model, name, round(score, 4)))
            finally:
                shutil.rmtree(tmp, ignore_errors=True)
        self.assertGreater(checked, 0, "no XT/STEP pairs found in testfiles/")
        self.assertEqual(bad, [], "%d of %d bodies differ" % (len(bad), checked))
        print("XT bodies matching STEP: %d / %d" % (checked - len(bad), checked))

    def test_stored_blobs_parse(self):
        count = 0
        for path in sorted(glob.glob(os.path.join(TESTFILES, "*.shapr"))):
            tmp = tempfile.mkdtemp()
            try:
                with zipfile.ZipFile(path) as z:
                    z.extract("workspace", tmp)
                db = sqlite3.connect(os.path.join(tmp, "workspace"))
                try:
                    for sql in ("select BodyData from HistoryImportedPrototypes",
                                "select Block from BodyRevisionBlocks"):
                        for (blob,) in db.execute(sql):
                            data = xt.parse(bytes(blob))
                            self.assertTrue(data.of_type("BODY"), path)
                            count += 1
                finally:
                    db.close()
            finally:
                shutil.rmtree(tmp, ignore_errors=True)
        self.assertGreater(count, 0)


def _score(got, ref):
    """Volume-weighted IoU, solids matched by position (multi-solid files)."""
    if not got or len(got) != len(ref):
        return 0.0
    if len(ref) == 1:
        return iou(got[0], ref[0])
    total = sum(r.Volume for r in ref)
    score, used = 0.0, set()
    for r in ref:
        i = min((i for i in range(len(got)) if i not in used),
                key=lambda i: (got[i].CenterOfMass - r.CenterOfMass).Length)
        used.add(i)
        score += iou(got[i], r) * r.Volume
    return score / total


suite = unittest.TestLoader().loadTestsFromTestCase(XTTest)
result = unittest.TextTestRunner(verbosity=2).run(suite)
print("TESTS", "OK" if result.wasSuccessful() else "FAILED")
