"""Conversion tests. Run with FreeCAD's interpreter:

    /Applications/FreeCAD.app/Contents/Resources/bin/freecadcmd -c \
        "exec(open('tests/test_convert.py').read())"

Each sample is converted into its own document, checked and closed before
the next, so memory stays bounded on the large samples.
"""

import glob
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(
    __file__ if "__file__" in globals() else "tests/test_convert.py")))
sys.path.insert(0, os.path.join(ROOT, "Shapr3DImport"))

import FreeCAD as App  # noqa: E402
import Part  # noqa: E402

from shapr.convert import convert  # noqa: E402
from shapr.reader import ShaprFile  # noqa: E402
from shapr.sketches import M2MM  # noqa: E402

SAMPLES = sorted(glob.glob(os.path.join(ROOT, "testfiles", "*.shapr")))


def converted(path):
    doc = App.newDocument()
    convert(path, doc)
    return doc


class ConvertTest(unittest.TestCase):
    def test_samples_present(self):
        self.assertTrue(SAMPLES, "no .shapr samples in testfiles/")

    def test_each_sample(self):
        for path in SAMPLES:
            with self.subTest(sample=os.path.basename(path)):
                src = ShaprFile(path)
                doc = converted(path)
                try:
                    self._check_sketches(doc, src, path)
                    self._check_bodies(doc, src, path)
                    bad = [o.Label for o in doc.Objects if "Invalid" in o.State or "Error" in o.State]
                    self.assertEqual(bad, [], "errored objects")
                finally:
                    App.closeDocument(doc.Name)

    def _check_sketches(self, doc, src, path):
        sketches = [o for o in doc.Objects if o.TypeId == "Sketcher::SketchObject"]
        self.assertEqual(len(sketches), len(src.sketches))
        for obj, sk in zip(sketches, src.sketches.values()):
            self.assertEqual(obj.solve(), 0, "%s does not solve" % obj.Label)
            curves = [i for i, g in enumerate(obj.Geometry) if not isinstance(g, Part.Point)]
            self.assertEqual(len(curves), len(sk.curves), obj.Label)
            for gi, c in zip(curves, sk.curves):
                for pos, p in ((1, c.start), (2, c.end), (3, c.center)):
                    if p is not None:
                        v = obj.getPoint(gi, pos)
                        self.assertAlmostEqual(v.x, p.x * M2MM, delta=1e-6)
                        self.assertAlmostEqual(v.y, p.y * M2MM, delta=1e-6)

    def _check_bodies(self, doc, src, path):
        group = doc.getObject("Bodies")
        bodies = [o for o in group.OutListRecursive if o.TypeId == "Part::Feature"]
        self.assertEqual(len(bodies), len(src.bodies), "exact body count")
        for o in bodies:
            s = o.Shape
            self.assertTrue(not s.isNull() and s.isValid(), o.Label)
            if s.Solids:
                self.assertGreater(s.Volume, 0, o.Label)
            else:  # sheet body
                self.assertGreater(s.Area, 0, o.Label)
        self.assertEqual(sum(not o.Visibility for o in bodies), sum(b.hidden for b in src.bodies))

    def test_sketch_edit_updates_extrusion(self):
        """Changing a sketch dimension must change the rebuilt solid on it."""
        for path in SAMPLES:
            doc = converted(path)
            try:
                for ext in doc.Objects:
                    if ext.TypeId != "Part::Extrusion" or not ext.Base.Regions:
                        continue
                    sk = ext.Base.Sketch
                    idx = next((i for i, c in enumerate(sk.Constraints)
                                if c.Type in ("Radius", "Distance") and c.Driving
                                and c.First in ext.Base.Regions), None)
                    if idx is None:
                        continue
                    before = ext.Shape.Volume
                    value = sk.Constraints[idx].Value
                    sk.setDatum(idx, App.Units.Quantity(value * 1.1, App.Units.Length))
                    doc.recompute()
                    self.assertNotAlmostEqual(before, ext.Shape.Volume, places=3, msg=ext.Label)
                    return
            finally:
                App.closeDocument(doc.Name)
        self.fail("no extrusion with a dimensioned profile found")


suite = unittest.TestLoader().loadTestsFromTestCase(ConvertTest)
result = unittest.TextTestRunner(verbosity=2).run(suite)
print("TESTS", "OK" if result.wasSuccessful() else "FAILED")
