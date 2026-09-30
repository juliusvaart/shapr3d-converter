"""Convert Shapr3D .shapr files to FreeCAD .FCStd.

Usage:
    freecadcmd shapr2fcstd.py --pass input.shapr [output.FCStd]

`--pass` stops freecadcmd from opening the arguments itself.
"""

import os
import sys
import traceback

HERE = os.path.dirname(os.path.abspath(sys.argv[1] if len(sys.argv) > 1 else "."))
sys.path.insert(0, os.path.join(HERE, "Shapr3DImport"))

import FreeCAD as App  # noqa: E402

from shapr.convert import convert  # noqa: E402


def main(argv):
    args = argv[argv.index("--pass") + 1:] if "--pass" in argv else argv[2:]
    if not args:
        App.Console.PrintMessage(__doc__)
        return 2
    src = os.path.abspath(args[0])
    dst = os.path.abspath(args[1]) if len(args) > 1 else os.path.splitext(src)[0] + ".FCStd"
    doc = App.newDocument("Shapr3DImport")
    try:
        convert(src, doc)
        doc.saveAs(dst)
    finally:
        App.closeDocument(doc.Name)
    App.Console.PrintMessage("saved %s\n" % dst)
    return 0


try:
    main(sys.argv)
except Exception:
    # freecadcmd hides script exceptions; print them explicitly.
    App.Console.PrintError(traceback.format_exc())
