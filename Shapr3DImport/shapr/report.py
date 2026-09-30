"""Collect import messages; print them and store them in the document."""

import FreeCAD as App


class Report:
    def __init__(self):
        self.lines = []

    def info(self, msg):
        self.lines.append("info: " + msg)

    def warn(self, msg):
        self.lines.append("warning: " + msg)

    def text(self):
        return "\n".join(self.lines)

    def publish(self, doc, source):
        header = "Shapr3D import report for %s\n\n" % source
        obj = doc.addObject("App::TextDocument", "ImportReport")
        obj.Label = "Shapr3D import report"
        obj.Text = header + self.text()
        App.Console.PrintMessage(header + self.text() + "\n")
        return obj
