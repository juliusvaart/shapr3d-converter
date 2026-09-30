"""FreeCAD import entry points for Shapr3D .shapr files."""

import os

import FreeCAD as App

from shapr.convert import convert


def open(filename):
    name = os.path.splitext(os.path.basename(filename))[0]
    doc = App.newDocument(_safe(name))
    doc.Label = name
    convert(filename, doc)
    return doc


def insert(filename, docname):
    try:
        doc = App.getDocument(docname)
    except NameError:
        doc = App.newDocument(docname)
    convert(filename, doc)
    return doc


def _safe(name):
    return "".join(c if c.isalnum() else "_" for c in name) or "Shapr3D"
