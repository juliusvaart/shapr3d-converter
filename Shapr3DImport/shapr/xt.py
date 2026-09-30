"""Parasolid XT transmit data reader (text and neutral binary).

Follows the Parasolid XT Format Reference: the data is a sequence of nodes,
each laid out as the schema describes. Files written by newer Parasolid
versions embed, for the first node of each type, the differences between
their schema and base schema SCH_13006; those diffs are applied on top of
the documented layouts in xt_schema.NODES.

Pure Python, no FreeCAD dependency.
"""

import struct

from .xt_schema import NODES

NULL_INT = -32764
NULL_DOUBLE = -3.14158e13
TERMINATOR = 1


class XTError(ValueError):
    pass


class Node:
    __slots__ = ("type", "name", "index", "fields")

    def __init__(self, type_, name, index, fields):
        self.type = type_
        self.name = name
        self.index = index
        self.fields = fields

    def __getitem__(self, key):
        return self.fields[key]

    def get(self, key, default=None):
        return self.fields.get(key, default)

    def __repr__(self):
        return "<%s #%d>" % (self.name, self.index)


# ---- low level readers ----

class _Binary:
    """Neutral binary: big-endian, IEEE doubles, 2-byte pointer indices."""

    def __init__(self, data, pos):
        self.d = data
        self.p = pos

    def _take(self, fmt, size):
        if self.p + size > len(self.d):
            raise XTError("unexpected end of data at %d" % self.p)
        v = struct.unpack_from(fmt, self.d, self.p)[0]
        self.p += size
        return v

    def byte(self):
        return self._take(">B", 1)

    def char(self):
        return chr(self.byte())

    def logical(self):
        return bool(self.byte())

    def short(self):
        return self._take(">h", 2)

    def int(self):
        return self._take(">i", 4)

    def double(self):
        return self._take(">d", 8)

    def ptr(self):
        r = self.short()
        q = 0
        if r < 0:
            q = self.short()
            r = -r
        return q * 32767 + r - 1

    posint = ptr

    def count(self):
        return self.byte()

    def sstring(self):
        n = self.byte()
        s = self.d[self.p:self.p + n].decode("latin-1")
        self.p += n
        return s

    def chars(self, n):
        s = self.d[self.p:self.p + n].decode("latin-1")
        self.p += n
        return s

    def nodetype(self):
        return self.short()

    def peek_op(self):
        return self.d[self.p:self.p + 1] in (b"C", b"D", b"I", b"A", b"Z")

    def varlen(self):
        return self.int()


_TEXT_ESCAPES = {"n": "\n", "-": " ", ";": ";", "^": "^"}


class _Text:
    """Text XT: numbers end with a space; chars and logicals do not."""

    def __init__(self, data, pos):
        self.d = data
        self.p = pos

    def _token(self):
        d, p = self.d, self.p
        end = d.find(" ", p)
        if end < 0:
            raise XTError("unexpected end of text data")
        self.p = end + 1
        return d[p:end]

    def _num(self, conv):
        if self.d[self.p] == "?":  # null value, no separating space
            self.p += 1
            return None
        return conv(self._token())

    def byte(self):
        return self._num(int)

    def char(self):
        c = self.d[self.p]
        self.p += 1
        if c == "^":
            c = _TEXT_ESCAPES.get(self.d[self.p], self.d[self.p])
            self.p += 1
        elif c == "\\":
            e = self.d[self.p]
            self.p += 1
            c = {"0": "\0", "n": "\r", "r": "\n", "\\": "\\"}.get(e, e)
            if e == "9":
                c = " " * 9
        return c

    def logical(self):
        c = self.d[self.p]
        self.p += 1
        return c == "T"

    short = int = ptr = posint = count = byte

    def double(self):
        return self._num(float)

    def sstring(self):
        n = self.byte()
        return self.chars(n)

    def chars(self, n):
        out = []
        size = 0
        while size < n:
            c = self.char()
            out.append(c)
            size += len(c)
        return "".join(out)

    def nodetype(self):
        return self.byte()

    def peek_op(self):
        return self.d[self.p] in "CDIAZ"

    def varlen(self):
        return self.byte()


# ---- schema handling ----

def _read_field_spec(r):
    name = r.sstring()
    ptr_class = r.short()
    n_elts = r.posint()
    ftype = "p" if ptr_class else r.sstring()
    if n_elts == 1:
        r.logical()  # xmt_code
    return (name, ftype, n_elts)


def _read_schema_diff(r, ntype):
    """Layout of `ntype` in this file, from its embedded schema diff."""
    n = r.count()
    base = NODES.get(ntype)
    if base is not None and n == 255:
        return base[0], list(base[1])
    if base is None or not r.peek_op():
        # Node type unknown to the base schema: full description follows.
        name = r.sstring()
        r.sstring()  # description
        return name, [_read_field_spec(r) for _ in range(n)]
    name, doc_fields = base
    ops = []
    while True:
        op = r.char()
        if op == "Z":
            break
        if op in "IA":
            ops.append((op, _read_field_spec(r)))
        elif op in "CD":
            ops.append((op, None))
        else:
            raise XTError("bad schema edit %r for node type %d" % (op, ntype))
    inserted = {spec[0] for op, spec in ops if spec}
    kept = [f for f in doc_fields if f[0] not in inserted]
    # A 'D' removes a base field. Usually that field is gone from the
    # documented schema too, but if the documentation still lists fields the
    # file's schema dropped, the first deletions consume those.
    n_copy = sum(1 for op, _ in ops if op == "C")
    doc_deletes = len(kept) - n_copy
    layout, k = [], 0
    for op, spec in ops:
        if op == "C":
            if k >= len(kept):
                raise XTError("schema diff for %s copies more fields than known" % name)
            layout.append(kept[k])
            k += 1
        elif op == "D" and doc_deletes > 0:
            k += 1
            doc_deletes -= 1
        elif op in "IA":
            layout.append(spec)
    if k != len(kept) or len(layout) != n:
        raise XTError("schema diff for %s does not match documented layout (%d/%d, %d/%d)"
                      % (name, k, len(kept), len(layout), n))
    return name, layout


# ---- node data ----

def _value(r, code):
    if code == "d":
        v = r.int()
        return None if v == NULL_INT else v
    if code == "p":
        return r.ptr()
    if code == "f":
        v = r.double()
        return None if v is None or v == NULL_DOUBLE else v
    if code in ("v", "h"):
        if isinstance(r, _Text) and r.d[r.p] == "?":
            r.p += 1  # a null vector is a single '?'
            return None
        x = (r.double(), r.double(), r.double())
        return None if x[0] == NULL_DOUBLE else x
    if code == "i":
        return (r.double(), r.double())
    if code == "b":
        return tuple(r.double() for _ in range(6))
    if code == "c":
        return r.char()
    if code == "l":
        return r.logical()
    if code in ("n", "w"):
        v = r.short()
        return None if v == NULL_INT else v
    if code == "u":
        return r.byte()
    raise XTError("unknown field type %r" % code)


def _values(r, code, n):
    if code == "c":
        return r.chars(n)
    return [_value(r, code) for _ in range(n)]


class XTData:
    """Parsed XT data: nodes by index; root is node 1."""

    def __init__(self, nodes, version, schema):
        self.nodes = nodes
        self.version = version
        self.schema = schema

    @property
    def root(self):
        return self.nodes[1]

    def __getitem__(self, index):
        return self.nodes.get(index) if index else None

    def of_type(self, name):
        return [n for n in self.nodes.values() if n.name == name]


def parse(data):
    """Parse XT bytes (neutral binary 'PS\\0\\0' or text transmit file)."""
    if data[:4] == b"PS\x00\x00":
        r = _Binary(data, 4)
        version = r.chars(r.short())
        schema = r.chars(r.int())
    else:
        text = data.decode("latin-1")
        start = text.find("**END_OF_HEADER")
        if start < 0:
            raise XTError("not an XT file: no header")
        body = text[text.index("\n", start) + 1:].replace("\r", "").replace("\n", "")
        if not body.startswith("T"):
            raise XTError("unsupported XT flavour %r" % body[:1])
        r = _Text(body, 1)
        version = r.chars(r.byte())
        schema = r.chars(r.byte())
    embedded = schema.count("_") >= 3
    if embedded:
        r.short()  # maximum number of node types
    r.int()  # user field size (only 0 supported below)

    layouts, nodes = {}, {}
    while True:
        ntype = r.nodetype()
        if ntype == TERMINATOR:
            break
        if ntype not in layouts:
            if embedded:
                layouts[ntype] = _read_schema_diff(r, ntype)
            elif ntype in NODES:
                layouts[ntype] = NODES[ntype]
            else:
                raise XTError("unknown node type %d" % ntype)
        name, fields = layouts[ntype]
        variable = any(f[2] == 1 for f in fields)
        n_var = r.varlen() if variable else 0
        index = r.ptr()
        values = {}
        for fname, code, n in fields:
            if n == 0:
                values[fname] = _value(r, code)
            else:
                values[fname] = _values(r, code, n_var if n == 1 else n)
        nodes[index] = Node(ntype, name, index, values)
    return XTData(nodes, version, schema)
