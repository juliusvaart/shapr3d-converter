"""Resolve Shapr3D topological names (HistoryNames graph).

Observed name types:
  0/1/18  created by a history feature: callKey{nodeID} / callKeyPrefix{nodeID}
  2       sketch curve {"curveID"}
  3       sketch region {"fins":[...], "finsOfInnerLoops":[[...]]}
  4       region edge {"filling", "fin"}
  5/6     derived body / face {"originals":[...]}
  7       body edge {"neighborFaceOriginals0/1"}
  8       body split piece {"weakName", "faceNames"}
  19      wrapper {"wrappedName", "kind"}
  21      projected topology in a sketch
"""

# Keys whose integer values are not name IDs.
_NON_NAME_KEYS = {"curveID", "sketchPlaneID", "standalonePointID", "kind", "qualifier",
                  "nodeID", "entropy", "entropyPrefix", "descID", "xIndex", "yIndex",
                  "pointKind", "splineInnerPointIndex", "patternDescriptorID", "sense",
                  "startParam", "endParam"}


class NameResolver:
    def __init__(self, names):
        self.names = names
        self._origin_cache = {}
        self._by_node = None

    def unwrap(self, nid):
        seen = set()
        while nid in self.names and self.names[nid][0] == 19 and nid not in seen:
            seen.add(nid)
            nid = self.names[nid][1]["wrappedName"]
        return nid

    def region(self, nid):
        """Sketch region -> (curveIDs, senses, body_edge_names, (wires, edges)) or None.

        senses[i] is 1 if the region lies left of curve i's direction, 0 if
        right, -1 if unknown/mixed.
        """
        t, d = self.names.get(self.unwrap(nid), (None, None))
        if t != 3:
            return None
        curves, senses, body_edges = [], [], []
        fins = list(d.get("fins", []))
        inner = d.get("finsOfInnerLoops", [])
        for loop in inner:
            fins.extend(loop)
        for fin in fins:
            for e in fin.get("originalEdgesWithSense", []):
                en = e["originalEdgeOrCurve"]
                et, ed = self.names.get(self.unwrap(en), (None, None))
                if et == 2:
                    sense = 1 if e.get("sense", True) else 0
                    if ed["curveID"] not in curves:
                        curves.append(ed["curveID"])
                        senses.append(sense)
                    else:
                        i = curves.index(ed["curveID"])
                        if senses[i] != sense:
                            senses[i] = -1
                else:
                    body_edges.append(en)
        return curves, senses, body_edges, (1 + len(inner), len(fins))

    def curve(self, nid):
        """Name of a sketch curve -> curveID, else None."""
        t, d = self.names.get(self.unwrap(nid), (None, None))
        return d["curveID"] if t == 2 else None

    def origins(self, nid):
        """Set of history node IDs that (transitively) created this name."""
        if nid in self._origin_cache:
            return self._origin_cache[nid]
        self._origin_cache[nid] = frozenset()  # cycle guard
        out = set()
        t, d = self.names.get(nid, (None, None))
        if d is not None:
            for key in ("callKey", "callKeyPrefix"):
                if key in d and "nodeID" in d[key]:
                    out.add(d[key]["nodeID"])
            for ref in self._refs(d):
                out |= self.origins(ref)
        res = frozenset(out)
        self._origin_cache[nid] = res
        return res

    def creator(self, nid):
        """History node that produced the body (or topology) this name refers to."""
        return self.creator_sources(nid)[0]

    def creator_sources(self, nid):
        """(creating node, names of that node's input topology) for a name."""
        seen = set()
        while nid in self.names and nid not in seen:
            seen.add(nid)
            t, d = self.names[nid]
            step = _creator_step(t, d)
            if isinstance(step, tuple):
                return step[1], d.get("sourceTopologyNames", [])
            if step is None:
                break
            nid = step
        return None, []

    def node_names(self, node):
        """Face names created by a history node: [(name, qualifier, first source)].

        Qualifiers seen: 0 side face (source: region edge), 1 end cap,
        2 start cap (on the sketch plane; source: region), 4 whole body.
        """
        if self._by_node is None:
            self._by_node = {}
            for nid, (t, d) in self.names.items():
                ck = d.get("callKey") if t == 0 and isinstance(d, dict) else None
                if isinstance(ck, dict) and "nodeID" in ck:
                    src = d.get("sourceTopologyNames") or [None]
                    self._by_node.setdefault(ck["nodeID"], []).append(
                        (nid, d.get("qualifier"), src[0]))
        return self._by_node.get(node, [])

    def face_keys(self, nid, renaming_nodes=()):
        """Face name -> [(node, qualifier, source name or None)].

        A face is keyed by the feature that created it; source None means
        "any face of that node with that qualifier" (callKeyPrefix names).
        Names created by nodes in renaming_nodes (body copies) stand for
        their source face.
        """
        seen, out, stack = set(), [], [nid]
        while stack:
            n = self.unwrap(stack.pop())
            if n in seen or n not in self.names:
                continue
            seen.add(n)
            t, d = self.names[n]
            if t == 6:
                stack.extend(reversed(d.get("originals", [])))
                continue
            ck = d.get("callKey") or d.get("callKeyPrefix")
            if t not in (0, 1) or not isinstance(ck, dict) or "nodeID" not in ck:
                continue
            src = d.get("sourceTopologyNames") or []
            if ck["nodeID"] in renaming_nodes and src:
                stack.extend(reversed(src))
                continue
            key = (ck["nodeID"], d.get("qualifier"), self.unwrap(src[0]) if t == 0 and src else None)
            if key not in out:
                out.append(key)
        return out

    def edge_faces(self, nid, renaming_nodes=()):
        """Body edge name (type 7) -> (face names on one side, on the other)."""
        seen = set()
        while nid not in seen:
            seen.add(nid)
            t, d = self.names.get(self.unwrap(nid), (None, None))
            if t == 7:
                a, b = d.get("neighborFaceOriginals0", []), d.get("neighborFaceOriginals1", [])
                if a and b:
                    return a, b
                nid = _first(d, "originals")
            elif t == 0 and d.get("callKey", {}).get("nodeID") in renaming_nodes:
                nid = _first(d, "sourceTopologyNames")
            else:
                break
        return None

    def fin_body_edge(self, nid):
        """Region edge name (type 4) -> its first body edge name, else None."""
        t, d = self.names.get(self.unwrap(nid), (None, None))
        if t != 4:
            return None
        for e in d.get("fin", {}).get("originalEdgesWithSense", []):
            en = e["originalEdgeOrCurve"]
            if self.names.get(self.unwrap(en), (None,))[0] == 7:
                return en
        return None

    def fin_curve(self, nid):
        """Region edge name (type 4) -> its first sketch curveID, else None."""
        t, d = self.names.get(self.unwrap(nid), (None, None))
        if t != 4:
            return None
        for e in d.get("fin", {}).get("originalEdgesWithSense", []):
            cid = self.curve(e["originalEdgeOrCurve"])
            if cid is not None:
                return cid
        return None

    def is_body_name(self, nid):
        """True if the name denotes a whole body (not a face/edge)."""
        t, d = self.names.get(self.unwrap(nid), (None, None))
        if t in (0, 1):  # feature output: qualifier 4 is the whole body
            return d.get("qualifier", 4) == 4
        return t in (5, 8)

    def _refs(self, d):
        stack = [d]
        while stack:
            x = stack.pop()
            if isinstance(x, dict):
                for k, v in x.items():
                    if k in _NON_NAME_KEYS:
                        continue
                    if isinstance(v, int) and not isinstance(v, bool):
                        if v in self.names:
                            yield v
                    else:
                        stack.append(v)
            elif isinstance(x, list):
                for v in x:
                    if isinstance(v, int) and not isinstance(v, bool):
                        if v in self.names:
                            yield v
                    else:
                        stack.append(v)


def _first(d, key):
    v = d.get(key)
    return v[0] if isinstance(v, list) and v else None


def _creator_step(t, d):
    """Next name along a body/face/edge name's primary lineage, or ('node', id)."""
    for key in ("callKey", "callKeyPrefix"):
        if isinstance(d.get(key), dict) and "nodeID" in d[key]:
            return ("node", d[key]["nodeID"])
    if t == 19:
        return d.get("wrappedName")
    if t == 8:
        return d.get("weakName")
    if t == 10:
        return d.get("parentBodyName")
    if t == 7:
        return _first(d, "originals") or _first(d, "neighborFaceOriginals0")
    if t == 13:
        return _first(d, "originals") or _first(d, "neighborFaces")
    return _first(d, "originals")
