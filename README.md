# Shapr3D → FreeCAD importer

Opens Shapr3D `.shapr` files in FreeCAD 1.1. The document it creates has three groups:

- **Sketches**: fully parametric Sketcher objects on the original planes.
- **Bodies**: the final solids, read exactly from the Parasolid data stored inside the `.shapr` file, named, hidden and organised into folders as in Shapr3D.
- **Parametric rebuild** (hidden): the modelling history replayed as Part features that stay linked to the sketches. Features it cannot rebuild yet are skipped and listed in the import report.

## Install

```sh
ln -s "$PWD/Shapr3DImport" "$HOME/Library/Application Support/FreeCAD/v1-1/Mod/Shapr3DImport"
```

Restart FreeCAD. After that, **File › Open** and **File › Import** accept `.shapr` files.

Keep the addon installed. FreeCAD only restores Python features (the rebuild's sketch-region profiles, face offsets, face moves and chamfers) from installed addons.

## Command line

```sh
/Applications/FreeCAD.app/Contents/Resources/bin/freecadcmd shapr2fcstd.py --pass input.shapr [output.FCStd]
```

`--pass` stops freecadcmd from also opening the arguments itself.

## What gets converted

### Bodies (exact)

A `.shapr` file is a zip that holds a SQLite `workspace`. Shapr3D stores its solids in that workspace in two places, both as Parasolid neutral binary transmit data:

- `HistoryImportedPrototypes` holds imported bodies.
- `BodyRevisionBlocks` holds one partition per body lineage in history-based models.

`shapr/xt.py` reads that data, and it also reads text `.x_t` files. Node layouts come from the documented base schema (`shapr/xt_schema.py`). The per-file schema edits (copy / delete / insert / append) are then applied on top.

`shapr/xt_step.py` translates each body into an in-memory STEP AP214 B-rep, which OCC's STEP reader then builds. That reader handles pcurves and periodic seams robustly.

The final set of bodies comes from replaying the `PersistedCalls` records at body level (`reader._replay_bodies`):

- which bodies occupy which partition;
- consumed and deleted bodies;
- copy-on-write;
- the lazy move and copy matrices that Shapr3D applies without rewriting partitions.

Names, the hidden flag and browser folders come from `Metadata`, `MetadataAssignments` and `HistoryFolders`.

### Sketches

| Shapr3D | FreeCAD |
|---|---|
| lines, arcs, circles, construction curves, symbol centers | `Sketcher::SketchObject` geometry |
| coincident, distance, point-on-curve, parallel, perpendicular, horizontal/vertical, midpoint, radius, locked points | Sketcher constraints. Each is added only if the source geometry satisfies it (to 1e-6 mm) and FreeCAD accepts it as non-redundant |

### Parametric rebuild

| Shapr3D | FreeCAD |
|---|---|
| Extrude / Revolve profile | `ShaprProfile` (Part::FeaturePython) that rebuilds the region from the live sketch |
| Extrude, Revolve (new / add / cut / intersect) | `Part::Extrusion`, `Part::Revolution`, `Part::MultiFuse`, `Part::Cut`, `Part::MultiCommon` |
| Union, Subtract | `Part::MultiFuse`, `Part::Cut` |
| Move/rotate of bodies (including copies) | `App::Link` with a placement |
| Face Offset (by distance, or to a distance from a reference face) | `ShaprFaceOffset`: the face moves along its normal and its neighbours extend or shrink with it |
| Move/rotate of faces | `ShaprFaceMove`: the face's plane moves and its neighbours follow |
| Chamfer | `ShaprChamfer`: a wedge cut, which also allows a chamfer as wide as the face |
| Fillet | `Part::Fillet` |
| Shell | `Part::Thickness` |

Face tools find their faces by the Shapr3D topological names, which name a face after the feature that created it. The rebuild tracks every such face's plane through later moves and offsets. Each face tool stores a geometric hint (point and normal) per face, so it finds its face again after a sketch edit renumbers the body's faces.

The rebuild skips Align, Split, Mirror and Sweep, and faces it cannot trace back to their creating feature.

Every import adds a **Shapr3D import report** text object to the document.

## Limitations

- Rolling-ball blend surfaces are rebuilt as a tube of the blend radius around the spine curve. This is exact for the blends in the samples, but approximate in general. Cliff-edge blends and foreign (PE) geometry are not supported.
- Intersection curves and SP-curves (the curves of tolerant edges) are sampled and interpolated. In the samples, the resulting volume error is at most 0.01 %.
- The matrix layout for imported bodies is unverified: every sample has an identity placement. Such placements are flagged in the report.
- Pattern and offset sketch constraints are not converted.
- The file layout was reverse-engineered from these samples. Other Shapr3D versions may store data this code does not expect.

## Tests

Run each test with FreeCAD's interpreter: `freecadcmd -c "exec(open('<script>').read())"`.

| Script | Checks | Time |
|---|---|---|
| `tests/test_convert.py` | Each sample converts. Sketches match the source and solve. Body counts and hidden flags are right. No object has errors. A sketch edit changes the rebuilt solid | ~50 s |
| `tests/test_xt.py` | Every X_T export matches its STEP. Every stored Parasolid blob parses | ~30 s |
| `tests/compare_step.py` | Converted bodies against the STEP exports, solid by solid | ~85 s |
| `tests/compare_rebuild.py` | Parametric rebuild against the exact bodies (reports a score) | ~3 min |

## Layout

- `Shapr3DImport/Init.py`, `importShapr.py`: FreeCAD addon entry points
- `shapr/reader.py`: zip and SQLite reading, history decoding, body replay, names and folders
- `shapr/xt.py`, `xt_schema.py`: Parasolid XT reader
- `shapr/xt_step.py`, `fit.py`: XT → STEP B-rep, with curve and surface fitting
- `shapr/names.py`: Shapr3D topological names
- `shapr/sketches.py`: sketches and constraints
- `shapr/profile.py`, `faceops.py`, `features.py`: the parametric rebuild
- `shapr/convert.py`: builds the document

## Credits

`xt_schema.py` is generated from the *Parasolid XT Format Reference* (Siemens, V35). It corrects the reference's errata that the sample files exposed: a missing FACE field type, the HALFEDGE `loop` field and the ELLIPSE field order.
