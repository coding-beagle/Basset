# Basset architecture

Basset is a parametric, history-based solid modeller in Rust. The long-term target is
feature parity with Fusion 360; the MVP deliberately ships few tools, each finished.

## Crate map

```
basset-math      f64 vectors (glam), Frame/Plane/Ray, tolerances, TriMesh interchange type
basset-sketch    2D sketch entities, constraints, solver, shapes, text, profile extraction
basset-kernel    solid modelling: Solid/Face topology, CSG, extrude/revolve/sweep/loft,
                  fillet/chamfer/combine/transform, tessellation, picking, mass properties
basset-core      Document, Components, Planes/Axes/Sketches/Bodies, Timeline + regeneration,
                  document save/open (.bass JSON)
basset-io        STL / 3MF export
basset-viewport  wgpu renderer: camera, mesh/line/point/triangle batches, grid, selection highlight
basset-app       winit + egui desktop application (Linux first)
```

Dependency direction is strictly downward: `math ← sketch, kernel ← core ← io, viewport ← app`.
`sketch` and `kernel` do not know about each other; `core` converts sketch profiles into
kernel profiles. This keeps both testable in isolation and lets the kernel be replaced.

## Conventions

* Units: millimetres and radians everywhere. UI converts.
* All geometry maths in `f64`. `f32` only inside GPU upload code.
* Comments explain *why*. Names explain *what*.
* Every public operation has unit tests; kernel tests assert volumes and bounding boxes
  because those catch winding, orientation, and boolean errors cheaply.
* Errors are typed (`thiserror`), never panics, for anything driven by user data.
* No `unwrap()` on user-derived data outside tests.

## Identity and the topological naming problem

Every entity is produced by a timeline feature and is identified by the `FeatureId` of the
feature that produced it. Faces are identified by a `FaceKey { op: OpId, role }` where
`OpId` is the feature id plus a sub-index (one per profile a feature extrudes) and
`role` is deterministic from the operation's inputs (`StartCap`, `EndCap`, `Side(curve)`,
`Fillet(n)`, `Chamfer(n)`). Booleans preserve `FaceKey`s of surviving face fragments.
Edges are identified by the unordered pair of `FaceKey`s they separate (`EdgeKey`).
Sketch profiles are referenced by a sample point inside the region, and planar faces used
as sketch planes get a frame anchored at the face centroid. A generator's inputs are
`RegionRef`s, so the same feature takes either a sketch region or a planar face; a face
used as a region tags each stretch of its outline with a hash of the face it borders, so
the lateral faces the generator grows there keep their keys when the body below changes.
This is what lets a fillet applied in step 5 survive an edit to the extrude in step 2.

## Kernel representation (MVP)

The MVP kernel is a **polygonal boundary representation with analytic provenance**: a
`Solid` is a set of `Face`s, each face is a set of planar polygons plus a `SurfaceKind`
(planar, cylindrical, …) recorded from the operation that made it. Booleans use BSP
partitioning. Curved surfaces are tessellated at creation with a documented resolution.
This gets extrude/revolve/sweep/loft/booleans/fillet/chamfer working end-to-end with one
consistent representation. The public API (`Solid`, operations, `FaceKey`) is designed so
an exact analytic B-rep can replace the internals later without changing `basset-core`.

Details worth knowing:

* Polygons are convex (caps are ear-clipped) because the BSP splitter relies on it.
* Booleans heal T-junctions afterwards so edge extraction and `Solid::is_closed` see a
  vertex-for-vertex matched shell; vertices merge within `MERGE_TOL` (1 µm).
* Fillets and chamfers are built as prismatic tool solids per edge chain (mitred at every
  polyline joint) and applied with a boolean: subtract for convex edges, union for
  concave ones. Where several blended edges meet the result is the intersection of their
  tools (a crease), not a spherical corner patch. Rolling over a face onto a neighbour
  (a radius larger than the adjacent face) is not detected.
* Overlapping coplanar faces with the same orientation survive a union twice; the
  volume is right but the face is doubled. Coplanar opposite faces (extruding from a
  face, cutting from a face) are handled.
* Curved faces are shaded smoothly by averaging facet normals within 45°; the analytic
  `SurfaceKind` is what selection reports.

## Sketch representation

Points are first-class entities; lines, arcs and circles reference points by id. The
solver flattens all free parameters into one vector and solves constraints as a
least-squares system with Levenberg–Marquardt, using forward-mode dual numbers for exact
Jacobians. Constraint residuals are written once, generically over the scalar type.

Profiles (closed regions usable by extrude etc.) are found by planar face tracing.
Curves are tessellated into polylines at extraction time and split wherever two of them
cross, so every region the drawing encloses is a profile, not only the ones the user drew
with matching endpoints. Each polyline segment is tagged with its source curve index so
the kernel can give the resulting side faces stable keys and correct surface kinds.
Splitting happens on the tessellated curves and both sides are cut at the identical
point, so the graph stays watertight and the boundary is as accurate as the tessellation
the kernel consumes anyway. Two fragments of one curve bounding the same region share a
curve tag, and so become one kernel face in two pieces rather than two faces.

Curves are trimmed and broken by cutting them at the analytic intersections with every
other curve, which divides a curve into pieces named by parameter ranges: trim drops the
picked piece, break keeps them all. Pieces share the point entities at the cuts, and the
picked curve's own entity is reused for its first surviving piece so dimensions written
on it survive; geometry that must change kind (a trimmed circle becomes an arc) carries
over the constraints the new kind still accepts. Patterns copy entities together with the
constraints written *between* them, so every copy holds its shape; copies are not linked
back to the seed, because a sketch-level pattern entity to regenerate from does not exist
yet and a silently broken copy would be worse than honest plain geometry.

A sketch also carries a table of named parameters — expressions over each other in a tiny
`+ - * / ()` language — and a binding from dimensions to expressions. Solving evaluates
the bindings first, so changing one parameter re-drives everything written over it.
Expressions are evaluated in the unit the dimension is typed in (degrees for angles), and
typing a plain number over a driven dimension releases it.

## Timeline

`Timeline` is an ordered list of `Feature`s plus a rollback cursor. Regeneration replays
features 0..cursor into a `ModelState`. Editing feature *i* invalidates the cached state
from *i* onward and replays; features whose references are missing after an edit are
marked `Failed` with a message rather than aborting the replay, mirroring Fusion's yellow
warning behaviour.

## File format

`.bass` is JSON: `{ "format_version": N, "generator": "...", "document": ... }`. Version 2
renamed the generators' `profiles` list to `regions` when a planar face became usable as a
region. The document is the timeline plus metadata; geometry is never stored because it is
fully regenerable. Old versions are migrated on load; newer versions are refused with a
clear error. Saves go through a temporary file and rename so a crash never truncates the
previous copy.

## Application

`basset-app` is a winit + wgpu + egui shell. The 3D viewport is rendered across the whole
window into an sRGB view, and egui panels are drawn on top with a load pass into the plain
view of the same surface (egui blends in gamma space, the viewport in linear). The
`Editor` owns the `Document`, camera, selection and one of two modes:

* **Model mode** picks faces, edges, corners, planes, sketch profiles, curves and points.
  A *selection mode* (Any / Face / Edge / Vertex / Sketch, keys `1`-`5`) says which of
  those a click may land on — Face covers both a body's face and a closed sketch region,
  which are the same thing to a generator — because dense geometry puts several of them within a few
  pixels of each other; it narrows a running tool's own filter, but never to nothing.
  Corners have no stable kernel key, so one is named by its position and lives only in the
  editor's selection — no feature stores a vertex. A modelling tool
  owns one timeline feature which it creates as soon as the input is complete and re-edits
  on every parameter change, so the viewport is a live preview; the document's transaction
  API makes the whole interaction one undo step and Cancel a rollback.
* **Sketch mode** draws on one plane with the same shape builders the sketch crate tests
  use, trims and breaks existing curves, patterns and moves a selection by typed offsets,
  names closed regions by a point inside them so `E` can hand them straight to Extrude, writes the working copy back into the feature after each change (so downstream
  features update live), and keeps its own undo stack for the session. The grid is drawn
  on the sketch plane and points that snap to no existing point snap to it; dragging on
  empty space is a rubber band, enclosing or crossing by its direction.

The navigation cube is drawn from the camera's own basis and hit-tested by casting the
pointer into a unit cube, so its 26 click targets are exactly the shapes drawn and need no
table of screen positions.

Panels never hold `&mut Editor` while borrowing document state: they queue commands that
run after the frame's UI closure returns.
