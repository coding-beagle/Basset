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
  vertex-for-vertex matched shell; vertices merge within `MERGE_TOL` (1 µm). Generators
  heal too, and then check that the shell is closed before returning it, so a profile
  that doubles back on itself is either repaired by the heal or fails its own feature
  rather than seeding every later operation with a hole.
* Fillets and chamfers are built as prismatic tool solids per edge chain (mitred at every
  polyline joint) and applied with a boolean: subtract for convex edges, union for
  concave ones. Where several blended edges meet the result is the intersection of their
  tools (a crease), not a spherical corner patch. Rolling over a face onto a neighbour
  (a radius larger than the adjacent face) is not detected.
* A BSP tree over a convex body is a list — the solid is the intersection of its face
  half-spaces, so every face plane has the rest behind it — and a boolean is quadratic
  in the facets of such a body. A fillet tool swept round a rim is one, and so is the
  cylinder it is cut from. The tree walks are iterative so depth costs no stack, and a
  blend feature budgets the facets it puts through its booleans (`MAX_FEATURE_POLYGONS`),
  coarsening its arcs to fit before it refuses, because the alternative was a radius box
  that could take the machine down.
* Overlapping coplanar faces with the same orientation survive a union twice; the
  volume is right but the face is doubled. Coplanar opposite faces (extruding from a
  face, cutting from a face) are handled.
* A boolean regroups fragments under the key of the face each came from, which would
  leave the flat top of two extrusions that finish at the same height as two faces, so
  `merge_continuous_faces` collapses faces of *different* operations that share an edge
  and continue across it into one, named by the group's lowest `FaceKey` — the earliest
  operation that made part of it. Faces of one operation are left alone: two coplanar
  sides there are two named stretches of one profile, and a fillet may be written on
  either.
* Booleans heal but do not validate, so a BSP result that leaks is neither reported nor
  visible: `display_edges` draws nothing along an edge only one polygon uses, on the
  grounds that painting the triangle soup helps nobody. Generators do validate, so the
  input to a boolean is sound; the boolean itself is the gap. `Solid::validate` is there
  when a caller wants the check.
* Curved faces are shaded smoothly by averaging facet normals within 45°; the analytic
  `SurfaceKind` is what selection reports.
* What the user sees as an edge is `Solid::display_edges`, worked out from the topology:
  every fold sharper than 45° and every change of surface, and nothing else. The mesh is
  never the source — welding the triangles on rounded coordinates misses vertices that a
  boolean left agreeing only to `MERGE_TOL`, and each miss draws a triangle edge as if
  the body had a hole. Two faces on one surface (a sketch line cut in two extrudes into
  two coplanar faces) meet at a *smooth* edge: nothing is drawn along it, nothing can be
  picked on it, and its ends are not corners. `Solid::edges` still reports it, because
  face boundaries are what `face_profile` traces and what booleans key from.

## Sketch representation

Points are first-class entities; lines, arcs and circles reference points by id. The
solver flattens all free parameters into one vector and solves constraints as a
least-squares system with Levenberg–Marquardt, using forward-mode dual numbers for exact
Jacobians. Constraint residuals are written once, generically over the scalar type.

The solve reports not only how many degrees of freedom remain — free parameters minus the
rank of the hard Jacobian — but which entities own them, by walking the Jacobian's null
space one free column at a time. The editor draws those entities in blue, and a sketch
that still has freedom left warns from the timeline through `FeatureStatus::Warned`: an
under-constrained sketch is a model-level fault, because it is an edit *elsewhere* that
moves the loose geometry and quietly changes what the profiles enclose.

Over-constraint is reported two ways, because it is two faults. A solve that fails names
the constraints that disagree (`SolveError::DidNotConverge::conflicting`, worst first);
a solve that succeeds names the constraints whose equations were all implied by the
ones ahead of them (`SolveReport::redundant`). The editor draws the first red and the
second orange — badges, leader lines and dimension value boxes alike — and the sketch
palette lists each set in a section of its own with hover-to-highlight and a delete, so
the answer to "which one?" is on the drawing and the fix is one click away.

Profiles (closed regions usable by extrude etc.) are found by planar face tracing.
Curves are tessellated into polylines at extraction time and split wherever two of them
cross, so every region the drawing encloses is a profile, not only the ones the user drew
with matching endpoints. Each polyline segment is tagged with its source curve index so
the kernel can give the resulting side faces stable keys and correct surface kinds.
Splitting happens on the tessellated curves and both sides are cut at the identical
point, so the graph stays watertight and the boundary is as accurate as the tessellation
the kernel consumes anyway. A curve is also split where another curve's *endpoint* lands
within the join tolerance of its interior without touching it — the 2D counterpart of
`Solid::heal` — because solver output lands a few 1e-7 off exact coordinates routinely,
and an unhealed near miss merges the regions either side of it into one self-overlapping
loop that extrudes without complaining. Two fragments of one curve bounding the same region share a
curve tag, and so become one kernel face in two pieces rather than two faces. Curves that
run along each other are cut at the ends of the stretch they share and the duplicate is
dropped, so an outline traced twice still encloses its regions; which copy survives, and
therefore which curve names the face, is arbitrary between identical curves.

Curves are trimmed and broken by cutting them at the analytic intersections with every
other curve, which divides a curve into pieces named by parameter ranges: trim drops the
picked piece, break keeps them all. Pieces share the point entities at the cuts, and the
picked curve's own entity is reused for its first surviving piece so dimensions written
on it survive; geometry that must change kind (a trimmed circle becomes an arc) carries
over the constraints the new kind still accepts. Patterns copy entities together with the
constraints written *between* them, so every copy holds its shape; copies are not linked
back to the seed, because a sketch-level pattern entity to regenerate from does not exist
yet and a silently broken copy would be worse than honest plain geometry.

Filleting a sketch corner (`fillet`) finds the arc by offsetting both curves by the
radius and intersecting the offsets: every point that distance from both is a possible
centre and the tangent points are its feet, which covers line–line, line–arc and arc–arc
with one construction instead of a case each. Up to four candidates come back, one per
corner of the crossing, so the caller passes the point on each curve it picked and the
candidate whose tangent points lie on those sides is the one meant — which is also why
the tool takes picks rather than just two ids. Everything that can fail is decided before
anything is edited (`plan`), so a radius that does not fit is refused with the sketch
untouched; then each curve's corner-end point is moved to its tangent point, which keeps
the curve's own entity and with it every dimension written on it, the arc is added sharing
those two points, and two `Tangent` constraints are written down. Without them the corner
is rounded only until the next solve. The corner point the two curves used to share is
pruned exactly as a trim prunes its orphans, so a dimension still written to it keeps it
alive rather than silently vanishing.

Offsetting (`offset`) orders the picked curves into one chain end to end — refusing a
branch, where three curves meet and there is no single answer — normalises a closed chain
counter-clockwise so that "outward" means something, moves every curve sideways, and then
resolves each corner. A corner that opens away from the side being offset to leaves a
gap, closed either by an arc of the offset radius about the source corner or by running
both edges out to their crossing; a corner that closes has the two edges overlapping, and
they are trimmed to that same crossing. The trap is that an offset bigger than the shape
is thick stays locally valid at every corner while the shape turns inside out, so each
piece is checked against its own untrimmed self (`Piece::agrees_with`): a piece trimmed
past its far end has reversed, and that is the offset running out of room rather than a
smaller copy of the drawing. That is still only local, though, and a slit narrower than
twice the distance closes over without any single corner noticing — so the finished
curves are also checked against each other for crossings (`crossing_free`) before any of
them is written down. A convex curve tighter than the offset is *not* an error: it
shrinks past a point and out of existence, its neighbours are joined to each other
instead, and that is what makes an offset of a filleted rectangle come out as a
square-cornered one rather than a refusal. Mitres are capped (`MITER_LIMIT`) because the
meeting point of a sharp enough corner runs away to infinity, and a cusp — where the
chain doubles back and the turn direction cannot be read off a cross product that is zero
— is named as a gap outright rather than left to the rounding. The result carries the constraints that
are true of it by construction — parallel, concentric, coincident corner centres, tangent
where the source was smooth — but is not linked back to its source, for the same reason a
pattern's copies are not.

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
  use, trims and breaks existing curves, rounds corners, patterns, offsets and moves a selection, names closed
  regions by a point inside them so `E` can hand them straight to Extrude, writes the
  working copy back into the feature after each change (so downstream
  features update live), and keeps its own undo stack for the session. The grid is drawn
  on the sketch plane and points that snap to no existing point snap to it; dragging on
  empty space is a rubber band, enclosing or crossing by its direction.

  Everything the user *does* there is a tool, including the geometric constraints: a
  constraint is picked first and its geometry after, and `constraints_for` decides what a
  set of picks means without caring about their order — which is also the question the
  toolbar asks to decide whether a button would do anything. The modal operations (move,
  pattern, offset, fillet) keep the sketch they started from and re-derive the result from it on every
  change, so editing a number twice replaces the result rather than compounding it;
  cancelling is putting that copy back, and keeping is pushing it onto the undo stack,
  which is why a modal operation is exactly one step of undo and never pops a checkpoint
  it did not take.

  They all have to watch out for the same trap. The solver does not fail when a
  constraint cannot be satisfied the way the user meant; it finds some *other*
  arrangement that satisfies it, and the cheapest one is usually the geometry folded flat.
  A move therefore checks that its result is still rigid (`rigid_error`) rather than
  trusting the residual, and a pattern rewrites each copy's constraints for the angle it
  was turned through (`pattern::turned`) rather than copying an axis constraint into a
  copy that contradicts it. Both failures used to look like a converged solve and a
  destroyed drawing.

  They all also get a window of their own (`panels::sketch_operation_dialog`) rather than a
  section at the bottom of the sketch palette. The controls an operation is driven by
  have to be on screen for as long as it is running, and the palette is a scrolling panel
  whose lower reaches are past the fold on an ordinary window — which made a pattern's
  origin unsettable, because the button that arms picking it could not be reached.

  Sketch mode has its own pick filter (`SketchPick`: all, curves, points, regions) beside
  model mode's, over the kinds of thing a sketch has. It applies to the Select tool only:
  drawing and snapping must still see every point whatever the user is choosing to
  select.

A dashed line carries the distance already travelled along its polyline
(`SegmentInstance::start`), because the dash pattern is measured in pixels along a
segment and a tessellated curve's segments are shorter than one dash — a pattern that
restarted at each segment put every one inside a dash and drew the curve solid, so
construction geometry was indistinguishable from ordinary geometry on anything small.
The renderer accumulates that distance itself, starting a new run wherever a segment does
not begin where the last one ended.

The manipulator both kinds of move are dragged by lives in `editor::gizmo`: arrows for the
directions a transform can travel along and rings for the axes it can turn about, chosen
from whatever is being moved (a sketch gets its plane's two arrows and one ring, a body
gets three of each). Only the grips are egui widgets; the shafts and rings are scene
geometry so they sit in 3D. A grip's drag is projected onto its axis *as that axis appears
on screen* and scaled by the world size of a pixel, which is what makes the handle follow
the pointer at any camera angle; a ring converts the same projection into an angle through
its radius. The manipulator only ever writes the numbers the dialog or palette shows, so
dragging and typing are two ways of saying one thing.

A `gizmo::Slider` is the other shape of handle: one grip sitting *at* the value rather
than on an arm of fixed length, so dragging it is dragging the number. An offset's
distance is one — `anchor + dir * distance` is a point of the result, and the anchor is a
point of the *source*, so the grip slides along one fixed line instead of wandering out
from under the pointer as it is dragged. It is still there at a distance the offset
refuses, which is how the user drags back out of one that does not fit, and taking it
through zero is how the side gets chosen, so there is no flip button to go and press.

A sketch fillet's radius is the other slider. Its anchor is the corner and its direction
the bisector, so the grip sits exactly the radius out along it: the distance from the
corner to the grip *is* the number, and pulling away from the corner grows the fillet the
way the arc itself travels. Unlike an offset there is no far side to cross into, so a drag
back through the corner stops at the smallest fillet there is rather than turning the arc
inside out.

Every tool button, sketch or modelling, in the toolbar or in a menu, carries a symbol
painted by `panels::tool_button` — the bundled fonts have no usable glyphs for these
shapes, and a drawn one reads the same on every machine. One `AnyTool` covers both tool
enums so there is a single icon mechanism rather than two that drift. Icon and label are
one widget with one id and one click sense, which is the fix for a bug that kept coming
back: a symbol drawn beside a button lights up under the pointer, and then the click does
nothing because only the word next to it was the button.

Every handle that snaps answers to shift. `SketchEditor::snapping()` is the single
question — the palette's toggle *and* shift not being held — and the drawing path, the
move's arrows and ring, and the offset's and fillet's sliders all ask it rather than reading the toggle
directly. The modifier reaches the sketch from two places, because the drawing path comes
from winit and the manipulators come from egui and neither sees the other's events; both
write it through `set_free_snap`. egui carries modifiers as a standing state set by
`ModifiersChanged`, not as a field on each pointer event, which is what lets a widget know
shift is down *while* it is being dragged — and is what a test driving a drag has to
reproduce.

The navigation cube is drawn from the camera's own basis and hit-tested by casting the
pointer into a unit cube, so its 26 click targets are exactly the shapes drawn and need no
table of screen positions.

Panels never hold `&mut Editor` while borrowing document state: they queue commands that
run after the frame's UI closure returns.

## Testing the application headlessly

`basset-app`'s `editor::harness` drives the program with no window and no GPU. It feeds
the editor real winit events (pointer moves, buttons, the wheel), runs the egui panels
for a frame and reads back the text they laid out — so a test can click a toolbar button
by its label — and builds the `Scene` the renderer would draw, which is how tests assert
on what is on screen. Model-space helpers turn a point in millimetres into the pixel it
occupies through the camera, so clicks exercise the same projection and pixel tolerances
picking uses. Two things a window supplies cannot be forged, because winit keeps their
fields private: `KeyEvent` and `Modifiers`. Keys go to `Editor::on_key` (what
`handle_window_event` does with them) and modifier state is written where the event would
have put it. `editor::tests` covers the interaction logic tool by tool; `editor::e2e`
covers the seams above it — events in, panels and scene out, and a document round trip
through a file.
