# Basset architecture

Basset is a parametric, history-based solid modeller in Rust. The long-term target is
feature parity with Fusion 360; the MVP deliberately ships few tools, each finished.

## Crate map

```
basset-math      f64 vectors (glam), Frame/Plane/Ray, tolerances, TriMesh interchange type
basset-sketch    2D sketch entities, constraints, solver, shapes, text, profile extraction
basset-kernel    solid modelling: Solid/Face topology, CSG, extrude/revolve/sweep/loft,
                  fillet/chamfer/thread/combine/transform, tessellation, picking, mass properties
basset-core      Document, Components, Planes/Axes/Sketches/Bodies, document parameters,
                  Timeline + regeneration, document save/open (.bass JSON), the diff
                  between two versions of a document
basset-io        STL / 3MF export
basset-fea       basic finite element analysis: voxel brick mesh of a Solid, linear elastic
                  static solve, von Mises stresses, SIMP topology optimisation, legacy VTK output
basset-render    appearances (metallic–roughness, transmission, clear coat, patterns) and their
                  library, procedural environments, scene settings, a progressive CPU path
                  tracer and PNG output
basset-viewport  wgpu renderer: camera, mesh/line/point/triangle batches, grid, selection highlight,
                  meshes with a colour per vertex and the ramp a stress plot paints with,
                  physically based environment shading, per-face material masks
basset-app       winit + egui desktop application (Linux first)
basset-mcp       Model Context Protocol server over stdio: the document model driven by an agent
```

Dependency direction is strictly downward: `math ← sketch, kernel, render ← core ← io, viewport ← app`;
`basset-mcp` sits beside the app, over `core`, `io` and `fea`, and knows nothing of the viewport.
`basset-fea` sits over `kernel` alone: it takes a `Solid` and face keys and knows nothing of
documents, so a study can be run on any body the kernel can make.
`basset-render` sits over `math` alone, like `fea` over `kernel`: it takes triangle meshes
with a face id per triangle and is told which appearance each face id wears, so it knows
nothing of documents or the kernel; `core` stores appearances as its types and builds a
trace scene from bodies' tessellations (`core::trace_scene`).
`sketch` and `kernel` do not know about each other; `core` converts sketch profiles into
kernel profiles. This keeps both testable in isolation and lets the kernel be replaced.
Where `core` has to put something of its own *into* a sketch — the document's parameter
table — it goes in as a closure rather than as a type, so the arrow still points one way;
see [Parameters](#parameters).

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
`Fillet(n)`, `Chamfer(n)`, `Thread(n)`). Booleans preserve `FaceKey`s of surviving face fragments.
Edges are identified by the unordered pair of `FaceKey`s they separate (`EdgeKey`).
Sketch profiles are referenced by a sample point inside the region together with a
signature of the curves that bounded it when it was picked (`ProfileRef::curves`, an
order-independent hash of their slot indexes). The point is the reference a user
understands, but it is a point in a drawing still being dimensioned: a rectangle shrunk
from 10 wide to 2 leaves the point it was clicked at out in the open. The curves keep
their identity through every such edit, so when the signature is known it is what the
region is found by, and the point only settles which of several regions the same curves
bound (a circle cut by one line is two). A reference whose curves no longer bound any
region falls back to the point, and if that encloses nothing either, to the nearest
region with a warning, since it was once inside one and the drawing moved under it; a
point-only reference (files from before the signature existed) that encloses nothing
fails as it always did, because guessing a region for a point never known to be inside
anything would turn a typo into a body. Such a reference is given its signature the
moment its sketch is about to be edited (`Document::sign_region_refs`): the region its
point finds then is the one the user sees, so recording its curves changes nothing yet,
and from then on a circle dragged off the point keeps its cut instead of handing it to
the square around it. Planar faces used
as sketch planes get a frame whose origin is where the world origin falls on the face's
plane, as in Fusion: nothing about the face's extent goes into it, so a sketch on a wall
stays where it was drawn when an edit upstream makes the wall taller. (Before format 10
the origin was the face's vertex average, which moved with every such edit and took the
sketch along; reading an older file replays it once the old way to learn where each
sketch sat and moves it into the new frame, see `Document::convert_face_frames_from_v9`.)
A sketch started on a face also opens with the face's outline copied in
(`core::project_face`): the kernel's `face_profile` is turned back into lines, arcs and
circles — a run of boundary pieces bordering one neighbour is one curve — with every
point fixed and a circle's diameter written, so the face is a reference the sketch can
dimension from, snap to and extrude, and not a loose drawing the timeline would warn
about. The copy is linked: each point and circle carries a key naming what it is on the
face (the corner between the stretches bordering two neighbours, the centre of the arc
along one), and every replay re-traces the face and moves the pinned copy to match
(`core::refresh_face_outline`) before the sketch is solved. A corner the face no longer
has stays where it was and the sketch warns; a point the user unpinned is theirs. A generator's inputs are
`RegionRef`s, so the same feature takes either a sketch region or a planar face; a face
used as a region tags each stretch of its outline with a hash of the face it borders, so
the lateral faces the generator grows there keep their keys when the body below changes.
This is what lets a fillet applied in step 5 survive an edit to the extrude in step 2.

An extrude "to" a face goes whichever way the face is. A sketch on a plane above a body,
extruded to the body's top, means downwards, so the reach is signed along the profile
normal and a target behind the profile is reached by travelling backwards; only a target
plane the profile straddles is refused, since the extrusion would thin to nothing along
the crossing line. A curved target is met at its first contact ahead if there is one,
otherwise at its first contact behind.

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
  (a radius larger than the adjacent face) is not detected. A fillet radius is signed,
  and the sign is the shape: positive is the round tangent to both faces, negative the
  *inverted* round — the sector of the dihedral angle about the edge line itself, a
  radius out along each face — which cuts a cove into a convex edge and lays a bead into
  a concave one. It is the same handle dragged the other way, as in Fusion's Press/Pull.
  A chamfer has no inverted form: the triangle it takes off already has its apex on the
  edge, so it stays unsigned.
* On compound bodies the chain does the work. Selected edges that continue one another
  tangentially — a straight edge a boolean seam split into two keys, the straights and
  arcs of a slot — become one tool mitred through the joins, each edge keeping its own
  blend face. A tool runs a hair past a chain end that opens into air and stops a hair
  short of one that does not (the inside corner of a T, the top face a bead climbs to),
  so it never notches a face it was not asked to touch; the price is that much of the
  edge left sharp, below anything drawn. Beads are applied before rounds whatever the
  pick order. Three shapes are refused with a typed error rather than built leaking: an
  edge folding by less than the 20° tangent threshold (a fillet's own run-out), an edge
  whose dihedral angle varies along it (a cylinder cut off askew — the section rotates
  against the facet seams), and a round whose end meets a bead in one feature (it would
  need a corner blend). Along a chain that turns, the arc is drawn no finer than 10° per
  facet: the boolean cuts the body's faces along the arc's tangent facets, and finer
  tangent facets on a curved chain disagree about where the face ends.
* A thread is a tool too: the ISO metric groove swept along a helix about a cylindrical
  face and subtracted, cut inward from a shaft (the face is the major diameter) and
  outward from a hole (the minor one, so a hole drilled at the tap size threads to size).
  It runs a turn past an end that opens into air and is clipped dead in the plane of an
  end that runs into material, a shoulder or a hole's floor. A helical flank cannot be
  faceted flat, so each facet of one is two triangles, and the BSP is at its weakest
  there: two near-parallel cuts across a body facet leave a sliver below its tolerance
  and the shell leaks. Three measures together closed all 155 threads of a sweep over
  sizes and pitches where the plain helix closed 100: the root steps a hundredth of the
  pitch along the axis on alternate rings, which creases every flank facet by a degree
  or so; each attempt is validated and retried drawn a fraction of a facet round with
  the other diagonals; the last retries draw 24 facets to the turn instead of 36. Fewer
  facets from the start would have been robust but inaccurate — the groove is an
  inscribed polygon, and at 12 to the turn its chords cut a fifth of an M10's depth.
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
  face boundaries are what `face_profile` traces and what booleans key from. The one
  line the viewport adds of its own is the silhouette, worked out per camera from the
  mesh's facet adjacency; a facet too thin for its own cross product to mean anything
  (a boolean cap's triangulation has them, since covering every healed vertex forces
  triangles across collinear runs) is oriented by the normal the mesh carries instead,
  or it reads as facing backwards and sprouts lines across the flat face it lies in.

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
with matching endpoints. The one contact chords cannot find is a tangency — a circle
touching a line from inside has its nearest chord vertex a sagitta short of the line and
no chord crossing it — so the analytic contact point of every tangent pair (line against
circle or arc, two circles or arcs) is seeded into both polylines as a vertex first, and
the chord scan then meets it like any crossing. Only tangencies are seeded: transversal
crossings the chords already find, and seeding those put a vertex a micron from the
endpoint a near-miss T-junction is cut at, which traced slivers between the two. At a
node where two edges leave tangent to each other the departure angle cannot order them,
and the face walk orders them by their bend instead: the edge curving further left is
further counter-clockwise. A circle inscribed in a square is the test for all of this,
five regions where there used to be none. Each polyline segment is tagged with its source curve index so
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
where the source was smooth — and one driving dimension, `Constraint::Offset`, holding it
at the distance. That is what makes an offset's size editable afterwards: it is clicked,
typed over and bound to a parameter like any other dimension, and a moved source takes
its offset along. The dimension measures one pair (source, result) per run of the result
joined smoothly, because along such a run the tangencies already carry the distance from
piece to piece and a pair for every piece would be reported redundant; a cut corner
carries nothing, so a mitred rectangle has four pairs and a rounded one has one. The
price of a solver-held offset is that its topology is fixed at creation: a new distance
that would swallow a curve or open a trimmed corner has no shape in the solver and fails
like any over-ambitious dimension, where offsetting afresh would work the corners out
again.

A sketch also carries a table of named parameters and a binding from dimensions to
expressions. Solving evaluates the bindings first, so changing one parameter re-drives
everything written over it, and typing a plain number over a driven dimension releases
it. The same table now also exists once at the document, behind every sketch and in front
of every feature; the next section is about both.

## Parameters

A named number — `bore = 12.5`, `wall = bore / 8` — is resolved in two scopes. A sketch
looks a name up in its own table first and asks the document only for what it does not
define, so a sketch parameter *shadows* a document one of the same name, as a local
shadows a global. That ordering was chosen over migrating the existing per-sketch tables
upwards because it needs no migration at all: every sketch that already had a `width`
still means its own, and a file written before the document had a table loads with an
empty one and behaves exactly as it did. The price is paid in one place and is
deliberate: a sketch parameter written over a document one cannot refer outward to it, so
`width = width * 2` is a cycle rather than a reference, because resolution is by name and
there is no syntax for saying which scope is meant. The error names the cycle, so at
least what happened is legible.

`basset-sketch` never learns that documents exist. The outer table arrives as a closure,
`basset_sketch::Outer` (`&dyn Fn(&str) -> Result<f64, SketchError>`), which
`basset_core::Parameters::lookup` hands over. `expr::eval` already took a lookup closure
for the sketch's own names, so this is that same seam carried one level up. A type would
have meant `sketch` depending on `core` — the reverse of `math ← sketch, kernel ← core ←
io` — and would have left the sketch crate untestable without a document behind it. The
`_with` half of the sketch API (`solve_with`, `parameter_value_with`,
`apply_parameters_with`, `failed_bindings_with`, `rename_parameter_with`) is each
operation with that closure supplied; the plain half passes `no_outer`, which knows no
names at all and is what the crate's own tests use.

The expression language is arithmetic and a named function table, and nothing else,
because every symbol in it has to be obvious to someone reading a dimension in a toolbar
three months later. `+ - * /` and brackets, `^` binding tighter and *right*-associative
so `2^3^2` is 512, exponent literals, and `sqrt abs floor ceil round sin cos tan asin
acos atan atan2 hypot min max deg rad`. Trigonometry is in radians in and out, as `f64`'s
own methods are, while the angle dimensions around it are in degrees: that mismatch is
not papered over with unit magic, `deg` and `rad` are in the table so the conversion is
written down where it happens. `pi` and `tau` are fallbacks rather than keywords — a name
is offered to the lookup first and only becomes a constant when the lookup reports it
unknown — so a user parameter called `pi` beats the constant, and any other lookup
failure still reaches the caller instead of being swallowed by a constant.

A feature's numbers are driven by the same expressions. `Feature::exprs` is a
`BTreeMap<NumericField, String>` beside the kind rather than an `Option<String>` next to
each number: the numbers live in the variants of `FeatureKind` and most of them are never
driven, so a neighbour field would have to be added to a dozen variants, written `None`
at every construction site, and could still come to disagree with the number it annotates.
A `NumericField` names a value by role — `Distance`, `Negative`, `Angle`, `Radius`,
`Pitch` — so one key means the same thing across the kinds that offer it (an extrude's
distance and second distance, a revolve's angle, a fillet's radius, a chamfer's distance,
a thread's pitch and length, an offset plane's distance, an angled plane's angle) and a panel can label it without matching on
the kind.

Replay resolves those expressions into a *copy* of the feature (`Regenerator::drive`,
handing back a `Cow` so the common case — a feature nobody drives — clones nothing).
Regeneration must not write back into the timeline: the expression is the input and the
number is derived from it, so a replay that edited the feature would make regeneration a
mutation and undo a lie. The stored number is kept in step separately, because it is the
only one a panel can read and releasing an expression has to keep the value the user
currently sees: `Document::refresh_driven_values` writes the evaluated numbers back after
every change to the table, including the wholesale swaps undo, redo and a rolled-back
transaction perform. It records no undo entry and moves no cursor, because it derives
nothing — it only catches the timeline up with a change that was recorded already. An
expression that stops evaluating does not fail its feature: the number it last had
stands and the feature is `Warned`, on the same grounds the sketch layer keeps a stale
dimension. Deleting a parameter should say what stopped being driven, not collapse half
the model to zero while the user works out what happened.

A parameter can drive anything at any point in the history, so there is no earlier
feature worth keeping: `Regenerator::set_parameters` throws the whole snapshot cache away
and replay starts from feature zero. The document therefore hands the table over only
when it has actually changed, since `state()` runs on every frame. Undo snapshots the
table alongside the timeline for the matching reason — restoring a timeline into a
document whose table had moved on would undo the edit and leave the model meaning
something neither version ever meant.

Expressions are evaluated in the unit the value is *typed* in: degrees for angles,
millimetres for everything else. `FeatureKind::numeric_field` and `set_numeric_field` are
the only place that conversion happens, because an expression is a number the user would
otherwise have typed into that box and so must mean what typing it there would have
meant. The feature path and the sketch path differ in exactly one respect, on purpose.
`Sketch::apply_binding` takes the magnitude and copysigns it back onto the angle the
constraint already held, because the sign of a sketch angle selects which solution the
solver lands on, and re-driving a dimension must not flip the drawing into its mirror. A
feature angle's sign is a direction the user chose rather than a solver branch, so there
it comes from the expression as written.

Renaming is what makes reference-by-name survivable. `expr::rename` is token-aware, so it
rewrites the name and nothing that merely looks like it — a function call keeps its name,
a longer identifier containing it is left alone, no number is touched — and copies the
rest of the text through byte for byte, spacing included, because the lexer keeps each
token's span and only spans are rewritten. `Document::rename_parameter` drives that over
its own rows, over every feature's expressions, and over every sketch's table and bound
dimensions; a sketch that *shadows* the old name is skipped, because there the references
mean the local parameter and must not follow the document's rename. It refuses outright
when a sketch both reads the old name and defines the new one for itself: rewriting would
*capture* those references onto the sketch's own row, quietly changing the drawing, or
turning it into a cycle where the sketch's own row is what mentioned the old name.
Neither table can see that alone — capture is a collision between two scopes — and the
document is the only place that holds both.

A body is named by the feature that made it (`BodyRef`), so the name the user gives it
in the browser is kept on that feature (`Feature::body_name`): it is deleted, reordered
and undone with the feature, and no table has to be pruned. A rename moves no geometry,
so the regenerator writes it into its cached states rather than replaying. Making a
component of a body (Fusion's "Create Components from Bodies") is a timeline step,
`ComponentFromBody`, that creates the component inside the one the body is in and moves
the body there from that point of the history on. The feature that made the body keeps
the component it said; a boolean, fillet or move edits a body in place and never
reassigns its component, so nothing downstream has to be rewritten, and rolling back past
the step puts the body back.

## Timeline

`Timeline` is an ordered list of `Feature`s plus a rollback cursor. Regeneration replays
features 0..cursor into a `ModelState`. Editing feature *i* invalidates the cached state
from *i* onward and replays; features whose references are missing after an edit are
marked `Failed` with a message rather than aborting the replay, mirroring Fusion's yellow
warning behaviour.

## File format

`.bass` is JSON: `{ "format_version": N, "generator": "...", "document": ... }`. Version 2
renamed the generators' `profiles` list to `regions` when a planar face became usable as a
region. Version 3 added the document's parameter table and the expressions driving feature
values, and its migration is the identity: both are new fields and both default, so a
version 2 document loads with an empty table and no driven values, which is exactly what
it had. The version was bumped all the same, because migration is only half of what a
version number is for. A file written now can carry parameters, and an older build reading
it would drop them silently and save back a document whose extrude distances no longer say
where they came from; refusing to open it, which the existing unsupported-version path
already does for anything newer than the build knows, is much better than that. Version 6
added `visibility` — the hidden bodies and sketches and whether the origin and grid are
shown — for the same reason and with the same identity migration; an older file opens
with the defaults a fresh editor shows. Visibility sits outside the undo snapshots, since
hiding something is not an edit. The editor keeps its own working copy and syncs it into
the document on save and back out on open. Version 8 added body names and the
component-from-body step on the same terms: both additive, an identity migration, bumped
so an older build refuses the file instead of dropping a body's name. Version 9 added the
curve-set signature a region reference carries beside its sample point, on the same
terms. The document is the
timeline plus that table plus metadata; geometry is never stored because it
is fully regenerable. Old versions are migrated on load; newer versions are refused with a
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
  API makes the whole interaction one undo step and Cancel a rollback. A rollback also puts
  back the redo stack that opening the transaction cleared, because a dialog the user
  cancelled did not happen and should not have cost them the thing they were about to redo;
  and every mutation that can be refused — an edit, rename, removal or reorder of a feature
  that names no feature, or a reorder that would cross a dependency — is validated before an
  undo entry is recorded, so a refusal no longer pushes a step that does nothing and clears
  the redo stack on its way past.
* **Sketch mode** draws on one plane with the same shape builders the sketch crate tests
  use, trims and breaks existing curves, rounds corners, patterns, offsets and moves a selection, names closed
  regions by a point inside them so `E` can hand them straight to Extrude, writes the
  working copy back into the feature after each change (so downstream
  features update live), and keeps its own undo stack for the session. The grid is drawn
  on the sketch plane and points that snap to no existing point snap to it; dragging on
  empty space is a rubber band, enclosing or crossing by its direction. Double-clicking a
  curve selects the whole shape it is chained into (`Sketch::connected_curves`), shift
  adding it to the selection. The viewport's clicks come from winit, which has no notion
  of a double-click, so the editor times the pair itself with egui's thresholds and
  applies the chain on top of the second click rather than instead of it — a
  double-click cannot be known until its second click arrives, by which time both clicks
  have done what single clicks do.

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

The document's parameter table is edited in the browser, beside the origin and the
components, because that is the panel that describes the *document* and the one panel up
whether or not a sketch is open — a table that only drove sketches would not have needed
lifting out of them. A window off a menu would have hidden the very names the feature
exists to keep in front of the user. The sketch palette lists the document's rows
read-only underneath its own and strikes through any the sketch shadows, since a name in
scope that the user cannot see is a name they have no way to know they may write. Editing
the document's table is disabled while a sketch or a tool dialog is open: both hold an
open document transaction, and a parameter changed inside one would be rolled back by an
unrelated Cancel.

Almost every number in the modelling dialogs goes through one helper (`tools::drag`),
which is what makes "any of them can be driven by a parameter" one change rather than one per
tool: the `ƒ` toggle swaps that number's drag box for an expression field, and while an
expression is set the number is read-only and shows what the expression works out to,
because there the expression is the input and the number only its result. The expression
is written onto the previewed feature on every sync rather than at OK, because the feature
*is* the preview and it is inside the tool's transaction either way. A field the kind
stops offering — the second distance of an extrude that is no longer two-sided — is
released rather than left driving nothing and warning on every regeneration afterwards.
The numbers that do not go through that helper (Move's six translate and rotate
components, and the whole sketch-operation dialog) therefore have no toggle; they would
each need a field identity of their own first.

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
table of screen positions. A click turns the camera as if the cube had been rolled to show
that side, carrying the current orientation along rather than resetting to +Z up; the
camera's `roll` angle about the view axis is what lets a square-on view be turned a quarter
at a time (the cube's curved arrows, `Shift+←`/`Shift+→`). Orbiting stands the view back
upright, since yaw and pitch are measured against world +Z.

The Simulation workspace (`editor::simulate`) runs a study of one body with `basset-fea`:
a static stress study, or a topology optimisation of the same set-up (`StudyKind`). The
editor has two *workspaces*, Design and Simulation, chosen by a pair of tabs at the left of
the menu bar the way Fusion's workspace selector works; `Workspace` is a property of the
view, not of the document, so a switch is neither saved nor undoable. Design is the
modeller described above. Simulation keeps the viewport and the menus but swaps the
toolbar for Study / Solve / Results groups (the kind of study, arming the fixed or loaded
face list, the material, the brick size and, for a topology study, the volume fraction;
Run and Stop with a spinner, the elapsed time, the solver's last report and a progress
bar; the plot toggle, what to colour it by, the deformation scale or density threshold,
and a VTK export), the browser for a *study tree* (the kind of study, the body, the
material by name, each held face and each loaded face with a remove button, the mesh, the
optimiser's target, and whether the results are fresh or stale), hides the timeline, and
puts a Study side panel on the right with every setting in full: two face lists each with
a button that arms it so the next faces clicked in the viewport join it (a face clicked
again leaves; a face is never in both); a force or a pressure; the material; the brick
size, defaulting to a twentieth of the body's longest side; the volume fraction and
iteration count of a topology study; and, after a run, the readout and a legend. In the
Simulation workspace the modelling tools, Measure and the timeline edits are refused —
every path a command can arrive by (key, menu, palette) goes through one gate on
`Command::is_modelling`, and the palette and the shortcut overlay hide what it would
refuse — so a face clicked there is only ever a pick for the study. Escape does not leave
the workspace; the Design tab does. Undo and redo stay on, since a study whose body was
undone is simply stale.

The material is picked from `fea::materials` by name: a combo grouped by family with a
filter box at the top, in the toolbar and the panel alike, showing the chosen entry's name
or "Custom" once E or ν has been typed over. The `Simulation` keeps the chosen
`MaterialSpec` beside the solver's two numbers, so a user who nudges the modulus of
6061-T6 is still weighing aluminium: the readout gives the mass (of the mesh, or of the
kept elements of a topology result, in grams or kilograms as the size warrants) and, where
the entry has a yield strength, the safety factor against it, red below one and amber
below two. The plot is coloured by a `Quantity` — von Mises stress by default, the
displacement magnitude, the safety factor (yield over nodal von Mises, capped at ten and
on the inverted ramp so the hot end is the end about to fail; offered only when the
material has a yield point), or for a topology result the density, which is its default.
The legend lies across the panel under the readings, hot at the right, and takes its
unit and its end labels from the quantity, so the bar and the body cannot disagree.

A study is editor state, not a timeline feature, for the reason Measure is: it changes
nothing about the model, and a feature that regenerates to no geometry would cost an undo
step for every retyped load and sit in the history as a step that does nothing. The
editor therefore holds one `Simulation` beside the Measure tool, its picks never reach
the selection, and no transaction is opened. Entering the workspace creates the study
(of the one selected body, or the only body) if there is none, and going back to Design
keeps it, results and all, so the user can edit the part and come back to re-run; the
viewport shows the study — its highlighted faces or its plot — only while the Simulation
workspace is up, and only a new or opened document drops it. What a feature would have
given for free — staying true to the model — the study has to earn: the results carry
the document revision they were computed at, and any edit since marks them stale, at
which point the panel and the tree say so and the viewport goes back to the plain body
rather than keep colouring it with stresses of a body that no longer exists. Results of
either kind go stale the same way.

The solve runs on a thread of its own. `simulate::run` hands a clone of the body's
`Arc<Solid>` and the `Study` (or the `TopologyStudy` built from it) to
`std::thread::spawn` and keeps a `Job` — the start time, the document revision captured
at launch, the receiving end of an `mpsc` channel, a cancel flag and the thread's handle
— on the `Simulation`. The thread calls `run_with` or `optimise_with` with an observer
that sends every `Progress` down the channel and returns the inverse of the flag, and
sends the answer last. `Editor::poll_simulation`, called at the top of every frame,
drains the channel: each report becomes the text under the spinner ("Solving: 1 250
iterations, residual 3.2e-5", "Optimising: iteration 12 of 40, compliance 15.1") and,
when the phase knows its length — an optimisation does, a conjugate gradient does not —
the fraction a progress bar shows; while the job is live it asks for another frame,
which is what animates the spinner and the clock in the toolbar, the panel and the tree.
Run is disabled and Stop enabled meanwhile. The answer is tagged with the revision the
solid was taken at, so a model edited while the solver was running simply gets stale
results, by the same rule as an edit after a run. Stop drops the job, which raises the
flag and drops the receiver: the observer returns `false` at its next report — the first
comes before the mesh is built, so a job stopped at once does no work — and the thread
gets `FeaError::Cancelled`, which the panel reads as "Stopped" rather than as an error,
and exits. The editor never joins the thread; the harness's `stop_and_join` does, so a
test can know the cancellation reached the solver. Headless, the harness's
`wait_for_solve` is the poll loop the event loop would otherwise be. A topology run ends
in an `Outcome` whose `results` are the static solve on the final design, so every reader
of stresses and displacements reads the same field for either kind, with the
`TopologyResults` beside it; its VTK export writes the whole grid with a density per
cell. While the plot is up the body's own mesh is not drawn and a mesh of the plotted
surface — the deformed brick skin of a static study, or the kept elements of a topology
study at the panel's threshold, undeformed — is uploaded through `upload_colored_mesh`
with one colour per vertex and drawn in its place, re-uploaded only when the run or any
part of the `PlotSpec` (quantity, scale, threshold, yield strength) changes. While it is
down, the held faces take the body's one highlight colour and the loaded ones are a
depth-tested fill over the face, because a second instance of the same geometry would
lose the depth test to the first.

The renderer learned one thing for this: a mesh may be uploaded with a linear RGB colour
per vertex (`Renderer::upload_colored_mesh`), and an instance of such a mesh ignores its
own colour and shows those, lit as every other body is and with highlights still painted
over them. Every vertex carries the twelve bytes whether or not it uses them, so there is
one vertex layout and one family of mesh pipelines rather than two of each; a per-draw
flag tells the shader which colour to read. `basset_viewport::stress_ramp` is the blue-to-red
ramp, kept in the viewport rather than in `fea` because it is about looking, not about
stress, and the legend in the dialog is painted with the same function so the bar and the
body cannot disagree.

The study's marks (`editor::study_marks`) are what says *how* a face is loaded, which a
highlighted face cannot: an orange arrow on each loaded face in the direction of the
force — the resultant at the faces' combined centroid, a shorter one at the centroid of
each polygon of a large face — or, for a pressure, a short arrow along every polygon's
normal, into the face for a positive pressure and out of it for suction; a ground mark
(triangle, base line and hatching) on each held face; and, while results are plotted, a
cross at the stress maximum and at the displacement maximum with "max 123.4 MPa" /
"max 0.0123 mm" beside them and the reaction drawn as an arrow leaving the held faces.
The markers stand on the *deformed* plot, so each is moved by its node's displacement —
the mean of its brick's eight for a stress, read at the brick's centre — times the plot
scale. All of it is derived from the `Simulation` every frame, so the arrows cannot
disagree with the panel's boxes. A force also gets a manipulator: three axis arrows with
grips at the resultant's foot, and a grip at the resultant's free end, reusing the move
gizmo's `grip_drag` and `along_axis` because it is the same gesture on a different
number. A pixel has to mean something in newtons, and the rule is that one arm's length
is the force's present magnitude (10 N at least), so the handle feels the same on a 10 N
study and a 10 kN one; the components snap to whole newtons through the same `Snapping`
every other handle answers to, and the resultant's grip dragged back through zero
reverses the force rather than stopping. The grips are egui areas, so they take the
press before a face pick can, as the move gizmo's do; the labels are not interactable,
so a label over the body never takes a click meant for the face under it. Nothing is
drawn in Design, where `study_view()` is `None`.

Panels never hold `&mut Editor` while borrowing document state: they queue commands that
run after the frame's UI closure returns.

### Comparing with git

A `.bass` file is JSON and diffs as JSON, which tells the user that `12.0` became `15.0`
somewhere in an extrude. The editor says the same thing in geometry. `basset-core::diff`
compares two documents: features by id (ids are allotted once and never reused, so the
same id in two versions is the same feature) with a feature counted as changed when its
serialised form differs; parameters by name and text; and bodies by the feature that
made them, face by face. Faces are matched by their `FaceKey` — the operation and role
that made them, the same names the topological naming scheme already keeps stable
across edits — and a face under the same key in both versions is compared by its
quantised vertex set, so a taller extrude reports its side faces and its end cap as
changed and its start cap as untouched. Both documents are replayed to their full length
(`Document::full_state`), because the file holds the whole timeline wherever the cursor
stands.

`basset-app::git` is what the editor needs from the repository, by shelling out to the
user's own `git`: the repository a folder is in (or `git init` to make one), the `.bass`
files it holds and the state of each (`ls-files` plus a `status --porcelain -z` pass over
it), the log with what each commit touched (`--name-only` under a record-separated
format), the bytes of one version of one file, and a commit of a chosen set of paths.
Nothing runs on a frame; the project is read when a file is opened, saved or committed
and when the Git menu opens. `basset-app::project` is the editor's view of it: the
project is the repository the open document is in, or one opened on purpose, and it
outlives the document — a new document keeps the project because it is most often a new
part of it, and Save As opens in the project's folder. The Project panel lists the parts
and the history from this; opening a part as it was at a commit reads the bytes out of
git into a document with no path, so looking costs nothing and keeping it is Save As.
Committing is separate from saving by design: the commit box ticks every changed part,
saves the open document if it is ticked, and records only what is ticked.
`basset-app::compare` holds the version being compared with — read from git,
regenerated — and the diff against the open document, recomputed only when the
document's revision counter moves (`Document::revision`, bumped by every mutation, undo
and redo included, inside a transaction or not), so a tool dialog's live preview is
diffed as it drags and an idle frame costs one integer comparison.

The scene draws the diff with what it already has. A changed body's new faces are its
instance's highlight set in green; the base solid is uploaded as a second mesh and drawn
in the `Overlay` style — translucent and not depth-tested, a style added for this — with
an invisible body colour and its removed faces as a red highlight set, so the old shape
of every moved face floats where it was *inside* the body that replaced it, which a
depth-tested ghost would hide. A removed body is the whole overlay in red; a new body is
tinted green. Selection wins
over the diff on a body with a face selected or hovered, because an instance has one
highlight colour and the user pointing at a face is asking about that face. The
timeline bars each changed chip in the colour of its change and stands a struck-through
chip in for each removed feature at the slot it occupied, before whatever now stands
there, as a text diff shows the old line above the new.

## Finite element analysis

`basset-fea` answers one question so far: how far does a body of one isotropic material
move, and how hard is it stressed, when some of its faces are held and forces or pressures
act on others. A `Study` is the material, the fixed `FaceKey`s, the loads and a target
element size; `run` returns displacements at every node and von Mises stress in every
element, the maxima and where they are, and the total reaction at the fixed faces, which
should equal and oppose the applied load and is reported so the caller can see that it does.

The mesh is a **voxel grid of identical bricks** rather than a tetrahedral mesh fitted to
the surface. Fitting tetrahedra to a faceted boundary that booleans have left agreeing
only to a micron is the hard half of a meshing library; a voxel grid cannot fail to mesh
anything the kernel can tessellate. A grid is laid over the body's bounding box with a
whole number of bricks along each axis, so a box meshes exactly; every cell whose centre
is inside the body is an element, decided by ray parity against the body's own
tessellation with one ray per column of cells, run a hair off the column's centre line so
it never passes through a triangle edge where two triangles would both report a hit.
Each exposed brick facet is tagged with the kernel face nearest its centre (a triangle
facing the same way is preferred, so a facet on a thin wall takes the near side), and that
is how a study's fixed faces and loads, named by `FaceKey`, find their nodes: the same
names a fillet is written on, surviving the same edits.

Because every element is the same brick, the stiffness matrix is computed once
(`element::Brick`, eight-node trilinear, 2×2×2 Gauss) and never assembled: the solver
multiplies by it element by element inside a Jacobi-preconditioned conjugate gradient,
with fixed degrees of freedom masked out. The cap on iterations is what turns a body left
free to float into an error with a hint rather than a hang. Stresses are read at each
brick's centre, where the trilinear element is most accurate, and averaged to the nodes
for a smooth plot. The known costs of the choice are a stair-stepped surface, which blurs
stress concentrations on curved and oblique faces, and a fully integrated brick's
stiffness in bending when a section is a brick or two thick: a cantilever five bricks deep
comes out a few percent stiffer than beam theory, and converges as the mesh is refined.
The tests pin a bar in tension to Hooke's law within 2% and a cantilever to beam theory
within 10%.

Results go out as legacy ASCII VTK (`fea::vtk`), which every viewer reads and needs no
dependency, and as a deformed surface mesh with a stress value per vertex
(`Results::deformed_surface`) with a stress value per vertex, which the editor's Simulate
dialog colours with `stress_ramp` and draws in the body's place. The `fea_static` tool of
the MCP server and that dialog are the two callers. Every entry point has a `_with` twin
(`run_with`, `optimise_with`) that takes an observer closure, handed a `Progress` (phase,
step, and the residual or compliance it is driving down) at every phase change and every
twenty-five conjugate gradient iterations, and returning `false` to stop the solve with
`FeaError::Cancelled`; it is a plain `FnMut`, so the caller decides which thread it runs on.

### Topology optimisation

`fea::topology` asks the inverse question: given a fraction of the body's volume, where
should the material go. It is SIMP — Solid Isotropic Material with Penalisation — the
method every commercial tool grew from: each element gets a density between almost nothing
and one, its stiffness is scaled by the density to a power (three), and the densities are
moved to minimise the compliance `fᵀu` under a volume constraint. The penalty makes an
element of middling density poor value for its volume, so the result tends to solid and
empty rather than a fog. A step is one static solve on the current densities, the
compliance sensitivities, which cost nothing beyond the element strain energies that solve
already gives, Sigmund's sensitivity filter over a ball a few elements wide, and an
optimality-criteria update with a move limit, damping and a bisection on the Lagrange
multiplier that lands the volume on target. The filter is what keeps the answer
mesh-independent and free of checkerboards, and it sets the minimum feature size.

The voxel solver takes to this well. The only change it needs is an optional stiffness
scale per element in `solve::System`, and the regular grid gives the filter's
neighbourhoods by index rather than by search. Elements with a facet on a fixed or loaded
face are passive — held at full density — so the boundary conditions cannot be optimised
away. Each solve is warm-started from the last and run to a looser tolerance than a static
study, with a tight final solve so the stresses reported on the finished design are as
good as `fea_static`'s. The stiffness of an empty element has a floor of one part in a
thousand rather than the `ρ_min^p` of the textbook, because a matrix-free conjugate
gradient crawls at a contrast of a billion and the answer is the same. `TopologyResults`
carries a density per element, the compliance at every solve, the static results on the
final design, and `surface(threshold)`: the skin of the kept elements, including the new
faces between kept and removed ones, with a density per vertex, in the shape of
`Results::deformed_surface` so the same code draws it. `vtk::write_topology` writes the
whole grid with a density per cell so the threshold can be chosen in the viewer.

### Materials

`Material` stays the two numbers the solver reads. `fea::materials` wraps them in a
`MaterialSpec` — a name, a group, a density and a yield strength — for about thirty common
engineering materials (structural and alloy steels, stainless, wrought and cast aluminium,
Ti-6Al-4V, copper alloys, cast irons, magnesium, the usual thermoplastics) with textbook
room-temperature values. Density and yield are what turn a study's answer into the two
numbers a designer asks for, a mass (`Results::mass_kg`) and a safety factor against yield
(`safety_factor`); they live beside `Material` rather than on it because the solver never
reads them. `find` resolves a typed name forgiving case, spaces, hyphens and `aluminum`,
and `MaterialSpec::of` names a pair of numbers that is a library entry so a UI can show
"Custom" otherwise. The generic `Steel` and `Aluminium` entries carry exactly
`Material::STEEL` and `Material::ALUMINIUM`, so documents and tool calls from before the
library existed resolve as they always did.

## Rendering

### Appearances in the document

An appearance is not a physical material: the density and stiffness a study reads come
from `basset-fea`'s library, and a part painted red is still steel. `basset_render::
Appearance` is the metallic–roughness model every real-time engine shares — base colour,
metallic, roughness — plus transmission and an index of refraction (glass, clear
plastic), a clear coat (gloss paint, varnished wood), emission (LEDs) and a procedural
pattern (brushed, wood grain, carbon weave, speckle) evaluated from world position,
because kernel faces have no texture coordinates to wrap an image with. Colours are
eight-bit sRGB and save as `#rrggbb`: that is what a swatch and a colour picker hand over,
and it round-trips exactly. The library (`basset_render::library`, `find`) gives metals
their measured reflectance at normal incidence as colour, so polish versus satin is a
matter of roughness alone.

`core::Appearances` is Fusion's arrangement: applying a library entry copies it *into the
design* by name, and assignments — a document default, one per body, overrides per face —
name the copy, so editing it repaints everything wearing it. Assignments are keyed by
`BodyRef` and `FaceKey`, the names the rest of the document already keeps stable, so a
painted top face stays painted when its extrude grows. An assignment to a body that is
gone is kept, so undoing the delete brings the body back in its colour.

Appearances are in the undo snapshot, because Ctrl+Z after painting the wrong body must
take back the paint and not the fillet before it, but they are not the model, and
undoing a colour must not replay the timeline. The document therefore stamps the model's
content (`model_stamp`, renewed by every model mutation in `push_undo` and carried by
every snapshot); `restore` regenerates, and bumps `revision`, only when the snapshot's
stamp differs from the current one. A paint is recorded by `record_appearance_undo`,
which leaves stamp and revision alone, so a stress plot stays fresh and a git comparison
is not recomputed across a change of colour. A run of edits to one appearance's
definition with nothing in between is one entry, so a roughness slider dragged across a
hundred frames undoes in one step. The scene settings (environment, brightness,
background, floor, depth of field) are saved but outside undo, as visibility is: dragging
the brightness is looking at the part, not changing it. `appearance_revision` counts both,
for whoever caches a picture of the model. Format 12 added all of it with an identity
migration.

### Environments

Fusion lights renders with photographed HDR panoramas; those would be the only binary
assets in the repository, so environments are described instead: a sky that is a gradient
from horizon to zenith and from horizon to nadir (`t = √|z|`, so the horizon band is
narrow), a few distant lights that are discs of uniform radiance (soft boxes a few tens of
degrees across, a sun a degree and a half), and a floor albedo. That is most of what makes a
product shot — the boxes' long reflections along a polished edge, the gradient in a curved
face — and it is exactly what both renderers can evaluate: the tracer samples the discs
directly and meets the sky by escaping into it, and the viewport shades the same gradient
and discs in closed form, so a highlight in the preview is where the render puts it. A
disc's irradiance on a facing surface is `L·π·sin²θ` in both.

### The path tracer

`basset-render` traces on the CPU, as the FEA solver solves on it: the GPU belongs to the
viewport, and a CPU tracer is the one that runs headless, in the MCP server and in tests.
A binned-SAH BVH (children side by side, iterative traversal) over `f64` triangles; a
unidirectional path tracer with next-event estimation towards the environment's discs,
combined with BSDF sampling by the power heuristic; the opaque lobes — Lambert weighted by
one minus metallic, GGX sampled through its visible normals, a smooth clear coat over both
— sampled and evaluated as one mixture so their combined density is what MIS sees; and,
with probability `transmission`, a rough dielectric interface treated as a delta for MIS.
Shadow rays pass through transmissive surfaces tinted rather than refracted, so glass casts
a tinted shadow instead of needing caustics. Fireflies are clamped per sample.

The floor is a shadow catcher, because Fusion's is invisible except for what the model
does to it. A camera ray that lands on it is shaded twice in one go — with the model in
the world and with only the environment — and the pixel keeps the background scaled by the
ratio of the two sums over all its samples (a ratio of sums converges; a mean of noisy
per-sample ratios would not). Under the model the ratio drops (the shadow), where the model
reflects in a glossy floor it moves, and far away the two agree and the floor vanishes into
the background with no fade to tune. Secondary rays meet the floor as a real surface, so a
chrome part reflects a floor and the floor bounces light into its underside. A solid
background is stored as the radiance the tone curve maps to the picked colour
(`aces_inverse`), so it comes out exactly as picked. Everything ends in ACES (Narkowicz's
fit) and sRGB.

Rendering is progressive (`RenderJob`): passes of one sample per pixel on scoped workers
taking rows from a shared iterator, accumulated, tone mapped and published after every
early pass and a few times a second after that; cancelling is a flag checked per row, so a
camera drag that restarts the in-canvas render every frame does not queue passes. Seeds
come from the pixel and the pass, so a render is the same image every time.

### The Render workspace

`editor::render` is the third workspace. Its click is the appearance *brush*: a swatch
clicked in the library arms it, a face clicked in the viewport is painted (or its body, by
the panel's Apply to), Escape puts it down; without a brush a click selects. It shares the
Simulation workspace's gate — modelling commands are refused by `Workspace::models` on
every path a command arrives by — and its viewport is the picture: environment lighting
(`Scene::lighting`), the sky or the solid background, no grid, planes, sketches or edges.
Appearances draw in Design too, under the studio lighting with their metallic, roughness
and clear coat modulating it (the default material reproduces the old shading bit for
bit); Simulation keeps the plain grey, where a face's colour is what the study says. A body
with face overrides is several instances of one mesh, each masked to its faces by a face
bitset beside the highlight bitset (`MeshInstance::face_mask`), only the first carrying the
edges; glass is `MeshStyle::Translucent`, blended at its own alpha.

The in-canvas render is a `RenderJob` at a fraction of the window's resolution (the
quality), restarted whenever its `CanvasKey` — camera, window, quality, model revision,
appearance revision, hidden bodies — changes, and painted under the panels over the whole
window, the same pixels the camera is measured in. Until the restarted job's first pass
arrives the raster preview shows through, so orbiting stays fluid. The traced scene is
cached under the key minus the camera, since the camera is what moves most and the BVH is
the costly part of a restart. Final renders are jobs of their own into a gallery that
belongs to the document, shown along the bottom in place of the timeline, opened in a
viewer and saved as PNG. The `snapshot_of_the_render_viewport` test (ignored; set
`BASSET_SNAPSHOT=out.png`) draws the workspace's raster preview through the real renderer
on a headless GPU.

## Driving the modeller from an agent

`basset-mcp` is a Model Context Protocol server: JSON-RPC over stdin and stdout, one
message per line, implemented directly on `serde_json` because the protocol a tool
server needs — `initialize`, `tools/list`, `tools/call`, `ping` — is a few hundred lines
and an SDK would be the only dependency the workspace could not build offline. It holds
one `Document` and exposes the operations the editor's dialogs perform: a sketch is a
feature holding a `Sketch`, drawn into by a batch of operations applied to a copy and
written back only if every one succeeds; an extrude names regions by sample point and
curve signature exactly as a click would; `body_info` and `check_document` return what
the regenerator and the kernel can say about the result — statuses, volumes, bounding
boxes, face and edge keys, and whether every shell is closed — so a failure that the
viewport would show as a yellow badge comes back as data; `fea_static` runs a study on a
body by face keys and reports the maxima, the reaction, the mass and the safety factor
against yield, with the material named from the library `fea_materials` lists, and
`fea_topology` optimises the layout of a given fraction of the body's material under the
same study. `set_appearance`, `define_appearance` and `scene_settings` paint and light the
model as the Render workspace does, and `render_image` path traces it to a PNG framed from
a named view, because a file is the only way an agent can look at the picture. Entities and constraints are
named by their slot index, resolved against the live sketch, since a wire id is meant to
be read back and quoted; faces are `feature.sub:Role` and edges two of those joined by
`|`, the parts a `FaceKey` is made of. `.mcp.json` at the workspace root registers the
server for Claude Code.

`testcases/library/` is a library of test geometries in that protocol: each file is a
script of tool calls that builds a model and the volumes, bounding boxes, region counts
and statuses it must come out with. `make geometries` runs them (it is an ordinary
`cargo test` of the `basset-mcp` crate). They are regression tests for the sketch and
extrude fundamentals — every extent kind, every origin plane, regions that touch, cross,
nest and overlap, booleans on faces, to-face extents, blends, parameters, and a sketch
re-dimensioned under an extrude — and, being plain JSON, they double as worked examples
of driving the modeller. A `$N` in a script's arguments is the feature call N made, and
`${N}` inside a string is substituted as text, which is how a face key quotes the body
it belongs to.

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
