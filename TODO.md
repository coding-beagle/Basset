A list of features to implement / make better, grouped by section

Sketch constraints:

Constraints are now visible: geometric ones are drawn as amber badges beside the geometry
they hold (selectable, with a delete in their context menu), everything the solver leaves
free is drawn in blue, the palette lists every constraint with hover-to-highlight, and a
loose sketch that a later feature builds from warns from the timeline and the status bar.
`SolveReport::under_constrained` is where the naming comes from: the null space of the
hard Jacobian, per parameter. A sketch that will not solve now names the constraints that
disagree rather than quoting a residual: `SolveError::DidNotConverge::conflicting` lists
them worst first (the per-equation residual, attributed back to the constraint that
compiled it), the palette offers each one for deletion, and their badges, leader lines and
value boxes are drawn red.

What is left here:

- Badge placement declutters: greedy ranked-slot placement in pixels on the sketch plane,
  five rings by eight directions about the entity's normal, ranked so that half a turn
  costs about one ring — a badge crosses to the far side of its line before walking out
  along the near one. Obstacles (curves, points, other badges, dimension value boxes) live
  in a hash grid, and a badge more than 22 px from its anchor grows a leader. Stability is
  ranked above packing: offsets are stored in pixels so a pan or zoom rescales the overlay
  without moving anything, and each badge is re-offered the slot it held last. Still open:
  leader lines are not themselves obstacles, so a long leader can cross a curve, and the
  collision space is the sketch plane rather than the true screen projection, so an
  orbited camera's foreshortening is ignored (the same approximation the sizing already
  made)
- Redundant constraints are now named: `SolveReport::redundant` lists the constraints
  whose every equation is linearly dependent, at the solution, on the equations ahead of
  them while the system is satisfied — deleting one changes neither the geometry nor the
  degrees of freedom. Distinct from conflicting, which is inconsistency rather than
  dependence and so only ever appears on a failed solve. The later of two interchangeable
  constraints is the one named, because implied equations go into the basis first. The
  editor draws them orange (badge, leader, value box) and the palette counts them and
  lists each with hover-to-highlight and a delete, the way it does conflicts. Also
  populated on every drag frame, and the grouped Gram–Schmidt behind it is
  O(rows²·cols), worth revisiting before sketches grow
- The degrees-of-freedom estimate is instantaneous (first order), so a point pinned only
  at second order — a zero-length distance, or two distance dimensions with the point
  exactly between them — is drawn blue although it cannot move. The fix is not a better
  tolerance: for each null-space direction `v` of `J_hard` the sketch is genuinely free
  along `v` only if the curvature term vanishes too, i.e. `vᵀ(∂²rₖ/∂x²)v == 0` for every
  hard equation `k`, which is exactly what those two cases fail. The Hessians are
  reachable — `dual::Dual` carrying second derivatives, or differencing the existing exact
  Jacobian along `v` at one extra `evaluate` per null direction, which is the cheap way.
  The hard part is not the test but the conclusion: a direction killed at second order
  still moves under the *solver*, so attribution needs a third state between "cannot move"
  and "can only move if something else moves first", not a yes/no. Until then the colour
  over-reports freedom, which is the safe direction
- `FREE_TOL`: done. `Mat::freedom` and `dependent_rows` equilibrate the Jacobian's columns
  to unit norm first, so full pivoting follows dependence rather than whichever feature is
  drawn largest and the tolerance thresholds a scale-invariant quantity. Regression test
  `freedom_is_reported_across_feature_sizes_that_span_a_thousandfold` (a 3000 mm lever
  driving a 1 mm detail) fails without it
- The badge layout is cached, keyed on a hash of point positions, radii, constraint ids,
  references and kinds, dimension label positions, the conflicting list and the view scale
  quantised to four steps per e-fold. A revision counter would be cheaper but would go
  stale, because `sketch` is a public field the panels, tools and tests all write to
  directly. Badges are hit-tested in `pointer_moved` rather than through one egui area
  each, which is also what lets a hovered badge light its geometry and the reverse.
  Measured at 319 constraints / 637 badges: laid out once, 200 further overlay reads
  from the cache

Sketch:

- Snapping infers what the drawing names — endpoints, midpoints, centres, crossings, the
  point on a curve, the origin, horizontal/vertical alignment with a touched point, a
  line's extension, tangent and perpendicular off the curve a chain continues from — under
  one ranking and one hold, both pure functions in `editor/snap.rs` and tested without a
  window. What is left there: the guides come from the last three points the pointer
  touched and from the origin, so an alignment with a point further back in the drawing
  has to be re-hovered first; a guide's dashes are drawn on the sketch plane rather than in
  screen space, so an orbited camera foreshortens them; there is no text beside the
  crosshair naming the snap, only the glyph, because sketch mode has no egui overlay of
  its own to put one in; crossings are found among the eight curves nearest the pointer,
  which is a cap rather than an index; and nothing can be *locked* — Fusion lets a guide
  be pinned by hovering it, and here it is only ever held.
- Off plane sketch faces are unselectable outside of the sketch.
- DXF import
- Trim/break: no Extend (dragging a curve out to meet another one) yet
- Sketch patterns are plain copies: there is no pattern entity to re-generate from, so
  editing the seed does not update the copies

Parameters:

Parameters are document-wide. A `.bass` file carries a table of named expressions beside
its timeline, and a name is resolved in two scopes: a sketch's own table first, the
document's behind it, so a sketch parameter shadows a document one of the same name.
That ordering is why lifting them up needed no migration — every sketch that already had
a `width` still means its own — and the format version went 2 to 3 with an identity
migration all the same, so that an older build refuses a file carrying parameters rather
than dropping them on the next save. `basset-sketch` still knows nothing about documents:
the outer table reaches it as a closure (`basset_sketch::Outer`), the same seam
`expr::eval` already had for names, carried one level up.

Features read the table too. `Feature::exprs` is a `BTreeMap<NumericField, String>`
beside the kind, so an extrude's distance and second distance, a revolve's angle, a
fillet's radius, a chamfer's distance, an offset plane's distance and an angled plane's
angle can each be driven. Replay resolves the text into a copy of the feature, never into
the timeline, and an expression that stops evaluating leaves the last number standing and
warns rather than failing the feature. The accessors work in the unit the user types —
degrees for angles, where the model holds radians — and unlike the sketch's angle
dimension the sign comes from the expression, because a feature angle's sign is a
direction the user chose rather than a solver branch. The expression language grew `^`
(right-associative), exponent literals, `pi` and `tau` as fallbacks a user parameter still
beats, and `sqrt abs floor ceil round sin cos tan asin acos atan atan2 hypot min max deg
rad`. Renaming rewrites every expression that mentions the name — document rows, feature
expressions, and every sketch's table and bound dimensions — and refuses a rename that
would capture references onto a narrower scope's row. Undo covers the table, and a change
to it invalidates regeneration from feature zero. In the app: a Parameters section in the
browser, an `ƒ` toggle on every feature-dialog number that goes through the shared
`drag` helper, the document's rows listed read-only in the sketch palette with the
shadowed ones struck through, and a delete that says what still reads the name.

What is left here:

- `Move`'s six translate and rotate components, and the whole sketch-operation dialog
  (sketch move, pattern, offset, fillet), still take plain numbers. They bypass the shared
  `drag` helper the `ƒ` toggle lives in, and each would need a field identity — the
  equivalent of a `NumericField` — before a toggle could be hung on it
- The sketch's own parameter panel has no rename, only add, re-express and delete, so
  `Sketch::rename_parameter_with` exists and is unused from the app
- Document parameters cannot be edited while a sketch or a tool dialog is open. Both hold
  an open document transaction, and an edit made inside one would be rolled back by a
  Cancel that had nothing to do with it. Lifting the restriction means sorting out the
  transaction story — a nested or independently committed edit — not adding more UI
- `Parameters::resolve`, and the identically shaped `Sketch::resolve`, clone the name
  stack for every reference they follow, so a diamond reference graph costs time
  exponential in its depth: thirty chained rows each mentioning the previous one twice is
  2³⁰ evaluations, enough to wedge a `set`. It always terminates, and at the handful of
  rows a document has it is free. The fix is to pass one stack down and pop on the way
  out, in both places at once so the two stay legible as the same algorithm
- `flag_sketch_faults` re-evaluates every bound dimension of every sketch on every
  `evaluate_prefix`, including on a frame where nothing was regenerated at all. Caching it
  needs an invalidation key over the document table and each sketch's bindings, which is
  the awkward part: the pass is recomputed from scratch precisely so that restoring a
  parameter, or deleting the feature that consumed a sketch, takes the warning away again
- `min` and `max` inherit `f64`'s NaN-dropping, so `max(sqrt(-1), 5)` is 5 rather than
  being refused. The finiteness check at the end of `eval` never sees the NaN, because the
  comparison already discarded it
- `referenced_names` cannot tell a user parameter named `pi` or `tau` from the constant —
  it has no lookup closure — so it leaves both out, and deleting such a parameter gives no
  "still in use" warning. Evaluation still prefers the parameter, and the cycle check does
  not rely on the list to terminate
- A sketch parameter written over a document one of the same name cannot refer outward to
  it: `width = width * 2` is a cycle, not a reference. Resolution is by name and there is
  no syntax for saying which scope is meant

Extrude tool:

- Multi-body targets: done — Join/Cut/Intersect carry a list of target bodies
  (`BodyOp::Cut(Vec<BodyRef>)` and friends, file format v5 wraps a v4 single target into
  a one-element list), the auto-target defaults to every body the extrusion's swept box
  touches, and the dialog shows the targets as a checklist. A body the tool never
  reaches still takes the boolean, as Fusion applies it: a missed cut is a no-op, a
  missed join keeps the tool as a second disjoint shell of that body. Still open: a
  join that bridges two targets does not merge them into one body the way Fusion's
  does — each target is unioned with the tool separately and they stay separate bodies;
  merging is `Combine`'s job for now. Clicking bodies in the viewport does not toggle
  the checklist either — the extrude tool's viewport picks mean regions, and
  overloading them was judged more surprising than the checklist
- "To face" extent: done for planar targets — the target is stored as the same `FaceRef`
  a sketch-on-face uses, resolved fresh on every replay, and a planar target is treated
  as its infinite plane. A plane parallel to the profile's is one exact distance; a
  tilted one is overshot and trimmed back with a boolean so the end lies on the plane
  exactly. Parallel-to-direction, behind-the-profile and never-reached targets are
  refused with per-feature errors rather than approximated
- Still open there: a *curved* target ends the extrusion flat, at the first contact —
  rays from the profile's boundary and an interior grid are cast against the target's
  facets and the shortest hit is the reach, so the result touches the surface without
  piercing it but does not wrap it. The honest end is the surface itself: extrude past
  the farthest hit and trim with the target's body, which the kernel's booleans could do
  but which grazes tangentially at the silhouette, exactly where a BSP boolean is at its
  flakiest. Also, while the extent is armed but no face is clicked yet, the preview
  keeps the last extent's shape (OK is disabled until a face is chosen, so nothing wrong
  can be committed)
- Silhouettes are drawn: `basset-viewport`'s `silhouette` module takes an edge to be on the
  outline when one of the two facets sharing it faces the camera and the other does not,
  which is view-dependent and so recomputed per camera change rather than baked at
  tessellation time. Adjacency is welded and listed once per mesh upload; per frame only
  one dot product per facet and one sign comparison per edge run, and the result is cached
  per drawn instance against the view that produced it. Perspective judges each facet
  against the direction from the eye to that facet, orthographic against the one view
  direction — an orthographic camera stood in for by an eye a long way off puts the outline
  of a large body visibly off its edge. Measured on a ball of 57 600 facets / 86 160
  adjacency records: 77 ms to build the adjacency, 82 µs per frame while the camera moves,
  0.4 µs while it is still; a body at the default tessellation (2 876 facets) costs 6 µs
  per moving frame
- The drawn-edge cut-off is now `TANGENT_EDGE_COS` (20°), separate from the shading crease
  `DISPLAY_CREASE_COS` (45°): two different surfaces meeting closer to tangent than that
  get no line, so a fillet no longer comes out ringed like a chamfer. `Edge::smooth`
  follows the same test, so a tangent boundary is not offered as a fillet target either
- Still open there: the adjacency is rebuilt on every upload of a body, which is a second
  weld of a mesh the kernel has already welded in `display_edges` — the 77 ms above lands
  on every regeneration of a dense body and would be better shared. The 20° is a fixed
  number, so a blend coarsened to the tool budget's two facets per arc (45° per facet) is
  still drawn as if it folded; the honest test compares the fold across the boundary with
  the folds *inside* the adjacent face, which is adaptive and needs no constant. And a
  silhouette edge that is also a drawn feature edge is drawn twice, over the same pixels:
  harmless, one wasted instance per such edge on a cube-shaped body

Fillet tool:

- The boolean's cost is measured, and bounded by time rather than by the machine. The
  BSP tree over a convex sweep (a rim tool, or the cylinder it sits on) is a list, which
  is the shape and not a poor choice of plane, so each boolean is quadratic in its facet
  count. The walks no longer recurse (the stack used to be the hard ceiling, at ~4.6k
  facets), polygons move through the tree rather than being copied at every level, and a
  polygon clear of the other solid's bounding box skips its tree; both rims of a
  cylinder now run 0.18 s at 2.6k facets, 0.5 s at 4.5k, 3.7 s at 9k and 105 s at 27k,
  at 106 MB peak where 25 GB used to be. `MAX_FEATURE_POLYGONS` is the choice of how
  long a preview may take (about a second), shared between the tools, and spent by
  coarsening the arcs before anything is refused. Still open: the asymptotic fix is a
  splitting plane that is not a face plane (see the `csg` module header for why the
  plain version costs more than it saves), and the whole feature is one thread, so a
  refused radius is instant but an accepted one still blocks the frame
- At a half-degree segment angle and finer the result leaks — 8 unmatched edges at
  0.5°, 4221 at 0.25° — on the old splitter as much as the new, so it is the healer's
  tolerance against facets tens of microns wide rather than the tree. Out of reach of the
  budget today, but a body tessellated that finely elsewhere would hit it
- Where several blended edges meet, the result is the intersection of their tools rather
  than a corner patch
- The size limit (`blend::size_limit`) now samples several points along each edge
  segment, so a tapered face is measured at its narrow end; bounds a doubly-curved
  neighbour by the nearest *folding* boundary rather than the face's bounding box; and
  ray-casts a five-ray fan from convex edges into the whole solid, so a cavity or thin
  wall behind the faces caps the radius. What is still loose, and what is now too tight:
  - The curved-face bound treats the tool as if it lay on the surface, but the tool is
    prismatic and straight while the surface curves away from it, so on a strongly
    curved band the honest bound is neither the surface reach nor the boundary
    distance; the interior fan is what actually catches the tool leaving the material,
    and it only samples five directions per point
  - That fan can pass either side of a small cavity corner that sits between two rays;
    it is answered by bounding the whole removed cross-section by its widest reach,
    which is conservative on the diagonal, and a cavity far behind the middle of a face
    is still only seen once a ray from the edge reaches it
  - The conservative fallback bounds reach by *distance* to a folding boundary even
    when that boundary lies beside the ray rather than ahead of it, so an edge near a
    cut corner on a curved band gets a much smaller limit than the material warrants
  - Concave edges get no interior bound: the tool adds material, so the hazard is not
    breaking through a wall but bridging a gap, and only the in-face reach guards that
  - Filleting an edge a boolean left across a fillet band leaks at some radii well
    inside any honest limit (0.1 on the half-cylinder fixture, where 0.05 and 0.25
    close): the healer against the band's mitred facets, not the limit, which is why
    `an_edge_across_a_curved_band…` asserts closure at a small radius rather than at
    the limit
- `TANGENT_EDGE_COS` (20°, `solid.rs`) also decides what counts as a *wall* for the
  curved-face fallback here, the same threshold that keeps near-tangent boundaries
  undrawn. The flip side stands: an edge whose faces genuinely fold by less than 20°
  gets no line, so there is nothing on screen to aim a fillet at, and a deliberately
  shallow edge is unpickable in practice. A deliberate trade — a blend coarsened to
  the tool budget's 45°-per-facet arcs must not come out ringed like a chamfer —
  recorded here rather than changed

Keyboard:

Commands are declared once, in `crates/basset-app/src/editor/commands.rs`: an id, a
label, the chords that run it, the mode it is live in and a test for whether it would do
anything now. `on_key` is a lookup into that table and runs the same `Command` a toolbar
button queues, the menus and tooltips print their key from it, `?` / `F1` lists it
filtered to the current mode, and `Ctrl+P` is a fuzzy search over it.

What is left here:

- Bindings are not user-configurable: the table is `const`, so re-binding means editing
  it and rebuilding. A keymap file read at startup, overlaid on the table, is the shape
  of the fix; the conflict check that the tests do over the table would have to move to
  runtime and say which of the two it dropped
- Pattern, Text and Finish Sketch have no key. The plain letters they would want are
  taken by shapes, and a second letter of the same word is worse than the palette
- Chords are one key with modifiers. Fusion's two-key sequences (`G` then a letter) have
  no home in `Chord`, which would need a pending-prefix state in the editor
- The palette matches on the label and the id with a subsequence score. It has no memory
  of what was picked last, so the common command does not rise to the top of a query that
  matches several
- Keys are printed in the sketch toolbar's tooltips rather than in its button names,
  because that row already wraps to three lines on an 800 px window and a fourth came
  off the sketch palette beside it — far enough to push the redundant-constraint section
  below the fold. The palette scrolls, so nothing was unreachable, but a report nobody
  scrolls to is a report nobody reads. The real fix is for the palette's warnings not to
  sit at the bottom of a long scroll in the first place
- `enabled` is shown, by dimming, but not enforced on a keystroke: a key whose command
  cannot act runs it anyway and the command says why. That is deliberate — the messages
  that say what to select first are worth more than a dead key — but it means the overlay
  can dim a row whose key still does something visible (a status line)
