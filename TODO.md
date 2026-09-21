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
  constraints is the one named, because implied equations go into the basis first.
  **No UI yet: nothing draws or lists them.** Also populated on every drag frame, and the
  grouped Gram–Schmidt behind it is O(rows²·cols), worth revisiting before sketches grow
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

- Off plane sketch faces are unselectable outside of the sketch.
- DXF import
- Trim/break: no Extend (dragging a curve out to meet another one) yet
- Sketch patterns are plain copies: there is no pattern entity to re-generate from, so
  editing the seed does not update the copies
- Parameters live on the sketch; document-wide parameters shared between sketches and
  feature dimensions are the next step

Extrude tool:

- Still open there: a silhouette is not drawn, so a cylinder standing against the
  background is bounded only by its shading; and `DISPLAY_CREASE_COS` (45°) is shared with
  the shading cut-off, which is right for a fold but arbitrary for a tangent edge, where
  a fillet meets the face it blends into

Fillet tool:

- Still needs optimisation fixes
- Slow for some geometries, needs optimisation. Worse than slow on a finely tessellated
  one: filleting both rims of a cylinder built at `chord_tolerance` 1e-4 and a 2° segment
  angle allocated ~25 GB before the OOM killer took the whole session with it, so the
  growth in the BSP boolean is superlinear in the tool's polygon count rather than merely
  steep. The fillet is applied as one boolean per edge chain against the accumulating
  result, so each rim's tool is split against every fragment the previous one made. Wants
  measuring — polygon count per boolean, against tessellation density — before it is
  optimised, and a guard that refuses or coarsens a blend whose tool would exceed some
  polygon budget, because a modeller must not be able to exhaust the machine from a
  radius box. Run `cargo test` under a memory cap (`systemd-run --user --scope -p
  MemoryMax=12G`) while this stands
- Where several blended edges meet, the result is the intersection of their tools rather
  than a corner patch, and a radius larger than the neighbouring face is not detected

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
- `enabled` is shown, by dimming, but not enforced on a keystroke: a key whose command
  cannot act runs it anyway and the command says why. That is deliberate — the messages
  that say what to select first are worth more than a dead key — but it means the overlay
  can dim a row whose key still does something visible (a status line)
