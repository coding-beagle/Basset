A list of features to implement / make better, grouped by section

Sketch constraints:

Constraints are now visible: geometric ones are drawn as amber badges beside the geometry
they hold (selectable, with a delete in their context menu), everything the solver leaves
free is drawn in blue, the palette lists every constraint with hover-to-highlight, and a
loose sketch that a later feature builds from warns from the timeline and the status bar.
`SolveReport::under_constrained` is where the naming comes from: the null space of the
hard Jacobian, per parameter.

What is left here:

- Badge placement is naive: several constraints on one entity stack outwards from its
  midpoint, which is enough for a tidy sketch and will collide on a dense one. No
  decluttering, and badges do not dodge the geometry or each other
- A conflicting sketch says "Constraints conflict" without naming the constraints that
  disagree. The solver knows the residual per equation, so the worst offenders could be
  listed the way the free parameters now are
- Nothing distinguishes a *redundant* constraint from a driving one, so a sketch can be
  quietly over-constrained but consistent
- The degrees-of-freedom estimate is instantaneous (first order), so a point pinned only
  at second order — a zero-length distance, or two distance dimensions with the point
  exactly between them — is drawn blue although it cannot move. Erring this way is the
  safe direction for a warning, but it is a lie in the colour
- `FREE_TOL` in linalg.rs is an absolute threshold on a ratio, so a sketch whose feature
  sizes span more than ~1e3 can under-report a genuinely free unknown. Fixing it properly
  means scaling the Jacobian's columns, not moving the tolerance
- Badges and the constraint overlay are rebuilt every frame, one egui area per badge. Fine
  for the sketches drawn so far; a few hundred constraints would want caching

Sketch:

- Profile tracer near-miss T-junctions are healed (`t_junctions` in profiles.rs splits a
  curve where another curve's endpoint lands within `JOIN_TOL` of its interior, the 2D
  equivalent of `Solid::heal`). testcases/crashes_when_sketch_changes_propagate.bass is
  the regression test. Still exact rather than tolerant: `segment_crossing`'s `0..=1`
  test, which is why the healing pass exists alongside it rather than instead of it
- DXF import
- Trim/break: no Extend (dragging a curve out to meet another one) yet
- Sketch patterns are plain copies: there is no pattern entity to re-generate from, so
  editing the seed does not update the copies
- Parameters live on the sketch; document-wide parameters shared between sketches and
  feature dimensions are the next step

Extrude tool:

- Draggable arrow in the viewport for the distance: done (`tools::handle`), and the dialog
  has the distance entry box beside it
- Extrudes need to clean up the geometry that they create.

Filet tool:

- Multiple edges select reliably; picking works against the unfilleted body while the preview shows (NOT DONE)
  - Are edges not automatically created when we create geometry?
- Slow for some geometries, needs optimisation
