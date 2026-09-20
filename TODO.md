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

- Clicking on tool symbol doesn't select it (in the variant dropdown), only the text (IMPORTANT!) - this is bad and misleading, make it so the tool symbol can be clicked to change tool as well.
- Patterns don't make sense. Make it closer to fusion.
- Badge placement is naive: several constraints on one entity stack outwards from its
  midpoint, which is enough for a tidy sketch and will collide on a dense one. No
  decluttering, and badges do not dodge the geometry or each other
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
- The geometry an extrude creates is clean: what the user sees as an edge comes from
  `Solid::display_edges` (topology) rather than from the triangles, so faces read as flat
  shapes and arcs as curves, and two faces on one surface — a sketch line cut in two —
  meet at a smooth edge that is neither drawn nor pickable. Generators heal and validate
  what they return, so a profile that doubles back on itself is repaired or fails its own
  feature instead of seeding a leaking shell
- Booleans still heal without validating, so a leak from the BSP splitter is now neither
  drawn nor reported (it used to show as stray triangle edges). Validating there would
  fail features that presently work, so it wants measuring before it is turned on
- Still open there: a silhouette is not drawn, so a cylinder standing against the
  background is bounded only by its shading; and `DISPLAY_CREASE_COS` (45°) is shared with
  the shading cut-off, which is right for a fold but arbitrary for a tangent edge, where
  a fillet meets the face it blends into

Fillet tool:

- Fillet arrow should go other way around.
- Multiple edges select reliably: picking runs against the state *before* the running
  feature (`Editor::refresh_pick_bodies` via `Document::state_before`), so a second pick
  lands on the unfilleted body while the preview shows. Covered by
  `fillet_picks_faces_as_edge_rings_and_edges_of_the_unfilleted_body`
- Slow for some geometries, needs optimisation
- Where several blended edges meet, the result is the intersection of their tools rather
  than a corner patch, and a radius larger than the neighbouring face is not detected

Export:

"Error 330 non manifold edges" - fix?
