# Basset

A fast, parametric, history-based 3D solid modeller written in Rust. Linux first.

The goal is feature parity with Fusion 360. The way there is deliberately narrow: few
tools, each finished, tested, and built on an object model that will not need to be
rewritten when the next tool arrives.

## Status: bootstrap / MVP

| Area | What exists |
| --- | --- |
| Object model | Document, Components, origin & construction Planes/Axes, Sketches, Bodies |
| Sketching | points, lines, arcs, circles, construction geometry, text; rectangles (2-point, centre), circles (centre, 2-point, 3-point), polygons, slots, arcs; geometric constraints and driving dimensions; Levenberg–Marquardt solver that names both the loose geometry and, when a sketch will not solve, the constraints that disagree; selection / hit testing including box select; grid snapping; closed-region detection with curves split at their crossings; trim and break; rectangular and circular patterns with a live preview; offset with rounded or squared corners; named parameters driving dimensions |
| Solids | extrude (one side / symmetric / two sides), revolve, sweep, loft, from a sketch region or a planar face; join / cut / intersect; fillet, chamfer, combine, move |
| Construction | offset plane, plane at an angle, sketch on a planar face |
| Timeline | insert at cursor, edit, suppress, reorder, delete, roll back / forward; edits replay forward with per-feature caching; per-feature failure reporting |
| Files | `.bass` documents (versioned JSON); STL and 3MF export of bodies and components |
| Viewport | wgpu renderer with orbit camera, MSAA, pixel-width lines, view-dependent silhouettes so a curved body is bounded against the background, a grid on any plane, face highlighting, translucent region fills, ray picking |
| App | winit + egui desktop shell: browser, timeline with rollback marker and context menu, live-preview tool dialogs, viewport transform manipulator (arrows and rotation rings) for sketch and body moves, navigation cube, sketch mode with shape tools, constraint tools and click-to-edit dimensions, native open/save/export dialogs |

See [ARCHITECTURE.md](ARCHITECTURE.md) for the crate layout, identity strategy and the
honest list of kernel limitations.

## Building

```sh
cargo build --release
cargo run --release -p basset-app            # optionally: -- path/to/model.bass
cargo test --workspace                       # includes headless tests of the app
cargo clippy --workspace --all-targets -- -D warnings
```

## Using the application

* **Navigate**: right-drag orbits, middle-drag pans (shift+middle orbits), wheel zooms,
  `F` fits the model. The View menu has the standard presets and an orthographic toggle.
  The navigation cube in the top right shows the current orientation: click a face, edge
  or corner of it for that view, drag it to orbit, or press its ⌂ for isometric.
* **Sketch**: Create → Sketch, click an origin plane (enable them in the browser) or a
  planar face of a body. The sketch tools sit in the toolbar at the top as icons; a
  shape with several ways to draw it (rectangle, circle, arc, slot) has one button that
  shows the kind used last, and holding it or right-clicking lists the others. Click
  points; clicks snap to existing points, which is how loops close. Right-click or Esc
  ends a line chain. Points that snap to nothing land on the grid, which is drawn on the
  sketch plane; the palette turns snapping off or pins the increment, and **holding shift
  lets go of the grid** for as long as it is down — for the one point that has to land
  between the lines, which is far commoner than wanting no grid at all. Shift frees the
  manipulators too, not just the drawing. Snapping to an existing *point* is never given
  up: it is how geometry gets joined, and shift is for escaping the grid rather than for
  drawing something that only looks attached. After a shape's
  first click, type its sizes (length, width and height, diameter…) in the entry boxes
  that appear, Tab between them and press Enter to place it; a typed size pins the
  preview while the pointer picks the direction, and becomes a driving dimension. With
  the Select tool, click geometry (shift-click for several), click inside a closed region
  to take the curves around it — the area itself fills so you can see what you are about
  to take — or drag a box (rightwards encloses, leftwards also takes what it touches);
  drag a point, a curve or the selection to move it under its constraints. The toolbar's
  Select buttons (keys `1`-`4`) say what a click or a box may take — all of it, curves
  only, points only, or the closed regions — because a corner, the curves meeting there
  and the area beyond them all sit within a few pixels of one another; narrowing the
  filter lets go of whatever it no longer covers. Drawing ignores it, so a line still
  snaps to a point while the filter says Curves. A point next
  to a curve wins the click over the curve running through it, so endpoints and centres
  are the easy things to hit. Each constraint in the toolbar is a tool: click it and then
  pick the geometry, in whatever order reads naturally, and it goes on as soon as the
  picks support it, the tool staying armed for the next pair; picking the geometry first
  still works, in which case the button applies it at once; the lit button puts the tool
  down again, as does Esc, and a pick the constraint could never use is refused with a
  note rather than swallowed. Equal, Parallel and Concentric chain: each pick after the
  first ties onto the one before, so five equal holes are five clicks. The toolbar's
  Construction button (`X`) reads what you have: with geometry selected it converts that
  geometry, and with nothing selected it arms the mode so the next shape is drawn as
  reference geometry, its preview dashed while you aim it. The Dimension
  tool reads what you pick, as in Fusion: a line, circle or arc followed by a click on
  empty space dimensions its length, diameter or radius; two parallel lines give their
  distance and two other lines their angle; a point or a circle's centre with a line
  gives their distance. Dimensions are drawn as in a drawing, with extension lines,
  arrowheads and leaders; drag the value to place one, click it to change it — with a
  number, or with an expression such as `bore / 2`, which binds it to the sketch's named
  parameters (the palette's Parameters section adds those, and a driven dimension is
  drawn with an ƒ). If the sketch cannot be solved, the constraints that disagree turn red
  on the drawing and the palette lists them with a delete beside each, rather than
  reporting a residual. Trim takes the piece of a curve you click, cut at the curves that
  cross it, and draws that piece in red before you commit to it; Break cuts a curve at
  its crossings and keeps everything. Move (or `M`) moves the selection: arrows and a
  rotation ring appear on it in the viewport to drag, snapping to the grid and to whole
  steps of angle, and entry boxes open on it at once — type 50, Tab, Enter, exactly as a
  shape's sizes are typed. Enter applies, Esc puts it back. Pattern repeats the
  selection in a grid or around a centre, copies keeping the constraints the seed was
  drawn with, except the ones that describe the sketch's axes rather than the shape — a
  turned copy has horizontal and vertical the other way round, and an odd angle has
  neither, so those are swapped or dropped rather than copied into a contradiction. The
  copies appear as you set the numbers, drawn as a preview, so a distance stated between
  copies or across the whole span is something you can see before OK keeps it; Select
  origin then places a circular pattern's centre with a click, snapping to a point if one
  is under the pointer. Instance counts are not capped — a bolt circle of two hundred is
  an ordinary thing to ask for — only the total work is, and a pattern that would exceed
  it says so. Offset (or `O`) draws a second chain of curves alongside the selection at a
  fixed distance, and the two corner styles are the two different drawings people mean by
  the word: rounded corners keep every *point* of the result the distance from what you
  drew, so an outward offset of a rectangle comes out with radiused corners, while square
  corners keep every *edge* that far from its own edge and run the edges out to meet, so
  an offset rectangle is still a rectangle. The distance is dragged by a handle on the
  result itself, so it is set by looking at the drawing rather than at a field in a
  panel; dragging back across the geometry and out the far side puts the offset on that
  side, which is the whole answer to "which way round is this" on an open path. It snaps
  to the grid, shift frees it, and the number in the palette reads back whatever the drag
  is showing. The result is new geometry rather than a linked copy, but it carries what is true
  of it — each edge parallel to its own edge, each arc concentric with its own arc, each
  rounded corner centred on the corner it rounds — so it comes out driven rather than a
  loose pile of blue. An offset larger than the shape can carry, or a corner too sharp to
  square off, is refused and said so instead of leaving a knot of crossed lines that looks
  like geometry. While any of the three is up the sketch belongs to it: geometry
  underneath cannot be dragged away, and Ctrl+Z puts the operation back the way Esc does.
  A move is rigid, so if the constraints will not take it — asking a rectangle held to the
  axes to turn — it is refused and said so, rather than the solver quietly finding some
  other shape that satisfies them.
  `E` extrudes the region under the pointer (or the ones you clicked inside):
  it finishes the sketch and opens Extrude with those regions already chosen. Finish
  Sketch commits.
* **Model**: the toolbar's Select buttons (or keys `1`-`5`) choose what a click picks —
  anything, faces, edges, vertices, or sketch geometry — which is how you reach a corner
  or a sketch point sitting under a face. A closed sketch region counts as a face, so Face
  mode picks either it or a body's face. Vertex and Sketch modes draw every point you
  could click. With a tool dialog open, click in the viewport to select what it asks for
  (for Extrude/Revolve/Sweep/Loft either a closed sketch region — every region enclosed
  by the curves, including the ones their crossings make — or a planar face of a body;
  edges for Fillet/Chamfer, or a face to take every edge around it; planes for Offset
  Plane, bodies for Combine/Move). The feature appears as soon as the input is complete
  and follows the dialog's parameters live; OK keeps it, Cancel removes it. Extrude,
  Fillet, Chamfer and Offset Plane also show an arrow in the viewport: drag its tip to
  set the distance or radius. Move shows the full manipulator — an arrow per axis and a
  ring per axis of rotation — driving the same numbers as its dialog. An extrude that lands on an existing body joins it unless
  you pick another operation, so stacked and overlapping extrudes make one body. While
  a preview shows, clicks still pick from the body as it was before the feature, so the
  second edge of a fillet is an edge of the original body.
* **Timeline**: click a feature to select it, double-click to edit, right-click for
  suppress / rename / delete / roll to here. The blue marker is the rollback cursor; new
  features are inserted at the marker.
* **Files**: `.bass` documents through the File menu or `Ctrl+S` / `Ctrl+O`; Export STL /
  3MF writes the selected bodies, or every visible body when nothing is selected.

Requires a Vulkan-capable GPU driver (Mesa is fine) for the application; the library
crates and their tests have no GPU requirement, and the renderer tests skip themselves
when no adapter is present.

## Layout

```
crates/
  basset-math      shared f64 maths and the TriMesh interchange type
  basset-sketch    2D constraint sketcher
  basset-kernel    solid modelling kernel
  basset-core      document, timeline, regeneration, .bass files
  basset-io        STL / 3MF export
  basset-viewport  wgpu renderer
  basset-app       desktop application
```

## Contributing conventions

* Comments explain *why*; identifiers explain *what*.
* Every operation that consumes user data returns a typed error; nothing panics on input.
* Kernel changes come with volume/bounding-box assertions; sketch changes with solver
  convergence tests; timeline changes with propagation tests.
* `cargo clippy --workspace --all-targets -- -D warnings` and `cargo fmt` must pass.
