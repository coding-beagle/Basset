# Basset

A fast, parametric, history-based 3D solid modeller written in Rust. Linux first.

The goal is feature parity with Fusion 360. The way there is deliberately narrow: few
tools, each finished, tested, and built on an object model that will not need to be
rewritten when the next tool arrives.

## Status: bootstrap / MVP

| Area | What exists |
| --- | --- |
| Object model | Document, Components, origin & construction Planes/Axes, Sketches, Bodies |
| Sketching | points, lines, arcs, circles, construction geometry, text; rectangles (2-point, centre), circles (centre, 2-point, 3-point), polygons, slots, arcs; geometric constraints and driving dimensions; Levenberg–Marquardt solver that names both the loose geometry and, when a sketch will not solve, the constraints that disagree; selection / hit testing including box select; grid snapping; closed-region detection with curves split at their crossings; trim and break; rectangular and circular patterns; named parameters driving dimensions |
| Solids | extrude (one side / symmetric / two sides), revolve, sweep, loft, from a sketch region or a planar face; join / cut / intersect; fillet, chamfer, combine, move |
| Construction | offset plane, plane at an angle, sketch on a planar face |
| Timeline | insert at cursor, edit, suppress, reorder, delete, roll back / forward; edits replay forward with per-feature caching; per-feature failure reporting |
| Files | `.bass` documents (versioned JSON); STL and 3MF export of bodies and components |
| Viewport | wgpu renderer with orbit camera, MSAA, pixel-width lines, a grid on any plane, face highlighting, translucent region fills, ray picking |
| App | winit + egui desktop shell: browser, timeline with rollback marker and context menu, live-preview tool dialogs, navigation cube, sketch mode with shape tools, constraints and click-to-edit dimensions, native open/save/export dialogs |

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
  sketch plane; the palette turns snapping off or pins the increment. After a shape's
  first click, type its sizes (length, width and height, diameter…) in the entry boxes
  that appear, Tab between them and press Enter to place it; a typed size pins the
  preview while the pointer picks the direction, and becomes a driving dimension. With
  the Select tool, click geometry (shift-click for several), click inside a closed region
  to take the curves around it, or drag a box (rightwards encloses, leftwards also takes
  what it touches); drag a point, a curve or the selection to move it under its
  constraints. The palette applies constraints to the selection; the toolbar toggles
  construction (`X`, lit when the selection is construction) or deletes. The Dimension
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
  its crossings and keeps everything. `M` moves the selection by offsets and a rotation
  you type — Enter applies, Esc puts it back — and the palette's Pattern section repeats
  the selection in a grid or around a centre, copies keeping the constraints the seed was
  drawn with. `E` extrudes the region under the pointer (or the ones you clicked inside):
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
  set the distance or radius. An extrude that lands on an existing body joins it unless
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
