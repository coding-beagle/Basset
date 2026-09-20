A list of features to implement / make better, grouped by section

Sketch:

- DXF import
- Trim/break: no Extend (dragging a curve out to meet another one) yet
- Sketch patterns are plain copies: there is no pattern entity to re-generate from, so
  editing the seed does not update the copies
- Parameters live on the sketch; document-wide parameters shared between sketches and
  feature dimensions are the next step

Extrude tool:

- Draggable arrow in the viewport for the distance (NOT DONE)
  - Needs an entry box for the extrusion distance
- Extrudes need to clean up the geometry that they create.

Filet tool:

- Multiple edges select reliably; picking works against the unfilleted body while the preview shows (NOT DONE)
  - Are edges not automatically created when we create geometry?
- Slow for some geometries, needs optimisation
