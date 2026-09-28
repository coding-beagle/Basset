//! Copies a planar face's outline into a sketch drawn on that face.
//!
//! A sketch started on a face opens with the face already in it: every edge of the face
//! is a sketch curve, pinned where the body puts it, so the user can dimension from the
//! face's corners, run a line to its edge, or extrude the face's own region again without
//! tracing it first. The kernel hands the outline over as a [`Profile`] whose loops are
//! straight pieces, each tagged with the neighbouring face it borders, and this module
//! turns those pieces back into the curves the user would have drawn: one line for a run
//! of collinear pieces, one arc or circle for a run that lies on a circle, and a polyline
//! for anything else.

use basset_kernel::{Contour, Profile};
use basset_math::Vec2;
use basset_sketch::{Constraint, EntityId, Sketch, SketchError};

/// How far a boundary vertex may sit off the line or circle its run is fitted to before
/// the run is drawn as it came. The kernel's healed shells agree to `MERGE_TOL` (1 µm),
/// so this is generous for a fit and still far under anything a sketch can see.
const FIT_TOL: f64 = 1e-5;

/// Adds the profile's outline to `sketch` and pins it in place. Returns every curve it
/// made, so a caller can tell projected geometry from what the user draws afterwards.
///
/// Points are fixed rather than dimensioned because the face is the reference, not a
/// thing the sketch drives: a circle's diameter is the one dimension written, because a
/// radius has no point to pin.
pub fn project_face(sketch: &mut Sketch, profile: &Profile) -> Result<Vec<EntityId>, SketchError> {
    let mut curves = Vec::new();
    for contour in profile.loops() {
        curves.extend(project_loop(sketch, contour)?);
    }
    Ok(curves)
}

/// A stretch of one loop that borders a single neighbouring face: the pieces between
/// one corner of the face and the next.
struct Run {
    /// Loop point indices, first to last; the last is the next run's first.
    points: Vec<usize>,
}

fn project_loop(sketch: &mut Sketch, contour: &Contour) -> Result<Vec<EntityId>, SketchError> {
    let n = contour.points.len();
    if n < 2 || contour.segments.len() != contour.edge_count() {
        return Ok(Vec::new());
    }
    let tag = |i: usize| contour.segments[i % n].curve;
    // Start the walk at a corner so no run is split across index 0; a loop with one tag
    // all the way round has no corner and is one run.
    let start = (0..n)
        .find(|&i| tag((i + n - 1) % n) != tag(i))
        .filter(|_| contour.closed);
    let runs = match start {
        None if contour.closed => vec![Run {
            points: (0..=n).map(|i| i % n).collect(),
        }],
        _ => {
            let first = start.unwrap_or(0);
            let count = contour.edge_count();
            let mut runs: Vec<Run> = Vec::new();
            for k in 0..count {
                let i = (first + k) % n;
                let extend = k > 0 && tag((first + k - 1) % n) == tag(i);
                if extend {
                    runs.last_mut().unwrap().points.push((i + 1) % n);
                } else {
                    runs.push(Run {
                        points: vec![i, (i + 1) % n],
                    });
                }
            }
            runs
        }
    };

    let mut ids: Vec<Option<EntityId>> = vec![None; n];
    let mut curves = Vec::new();
    let single = runs.len() == 1 && contour.closed;
    for run in &runs {
        let pts: Vec<Vec2> = run.points.iter().map(|&i| contour.points[i]).collect();
        let mut point = |i: usize, sketch: &mut Sketch| -> Result<EntityId, SketchError> {
            if let Some(id) = ids[i] {
                return Ok(id);
            }
            let id = sketch.add_point(contour.points[i]);
            sketch.add_constraint(Constraint::Fix(id))?;
            ids[i] = Some(id);
            Ok(id)
        };
        // A loop with one neighbour all the way round is a circle if the points say so.
        // The fitter sees the first pieces again at the end, so the closing joint is
        // tested like every other.
        let wrapped: Vec<Vec2> = pts
            .iter()
            .chain(pts.iter().skip(1).take(1))
            .copied()
            .collect();
        if single && let Some((center, radius)) = fit_circle(&wrapped) {
            let c = sketch.add_point(center);
            sketch.add_constraint(Constraint::Fix(c))?;
            let circle = sketch.add_circle(c, radius)?;
            sketch.add_constraint(Constraint::Diameter {
                curve: circle,
                value: 2.0 * radius,
            })?;
            curves.push(circle);
            continue;
        }
        let (first, last) = (run.points[0], *run.points.last().unwrap());
        if collinear(&pts) {
            let a = point(first, sketch)?;
            let b = point(last, sketch)?;
            curves.push(sketch.add_line(a, b)?);
            continue;
        }
        if pts.len() >= 3
            && let Some((center, _)) = fit_circle(&pts)
        {
            let c = sketch.add_point(center);
            sketch.add_constraint(Constraint::Fix(c))?;
            let a = point(first, sketch)?;
            let b = point(last, sketch)?;
            // A sketch arc runs counter-clockwise from start to end, so a run that
            // travels clockwise round its centre is the same arc drawn from the far end.
            let ccw = (pts[0] - center).perp_dot(pts[1] - center) > 0.0;
            let (s, e) = if ccw { (a, b) } else { (b, a) };
            curves.push(sketch.add_arc(c, s, e)?);
            continue;
        }
        for w in run.points.windows(2) {
            let a = point(w[0], sketch)?;
            let b = point(w[1], sketch)?;
            curves.push(sketch.add_line(a, b)?);
        }
    }
    Ok(curves)
}

/// Whether every point lies on the line through the first and last.
fn collinear(pts: &[Vec2]) -> bool {
    let (a, b) = (pts[0], pts[pts.len() - 1]);
    let d = b - a;
    let len = d.length();
    if len < FIT_TOL {
        return false;
    }
    pts.iter()
        .all(|p| (d.perp_dot(*p - a) / len).abs() <= FIT_TOL)
}

/// The widest turn between two consecutive pieces that can still be a tessellated arc.
/// The kernel facets a circle at ten degrees or finer, so a run that turns more sharply
/// than this at a joint is a corner, not a curve — the four corners of a square sit on a
/// circle too, and without this test a square cap would come back as one.
const MAX_ARC_TURN: f64 = std::f64::consts::FRAC_PI_6;

/// The circle through the first, middle and last points, if every point sits on it and
/// the pieces bend gently enough to be a tessellated arc. `None` for fewer than three
/// points, for collinear ones, and for anything that is not a circle to within `FIT_TOL`.
fn fit_circle(pts: &[Vec2]) -> Option<(Vec2, f64)> {
    if pts.len() < 3 {
        return None;
    }
    let gentle = pts.windows(3).all(|w| {
        let (d0, d1) = (w[1] - w[0], w[2] - w[1]);
        d0.angle_to(d1).abs() <= MAX_ARC_TURN
    });
    if !gentle {
        return None;
    }
    let (a, b, c) = (pts[0], pts[pts.len() / 2], pts[pts.len() - 1]);
    let (center, radius) = basset_sketch::shapes::circumcircle(a, b, c)?;
    pts.iter()
        .all(|p| (p.distance(center) - radius).abs() <= FIT_TOL)
        .then_some((center, radius))
}

#[cfg(test)]
mod tests {
    use super::*;
    use basset_kernel::{Segment, Tessellation as KernelTess};
    use basset_math::Frame;
    use basset_sketch::Entity;
    use basset_sketch::Tessellation;

    fn kinds(sketch: &Sketch, curves: &[EntityId]) -> Vec<&'static str> {
        curves
            .iter()
            .map(|id| sketch.entity(*id).unwrap().entity.kind_name())
            .collect()
    }

    fn all_points_fixed(sketch: &Sketch) -> bool {
        sketch
            .entities()
            .filter(|(_, e)| e.entity.is_point())
            .all(|(id, _)| {
                sketch
                    .constraints()
                    .any(|(_, c)| matches!(c, Constraint::Fix(p) if *p == id))
            })
    }

    #[test]
    fn a_rectangle_becomes_four_pinned_lines_sharing_corners() {
        let square = Contour {
            points: vec![
                Vec2::new(0.0, 0.0),
                Vec2::new(10.0, 0.0),
                Vec2::new(10.0, 5.0),
                Vec2::new(0.0, 5.0),
            ],
            segments: (1..=4).map(Segment::line).collect(),
            closed: true,
        };
        let mut sketch = Sketch::new();
        let curves = project_face(&mut sketch, &Profile::new(Frame::XY, square)).unwrap();
        assert_eq!(kinds(&sketch, &curves), ["line"; 4]);
        assert_eq!(
            sketch
                .entities()
                .filter(|(_, e)| e.entity.is_point())
                .count(),
            4,
            "corners are shared, not doubled"
        );
        assert!(all_points_fixed(&sketch));
        let report = sketch.solve().unwrap();
        assert_eq!(report.degrees_of_freedom, 0);
        let profiles = sketch.profiles(&Tessellation::default());
        assert_eq!(profiles.len(), 1);
        assert!((profiles[0].area() - 50.0).abs() < 1e-9);
    }

    /// A boolean leaves a straight edge in pieces; the pieces bordering one face are
    /// one line again.
    #[test]
    fn collinear_pieces_of_one_neighbour_merge_into_one_line() {
        let contour = Contour {
            points: vec![
                Vec2::new(0.0, 0.0),
                Vec2::new(4.0, 0.0),
                Vec2::new(10.0, 0.0),
                Vec2::new(10.0, 5.0),
                Vec2::new(0.0, 5.0),
            ],
            segments: [1, 1, 2, 3, 4].map(Segment::line).to_vec(),
            closed: true,
        };
        let mut sketch = Sketch::new();
        let curves = project_face(&mut sketch, &Profile::new(Frame::XY, contour)).unwrap();
        assert_eq!(curves.len(), 4);
        assert_eq!(
            sketch
                .entities()
                .filter(|(_, e)| e.entity.is_point())
                .count(),
            4
        );
    }

    #[test]
    fn a_round_hole_becomes_a_circle_with_its_diameter_written() {
        let outer = Contour::polygon(
            vec![
                Vec2::new(-5.0, -5.0),
                Vec2::new(5.0, -5.0),
                Vec2::new(5.0, 5.0),
                Vec2::new(-5.0, 5.0),
            ],
            1,
        );
        let hole = Contour::circle(Vec2::new(1.0, 0.5), 2.0, 9, &KernelTess::default());
        let mut profile = Profile::new(Frame::XY, outer);
        profile.holes.push(hole);
        let mut sketch = Sketch::new();
        let curves = project_face(&mut sketch, &profile).unwrap();
        let circles: Vec<_> = curves
            .iter()
            .filter_map(|id| match sketch.entity(*id).unwrap().entity {
                Entity::Circle { center, radius } => Some((center, radius)),
                _ => None,
            })
            .collect();
        assert_eq!(circles.len(), 1, "{:?}", kinds(&sketch, &curves));
        let (center, radius) = circles[0];
        assert!((radius - 2.0).abs() < 1e-6);
        assert!(
            sketch
                .point_pos(center)
                .unwrap()
                .distance(Vec2::new(1.0, 0.5))
                < 1e-6
        );
        assert!(sketch.constraints().any(|(_, c)| matches!(
            c,
            Constraint::Diameter { value, .. } if (value - 4.0).abs() < 1e-6
        )));
        assert!(all_points_fixed(&sketch));
        assert_eq!(sketch.solve().unwrap().degrees_of_freedom, 0);
        // Square minus disc: the projected face is a region the user can extrude.
        let profiles = sketch.profiles(&Tessellation::default());
        let ring = profiles.iter().map(|p| p.area()).fold(0.0_f64, f64::max);
        assert!(
            (ring - (100.0 - std::f64::consts::PI * 4.0)).abs() < 0.2,
            "{ring}"
        );
    }

    /// The rounded end of a slot: many pieces on one circle, bounded by two lines.
    #[test]
    fn a_run_on_a_circle_becomes_an_arc_joined_to_its_neighbours() {
        let arc = |cx: f64, from: f64, to: f64| -> Vec<Vec2> {
            (0..=8)
                .map(|i| {
                    let t = from + (to - from) * i as f64 / 8.0;
                    Vec2::new(cx + 2.0 * t.cos(), 2.0 * t.sin())
                })
                .collect()
        };
        let half = std::f64::consts::FRAC_PI_2;
        let mut points = Vec::new();
        let mut segments = Vec::new();
        // Right cap, counter-clockwise from the bottom, then the top line, then the
        // left cap, then the bottom line.
        for p in &arc(5.0, -half, half)[..8] {
            points.push(*p);
            segments.push(Segment::line(1));
        }
        points.push(Vec2::new(5.0, 2.0));
        segments.push(Segment::line(2));
        for p in &arc(-5.0, half, 3.0 * half)[..8] {
            points.push(*p);
            segments.push(Segment::line(3));
        }
        points.push(Vec2::new(-5.0, -2.0));
        segments.push(Segment::line(4));
        let contour = Contour {
            points,
            segments,
            closed: true,
        };
        let mut sketch = Sketch::new();
        let curves = project_face(&mut sketch, &Profile::new(Frame::XY, contour)).unwrap();
        let mut got = kinds(&sketch, &curves);
        got.sort_unstable();
        assert_eq!(got, ["arc", "arc", "line", "line"]);
        assert!(all_points_fixed(&sketch));
        assert_eq!(sketch.solve().unwrap().degrees_of_freedom, 0);
        let profiles = sketch.profiles(&Tessellation::default());
        assert_eq!(profiles.len(), 1, "the arcs meet the lines end to end");
        let expected = 10.0 * 4.0 + std::f64::consts::PI * 4.0;
        assert!(
            (profiles[0].area() - expected).abs() < 0.2,
            "{}",
            profiles[0].area()
        );
    }

    /// Pieces that are neither straight nor circular are kept as they are, and still
    /// close the loop.
    #[test]
    fn a_freeform_run_is_kept_piece_by_piece() {
        let contour = Contour {
            points: vec![
                Vec2::new(0.0, 0.0),
                Vec2::new(10.0, 0.0),
                Vec2::new(9.0, 3.0),
                Vec2::new(10.0, 6.0),
                Vec2::new(7.0, 9.0),
                Vec2::new(0.0, 9.0),
            ],
            segments: [1, 2, 2, 2, 3, 4].map(Segment::line).to_vec(),
            closed: true,
        };
        let mut sketch = Sketch::new();
        let curves = project_face(&mut sketch, &Profile::new(Frame::XY, contour)).unwrap();
        assert_eq!(kinds(&sketch, &curves), ["line"; 6]);
        assert_eq!(sketch.profiles(&Tessellation::default()).len(), 1);
    }
}
