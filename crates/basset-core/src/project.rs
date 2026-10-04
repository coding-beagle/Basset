//! Copies a planar face's outline into a sketch drawn on that face, and keeps the copy on
//! the face as the face changes.
//!
//! A sketch started on a face opens with the face already in it: every edge of the face
//! is a sketch curve, pinned where the body puts it, so the user can dimension from the
//! face's corners, run a line to its edge, or extrude the face's own region again without
//! tracing it first. The kernel hands the outline over as a [`Profile`] whose loops are
//! straight pieces, each tagged with the neighbouring face it borders, and this module
//! turns those pieces back into the curves the user would have drawn: one line for a run
//! of collinear pieces, one arc or circle for a run that lies on a circle, and a polyline
//! for anything else.
//!
//! The copy is linked, not traced once. Every point and circle it makes carries a key
//! saying what it is on the face — the corner where the stretch bordering one neighbour
//! meets the stretch bordering the next, the centre of the arc along a stretch — and
//! [`refresh_face_outline`] uses those keys on every replay to put the copy back on the
//! face as it now is. A neighbour is named by the tag `face_profile` gives its stretch,
//! which survives edits of the body, so a wall extruded taller keeps the names of its
//! corners and its copy simply follows it up.

use std::collections::{HashMap, HashSet};

use basset_kernel::{Contour, Profile};
use basset_math::Vec2;
use basset_sketch::{Constraint, Entity, EntityId, Sketch, SketchError};

/// How far a boundary vertex may sit off the line or circle its run is fitted to before
/// the run is drawn as it came. The kernel's healed shells agree to `MERGE_TOL` (1 µm),
/// so this is generous for a fit and still far under anything a sketch can see.
const FIT_TOL: f64 = 1e-5;

/// How close a pinned point of an old, unlinked copy must sit to a point of the face's
/// outline to be taken for a copy of it. Converting an old file moves the sketch by a
/// computed offset, so the two agree to rounding, not exactly.
const ADOPT_TOL: f64 = 1e-6;

/// Adds the profile's outline to `sketch`, pins it in place and links it to the face.
/// Returns every curve it made, so a caller can tell projected geometry from what the
/// user draws afterwards.
///
/// Points are fixed rather than dimensioned because the face is the reference, not a
/// thing the sketch drives: a circle's diameter is the one dimension written, because a
/// radius has no point to pin.
pub fn project_face(sketch: &mut Sketch, profile: &Profile) -> Result<Vec<EntityId>, SketchError> {
    let mut seen = Seen::default();
    let mut curves = Vec::new();
    for contour in profile.loops() {
        curves.extend(project_loop(sketch, contour, &mut seen)?);
    }
    Ok(curves)
}

/// Moves a sketch's linked copy of a face's outline to where `profile` — the face as it
/// is now, in the sketch's frame — puts it.
///
/// A pinned point goes where the face has it, and a circle takes the face's diameter.
/// A point the user has unpinned is theirs now and is left alone. Returns the linked
/// entities the face no longer has — a corner a later edit took away — which stay where
/// they were, for the caller to say so.
pub fn refresh_face_outline(
    sketch: &mut Sketch,
    profile: &Profile,
) -> Result<Vec<EntityId>, SketchError> {
    if sketch.links().next().is_none() {
        return Ok(Vec::new());
    }
    let mut face = Sketch::new();
    project_face(&mut face, profile)?;
    let found: HashMap<u64, EntityId> = face.links().map(|(id, key)| (key, id)).collect();
    let pinned = pinned_points(sketch);
    let mut missing = Vec::new();
    for (id, key) in sketch.links().collect::<Vec<_>>() {
        let (Some(mine), Some(theirs)) = (
            sketch.entity(id).map(|e| e.entity.clone()),
            found.get(&key).and_then(|f| face.entity(*f)),
        ) else {
            missing.push(id);
            continue;
        };
        match (mine, &theirs.entity) {
            (Entity::Point { .. }, Entity::Point { pos }) => {
                if pinned.contains(&id) {
                    sketch.set_point_pos(id, *pos)?;
                }
            }
            (Entity::Circle { .. }, Entity::Circle { radius, .. }) => {
                let diameters: Vec<_> = sketch
                    .constraints()
                    .filter(|(_, c)| {
                        matches!(c, Constraint::Diameter { curve, value }
                            if *curve == id && (value - 2.0 * radius).abs() > f64::EPSILON)
                    })
                    .map(|(c, _)| c)
                    .collect();
                for c in diameters {
                    sketch.set_dimension_value(c, 2.0 * radius)?;
                }
            }
            _ => missing.push(id),
        }
    }
    Ok(missing)
}

/// Links an old, unlinked copy of a face's outline — one made before copies were linked —
/// so it follows the face from now on. Returns how many entities it linked.
///
/// The copy has no keys, so it is recognised by where it is: a pinned point sitting on a
/// point of the face's outline as `profile` has it now is taken for a copy of that point,
/// and a circle whose centre was so taken and whose radius matches for a copy of that
/// circle. A copy the face has since moved away from is not recognised and stays as it
/// was, pinned where it was drawn.
pub fn adopt_face_outline(sketch: &mut Sketch, profile: &Profile) -> Result<usize, SketchError> {
    let mut face = Sketch::new();
    project_face(&mut face, profile)?;
    let mut taken: HashSet<u64> = sketch.links().map(|(_, key)| key).collect();
    let mut adopted = 0;
    let mut centres: HashMap<EntityId, EntityId> = HashMap::new();
    for id in pinned_points(sketch) {
        if sketch.link_key(id).is_some() {
            continue;
        }
        let Some(pos) = sketch.point_pos(id) else {
            continue;
        };
        let twin = face.links().find(|(f, key)| {
            !taken.contains(key)
                && face
                    .point_pos(*f)
                    .is_some_and(|p| p.distance(pos) <= ADOPT_TOL)
        });
        if let Some((f, key)) = twin {
            sketch.link(id, key)?;
            taken.insert(key);
            centres.insert(id, f);
            adopted += 1;
        }
    }
    let circles: Vec<(EntityId, EntityId, f64)> = sketch
        .entities()
        .filter_map(|(id, e)| match e.entity {
            Entity::Circle { center, radius } if sketch.link_key(id).is_none() => {
                Some((id, center, radius))
            }
            _ => None,
        })
        .collect();
    for (id, center, radius) in circles {
        let Some(face_centre) = centres.get(&center) else {
            continue;
        };
        let twin = face.links().find(|(f, key)| {
            !taken.contains(key)
                && matches!(face.entity(*f).map(|e| &e.entity),
                    Some(Entity::Circle { center: c, radius: r })
                        if c == face_centre && (r - radius).abs() <= ADOPT_TOL)
        });
        if let Some((_, key)) = twin {
            sketch.link(id, key)?;
            taken.insert(key);
            adopted += 1;
        }
    }
    Ok(adopted)
}

fn pinned_points(sketch: &Sketch) -> Vec<EntityId> {
    let mut pinned: Vec<EntityId> = sketch
        .constraints()
        .filter_map(|(_, c)| match c {
            Constraint::Fix(p) => Some(*p),
            _ => None,
        })
        .collect();
    pinned.sort_unstable();
    pinned.dedup();
    pinned
}

/// What a linked entity is on the face, before it is hashed into a key.
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
enum Place {
    /// Where the stretch bordering `before` gives way to the one bordering `after`.
    Corner { before: u32, after: u32 },
    /// The `index`th joint along a stretch drawn piece by piece.
    Vertex { run: u32, index: u32 },
    /// The centre of the arc or circle along a stretch.
    Centre { run: u32 },
    /// The circle a whole loop bordering one neighbour makes.
    Circle { run: u32 },
}

/// Counts the places already keyed, so a face bordering the same neighbour along two
/// separate stretches — a U-shaped neighbour — gives each its own keys.
#[derive(Default)]
struct Seen(HashMap<Place, u32>);

impl Seen {
    /// How many times `place` has come up before this one.
    fn nth(&mut self, place: Place) -> u32 {
        let count = self.0.entry(place).or_insert(0);
        *count += 1;
        *count - 1
    }
}

/// FNV-1a over the place, rather than `DefaultHasher`, because the key is written into
/// the file and has to mean the same thing to a future build.
fn place_key(place: Place, nth: u32) -> u64 {
    let (kind, a, b) = match place {
        Place::Corner { before, after } => (0u8, before, after),
        Place::Vertex { run, index } => (1, run, index),
        Place::Centre { run } => (2, run, 0),
        Place::Circle { run } => (3, run, 0),
    };
    let mut hash: u64 = 0xcbf2_9ce4_8422_2325;
    let mut eat = |byte: u8| {
        hash ^= u64::from(byte);
        hash = hash.wrapping_mul(0x0000_0100_0000_01b3);
    };
    eat(kind);
    for word in [a, b, nth] {
        for byte in word.to_le_bytes() {
            eat(byte);
        }
    }
    hash
}

/// A stretch of one loop that borders a single neighbouring face: the pieces between
/// one corner of the face and the next.
struct Run {
    /// Loop point indices, first to last; the last is the next run's first.
    points: Vec<usize>,
}

fn project_loop(
    sketch: &mut Sketch,
    contour: &Contour,
    seen: &mut Seen,
) -> Result<Vec<EntityId>, SketchError> {
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

    let single = runs.len() == 1 && contour.closed;
    // Name every point before drawing anything: a corner by the two stretches meeting
    // there, a joint inside a stretch by its place along it. An open loop's two ends
    // border nothing on their outer side.
    let mut keys: Vec<u64> = vec![0; n];
    let mut run_keys: Vec<u32> = Vec::with_capacity(runs.len());
    for (r, run) in runs.iter().enumerate() {
        let here = tag(run.points[0]);
        // A stretch's own places are told apart from another stretch bordering the same
        // neighbour by which stretch it is, so they need no counting of their own.
        let nth = seen.nth(Place::Centre { run: here });
        run_keys.push(nth);
        let inner = if single {
            &run.points[..run.points.len() - 1]
        } else {
            let before = match r {
                0 if !contour.closed => u32::MAX,
                0 => tag(runs[runs.len() - 1].points[0]),
                _ => tag(runs[r - 1].points[0]),
            };
            let corner = Place::Corner {
                before,
                after: here,
            };
            keys[run.points[0]] = place_key(corner, seen.nth(corner));
            if r == runs.len() - 1 && !contour.closed {
                let end = Place::Corner {
                    before: here,
                    after: u32::MAX,
                };
                keys[*run.points.last().unwrap()] = place_key(end, seen.nth(end));
            }
            &run.points[1..run.points.len() - 1]
        };
        let skip = usize::from(!single);
        for (k, &i) in inner.iter().enumerate() {
            let index = (k + skip) as u32;
            keys[i] = place_key(Place::Vertex { run: here, index }, nth);
        }
    }

    let mut ids: Vec<Option<EntityId>> = vec![None; n];
    let mut curves = Vec::new();
    for (run, nth) in runs.iter().zip(run_keys) {
        let here = tag(run.points[0]);
        let pts: Vec<Vec2> = run.points.iter().map(|&i| contour.points[i]).collect();
        let mut point = |i: usize, sketch: &mut Sketch| -> Result<EntityId, SketchError> {
            if let Some(id) = ids[i] {
                return Ok(id);
            }
            let id = sketch.add_point(contour.points[i]);
            sketch.add_constraint(Constraint::Fix(id))?;
            sketch.link(id, keys[i])?;
            ids[i] = Some(id);
            Ok(id)
        };
        let centre = |sketch: &mut Sketch, at: Vec2| -> Result<EntityId, SketchError> {
            let c = sketch.add_point(at);
            sketch.add_constraint(Constraint::Fix(c))?;
            sketch.link(c, place_key(Place::Centre { run: here }, nth))?;
            Ok(c)
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
            let c = centre(sketch, center)?;
            let circle = sketch.add_circle(c, radius)?;
            sketch.link(circle, place_key(Place::Circle { run: here }, nth))?;
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
            let c = centre(sketch, center)?;
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

    fn rectangle(w: f64, h: f64, tags: [u32; 4]) -> Profile {
        let contour = Contour {
            points: vec![
                Vec2::new(0.0, 0.0),
                Vec2::new(w, 0.0),
                Vec2::new(w, h),
                Vec2::new(0.0, h),
            ],
            segments: tags.map(Segment::line).to_vec(),
            closed: true,
        };
        Profile::new(Frame::XY, contour)
    }

    /// The face grows taller: the pinned corners go with it, a corner the user unpinned
    /// stays theirs, and corners whose neighbours changed are reported, not guessed at.
    #[test]
    fn a_refresh_moves_pinned_corners_and_reports_the_ones_that_are_gone() {
        let mut sketch = Sketch::new();
        project_face(&mut sketch, &rectangle(10.0, 4.0, [1, 2, 3, 4])).unwrap();
        let corner_at = |sketch: &Sketch, at: Vec2| {
            sketch
                .links()
                .map(|(id, _)| id)
                .find(|id| sketch.point_pos(*id).unwrap().distance(at) < 1e-9)
                .unwrap()
        };
        let freed = corner_at(&sketch, Vec2::new(0.0, 4.0));
        let pin = sketch
            .constraints()
            .find_map(|(c, k)| matches!(k, Constraint::Fix(p) if *p == freed).then_some(c))
            .unwrap();
        sketch.remove_constraint(pin);

        let gone = refresh_face_outline(&mut sketch, &rectangle(10.0, 8.0, [1, 2, 3, 4])).unwrap();
        assert!(gone.is_empty(), "{gone:?}");
        let tops: Vec<f64> = sketch
            .links()
            .filter_map(|(id, _)| sketch.point_pos(id))
            .map(|p| p.y)
            .collect();
        assert!(tops.contains(&8.0), "the pinned top corner rose: {tops:?}");
        assert_eq!(sketch.point_pos(freed), Some(Vec2::new(0.0, 4.0)));

        // The top now borders a different neighbour: both top corners lost their names.
        let gone = refresh_face_outline(&mut sketch, &rectangle(10.0, 8.0, [1, 2, 9, 4])).unwrap();
        assert_eq!(gone.len(), 2, "{gone:?}");
    }

    /// An old copy, pinned but never linked, is recognised where it still lies on the face.
    #[test]
    fn an_unlinked_copy_is_adopted_where_it_matches_the_face() {
        let mut sketch = Sketch::new();
        let r = basset_sketch::shapes::rectangle_two_point(
            &mut sketch,
            Vec2::ZERO,
            Vec2::new(10.0, 4.0),
        );
        for c in r.corners {
            sketch.add_constraint(Constraint::Fix(c)).unwrap();
        }
        // A pinned point of the user's own, nowhere on the outline.
        let own = sketch.add_point(Vec2::new(3.0, 1.0));
        sketch.add_constraint(Constraint::Fix(own)).unwrap();
        let adopted = adopt_face_outline(&mut sketch, &rectangle(10.0, 4.0, [1, 2, 3, 4])).unwrap();
        assert_eq!(adopted, 4);
        assert!(sketch.link_key(own).is_none());
        assert!(r.corners.iter().all(|c| sketch.link_key(*c).is_some()));
        refresh_face_outline(&mut sketch, &rectangle(10.0, 8.0, [1, 2, 3, 4])).unwrap();
        assert_eq!(sketch.point_pos(r.corners[2]), Some(Vec2::new(10.0, 8.0)));
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
