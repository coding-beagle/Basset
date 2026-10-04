//! Modelled threads on cylindrical faces.
//!
//! A thread is a groove swept along a helix about the face's axis and subtracted from the
//! body, so it is built like a blend: a tool solid and one boolean. The groove's section
//! is the ISO metric basic profile — 60° flanks, `5/8·H` deep where `H = √3/2·P` is the
//! height of the sharp triangle — taken from the face inwards on a shaft (the face is the
//! major diameter) and outwards into the wall around a hole (the face is the minor one).
//! That is the one choice that makes both work from the same pick: a 10 mm shaft threads
//! to M10, and a hole drilled at the tap size for M10 threads to M10 too.
//!
//! The groove is wider than the gap the profile leaves at the face, by running its sides
//! on past the face for a short way ([`radial_margin`]), so the tool's flanks cross the
//! body's facets instead of sitting on them; the margin stays well short of the pitch,
//! so neighbouring turns of the tool never touch and the tool is a sound solid.
//!
//! Where the thread stops along the axis is decided by what is beyond each end of the
//! face. An end that opens into air — a shaft's tip, a hole's mouth, a chamfer leading
//! off either — lets the groove run on a turn and out, so the tool closes in the air
//! clear of the body's end face. An end that runs into more material — the shoulder
//! under a bolt head, the floor of a blind hole — stops the groove dead in that plane, as
//! a modelled thread in Fusion does, rather than notch the shoulder. A thread shorter
//! than the face starts at the open end and stops dead where its length runs out.
//!
//! The groove's flanks are twisted surfaces, and the BSP boolean is at its weakest
//! against them; [`RIPPLE`], [`TURN_SEGMENTS`] and [`ATTEMPTS`] are what it takes to get a
//! closed shell back, and the result is checked rather than trusted.
//!
//! Faces of the groove: `Thread(0)` is its root (a cylinder), `Thread(1)` and `Thread(2)`
//! the flanks facing along and against the axis, `Thread(3)` the tool's crest (always
//! outside the body, so it never survives) and `Thread(4)`/`Thread(5)` the planes the
//! groove stops dead in at the low and high end.

use std::f64::consts::TAU;

use basset_math::{Frame, Vec2, Vec3};

use crate::csg::{BoolOp, boolean};
use crate::error::KernelError;
use crate::geometry::{Contour, Extent, Profile};
use crate::ids::{FaceKey, FaceRole, OpId};
use crate::solid::{Solid, SolidBuilder, SurfaceKind};

/// Facets one thread tool may carry: 36 to the turn, four to a ring, so about 80 turns.
///
/// A fine pitch typed over a long face multiplies out quickly, and the boolean's time
/// grows faster than the count. Measured on one core: M10 × 1.5 over 30 mm (3 200
/// facets) in 0.25 s, over 60 mm (6 000) in 0.8 s, M10 × 0.5 over 40 mm (11 800) in
/// 2.2 s, and M6 × 0.5 over 80 mm (23 000) in 22 s. The budget keeps a preview to a
/// couple of seconds.
const MAX_THREAD_POLYGONS: usize = 12_000;

/// Facets per turn of the helix, by attempt.
///
/// The default tessellation's 10°, because the groove is an inscribed polygon and its
/// chords cut inside the true radii by `r·(1 − cos(π/n))`: at 12 to the turn that was a
/// fifth of an M10's depth, and the groove took 14% too much off a shaft and 23% too
/// little out of a hole. The last attempts go coarser, which widens the angle between
/// the triangles of a flank facet (see [`RIPPLE`]) and closes the few threads 36 cannot,
/// all of them fine pitches on large diameters.
const TURN_SEGMENTS: [usize; ATTEMPTS] = [36, 36, 36, 24, 24];

/// How far, as a share of the pitch, the groove's root steps along the axis and back on
/// alternate rings.
///
/// A facet of a helical flank is twisted, so it goes in as two triangles, and left to
/// the helix those lie at about `0.75·P / (n·r)` radians to each other — a twentieth of
/// a degree for M20 × 0.5. The boolean cuts the body's facets along both triangles'
/// planes, and two cuts that close to parallel leave a sliver below its tolerance where
/// they meet, which is where the shell leaks. Stepping the root corners back and forth
/// puts a crease of a degree or so into every flank facet instead, while the root's own
/// facets stay flat, since both of a ring's root corners move together. A hundredth of
/// the pitch is microns on any thread anyone makes.
///
/// Measured on shafts and tapped holes of 1.6 to 20 mm at pitches of 0.35 to 3 mm, 155
/// threads in all: the plain helix closed 100 on the first try and 132 with retries; with
/// the ripple, 130 and 153; with the coarser fallbacks of [`TURN_SEGMENTS`], all 155.
const RIPPLE: f64 = 0.01;

/// What the user asked for.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ThreadSpec {
    /// Axial advance per turn, mm.
    pub pitch: f64,
    /// How much of the face is threaded, from the start end; `None` is all of it.
    pub length: Option<f64>,
    /// Turns the other way: advancing along the axis while turning clockwise about it.
    pub left_handed: bool,
    /// Measures `length` from the other end of the face.
    pub reversed: bool,
}

/// A cylindrical face, as a thread sees it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CylinderFace {
    pub origin: Vec3,
    /// Unit direction of the axis.
    pub axis: Vec3,
    pub radius: f64,
    /// The face's span along the axis, measured from `origin`.
    pub low: f64,
    pub high: f64,
    /// A shaft (material inside the face) rather than a hole (material outside it).
    pub external: bool,
    /// Whether each end — low, then high — opens into air rather than into more material.
    pub open: [bool; 2],
    /// How far inside the true cylinder the face's facets reach at their middles.
    pub sagitta: f64,
}

impl CylinderFace {
    pub fn length(&self) -> f64 {
        self.high - self.low
    }

    /// The thread size this face is: the major diameter, which is the face itself on a
    /// shaft and the face plus twice the thread's depth in a hole.
    pub fn nominal_diameter(&self, pitch: f64) -> f64 {
        match self.external {
            true => 2.0 * self.radius,
            false => 2.0 * (self.radius + depth(pitch)),
        }
    }

    /// The ISO metric coarse pitch for this face: the standard size whose major diameter
    /// (shaft) or minor diameter (hole) is nearest the face's.
    pub fn coarse_pitch(&self) -> f64 {
        let size = 2.0 * self.radius;
        let fit = |&(d, p): &(f64, f64)| match self.external {
            true => (d - size).abs(),
            false => (d - 2.0 * depth(p) - size).abs(),
        };
        ISO_COARSE
            .iter()
            .min_by(|a, b| fit(a).total_cmp(&fit(b)))
            .map_or(1.0, |&(_, p)| p)
    }

    /// The point at axial position `t` and angle `phi` (from `u`), `rho` from the axis.
    fn at(&self, (u, v): (Vec3, Vec3), rho: f64, phi: f64, t: f64) -> Vec3 {
        self.origin + self.axis * t + (u * phi.cos() + v * phi.sin()) * rho
    }
}

/// Nominal diameter and coarse pitch of the ISO metric sizes, first choice and second.
const ISO_COARSE: [(f64, f64); 30] = [
    (1.0, 0.25),
    (1.2, 0.25),
    (1.6, 0.35),
    (2.0, 0.4),
    (2.5, 0.45),
    (3.0, 0.5),
    (3.5, 0.6),
    (4.0, 0.7),
    (5.0, 0.8),
    (6.0, 1.0),
    (8.0, 1.25),
    (10.0, 1.5),
    (12.0, 1.75),
    (14.0, 2.0),
    (16.0, 2.0),
    (18.0, 2.5),
    (20.0, 2.5),
    (22.0, 2.5),
    (24.0, 3.0),
    (27.0, 3.0),
    (30.0, 3.5),
    (33.0, 3.5),
    (36.0, 4.0),
    (39.0, 4.0),
    (42.0, 4.5),
    (45.0, 4.5),
    (48.0, 5.0),
    (52.0, 5.0),
    (56.0, 5.5),
    (64.0, 6.0),
];

/// How deep the groove is: `5/8·H` of the basic profile.
pub fn depth(pitch: f64) -> f64 {
    5.0 / 8.0 * (3f64.sqrt() / 2.0) * pitch
}

/// How far past the face the groove's sides run before the tool closes: clear of the
/// facets by a sixteenth of a pitch, so the tool's crest never grazes them, and short of
/// the distance at which a groove widening at 60° would meet the next turn.
fn radial_margin(face: &CylinderFace, pitch: f64) -> f64 {
    let facets = if face.external { 0.0 } else { face.sagitta };
    (facets + pitch / 16.0).min(pitch / 10.0)
}

/// The angle the helix starts at: as far as it can be from every corner of the face.
///
/// The helix is drawn with as many facets a turn as the cylinder it cuts, usually from
/// the same frame, so left alone every section of the tool would pass exactly through a
/// seam between two of the body's facets, and the boolean would be cutting along lines
/// where the body already has edges. Halfway between seams it never has to.
fn helix_phase(solid: &Solid, key: FaceKey, face: &CylinderFace, step: f64) -> f64 {
    let Some(f) = solid.face(key) else {
        return 0.0;
    };
    let frame = Frame::from_normal(face.origin, face.axis);
    let corners: Vec<f64> = f
        .polygons
        .iter()
        .flat_map(|p| p.vertices.iter())
        .map(|v| {
            let d = *v - face.origin;
            d.dot(frame.y).atan2(d.dot(frame.x)).rem_euclid(step)
        })
        .collect();
    const TRIES: usize = 32;
    let clearance = |phase: f64| {
        corners
            .iter()
            .map(|c| {
                let d = (c - phase).rem_euclid(step);
                d.min(step - d)
            })
            .fold(f64::INFINITY, f64::min)
    };
    (0..TRIES)
        .map(|i| step * i as f64 / TRIES as f64)
        .max_by(|a, b| clearance(*a).total_cmp(&clearance(*b)))
        .unwrap_or(0.0)
}

/// Reads a face of `solid` as a cylinder to thread.
pub fn cylinder_face(solid: &Solid, key: FaceKey) -> Result<CylinderFace, KernelError> {
    let face = solid.face(key).ok_or(KernelError::MissingFace(key))?;
    let SurfaceKind::Cylindrical {
        origin,
        axis,
        radius,
    } = face.surface
    else {
        return Err(KernelError::NotCylindricalFace(key));
    };
    let axis = axis.normalize();
    let along = |p: Vec3| (p - origin).dot(axis);
    let (mut low, mut high) = (f64::INFINITY, f64::NEG_INFINITY);
    for v in face.polygons.iter().flat_map(|p| p.vertices.iter()) {
        low = low.min(along(*v));
        high = high.max(along(*v));
    }
    if high <= low {
        return Err(KernelError::NotCylindricalFace(key));
    }
    // A shaft's face points away from its axis and a hole's towards it. Weighed by area
    // so a sliver a boolean left does not get the casting vote.
    let outward: f64 = face
        .polygons
        .iter()
        .map(|p| {
            let c = p.centroid() - origin;
            let radial = c - axis * c.dot(axis);
            p.plane.normal.dot(radial) * p.area()
        })
        .sum();
    let mut cylinder = CylinderFace {
        origin,
        axis,
        radius,
        low,
        high,
        external: outward > 0.0,
        open: [false; 2],
        sagitta: face
            .polygons
            .iter()
            .map(|p| {
                let off = (p.plane.origin - origin).dot(p.plane.normal);
                let across = (p.plane.normal - axis * p.plane.normal.dot(axis)).length();
                (radius - off.abs() / across.max(1e-12)).max(0.0)
            })
            .fold(0.0, f64::max),
    };
    cylinder.open = [
        end_is_open(solid, key, &cylinder, low, -1.0),
        end_is_open(solid, key, &cylinder, high, 1.0),
    ];
    Ok(cylinder)
}

/// Whether the face's end at `t` (the low end when `side` is −1, the high one when +1)
/// opens into air: every face beyond it turns away along the axis, the way a shaft's
/// end cap, a hole's rim or a chamfer leading off either does. A shoulder or a hole's
/// floor faces back at the threaded stretch, and a cylinder carrying on beyond the end
/// is no turn at all; both are more material, and the groove stops at them.
fn end_is_open(solid: &Solid, key: FaceKey, face: &CylinderFace, t: f64, side: f64) -> bool {
    const ON_END: f64 = 1e-4;
    const TURNS_AWAY: f64 = 0.1;
    let along = |p: Vec3| (p - face.origin).dot(face.axis);
    let outward = face.axis * side;
    let mut any = false;
    for edge in solid.edges().iter().filter(|e| e.key.touches(key)) {
        for seg in &edge.segments {
            if (along(seg.start) - t).abs() > ON_END || (along(seg.end) - t).abs() > ON_END {
                continue;
            }
            let beyond = if edge.key.a == key {
                seg.normal_b
            } else {
                seg.normal_a
            };
            if edge.smooth || beyond.dot(outward) < TURNS_AWAY {
                return false;
            }
            any = true;
        }
    }
    any
}

/// Cuts a thread into the cylindrical face `key` of `solid`.
pub fn thread(
    op: OpId,
    solid: &Solid,
    key: FaceKey,
    spec: &ThreadSpec,
) -> Result<Solid, KernelError> {
    let pitch = spec.pitch;
    if pitch <= 0.0 || !pitch.is_finite() {
        return Err(KernelError::NonPositivePitch);
    }
    let face = cylinder_face(solid, key)?;
    let h = depth(pitch);
    if face.external && face.radius - h < 0.1 * face.radius {
        return Err(KernelError::ThreadTooDeep {
            pitch,
            depth: h,
            radius: face.radius,
        });
    }

    // The stretch of the axis to thread, and where it stops dead rather than running out.
    let available = face.length();
    let (mut low, mut high, mut open) = (face.low, face.high, face.open);
    if let Some(length) = spec.length {
        if length.is_nan() || length <= 0.0 || length > available + 1e-6 {
            return Err(KernelError::ThreadLength { length, available });
        }
        // From the end that opens, as a bolt is threaded from its tip and a hole from its
        // mouth; from the high end when both do or neither does.
        let from_high = !(face.open[0] && !face.open[1]) != spec.reversed;
        if from_high {
            low = high - length;
            open[0] = false;
        } else {
            high = low + length;
            open[1] = false;
        }
    }
    // Past an open end the tool runs on a turn into the air and closes there, clear of
    // everything. Into material it is clipped to stop dead in the plane of the end, and
    // the clip is only paid for when an end needs it: it is a boolean over every facet of
    // the tool, and one more chance of a sliver.
    let clip = match open {
        [true, true] => None,
        _ => {
            let beyond = 2.0 * pitch;
            let clip_low = if open[0] { low - beyond } else { low };
            let clip_high = if open[1] { high + beyond } else { high };
            Some(slab(op, &face, pitch, (clip_low, clip_high))?)
        }
    };
    let mut last = None;
    for attempt in 0..ATTEMPTS {
        let helix = Helix {
            pitch,
            left_handed: spec.left_handed,
            span: (low, high),
            attempt,
        };
        let mut tool = helix_tool(op, solid, key, &face, &helix)?;
        if let Some(clip) = &clip {
            tool = boolean(&tool, clip, BoolOp::Intersect)?;
        }
        let result = boolean(solid, &tool, BoolOp::Subtract)?;
        match result.validate() {
            Ok(()) => return Ok(result),
            Err(e) => last = Some(e),
        }
    }
    Err(last.unwrap_or(KernelError::EmptyResult))
}

/// How many differently drawn helices a thread tries before it reports a leak.
///
/// What leaks is a coincidence — a corner of the tool landing within the boolean's
/// tolerance of a cut it made a moment before — so the same thread drawn a fraction of a
/// facet round, with its flanks split along the other diagonals, usually misses it. Each
/// try is checked; the first closed shell is the answer. See [`RIPPLE`] for the numbers.
const ATTEMPTS: usize = 5;

/// One way of drawing the groove's helix.
struct Helix {
    pitch: f64,
    left_handed: bool,
    /// The stretch of the axis the groove must cover.
    span: (f64, f64),
    /// Which retry this is: each turns the helix on a little and splits its flanks along
    /// the other diagonals.
    attempt: usize,
}

/// The groove swept along the helix, a turn past each end of `low..high`: whole all round
/// wherever the slab clips it, and closed off in the air past an end it runs out of.
fn helix_tool(
    op: OpId,
    solid: &Solid,
    key: FaceKey,
    face: &CylinderFace,
    helix: &Helix,
) -> Result<Solid, KernelError> {
    let Helix {
        pitch,
        left_handed,
        span: (low, high),
        attempt,
    } = *helix;
    let h = depth(pitch);
    let margin = radial_margin(face, pitch);
    let flank = (TAU / 12.0).tan(); // tan 30°: half the 60° included angle
    // Signed so `face.radius + inward * x` goes into the material.
    let inward = if face.external { -1.0 } else { 1.0 };
    // The groove's width at the face: the gap the basic profile leaves between teeth. A
    // shaft's crest is a P/8 flat, so its groove is 7P/8 wide there; a nut's crest, on
    // the minor diameter, is a P/4 flat, leaving 3P/4.
    let at_face = if face.external {
        7.0 / 8.0 * pitch
    } else {
        3.0 / 4.0 * pitch
    };
    let root = at_face - 2.0 * h * flank;
    let mouth = at_face + 2.0 * margin * flank;
    let rho_root = face.radius + inward * h;
    let rho_mouth = face.radius - inward * margin;
    // Crest along the axis, flank up, root, flank down: (distance from axis, offset).
    let section = [
        (rho_mouth, -mouth / 2.0),
        (rho_mouth, mouth / 2.0),
        (rho_root, root / 2.0),
        (rho_root, -root / 2.0),
    ];
    let roles = [3, 1, 0, 2];
    let root_corners = 2..4;

    let per_turn = TURN_SEGMENTS[attempt];
    let ripple = RIPPLE * pitch;
    let start = low - pitch;
    let turns = (high + pitch - start) / pitch;
    let steps = (turns * per_turn as f64).ceil() as usize;
    let needed = steps * section.len() + 2;
    if needed > MAX_THREAD_POLYGONS {
        return Err(KernelError::ThreadTooDense {
            needed,
            budget: MAX_THREAD_POLYGONS,
        });
    }
    let frame = Frame::from_normal(face.origin, face.axis);
    let uv = (frame.x, frame.y);
    let hand = if left_handed { -1.0 } else { 1.0 };
    let step = TAU / per_turn as f64;
    // Retries move round by an irrational share of a facet, so no two land alike.
    let phase = helix_phase(solid, key, face, step) + step * (attempt as f64 * 0.382).fract();
    let ring = |i: usize| -> Vec<Vec3> {
        let turn = i as f64 / per_turn as f64;
        let phi = phase + hand * TAU * turn;
        let t = start + pitch * turn;
        let wobble = if i.is_multiple_of(2) { ripple } else { -ripple };
        section
            .iter()
            .enumerate()
            .map(|(j, &(rho, dt))| {
                let dt = if root_corners.contains(&j) {
                    dt + wobble
                } else {
                    dt
                };
                face.at(uv, rho, phi, t + dt)
            })
            .collect()
    };
    let surface = |role: u32| match role {
        0 => SurfaceKind::Cylindrical {
            origin: face.origin,
            axis: face.axis,
            radius: rho_root,
        },
        3 => SurfaceKind::Cylindrical {
            origin: face.origin,
            axis: face.axis,
            radius: rho_mouth,
        },
        _ => SurfaceKind::Freeform,
    };

    let mut b = SolidBuilder::default();
    let mut cur = ring(0);
    let first = cur.clone();
    for i in 1..=steps {
        let next = ring(i);
        for j in 0..section.len() {
            let k = (j + 1) % section.len();
            let role = roles[j];
            let quad = if attempt.is_multiple_of(2) {
                [cur[j], cur[k], next[k], next[j]]
            } else {
                [cur[k], next[k], next[j], cur[j]]
            };
            b.push_quad(
                FaceKey::new(op, FaceRole::Thread(role)),
                surface(role),
                quad,
            );
        }
        cur = next;
    }
    // The ends of the sweep: a turn out past the body or the slab, so they never survive.
    let mut start_cap = first;
    start_cap.reverse();
    let cap_key = FaceKey::new(op, FaceRole::Generic(0));
    let planar = SurfaceKind::Planar { normal: Vec3::ZERO };
    b.push(cap_key, planar, start_cap);
    b.push(FaceKey::new(op, FaceRole::Generic(1)), planar, cur);
    let mut tool = b.finish();
    tool.heal();
    tool.validate()?;
    Ok(tool)
}

/// The block the tool is clipped to: `low..high` along the axis, and wide enough across
/// to hold the whole groove. Its ends become the planes a groove stops dead in.
fn slab(
    op: OpId,
    face: &CylinderFace,
    pitch: f64,
    (low, high): (f64, f64),
) -> Result<Solid, KernelError> {
    let half = face.radius + 2.0 * depth(pitch) + pitch;
    let mut frame = Frame::from_normal(face.origin, face.axis);
    frame.origin = face.at((frame.x, frame.y), 0.0, 0.0, low);
    let square = Contour::polygon(
        vec![
            Vec2::new(-half, -half),
            Vec2::new(half, -half),
            Vec2::new(half, half),
            Vec2::new(-half, half),
        ],
        0,
    );
    let mut block = crate::generate::extrude(
        op,
        &Profile::new(frame, square),
        Extent::OneSide(high - low),
    )?;
    for f in &mut block.faces {
        f.key.role = match f.key.role {
            FaceRole::StartCap => FaceRole::Thread(4),
            FaceRole::EndCap => FaceRole::Thread(5),
            _ => FaceRole::Generic(2),
        };
    }
    Ok(block)
}

#[cfg(test)]
mod tests {
    use std::f64::consts::PI;

    use super::*;
    use crate::generate::revolve;
    use crate::geometry::{Axis, Contour, Segment, SegmentKind, Tessellation};
    use crate::primitives::{cuboid, cylinder};

    fn side() -> FaceKey {
        FaceKey::new(OpId::new(1), FaceRole::Side(0))
    }

    fn spec(pitch: f64) -> ThreadSpec {
        ThreadSpec {
            pitch,
            length: None,
            left_handed: false,
            reversed: false,
        }
    }

    /// Volume a groove takes per mm of axis: its section swept once round per pitch, with
    /// the section's mid-depth radius standing in for its centroid's.
    fn groove_volume_per_mm(r: f64, pitch: f64, at_face: f64) -> f64 {
        let h = depth(pitch);
        let root = at_face - 2.0 * h * (PI / 6.0).tan();
        let area = (at_face + root) / 2.0 * h;
        area * TAU * (r - h / 2.0) / pitch
    }

    #[test]
    fn a_shaft_reads_as_external_and_a_bore_as_internal() {
        let tess = Tessellation::default();
        let shaft = cylinder(OpId::new(1), Vec3::ZERO, Vec3::Z, 5.0, 20.0, &tess);
        let f = cylinder_face(&shaft, side()).unwrap();
        assert!(f.external);
        assert!((f.radius - 5.0).abs() < 1e-9);
        assert!((f.length() - 20.0).abs() < 1e-9);
        assert_eq!(f.open, [true, true]);
        assert_eq!(f.coarse_pitch(), 1.5, "a 10 mm shaft is M10");

        let plate = cuboid(OpId::new(2), Vec3::splat(-10.0), Vec3::new(10.0, 10.0, 0.0));
        let drill = cylinder(
            OpId::new(1),
            Vec3::new(0.0, 0.0, -11.0),
            Vec3::Z,
            4.25,
            12.0,
            &tess,
        );
        let holed = boolean(&plate, &drill, BoolOp::Subtract).unwrap();
        let f = cylinder_face(&holed, side()).unwrap();
        assert!(!f.external);
        assert_eq!(f.open, [true, true]);
        // An 8.5 mm tap drill is M10's.
        assert_eq!(f.coarse_pitch(), 1.5);
        assert!((f.nominal_diameter(1.5) - 10.124).abs() < 0.01);
    }

    #[test]
    fn an_external_thread_takes_its_groove_off_the_shaft_and_stays_closed() {
        let tess = Tessellation::default();
        let shaft = cylinder(OpId::new(1), Vec3::ZERO, Vec3::Z, 5.0, 20.0, &tess);
        let threaded = thread(OpId::new(2), &shaft, side(), &spec(1.5)).unwrap();
        threaded.validate().unwrap();
        let removed = shaft.volume() - threaded.volume();
        let expected = groove_volume_per_mm(5.0, 1.5, 7.0 / 8.0 * 1.5) * 20.0;
        assert!(
            (removed - expected).abs() < 0.1 * expected,
            "removed {removed:.2} mm³, expected about {expected:.2}"
        );
        // Nothing grows: the thread only cuts.
        let (a, b) = (shaft.aabb(), threaded.aabb());
        assert!(b.max.z <= a.max.z + 1e-6 && b.min.z >= a.min.z - 1e-6);
        assert!(
            threaded
                .face(FaceKey::new(OpId::new(2), FaceRole::Thread(0)))
                .is_some()
        );
        assert!(
            threaded
                .face(FaceKey::new(OpId::new(2), FaceRole::Thread(1)))
                .is_some()
        );
        assert!(
            threaded
                .face(FaceKey::new(OpId::new(2), FaceRole::Thread(2)))
                .is_some()
        );
    }

    #[test]
    fn an_internal_thread_cuts_into_the_wall_of_a_through_hole() {
        let tess = Tessellation::default();
        let plate = cuboid(OpId::new(2), Vec3::splat(-10.0), Vec3::new(10.0, 10.0, 0.0));
        let drill = cylinder(
            OpId::new(1),
            Vec3::new(0.0, 0.0, -11.0),
            Vec3::Z,
            4.25,
            12.0,
            &tess,
        );
        let holed = boolean(&plate, &drill, BoolOp::Subtract).unwrap();
        let tapped = thread(OpId::new(3), &holed, side(), &spec(1.5)).unwrap();
        tapped.validate().unwrap();
        let removed = holed.volume() - tapped.volume();
        let expected = groove_volume_per_mm(4.25 + depth(1.5), 1.5, 3.0 / 4.0 * 1.5) * 10.0;
        assert!(
            (removed - expected).abs() < 0.15 * expected,
            "removed {removed:.2} mm³, expected about {expected:.2}"
        );
    }

    #[test]
    fn a_thread_stops_dead_at_a_shoulder_and_leaves_the_shoulder_whole() {
        let tess = Tessellation::default();
        // A head of radius 8 under a shank of radius 5, revolved about Z.
        let mut outline = Contour::polygon(
            vec![
                Vec2::new(0.0, 0.0),
                Vec2::new(8.0, 0.0),
                Vec2::new(8.0, 4.0),
                Vec2::new(5.0, 4.0),
                Vec2::new(5.0, 20.0),
                Vec2::new(0.0, 20.0),
            ],
            0,
        );
        for (i, s) in outline.segments.iter_mut().enumerate() {
            *s = Segment {
                curve: i as u32,
                kind: SegmentKind::Line,
            };
        }
        // Profile x is the distance from the axis and y the height along it.
        let plane = Frame::from_axes(Vec3::ZERO, Vec3::X, Vec3::Z);
        let bolt = revolve(
            OpId::new(1),
            &Profile::new(plane, outline),
            &Axis::new(Vec3::ZERO, Vec3::Z),
            TAU,
            &tess,
        )
        .unwrap();
        let shank = FaceKey::new(OpId::new(1), FaceRole::Side(3));
        let f = cylinder_face(&bolt, shank).unwrap();
        assert!(f.external);
        assert_eq!(
            f.open,
            [false, true],
            "the head is material, the tip is air"
        );
        let threaded = thread(OpId::new(2), &bolt, shank, &spec(1.5)).unwrap();
        threaded.validate().unwrap();
        // The head is untouched: the groove does not reach below the shoulder.
        let head = FaceKey::new(OpId::new(1), FaceRole::Side(1));
        assert!(
            (threaded.face(head).unwrap().area() - bolt.face(head).unwrap().area()).abs() < 1e-6
        );
        let low = threaded
            .faces
            .iter()
            .filter(|face| face.key.op == OpId::new(2))
            .flat_map(|face| face.polygons.iter().flat_map(|p| p.vertices.iter()))
            .map(|v| v.z)
            .fold(f64::INFINITY, f64::min);
        assert!(low >= 4.0 - 1e-6, "groove reaches z = {low}");
    }

    #[test]
    fn a_partial_thread_starts_at_the_open_end() {
        let tess = Tessellation::default();
        let shaft = cylinder(OpId::new(1), Vec3::ZERO, Vec3::Z, 5.0, 20.0, &tess);
        let short = ThreadSpec {
            length: Some(8.0),
            ..spec(1.5)
        };
        let threaded = thread(OpId::new(2), &shaft, side(), &short).unwrap();
        threaded.validate().unwrap();
        let groove_z: Vec<f64> = threaded
            .faces
            .iter()
            .filter(|face| face.key.op == OpId::new(2))
            .flat_map(|face| face.polygons.iter().flat_map(|p| p.vertices.iter()))
            .map(|v| v.z)
            .collect();
        let low = groove_z.iter().copied().fold(f64::INFINITY, f64::min);
        assert!((low - 12.0).abs() < 1e-6, "groove starts at z = {low}");
        let reversed = ThreadSpec {
            reversed: true,
            ..short
        };
        let threaded = thread(OpId::new(2), &shaft, side(), &reversed).unwrap();
        let high = threaded
            .faces
            .iter()
            .filter(|face| face.key.op == OpId::new(2))
            .flat_map(|face| face.polygons.iter().flat_map(|p| p.vertices.iter()))
            .map(|v| v.z)
            .fold(f64::NEG_INFINITY, f64::max);
        assert!((high - 8.0).abs() < 1e-6, "groove ends at z = {high}");
    }

    #[test]
    fn left_and_right_hands_are_mirror_images() {
        let tess = Tessellation::default();
        let shaft = cylinder(OpId::new(1), Vec3::ZERO, Vec3::Z, 5.0, 10.0, &tess);
        let right = thread(OpId::new(2), &shaft, side(), &spec(2.0)).unwrap();
        let left_spec = ThreadSpec {
            left_handed: true,
            ..spec(2.0)
        };
        let left = thread(OpId::new(2), &shaft, side(), &left_spec).unwrap();
        left.validate().unwrap();
        assert!((right.volume() - left.volume()).abs() < 1e-3 * right.volume());
        // A right-hand groove climbs as it turns counter-clockwise about +Z. Along the
        // root's helical edges the rise and the turn have the hand's sign; its axial
        // edges turn by nothing and say nothing.
        let climb = |s: &Solid| -> f64 {
            let root = s
                .face(FaceKey::new(OpId::new(2), FaceRole::Thread(0)))
                .unwrap();
            let mut sum = 0.0;
            for p in &root.polygons {
                for (i, a) in p.vertices.iter().enumerate() {
                    let b = p.vertices[(i + 1) % p.vertices.len()];
                    let mut turn = b.y.atan2(b.x) - a.y.atan2(a.x);
                    if turn > PI {
                        turn -= TAU;
                    } else if turn < -PI {
                        turn += TAU;
                    }
                    sum += turn * (b.z - a.z);
                }
            }
            sum
        };
        assert!(climb(&right) > 0.0);
        assert!(climb(&left) < 0.0);
    }

    #[test]
    fn a_flat_face_a_coarse_pitch_and_an_overlong_length_are_refused() {
        let tess = Tessellation::default();
        let shaft = cylinder(OpId::new(1), Vec3::ZERO, Vec3::Z, 2.0, 10.0, &tess);
        let cap = FaceKey::new(OpId::new(1), FaceRole::EndCap);
        assert_eq!(
            thread(OpId::new(2), &shaft, cap, &spec(0.4)),
            Err(KernelError::NotCylindricalFace(cap))
        );
        assert!(matches!(
            thread(OpId::new(2), &shaft, side(), &spec(4.0)),
            Err(KernelError::ThreadTooDeep { .. })
        ));
        assert!(matches!(
            thread(OpId::new(2), &shaft, side(), &spec(0.0)),
            Err(KernelError::NonPositivePitch)
        ));
        let long = ThreadSpec {
            length: Some(11.0),
            ..spec(0.4)
        };
        assert!(matches!(
            thread(OpId::new(2), &shaft, side(), &long),
            Err(KernelError::ThreadLength { .. })
        ));
        let dense = ThreadSpec {
            pitch: 0.01,
            ..spec(0.4)
        };
        assert!(matches!(
            thread(OpId::new(2), &shaft, side(), &dense),
            Err(KernelError::ThreadTooDense { .. })
        ));
    }

    /// The standard sizes, both ways round, and a blind hole whose floor the groove must
    /// stop at. Each has to come back closed; a leak here is the boolean losing an edge,
    /// which the retries exist to get past.
    #[test]
    fn standard_threads_close_on_shafts_and_in_through_and_blind_holes() {
        let tess = Tessellation::default();
        let plate = cuboid(OpId::new(2), Vec3::splat(-12.0), Vec3::new(12.0, 12.0, 0.0));
        for (size, pitch) in [(3.0, 0.5), (6.0, 1.0), (10.0, 1.5), (16.0, 2.0)] {
            let shaft = cylinder(OpId::new(1), Vec3::ZERO, Vec3::Z, size / 2.0, 12.0, &tess);
            let tap = size / 2.0 - depth(pitch);
            let through = cylinder(
                OpId::new(1),
                Vec3::new(0.0, 0.0, -13.0),
                Vec3::Z,
                tap,
                14.0,
                &tess,
            );
            let blind = cylinder(
                OpId::new(1),
                Vec3::new(0.0, 0.0, -8.0),
                Vec3::Z,
                tap,
                9.0,
                &tess,
            );
            for (name, body) in [
                ("shaft", shaft),
                (
                    "through hole",
                    boolean(&plate, &through, BoolOp::Subtract).unwrap(),
                ),
                (
                    "blind hole",
                    boolean(&plate, &blind, BoolOp::Subtract).unwrap(),
                ),
            ] {
                let threaded = thread(OpId::new(3), &body, side(), &spec(pitch))
                    .unwrap_or_else(|e| panic!("M{size} {name}: {e}"));
                assert!(
                    threaded.volume() < body.volume(),
                    "M{size} {name} took nothing off"
                );
            }
        }
    }
}
