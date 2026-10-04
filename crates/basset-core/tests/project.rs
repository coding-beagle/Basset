//! A sketch started on a face of a real body opens with that face's outline in it, as
//! the kernel traced it: lines where the neighbours are flat and a circle where a hole
//! goes through, pinned against the user's drags and linked to the face, so an edit
//! upstream that reshapes the face reshapes the copy with it.

use std::f64::consts::PI;

use basset_core::{
    BodyOp, BodyRef, ComponentId, Document, Extent, FaceKey, FaceRef, FaceRole, FeatureId,
    FeatureKind, FeatureStatus, OriginPlane, PlaneRef, ProfileRef, RegionRef, Sketch, file,
    project_face,
};
use basset_kernel::{OpId, SurfaceKind};
use basset_math::{Vec2, Vec3};
use basset_sketch::{Constraint, Entity, Tessellation, shapes};

#[test]
fn a_faces_outline_comes_back_as_the_curves_that_made_it() {
    let mut doc = Document::new("project");
    let mut plate = Sketch::new();
    shapes::rectangle_two_point(&mut plate, Vec2::ZERO, Vec2::new(10.0, 10.0));
    shapes::circle_center(&mut plate, Vec2::new(5.0, 5.0), 2.0);
    let sketch = doc.add_feature(FeatureKind::Sketch {
        plane: PlaneRef::Origin(OriginPlane::XY),
        component: ComponentId::ROOT,
        sketch: plate,
    });
    let body = doc.add_feature(FeatureKind::Extrude {
        regions: vec![RegionRef::Profile(ProfileRef::new(
            sketch,
            Vec2::new(1.0, 1.0),
        ))],
        extent: Extent::OneSide(2.0),
        operation: BodyOp::NewBody,
        component: ComponentId::ROOT,
    });
    let state = doc.state();
    let solid = &state.body(BodyRef(body)).unwrap().solid;
    let top = FaceKey::new(OpId::new(body.0), FaceRole::EndCap);
    let profile = solid.face_profile(top).unwrap();

    let mut on_face = Sketch::new();
    let curves = project_face(&mut on_face, &profile).unwrap();
    let mut kinds: Vec<_> = curves
        .iter()
        .map(|c| on_face.entity(*c).unwrap().entity.kind_name())
        .collect();
    kinds.sort_unstable();
    assert_eq!(kinds, ["circle", "line", "line", "line", "line"]);

    // The circle is the hole, drawn in the face's own frame: put back into the world it
    // sits where the hole was cut.
    let (center, radius) = curves
        .iter()
        .find_map(|c| match on_face.entity(*c).unwrap().entity {
            Entity::Circle { center, radius } => Some((center, radius)),
            _ => None,
        })
        .unwrap();
    assert!((radius - 2.0).abs() < 1e-6, "{radius}");
    let world = profile.frame.to_world(on_face.point_pos(center).unwrap());
    assert!(world.distance(Vec3::new(5.0, 5.0, 2.0)) < 1e-6, "{world:?}");

    // Every point is pinned, so the copy is a reference and not a loose sketch.
    let fixed = |id| {
        on_face
            .constraints()
            .any(|(_, c)| matches!(c, Constraint::Fix(p) if *p == id))
    };
    assert!(
        on_face
            .entities()
            .filter(|(_, e)| e.entity.is_point())
            .all(|(id, _)| fixed(id))
    );
    assert_eq!(on_face.solve().unwrap().degrees_of_freedom, 0);

    // The face's region is there to be extruded again: square minus hole.
    let profiles = on_face.profiles(&Tessellation::default());
    let ring = profiles.iter().map(|p| p.area()).fold(0.0_f64, f64::max);
    assert!((ring - (100.0 - PI * 4.0)).abs() < 0.2, "{ring}");
}

/// A 10 × 10 block `height` tall on the XY plane, and the id of its extrude.
fn block(doc: &mut Document, height: f64) -> FeatureId {
    let mut base = Sketch::new();
    shapes::rectangle_two_point(&mut base, Vec2::ZERO, Vec2::new(10.0, 10.0));
    let sketch = doc.add_feature(FeatureKind::Sketch {
        plane: PlaneRef::Origin(OriginPlane::XY),
        component: ComponentId::ROOT,
        sketch: base,
    });
    doc.add_feature(FeatureKind::Extrude {
        regions: vec![RegionRef::Profile(ProfileRef::new(
            sketch,
            Vec2::new(1.0, 1.0),
        ))],
        extent: Extent::OneSide(height),
        operation: BodyOp::NewBody,
        component: ComponentId::ROOT,
    })
}

/// A pinned circle of radius 1 at `at`, so the sketch holding it is fully constrained.
fn pinned_hole(sketch: &mut Sketch, at: Vec2) {
    let hole = shapes::circle_center(sketch, at, 1.0);
    sketch.add_constraint(Constraint::Fix(hole.center)).unwrap();
    sketch
        .add_constraint(Constraint::Diameter {
            curve: hole.circle,
            value: 2.0,
        })
        .unwrap();
}

fn cut(doc: &mut Document, sketch: FeatureId, sample: Vec2, body: FeatureId) -> FeatureId {
    doc.add_feature(FeatureKind::Extrude {
        regions: vec![RegionRef::Profile(ProfileRef::new(sketch, sample))],
        extent: Extent::OneSide(-2.0),
        operation: BodyOp::Cut(vec![BodyRef(body)]),
        component: ComponentId::ROOT,
    })
}

/// The wall of a block is made taller after a sketch was drawn on it. The copy of the
/// wall's outline in the sketch grows with it, and the hole drawn there stays where it
/// was in the world: neither the face's new extent nor its new middle moves it.
#[test]
fn a_walls_outline_follows_the_wall_and_what_is_drawn_on_it_stays_put() {
    let mut doc = Document::new("wall");
    let body = block(&mut doc, 4.0);
    let state = doc.state();
    let solid = &state.body(BodyRef(body)).unwrap().solid;
    let wall = solid
        .faces
        .iter()
        .find(|f| matches!(f.surface, SurfaceKind::Planar { normal } if normal.x > 0.99))
        .unwrap()
        .key;
    let mut on_wall = Sketch::new();
    project_face(&mut on_wall, &solid.face_profile(wall).unwrap()).unwrap();
    // The wall faces +x, so its sketch runs along world y and up world z from the point
    // where the world origin falls on it: (5, 2) is half way along and 2 up.
    pinned_hole(&mut on_wall, Vec2::new(5.0, 2.0));
    let sketch = doc.add_feature(FeatureKind::Sketch {
        plane: PlaneRef::Face(FaceRef {
            body: BodyRef(body),
            key: wall,
        }),
        component: ComponentId::ROOT,
        sketch: on_wall,
    });
    let hole = cut(&mut doc, sketch, Vec2::new(5.0, 2.0), body);
    let before = doc.state().body(BodyRef(body)).unwrap().solid.volume();
    assert!((before - (400.0 - PI * 2.0)).abs() < 0.1, "{before}");

    doc.edit_feature_kind(body, |k| {
        if let FeatureKind::Extrude { extent, .. } = k {
            *extent = Extent::OneSide(8.0);
        }
    })
    .unwrap();
    let state = doc.state();
    for id in [sketch, hole] {
        assert_eq!(state.status(id), Some(&FeatureStatus::Ok), "{id:?}");
    }
    let solved = &state.sketches[&sketch];
    let outline: Vec<Vec2> = solved
        .sketch
        .links()
        .filter_map(|(id, _)| solved.sketch.point_pos(id))
        .collect();
    assert_eq!(outline.len(), 4, "{outline:?}");
    let top = outline.iter().map(|p| p.y).fold(f64::MIN, f64::max);
    let bottom = outline.iter().map(|p| p.y).fold(f64::MAX, f64::min);
    assert!(
        (top - 8.0).abs() < 1e-6 && bottom.abs() < 1e-6,
        "the copy spans the wall as it is now: {outline:?}"
    );
    let centre = solved
        .sketch
        .entities()
        .find_map(|(_, e)| match e.entity {
            Entity::Circle { center, .. } => solved.sketch.point_pos(center),
            _ => None,
        })
        .unwrap();
    let world = solved.frame.to_world(centre);
    assert!(
        world.distance(Vec3::new(10.0, 5.0, 2.0)) < 1e-9,
        "{world:?}"
    );
    let after = state.body(BodyRef(body)).unwrap().solid.volume();
    assert!((after - (800.0 - PI * 2.0)).abs() < 0.1, "{after}");
}

/// A file from before version 10 put a sketch on a face with its origin at the face's
/// vertex average. Read now, the sketch is carried into the new frame without anything
/// moving, and its pinned copy of the face's outline is linked to the face.
#[test]
fn a_version_9_face_sketch_converts_without_moving() {
    let mut doc = Document::new("old");
    let body = block(&mut doc, 4.0);
    let top = FaceKey::new(OpId::new(body.0), FaceRole::EndCap);
    let legacy = doc
        .state()
        .body(BodyRef(body))
        .unwrap()
        .solid
        .face(top)
        .unwrap()
        .legacy_frame()
        .unwrap();
    // What the old editor drew: the top's outline and a hole at its middle, both in the
    // frame centred on the face, where the middle is (0, 0).
    let local = |x: f64, y: f64| {
        let p = legacy.to_local(Vec3::new(x, y, 4.0));
        Vec2::new(p.x, p.y)
    };
    let mut on_top = Sketch::new();
    let outline = shapes::rectangle_two_point(&mut on_top, local(0.0, 0.0), local(10.0, 10.0));
    for corner in outline.corners {
        on_top.add_constraint(Constraint::Fix(corner)).unwrap();
    }
    pinned_hole(&mut on_top, local(5.0, 5.0));
    assert!(local(5.0, 5.0).length() < 1e-9);
    let sketch = doc.add_feature(FeatureKind::Sketch {
        plane: PlaneRef::Face(FaceRef {
            body: BodyRef(body),
            key: top,
        }),
        component: ComponentId::ROOT,
        sketch: on_top,
    });
    cut(&mut doc, sketch, Vec2::ZERO, body);

    let mut bytes = Vec::new();
    file::write(&mut bytes, &doc).unwrap();
    let mut json: serde_json::Value = serde_json::from_slice(&bytes).unwrap();
    json["format_version"] = 9.into();
    let mut read = file::read(serde_json::to_vec(&json).unwrap().as_slice()).unwrap();

    let FeatureKind::Sketch { sketch: on_top, .. } = &read.timeline().get(sketch).unwrap().kind
    else {
        panic!("not a sketch")
    };
    assert_eq!(on_top.links().count(), 4, "the four corners are linked");
    let state = read.state();
    assert!(state.failed_features().next().is_none());
    let solved = &state.sketches[&sketch];
    let centre = solved
        .sketch
        .entities()
        .find_map(|(_, e)| match e.entity {
            Entity::Circle { center, .. } => solved.sketch.point_pos(center),
            _ => None,
        })
        .unwrap();
    let world = solved.frame.to_world(centre);
    assert!(world.distance(Vec3::new(5.0, 5.0, 4.0)) < 1e-9, "{world:?}");
    let solid = &state.body(BodyRef(body)).unwrap().solid;
    assert!((solid.volume() - (400.0 - PI * 2.0)).abs() < 0.1);
    let hole = solid.centroid();
    // The cut's region was found where the hole now is in the sketch.
    assert!(
        (hole.x - 5.0).abs() < 1e-6 && (hole.y - 5.0).abs() < 1e-6,
        "{hole:?}"
    );
}
