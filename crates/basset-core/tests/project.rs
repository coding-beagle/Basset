//! A sketch started on a face of a real body opens with that face's outline in it, as
//! the kernel traced it: lines where the neighbours are flat and a circle where a hole
//! goes through, pinned so nothing later can move them.

use std::f64::consts::PI;

use basset_core::{
    BodyOp, BodyRef, ComponentId, Document, Extent, FaceKey, FaceRole, FeatureKind, OriginPlane,
    PlaneRef, ProfileRef, RegionRef, Sketch, project_face,
};
use basset_kernel::OpId;
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
        regions: vec![RegionRef::Profile(ProfileRef {
            sketch,
            sample: Vec2::new(1.0, 1.0),
        })],
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
