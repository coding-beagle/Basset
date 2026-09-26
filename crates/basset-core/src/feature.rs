//! Parametric inputs of every timeline operation.
//!
//! A feature stores only what the user chose, never derived geometry, so that changing
//! any input and replaying the timeline is the single source of truth. Adding a tool to
//! Basset means adding a variant here and an evaluator in `regen`; nothing else.

use std::collections::BTreeMap;

use basset_math::Affine3;
use basset_sketch::Sketch;
use serde::{Deserialize, Serialize};

use crate::ids::{ComponentId, FeatureId};
use crate::refs::{AxisRef, BodyRef, EdgeRef, FaceRef, PathRef, PlaneRef, RegionRef};

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Feature {
    pub id: FeatureId,
    pub name: String,
    /// Suppressed features are skipped during replay but keep their place and inputs.
    pub suppressed: bool,
    pub kind: FeatureKind,
    /// Feature values driven by an expression over the document's parameters, keyed by the
    /// field they drive.
    ///
    /// A side table rather than a field beside each number, because the numbers live in
    /// the variants of `FeatureKind` and most of them are never driven: giving every one
    /// an `Option<String>` neighbour would add a field to a dozen variants, make every
    /// construction site say `None` a dozen times, and still leave the two halves free to
    /// disagree. Keyed by [`NumericField`], the driven values are exactly the entries that
    /// exist, an undriven feature costs nothing on disk, and a kind that gains a number
    /// later needs no change here at all.
    ///
    /// The string is what the user wrote; the evaluated number is written into the kind,
    /// so replay and every reader of the feature see a plain number whether or not an
    /// expression put it there.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub exprs: BTreeMap<NumericField, String>,
}

/// A number inside a [`FeatureKind`] that an expression can drive.
///
/// Named by role rather than by field, so one key means the same thing across the kinds
/// that offer it and the UI can label it without matching on the kind.
#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug, Serialize, Deserialize)]
pub enum NumericField {
    Distance,
    /// The second distance of a two-sided extent, the one opposite the normal.
    Negative,
    Angle,
    Radius,
}

impl NumericField {
    /// What a panel calls this field.
    pub fn label(self) -> &'static str {
        match self {
            NumericField::Distance => "Distance",
            NumericField::Negative => "Second distance",
            NumericField::Angle => "Angle",
            NumericField::Radius => "Radius",
        }
    }
}

impl std::fmt::Display for NumericField {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.label())
    }
}

/// How far a profile is pushed. Distances are along the profile plane's normal.
#[derive(Copy, Clone, PartialEq, Debug, Serialize, Deserialize)]
pub enum Extent {
    OneSide(f64),
    Symmetric(f64),
    TwoSides {
        positive: f64,
        negative: f64,
    },
    /// Up to a face of an existing body, Fusion's "to object": the reach is worked out
    /// from the target at replay time, so the extrusion follows the target through
    /// edits. The face is named by the same [`FaceRef`] a sketch-on-face stores, whose
    /// `FaceKey` is derived from the operation that made the face and so survives
    /// regeneration.
    ToFace(FaceRef),
}

/// What a body-creating feature does with the solid it produces.
///
/// The boolean variants carry every body the solid is applied to, as Fusion's do: a cut
/// whose path passes through several bodies cuts each of the ones listed, with the same
/// tool solid. An empty list is a feature error at replay time, not a panic — the dialog
/// never builds one, but an edited file can say anything.
#[derive(Clone, PartialEq, Debug, Serialize, Deserialize)]
pub enum BodyOp {
    NewBody,
    Join(Vec<BodyRef>),
    Cut(Vec<BodyRef>),
    Intersect(Vec<BodyRef>),
}

impl BodyOp {
    /// The bodies the produced solid is applied to; empty for a new body.
    pub fn targets(&self) -> &[BodyRef] {
        match self {
            BodyOp::NewBody => &[],
            BodyOp::Join(b) | BodyOp::Cut(b) | BodyOp::Intersect(b) => b,
        }
    }
}

#[derive(Copy, Clone, PartialEq, Eq, Debug, Serialize, Deserialize)]
pub enum CombineOp {
    Join,
    Cut,
    Intersect,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum FeatureKind {
    NewComponent {
        name: String,
        parent: ComponentId,
    },
    Sketch {
        plane: PlaneRef,
        component: ComponentId,
        sketch: Sketch,
    },
    OffsetPlane {
        base: PlaneRef,
        distance: f64,
    },
    AngledPlane {
        base: PlaneRef,
        axis: AxisRef,
        angle: f64,
    },
    Extrude {
        regions: Vec<RegionRef>,
        extent: Extent,
        operation: BodyOp,
        component: ComponentId,
    },
    Revolve {
        regions: Vec<RegionRef>,
        axis: AxisRef,
        angle: f64,
        operation: BodyOp,
        component: ComponentId,
    },
    Sweep {
        regions: Vec<RegionRef>,
        path: PathRef,
        operation: BodyOp,
        component: ComponentId,
    },
    Loft {
        regions: Vec<RegionRef>,
        operation: BodyOp,
        component: ComponentId,
    },
    Fillet {
        edges: Vec<EdgeRef>,
        radius: f64,
    },
    Chamfer {
        edges: Vec<EdgeRef>,
        distance: f64,
    },
    Combine {
        target: BodyRef,
        tools: Vec<BodyRef>,
        operation: CombineOp,
        keep_tools: bool,
    },
    Move {
        body: BodyRef,
        transform: Affine3,
    },
}

impl FeatureKind {
    /// Human-readable default name used when the user does not provide one.
    pub fn default_name(&self) -> &'static str {
        match self {
            FeatureKind::NewComponent { .. } => "Component",
            FeatureKind::Sketch { .. } => "Sketch",
            FeatureKind::OffsetPlane { .. } => "Offset Plane",
            FeatureKind::AngledPlane { .. } => "Angled Plane",
            FeatureKind::Extrude { .. } => "Extrude",
            FeatureKind::Revolve { .. } => "Revolve",
            FeatureKind::Sweep { .. } => "Sweep",
            FeatureKind::Loft { .. } => "Loft",
            FeatureKind::Fillet { .. } => "Fillet",
            FeatureKind::Chamfer { .. } => "Chamfer",
            FeatureKind::Combine { .. } => "Combine",
            FeatureKind::Move { .. } => "Move",
        }
    }

    /// Which of this kind's numbers an expression can drive, in the order a panel shows
    /// them.
    pub fn numeric_fields(&self) -> &'static [NumericField] {
        use NumericField::{Angle, Distance, Negative, Radius};
        match self {
            FeatureKind::OffsetPlane { .. } | FeatureKind::Chamfer { .. } => &[Distance],
            FeatureKind::AngledPlane { .. } | FeatureKind::Revolve { .. } => &[Angle],
            FeatureKind::Extrude { extent, .. } => match extent {
                Extent::OneSide(_) | Extent::Symmetric(_) => &[Distance],
                Extent::TwoSides { .. } => &[Distance, Negative],
                // The reach comes from the target, so there is no number to drive.
                Extent::ToFace(_) => &[],
            },
            FeatureKind::Fillet { .. } => &[Radius],
            _ => &[],
        }
    }

    /// The current value of one field, in the unit the user types it in.
    ///
    /// `None` when this kind has no such field. See [`FeatureKind::set_numeric_field`] for
    /// the units.
    pub fn numeric_field(&self, field: NumericField) -> Option<f64> {
        match (self, field) {
            (FeatureKind::OffsetPlane { distance, .. }, NumericField::Distance) => Some(*distance),
            (FeatureKind::Chamfer { distance, .. }, NumericField::Distance) => Some(*distance),
            (FeatureKind::Fillet { radius, .. }, NumericField::Radius) => Some(*radius),
            (FeatureKind::AngledPlane { angle, .. }, NumericField::Angle)
            | (FeatureKind::Revolve { angle, .. }, NumericField::Angle) => Some(angle.to_degrees()),
            (FeatureKind::Extrude { extent, .. }, NumericField::Distance) => match extent {
                Extent::OneSide(d) | Extent::Symmetric(d) => Some(*d),
                Extent::TwoSides { positive, .. } => Some(*positive),
                Extent::ToFace(_) => None,
            },
            (
                FeatureKind::Extrude {
                    extent: Extent::TwoSides { negative, .. },
                    ..
                },
                NumericField::Negative,
            ) => Some(*negative),
            _ => None,
        }
    }

    /// Writes one field, in the unit the user types it in. Returns whether this kind has
    /// that field at all.
    ///
    /// # Units
    ///
    /// These two accessors work in the unit the user *types*: degrees for
    /// [`NumericField::Angle`], millimetres for everything else. The model stores radians,
    /// so the conversion happens here and nowhere else. That is the same rule
    /// `Sketch::apply_binding` follows for a dimension driven by an expression, and the
    /// same one the UI already follows by holding an `angle_deg` in its parameters: an
    /// expression is a number the user would otherwise have typed into that box, so it
    /// must mean what typing it there would have meant. Unlike the sketch's angle
    /// constraint, a feature angle's sign is a direction the user chose rather than a
    /// solver branch, so the sign is taken from the expression and not preserved.
    pub fn set_numeric_field(&mut self, field: NumericField, value: f64) -> bool {
        match (self, field) {
            (FeatureKind::OffsetPlane { distance, .. }, NumericField::Distance)
            | (FeatureKind::Chamfer { distance, .. }, NumericField::Distance) => {
                *distance = value;
                true
            }
            (FeatureKind::Fillet { radius, .. }, NumericField::Radius) => {
                *radius = value;
                true
            }
            (FeatureKind::AngledPlane { angle, .. }, NumericField::Angle)
            | (FeatureKind::Revolve { angle, .. }, NumericField::Angle) => {
                *angle = value.to_radians();
                true
            }
            (FeatureKind::Extrude { extent, .. }, NumericField::Distance) => match extent {
                Extent::OneSide(d) | Extent::Symmetric(d) => {
                    *d = value;
                    true
                }
                Extent::TwoSides { positive, .. } => {
                    *positive = value;
                    true
                }
                Extent::ToFace(_) => false,
            },
            (
                FeatureKind::Extrude {
                    extent: Extent::TwoSides { negative, .. },
                    ..
                },
                NumericField::Negative,
            ) => {
                *negative = value;
                true
            }
            _ => false,
        }
    }

    /// Whether this feature creates a body named after itself (as opposed to editing one).
    pub fn creates_body(&self) -> bool {
        match self {
            FeatureKind::Extrude { operation, .. }
            | FeatureKind::Revolve { operation, .. }
            | FeatureKind::Sweep { operation, .. }
            | FeatureKind::Loft { operation, .. } => matches!(operation, BodyOp::NewBody),
            _ => false,
        }
    }

    /// Every feature id this feature depends on. Used to flag dependants when a feature
    /// fails or is deleted, and to keep deletion honest about what else will break.
    pub fn dependencies(&self) -> Vec<FeatureId> {
        let mut out = Vec::new();
        let mut plane = |p: &PlaneRef| match p {
            PlaneRef::Origin(_) => {}
            PlaneRef::Feature(f) => out.push(*f),
            PlaneRef::Face(face) => out.push(face.body.0),
        };
        match self {
            FeatureKind::NewComponent { parent, .. } => {
                if *parent != ComponentId::ROOT {
                    out.push(FeatureId(parent.0));
                }
            }
            FeatureKind::Sketch { plane: p, .. } => plane(p),
            FeatureKind::OffsetPlane { base, .. } => plane(base),
            FeatureKind::AngledPlane { base, axis, .. } => {
                plane(base);
                if let AxisRef::SketchLine { sketch, .. } = axis {
                    out.push(*sketch);
                }
            }
            FeatureKind::Extrude {
                regions,
                extent,
                operation,
                ..
            } => {
                out.extend(regions.iter().map(RegionRef::source));
                if let Extent::ToFace(face) = extent {
                    out.push(face.body.0);
                }
                out.extend(operation.targets().iter().map(|b| b.0));
            }
            FeatureKind::Loft {
                regions, operation, ..
            } => {
                out.extend(regions.iter().map(RegionRef::source));
                out.extend(operation.targets().iter().map(|b| b.0));
            }
            FeatureKind::Revolve {
                regions,
                axis,
                operation,
                ..
            } => {
                out.extend(regions.iter().map(RegionRef::source));
                if let AxisRef::SketchLine { sketch, .. } = axis {
                    out.push(*sketch);
                }
                out.extend(operation.targets().iter().map(|b| b.0));
            }
            FeatureKind::Sweep {
                regions,
                path,
                operation,
                ..
            } => {
                out.extend(regions.iter().map(RegionRef::source));
                out.push(path.sketch);
                out.extend(operation.targets().iter().map(|b| b.0));
            }
            FeatureKind::Fillet { edges, .. } | FeatureKind::Chamfer { edges, .. } => {
                out.extend(edges.iter().map(|e| e.body.0));
            }
            FeatureKind::Combine { target, tools, .. } => {
                out.push(target.0);
                out.extend(tools.iter().map(|b| b.0));
            }
            FeatureKind::Move { body, .. } => out.push(body.0),
        }
        out.sort_unstable();
        out.dedup();
        out
    }
}

#[cfg(test)]
mod tests {
    use std::f64::consts::PI;

    use crate::refs::{OriginAxis, OriginPlane};

    use super::*;

    fn every_kind() -> Vec<FeatureKind> {
        let plane = PlaneRef::Origin(OriginPlane::XY);
        let axis = AxisRef::Origin(OriginAxis::X);
        vec![
            FeatureKind::OffsetPlane {
                base: plane,
                distance: 1.0,
            },
            FeatureKind::AngledPlane {
                base: plane,
                axis,
                angle: 0.0,
            },
            FeatureKind::Extrude {
                regions: Vec::new(),
                extent: Extent::OneSide(1.0),
                operation: BodyOp::NewBody,
                component: ComponentId::ROOT,
            },
            FeatureKind::Extrude {
                regions: Vec::new(),
                extent: Extent::Symmetric(1.0),
                operation: BodyOp::NewBody,
                component: ComponentId::ROOT,
            },
            FeatureKind::Extrude {
                regions: Vec::new(),
                extent: Extent::TwoSides {
                    positive: 1.0,
                    negative: 2.0,
                },
                operation: BodyOp::NewBody,
                component: ComponentId::ROOT,
            },
            FeatureKind::Extrude {
                regions: Vec::new(),
                extent: Extent::ToFace(crate::refs::FaceRef {
                    body: BodyRef(FeatureId(1)),
                    key: basset_kernel::FaceKey::new(
                        basset_kernel::OpId::new(1),
                        basset_kernel::FaceRole::EndCap,
                    ),
                }),
                operation: BodyOp::NewBody,
                component: ComponentId::ROOT,
            },
            FeatureKind::Revolve {
                regions: Vec::new(),
                axis,
                angle: 0.0,
                operation: BodyOp::NewBody,
                component: ComponentId::ROOT,
            },
            FeatureKind::Fillet {
                edges: Vec::new(),
                radius: 1.0,
            },
            FeatureKind::Chamfer {
                edges: Vec::new(),
                distance: 1.0,
            },
            FeatureKind::Loft {
                regions: Vec::new(),
                operation: BodyOp::NewBody,
                component: ComponentId::ROOT,
            },
        ]
    }

    #[test]
    fn every_offered_field_of_every_kind_reads_back_what_was_written() {
        for mut kind in every_kind() {
            for (n, field) in kind.numeric_fields().iter().copied().enumerate() {
                let written = 3.0 + n as f64;
                assert!(
                    kind.set_numeric_field(field, written),
                    "{field} is offered by {} but cannot be written",
                    kind.default_name()
                );
                let read = kind.numeric_field(field).expect("an offered field reads");
                // Not exactly equal: an angle goes to radians and back on the way.
                assert!(
                    (read - written).abs() < 1e-12,
                    "{field} of {} read back as {read}, not {written}",
                    kind.default_name()
                );
            }
        }
    }

    #[test]
    fn a_field_a_kind_does_not_offer_is_neither_read_nor_written() {
        use NumericField::{Angle, Distance, Negative, Radius};
        for mut kind in every_kind() {
            let offered = kind.numeric_fields().to_vec();
            for field in [Distance, Negative, Angle, Radius] {
                if offered.contains(&field) {
                    continue;
                }
                assert_eq!(kind.numeric_field(field), None);
                assert!(!kind.set_numeric_field(field, 7.0));
            }
        }
    }

    #[test]
    fn the_two_sides_of_an_extrude_are_separate_fields() {
        let mut kind = FeatureKind::Extrude {
            regions: Vec::new(),
            extent: Extent::TwoSides {
                positive: 1.0,
                negative: 2.0,
            },
            operation: BodyOp::NewBody,
            component: ComponentId::ROOT,
        };
        kind.set_numeric_field(NumericField::Distance, 10.0);
        assert_eq!(kind.numeric_field(NumericField::Negative), Some(2.0));
        kind.set_numeric_field(NumericField::Negative, 20.0);
        assert_eq!(kind.numeric_field(NumericField::Distance), Some(10.0));
    }

    /// The accessors work in the unit the user types, and angles are the only field where
    /// that differs from what the model stores.
    #[test]
    fn an_angle_is_written_and_read_in_degrees_while_the_model_holds_radians() {
        let mut kind = FeatureKind::Revolve {
            regions: Vec::new(),
            axis: AxisRef::Origin(OriginAxis::Z),
            angle: 0.0,
            operation: BodyOp::NewBody,
            component: ComponentId::ROOT,
        };
        kind.set_numeric_field(NumericField::Angle, 90.0);
        let FeatureKind::Revolve { angle, .. } = &kind else {
            unreachable!()
        };
        assert!(
            (angle - PI / 2.0).abs() < 1e-12,
            "{angle} is not 90 degrees"
        );
        assert!((kind.numeric_field(NumericField::Angle).unwrap() - 90.0).abs() < 1e-12);

        let mut plane = FeatureKind::AngledPlane {
            base: PlaneRef::Origin(OriginPlane::XY),
            axis: AxisRef::Origin(OriginAxis::X),
            angle: PI,
        };
        assert!((plane.numeric_field(NumericField::Angle).unwrap() - 180.0).abs() < 1e-12);
        // A sign is a direction the user chose, so it comes from the expression as typed.
        plane.set_numeric_field(NumericField::Angle, -45.0);
        let FeatureKind::AngledPlane { angle, .. } = &plane else {
            unreachable!()
        };
        assert!((angle + PI / 4.0).abs() < 1e-12);
    }
}
