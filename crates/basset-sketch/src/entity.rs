//! Sketch entities. Points are first-class and every curve references points by id, so
//! moving a point moves every curve attached to it and the solver only ever sees point
//! coordinates and circle radii as unknowns.

use basset_math::Vec2;
use serde::{Deserialize, Serialize};

use crate::EntityId;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum Entity {
    Point {
        pos: Vec2,
    },
    Line {
        start: EntityId,
        end: EntityId,
    },
    Circle {
        center: EntityId,
        radius: f64,
    },
    /// Counter-clockwise from `start` to `end`. The radius is `|start − center|`; the
    /// solver keeps `|end − center|` equal to it through an implicit constraint.
    Arc {
        center: EntityId,
        start: EntityId,
        end: EntityId,
    },
    /// Text laid out along the +x direction rotated by `angle` (radians) with its
    /// baseline origin at `anchor`. `height` is the em size in mm.
    Text {
        anchor: EntityId,
        text: String,
        height: f64,
        angle: f64,
    },
}

impl Entity {
    /// Human-readable kind, used in error messages.
    pub fn kind_name(&self) -> &'static str {
        match self {
            Entity::Point { .. } => "point",
            Entity::Line { .. } => "line",
            Entity::Circle { .. } => "circle",
            Entity::Arc { .. } => "arc",
            Entity::Text { .. } => "text",
        }
    }

    /// Ids of the entities this one depends on (the points a curve is built from).
    pub fn references(&self) -> Vec<EntityId> {
        match *self {
            Entity::Point { .. } => Vec::new(),
            Entity::Line { start, end } => vec![start, end],
            Entity::Circle { center, .. } => vec![center],
            Entity::Arc { center, start, end } => vec![center, start, end],
            Entity::Text { anchor, .. } => vec![anchor],
        }
    }

    pub fn is_point(&self) -> bool {
        matches!(self, Entity::Point { .. })
    }

    pub fn is_line(&self) -> bool {
        matches!(self, Entity::Line { .. })
    }

    /// Circles and arcs: anything with a centre and a radius.
    pub fn is_circular(&self) -> bool {
        matches!(self, Entity::Circle { .. } | Entity::Arc { .. })
    }

    /// Lines and arcs: curves with two endpoints that can be chained.
    pub fn is_open_curve(&self) -> bool {
        matches!(self, Entity::Line { .. } | Entity::Arc { .. })
    }

    /// Lines, arcs and circles: anything with a shape that participates in profiles.
    pub fn is_curve(&self) -> bool {
        matches!(
            self,
            Entity::Line { .. } | Entity::Arc { .. } | Entity::Circle { .. }
        )
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct EntityData {
    pub entity: Entity,
    /// Construction geometry is solved and drawn but excluded from profiles.
    pub construction: bool,
}
