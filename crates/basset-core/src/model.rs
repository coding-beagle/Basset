//! The evaluated state of a document at the timeline cursor.
//!
//! Everything here is derived. Solids and solved sketches are shared through `Arc` so
//! that the regenerator can snapshot the state after every feature cheaply; a snapshot
//! is what lets an edit to feature *i* replay only features *i..* instead of everything.

use std::collections::BTreeMap;
use std::sync::Arc;

use basset_kernel::{Profile, Solid};
use basset_math::Frame;
use basset_sketch::Sketch;
use serde::{Deserialize, Serialize};

use crate::ids::{ComponentId, FeatureId};
use crate::refs::BodyRef;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum FeatureStatus {
    Ok,
    Suppressed,
    /// Replay continued past this feature; its outputs are missing and dependants will
    /// usually fail too. The message is what the UI shows in the timeline tooltip.
    Failed(String),
}

#[derive(Clone, Debug)]
pub struct Component {
    pub id: ComponentId,
    pub name: String,
    pub parent: Option<ComponentId>,
}

#[derive(Clone, Debug)]
pub struct Body {
    pub id: BodyRef,
    pub name: String,
    pub component: ComponentId,
    pub solid: Arc<Solid>,
}

#[derive(Clone, Debug)]
pub struct SolvedSketch {
    pub frame: Frame,
    pub component: ComponentId,
    /// The sketch after constraint solving; the timeline keeps the pre-solve inputs.
    pub sketch: Sketch,
    /// Closed regions in kernel form (frame attached), ready for extrude/revolve/loft.
    pub profiles: Vec<Profile>,
}

#[derive(Clone, Debug, Default)]
pub struct ModelState {
    pub components: BTreeMap<ComponentId, Component>,
    pub planes: BTreeMap<FeatureId, Frame>,
    pub sketches: BTreeMap<FeatureId, Arc<SolvedSketch>>,
    pub bodies: BTreeMap<BodyRef, Body>,
    pub statuses: BTreeMap<FeatureId, FeatureStatus>,
}

impl ModelState {
    pub fn with_root() -> Self {
        let mut s = Self::default();
        s.components.insert(
            ComponentId::ROOT,
            Component {
                id: ComponentId::ROOT,
                name: "Root".into(),
                parent: None,
            },
        );
        s
    }

    pub fn body(&self, id: BodyRef) -> Option<&Body> {
        self.bodies.get(&id)
    }

    pub fn status(&self, id: FeatureId) -> Option<&FeatureStatus> {
        self.statuses.get(&id)
    }

    pub fn failed_features(&self) -> impl Iterator<Item = (FeatureId, &str)> {
        self.statuses.iter().filter_map(|(id, s)| match s {
            FeatureStatus::Failed(msg) => Some((*id, msg.as_str())),
            _ => None,
        })
    }

    /// Bodies belonging to a component and, transitively, its children.
    pub fn bodies_in(&self, component: ComponentId) -> Vec<&Body> {
        self.bodies
            .values()
            .filter(|b| self.is_within(b.component, component))
            .collect()
    }

    fn is_within(&self, mut c: ComponentId, ancestor: ComponentId) -> bool {
        loop {
            if c == ancestor {
                return true;
            }
            match self.components.get(&c).and_then(|comp| comp.parent) {
                Some(p) => c = p,
                None => return false,
            }
        }
    }
}
