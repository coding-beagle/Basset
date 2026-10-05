//! What each body and face looks like, and the scene a render is lit by.
//!
//! Fusion keeps a design's appearances beside its geometry: a library entry applied to a
//! body is *copied into the design* ("In this design"), and editing that copy changes
//! every body wearing it without touching the library. [`Appearances`] is that list plus
//! the assignments: a document-wide default (Fusion's appearance on the root
//! component), one appearance per body, and overrides per face. Assignments name an
//! appearance by its name in the list, so editing it is one edit.
//!
//! None of it is geometry, so none of it is in the timeline. Assignments are keyed by the
//! same stable names the rest of the document uses — a [`BodyRef`] is the feature that
//! made the body and a [`FaceKey`] the operation and role that made the face — so a
//! colour given to the top of an extrude stays on it when the extrude is made taller,
//! exactly as a fillet written on its edge does. An assignment to a body or face that
//! no longer exists is not an error and is kept: undoing the delete brings the body
//! back in the colour it had.

use std::collections::BTreeMap;

use basset_kernel::{FaceKey, Tessellated};
use basset_render::{Appearance, SceneBuilder, SceneSettings, TraceScene};
use serde::{Deserialize, Serialize};

use crate::refs::BodyRef;

/// [`Appearance::DEFAULT`] with an address, for lookups that fall back to it by reference.
static DEFAULT: Appearance = Appearance::DEFAULT;

/// One face given an appearance of its own.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct FaceAppearance {
    pub body: BodyRef,
    pub face: FaceKey,
    pub appearance: String,
}

/// The design's appearances and where they are applied. Every field defaults, so a file
/// from before appearances were saved opens with every body in the default grey.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Appearances {
    /// "In this design": every appearance the document has taken a copy of, by name.
    pub defined: BTreeMap<String, Appearance>,
    /// What a body with no appearance of its own wears. `None` is
    /// [`Appearance::DEFAULT`], the grey the modelling viewport draws.
    pub default: Option<String>,
    pub bodies: BTreeMap<BodyRef, String>,
    /// Kept sorted by body and face, so the same assignments always save as the same
    /// bytes and a file under version control does not churn.
    pub faces: Vec<FaceAppearance>,
}

impl Appearances {
    pub fn is_empty(&self) -> bool {
        self.defined.is_empty()
            && self.default.is_none()
            && self.bodies.is_empty()
            && self.faces.is_empty()
    }

    pub fn get(&self, name: &str) -> Option<&Appearance> {
        self.defined.get(name)
    }

    /// The appearance a name refers to, falling back to the default grey when the name
    /// is not defined (a hand-edited file, or a definition removed under it).
    fn named(&self, name: Option<&str>) -> &Appearance {
        name.and_then(|n| self.defined.get(n)).unwrap_or(&DEFAULT)
    }

    /// What the document as a whole is painted with.
    pub fn document_default(&self) -> &Appearance {
        self.named(self.default.as_deref())
    }

    /// What a body is painted with, before any face of it says otherwise.
    pub fn body(&self, body: BodyRef) -> &Appearance {
        match self.bodies.get(&body) {
            Some(name) => self.named(Some(name.as_str())),
            None => self.document_default(),
        }
    }

    /// The appearance assigned to the body itself, if any (not the document default).
    pub fn body_assignment(&self, body: BodyRef) -> Option<&str> {
        self.bodies.get(&body).map(String::as_str)
    }

    pub fn face_assignment(&self, body: BodyRef, face: FaceKey) -> Option<&str> {
        self.faces
            .iter()
            .find(|f| f.body == body && f.face == face)
            .map(|f| f.appearance.as_str())
    }

    /// What one face is painted with: its own appearance, else its body's.
    pub fn face(&self, body: BodyRef, face: FaceKey) -> &Appearance {
        match self.face_assignment(body, face) {
            Some(name) => self.named(Some(name)),
            None => self.body(body),
        }
    }

    /// The faces of a body that override its appearance.
    pub fn face_overrides(&self, body: BodyRef) -> impl Iterator<Item = &FaceAppearance> {
        self.faces.iter().filter(move |f| f.body == body)
    }

    /// How many places use an appearance: bodies, faces, and the document default.
    pub fn uses(&self, name: &str) -> usize {
        self.bodies.values().filter(|n| *n == name).count()
            + self.faces.iter().filter(|f| f.appearance == name).count()
            + usize::from(self.default.as_deref() == Some(name))
    }

    /// Takes a copy of an appearance into the design if there is none of that name, and
    /// returns the name to assign. A copy already there wins: it may have been edited,
    /// and re-applying the library entry must not quietly undo the edit.
    pub(crate) fn define(&mut self, appearance: &Appearance) -> String {
        let name = appearance.display_name().to_owned();
        self.defined.entry(name.clone()).or_insert_with(|| {
            let mut copy = appearance.clone().sanitised();
            copy.name = name.clone();
            copy
        });
        name
    }

    pub(crate) fn assign_face(&mut self, body: BodyRef, face: FaceKey, name: Option<String>) {
        self.faces.retain(|f| !(f.body == body && f.face == face));
        if let Some(appearance) = name {
            self.faces.push(FaceAppearance {
                body,
                face,
                appearance,
            });
            self.faces.sort_by_key(|f| (f.body, f.face));
        }
    }

    /// Points every assignment of `from` at `to`.
    pub(crate) fn rename_references(&mut self, from: &str, to: &str) {
        for n in self.bodies.values_mut() {
            if n == from {
                *n = to.to_owned();
            }
        }
        for f in &mut self.faces {
            if f.appearance == from {
                f.appearance = to.to_owned();
            }
        }
        if self.default.as_deref() == Some(from) {
            self.default = Some(to.to_owned());
        }
    }
}

/// Builds what the path tracer traces from bodies' tessellations and the appearances
/// they wear. The caller supplies the tessellations — the editor has them on screen
/// already, the MCP server makes them fresh — and decides which bodies are in the
/// picture.
pub fn trace_scene<'a>(
    bodies: impl IntoIterator<Item = (BodyRef, &'a Tessellated)>,
    appearances: &Appearances,
    settings: &SceneSettings,
) -> TraceScene {
    let mut builder = SceneBuilder::new();
    let mut indices: BTreeMap<String, u32> = BTreeMap::new();
    let mut index_of = |builder: &mut SceneBuilder, a: &Appearance| -> u32 {
        // An appearance used by several bodies is one material to the tracer.
        let key = a.display_name().to_owned();
        if let Some(i) = indices.get(&key) {
            return *i;
        }
        let i = builder.add_appearance(a);
        indices.insert(key, i);
        i
    };
    for (body, tess) in bodies {
        let per_face: Vec<u32> = tess
            .face_keys
            .iter()
            .map(|key| index_of(&mut builder, appearances.face(body, *key)))
            .collect();
        let fallback = index_of(&mut builder, appearances.body(body));
        builder.add_mesh(&tess.mesh, |face| {
            per_face.get(face as usize).copied().unwrap_or(fallback)
        });
    }
    builder.build(settings)
}

#[cfg(test)]
mod tests {
    use basset_kernel::{FaceRole, OpId};

    use super::*;
    use crate::ids::FeatureId;

    fn red() -> Appearance {
        basset_render::find("Paint - Gloss Red").unwrap().clone()
    }

    #[test]
    fn faces_fall_back_to_their_body_and_bodies_to_the_document() {
        let mut a = Appearances::default();
        let body = BodyRef(FeatureId(3));
        let top = FaceKey::new(OpId::new(3), FaceRole::EndCap);
        let side = FaceKey::new(OpId::new(3), FaceRole::StartCap);
        assert_eq!(a.face(body, top), &Appearance::DEFAULT);

        let chrome = basset_render::find("Chrome").unwrap();
        let name = a.define(chrome);
        a.default = Some(name);
        assert_eq!(a.face(body, top).name, "Chrome");

        let red = a.define(&red());
        a.bodies.insert(body, red.clone());
        assert_eq!(a.face(body, side).name, "Paint - Gloss Red");

        let gold = a.define(basset_render::find("Gold - Polished").unwrap());
        a.assign_face(body, top, Some(gold));
        assert_eq!(a.face(body, top).name, "Gold - Polished");
        assert_eq!(a.face(body, side).name, "Paint - Gloss Red");
        assert_eq!(a.uses("Paint - Gloss Red"), 1);

        a.assign_face(body, top, None);
        assert_eq!(a.face(body, top).name, "Paint - Gloss Red");
    }

    #[test]
    fn defining_again_keeps_the_edited_copy() {
        let mut a = Appearances::default();
        let name = a.define(&red());
        a.defined.get_mut(&name).unwrap().roughness = 0.9;
        a.define(&red());
        assert_eq!(a.get(&name).unwrap().roughness, 0.9);
    }

    #[test]
    fn assignments_round_trip_through_json() {
        let mut a = Appearances::default();
        let name = a.define(&red());
        a.bodies.insert(BodyRef(FeatureId(7)), name.clone());
        a.assign_face(
            BodyRef(FeatureId(7)),
            FaceKey::new(OpId::new(7), FaceRole::Side(2)),
            Some(name),
        );
        let json = serde_json::to_string(&a).unwrap();
        let back: Appearances = serde_json::from_str(&json).unwrap();
        assert_eq!(back, a);
    }
}
