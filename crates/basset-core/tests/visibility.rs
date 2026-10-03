//! What the user hid is saved with the document and is not part of its history.

use basset_core::{BodyRef, Document, FeatureId, Visibility, file};

fn tidied() -> Visibility {
    Visibility {
        hidden_bodies: [BodyRef(FeatureId(4)), BodyRef(FeatureId(2))].into(),
        hidden_sketches: [FeatureId(1)].into(),
        show_origin: true,
        show_grid: false,
        show_axes: false,
    }
}

#[test]
fn hidden_items_survive_a_round_trip_through_a_file() {
    let mut doc = Document::new("tidied");
    doc.set_visibility(tidied());
    let mut bytes = Vec::new();
    file::write(&mut bytes, &doc).expect("writing to memory");
    let reopened = file::read(bytes.as_slice()).expect("reading back what was written");
    assert_eq!(*reopened.visibility(), tidied());
}

#[test]
fn a_new_document_hides_nothing_but_the_origin() {
    let doc = Document::new("fresh");
    let v = doc.visibility();
    assert!(v.hidden_bodies.is_empty() && v.hidden_sketches.is_empty());
    assert!(!v.show_origin && v.show_grid && v.show_axes);
}

#[test]
fn undo_leaves_visibility_alone() {
    let mut doc = Document::new("tidied");
    doc.set_parameter("w", "10").expect("a valid parameter");
    doc.set_visibility(tidied());
    assert!(doc.undo());
    assert_eq!(
        *doc.visibility(),
        tidied(),
        "hiding something is not an edit for undo to take back"
    );
}
