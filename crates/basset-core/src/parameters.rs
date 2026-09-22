//! Parameters shared by the whole document.
//!
//! A sketch has always been able to name a number (`hole_spacing = 12.5`) and drive its
//! dimensions from it, but a model is rarely one sketch: the plate thickness that sets an
//! extrude distance is usually the same number as the one a later sketch measures from,
//! and repeating it in every sketch means editing it in every sketch. So the same table
//! lives once more at the document, behind every sketch and in front of every feature.
//!
//! # Where a name is looked up
//!
//! A sketch resolves a name in its own table first and asks the document only for what it
//! does not define, so a sketch parameter *shadows* a document one of the same name. That
//! ordering is what made lifting parameters up need no migration at all: every sketch that
//! already had a `width` still means its own. `basset-sketch` never learns that documents
//! exist — it takes the outer table as a closure ([`basset_sketch::Outer`]), which
//! [`Parameters::lookup`] hands it.
//!
//! # Deliberate limitations
//!
//! * Resolution is by name with no scope syntax, so a sketch parameter cannot refer to the
//!   document parameter it shadows: `width = width * 2` is a cycle and is reported as one.
//! * The table is flat and insertion-ordered, not a dependency graph. A parameter may
//!   refer to one defined after it; the order is the order the user typed, because that is
//!   the order they will read it in.
//! * Evaluation is not cached. Every read walks the expressions again, which is fine for
//!   the handful of parameters a document has and keeps "the value is always what the text
//!   says" true without an invalidation scheme to get wrong.

use basset_sketch::{Parameter, SketchError, expr};
use serde::{Deserialize, Serialize};

/// The document's parameter table, in the order the user added to it.
///
/// Serialised as a plain array of `{name, expr}` so the `.bass` file reads as the table
/// does.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(transparent)]
pub struct Parameters {
    rows: Vec<Parameter>,
}

impl Parameters {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn rows(&self) -> &[Parameter] {
        &self.rows
    }

    pub fn get(&self, name: &str) -> Option<&Parameter> {
        self.rows.iter().find(|p| p.name == name)
    }

    pub fn len(&self) -> usize {
        self.rows.len()
    }

    pub fn is_empty(&self) -> bool {
        self.rows.is_empty()
    }

    /// Value of one named parameter.
    pub fn value(&self, name: &str) -> Result<f64, SketchError> {
        self.resolve(name, &mut Vec::new())
    }

    /// Evaluates an expression written over this table.
    pub fn evaluate(&self, text: &str) -> Result<f64, SketchError> {
        expr::eval(text, &|name| self.resolve(name, &mut Vec::new()))
    }

    /// Adds a parameter or replaces the expression of an existing one.
    ///
    /// The expression is evaluated before it is kept, so an unknown name or a cycle is
    /// reported to the user instead of quietly un-driving every dimension and feature that
    /// depends on this name. A rejected edit puts the previous row back where it was: the
    /// table is a list the user reads top to bottom, and a failed edit must not reorder it.
    pub fn set(&mut self, name: &str, expression: &str) -> Result<f64, SketchError> {
        if !expr::is_valid_name(name) {
            return Err(SketchError::InvalidArgument(format!(
                "{name:?} is not a valid parameter name: use a letter or underscore \
                 followed by letters, digits or underscores"
            )));
        }
        let previous = self
            .rows
            .iter()
            .position(|p| p.name == name)
            .map(|at| (at, self.rows[at].clone()));
        match self.rows.iter_mut().find(|p| p.name == name) {
            Some(p) => p.expr = expression.to_string(),
            None => self.rows.push(Parameter {
                name: name.to_string(),
                expr: expression.to_string(),
            }),
        }
        match self.value(name) {
            Ok(value) => Ok(value),
            Err(e) => {
                self.rows.retain(|p| p.name != name);
                if let Some((at, p)) = previous {
                    self.rows.insert(at.min(self.rows.len()), p);
                }
                Err(e)
            }
        }
    }

    /// Removes a parameter. Anything driven by an expression that mentions it keeps the
    /// value it last had and is reported as warned on the next regeneration, because a
    /// model that silently collapsed to zero would be far worse than one that complains.
    pub fn remove(&mut self, name: &str) -> bool {
        let before = self.rows.len();
        self.rows.retain(|p| p.name != name);
        before != self.rows.len()
    }

    /// Renames a parameter and rewrites every expression in this table that mentions it.
    ///
    /// Renaming is not remove-then-add: references are by name, so anything short of a
    /// rewrite turns every dependent expression into an unknown name. The document rewrites
    /// features and sketches on top of this; see [`crate::Document::rename_parameter`].
    pub fn rename(&mut self, from: &str, to: &str) -> Result<(), SketchError> {
        if from == to {
            return Ok(());
        }
        if !expr::is_valid_name(to) {
            return Err(SketchError::InvalidArgument(format!(
                "{to:?} is not a valid parameter name: use a letter or underscore \
                 followed by letters, digits or underscores"
            )));
        }
        if self.get(from).is_none() {
            return Err(SketchError::UnknownParameter(from.to_string()));
        }
        if self.get(to).is_some() {
            return Err(SketchError::InvalidArgument(format!(
                "this document already has a parameter named {to:?}"
            )));
        }
        for p in &mut self.rows {
            if p.name == from {
                p.name = to.to_string();
            }
            p.expr = expr::rename(&p.expr, from, to);
        }
        Ok(())
    }

    /// Whether any row's expression reads `name`. Deleting a parameter something still
    /// reads is worth warning about.
    pub fn mentions(&self, name: &str) -> bool {
        self.rows
            .iter()
            .any(|p| expr::referenced_names(&p.expr).iter().any(|n| n == name))
    }

    /// The table as the sketch layer wants it: a closure to pass as
    /// [`basset_sketch::Outer`]. `&params.lookup()` coerces to it.
    pub fn lookup(&self) -> impl Fn(&str) -> Result<f64, SketchError> + '_ {
        move |name: &str| self.value(name)
    }

    /// Value of one name, carrying the chain of names currently being resolved so a
    /// parameter that refers to itself — directly or round a loop — is an error rather
    /// than a hang. The same explicit stack the sketch table uses, for the same reason.
    fn resolve(&self, name: &str, stack: &mut Vec<String>) -> Result<f64, SketchError> {
        if stack.iter().any(|n| n == name) {
            stack.push(name.to_string());
            return Err(SketchError::CircularParameter(stack.join(" → ")));
        }
        let Some(parameter) = self.get(name) else {
            return Err(SketchError::UnknownParameter(name.to_string()));
        };
        let expression = parameter.expr.clone();
        stack.push(name.to_string());
        let value = expr::eval(&expression, &|inner| {
            self.resolve(inner, &mut stack.clone())
        });
        stack.pop();
        value
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn table(rows: &[(&str, &str)]) -> Parameters {
        let mut p = Parameters::new();
        for (name, expr) in rows {
            p.set(name, expr).expect("row is valid");
        }
        p
    }

    #[test]
    fn a_parameter_may_be_written_over_another_parameter() {
        let p = table(&[("thickness", "4"), ("plate", "thickness * 3")]);
        assert_eq!(p.value("plate").unwrap(), 12.0);
        assert_eq!(p.evaluate("plate + thickness").unwrap(), 16.0);
    }

    #[test]
    fn setting_a_parameter_twice_keeps_its_place_in_the_table() {
        let mut p = table(&[("a", "1"), ("b", "2"), ("c", "3")]);
        p.set("b", "20").unwrap();
        let names: Vec<&str> = p.rows().iter().map(|r| r.name.as_str()).collect();
        assert_eq!(names, ["a", "b", "c"]);
        assert_eq!(p.value("b").unwrap(), 20.0);
    }

    #[test]
    fn a_rejected_edit_leaves_the_table_exactly_as_it_was() {
        let mut p = table(&[("a", "1"), ("b", "a + 1"), ("c", "3")]);
        let before = p.clone();
        assert!(p.set("b", "nonsense_name * 2").is_err());
        assert_eq!(p, before);
        assert!(p.set("b", "b + 1").is_err());
        assert_eq!(p, before);
    }

    #[test]
    fn a_new_parameter_that_does_not_evaluate_is_not_added_at_all() {
        let mut p = table(&[("a", "1")]);
        assert!(matches!(
            p.set("b", "missing"),
            Err(SketchError::UnknownParameter(_))
        ));
        assert_eq!(p.len(), 1);
        assert!(p.get("b").is_none());
    }

    #[test]
    fn a_cycle_round_several_parameters_is_reported_rather_than_hanging() {
        let mut p = table(&[("a", "1"), ("b", "a")]);
        assert!(p.set("a", "b").is_err());
        assert!(matches!(
            table_with_cycle().value("a"),
            Err(SketchError::CircularParameter(_))
        ));
    }

    /// A cycle can only be built by bypassing [`Parameters::set`], which refuses one, so
    /// the read path is tested against a table assembled by hand — as a loaded file could
    /// be if it were edited outside Basset.
    fn table_with_cycle() -> Parameters {
        Parameters {
            rows: vec![
                Parameter {
                    name: "a".into(),
                    expr: "b".into(),
                },
                Parameter {
                    name: "b".into(),
                    expr: "a".into(),
                },
            ],
        }
    }

    #[test]
    fn an_invalid_name_is_refused_before_anything_is_stored() {
        let mut p = Parameters::new();
        assert!(p.set("2wide", "1").is_err());
        assert!(p.set("has space", "1").is_err());
        assert!(p.is_empty());
    }

    #[test]
    fn renaming_rewrites_the_expressions_that_mention_the_old_name() {
        let mut p = table(&[("w", "10"), ("area", "w * w"), ("w_2", "3")]);
        p.set("other", "w_2 + 1").unwrap();
        p.rename("w", "width").unwrap();
        assert_eq!(p.get("width").unwrap().expr, "10");
        assert_eq!(p.get("area").unwrap().expr, "width * width");
        // A name that merely starts with the old one is not a reference to it.
        assert_eq!(p.get("other").unwrap().expr, "w_2 + 1");
        assert_eq!(p.value("area").unwrap(), 100.0);
    }

    #[test]
    fn renaming_refuses_an_unknown_name_an_invalid_name_or_a_collision() {
        let mut p = table(&[("a", "1"), ("b", "2")]);
        assert!(matches!(
            p.rename("missing", "c"),
            Err(SketchError::UnknownParameter(_))
        ));
        assert!(p.rename("a", "2bad").is_err());
        assert!(p.rename("a", "b").is_err());
        assert_eq!(p, table(&[("a", "1"), ("b", "2")]));
    }

    #[test]
    fn removing_a_parameter_leaves_the_expressions_that_read_it_unevaluable() {
        let mut p = table(&[("a", "1"), ("b", "a + 1")]);
        assert!(p.mentions("a"));
        assert!(p.remove("a"));
        assert!(!p.remove("a"));
        assert!(matches!(
            p.value("b"),
            Err(SketchError::UnknownParameter(_))
        ));
    }

    #[test]
    fn the_lookup_closure_resolves_through_the_table() {
        let p = table(&[("a", "2"), ("b", "a * 5")]);
        let lookup = p.lookup();
        let outer: basset_sketch::Outer = &lookup;
        assert_eq!(outer("b").unwrap(), 10.0);
        assert!(outer("nope").is_err());
    }

    #[test]
    fn the_table_serialises_as_a_plain_array_of_rows() {
        let p = table(&[("a", "1"), ("b", "a + 1")]);
        let json = serde_json::to_value(&p).unwrap();
        assert_eq!(
            json,
            serde_json::json!([{"name": "a", "expr": "1"}, {"name": "b", "expr": "a + 1"}])
        );
        let back: Parameters = serde_json::from_value(json).unwrap();
        assert_eq!(back, p);
    }
}
