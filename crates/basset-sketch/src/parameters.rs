//! Named constants and expression-driven dimensions.
//!
//! A sketch carries a small table of named parameters (`hole_spacing = 12.5`) and a
//! binding from dimensions to expressions written over them. Solving evaluates the
//! bindings first, so changing one parameter re-drives every dimension that mentions it
//! — which is the point: a drawing states its intent once and the rest follows.
//!
//! Expressions are evaluated in the unit the dimension is *typed* in, degrees for angles
//! and millimetres for everything else, because the number the user writes in a panel
//! has to be the number they would have typed into the dimension box.
//!
//! # Two scopes
//!
//! A name is looked up in the sketch's own table first and, failing that, in an *outer*
//! table supplied by the caller — in practice the document's parameters, which every
//! sketch and every feature in the file shares. The outer table arrives as a closure
//! ([`Outer`]) rather than as data, so this crate still knows nothing about documents:
//! `expr::eval` already took a lookup, and this is that same seam carried one level up.
//!
//! The sketch's own table *shadows* the outer one, as a local variable shadows a global.
//! That is what lets a document define `thickness` for the whole model while one sketch
//! overrides it, and it is why lifting parameters to the document needed no migration:
//! every sketch that already had a `width` keeps meaning its own.
//!
//! Shadowing has one sharp edge, which is deliberate. A sketch parameter written over a
//! document one of the same name cannot refer to it — `width = width * 2` is a cycle,
//! not a reference outward — because resolution is by name and there is no syntax to
//! say which scope is meant. The error names the cycle, so what happens is at least
//! legible.
//!
//! # Renaming
//!
//! Parameters are referred to by name inside expression text, so renaming one has to
//! rewrite every expression that mentions it or the rename silently breaks the drawing.
//! [`Sketch::rename_parameter`] does that for the sketch's own table, and
//! [`Sketch::rewrite_outer_parameter`] does it on behalf of the document — skipping any
//! sketch that shadows the name, because there the references mean the local parameter
//! and must not follow the document's rename.

use serde::{Deserialize, Serialize};

use crate::{Constraint, ConstraintId, Sketch, SketchError, expr};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Parameter {
    pub name: String,
    /// What the user wrote. Kept verbatim so a panel shows `width / 2`, not `25`.
    pub expr: String,
}

/// A table of names the sketch does not define itself — the document's parameters.
///
/// A closure rather than a type, so `basset-sketch` stays below `basset-core` in the
/// dependency order and can still be tested with nothing behind it.
pub type Outer<'a> = &'a dyn Fn(&str) -> Result<f64, SketchError>;

/// The outer table of a sketch read on its own: empty. Every name has to come from the
/// sketch itself, which is what the crate's own tests and any caller without a document
/// want.
pub fn no_outer(name: &str) -> Result<f64, SketchError> {
    Err(SketchError::UnknownParameter(name.to_string()))
}

impl Sketch {
    pub fn parameters(&self) -> &[Parameter] {
        &self.parameters
    }

    pub fn parameter(&self, name: &str) -> Option<&Parameter> {
        self.parameters.iter().find(|p| p.name == name)
    }

    /// Current value of a named parameter, with nothing outside the sketch.
    pub fn parameter_value(&self, name: &str) -> Result<f64, SketchError> {
        self.parameter_value_with(name, &no_outer)
    }

    /// Current value of a named parameter, falling back to the document's table.
    pub fn parameter_value_with(&self, name: &str, outer: Outer) -> Result<f64, SketchError> {
        self.resolve(name, &mut Vec::new(), outer)
    }

    /// Adds a parameter or replaces the expression of an existing one. The expression is
    /// evaluated before it is kept, so a name that does not exist or a cycle is reported
    /// to the user instead of quietly breaking every dimension that depends on it.
    pub fn set_parameter(&mut self, name: &str, expression: &str) -> Result<f64, SketchError> {
        self.set_parameter_with(name, expression, &no_outer)
    }

    /// [`Sketch::set_parameter`], resolving names the sketch does not define through the
    /// document's table.
    pub fn set_parameter_with(
        &mut self,
        name: &str,
        expression: &str,
        outer: Outer,
    ) -> Result<f64, SketchError> {
        if !expr::is_valid_name(name) {
            return Err(SketchError::InvalidArgument(format!(
                "{name:?} is not a valid parameter name: use a letter or underscore \
                 followed by letters, digits or underscores"
            )));
        }
        let previous = self
            .parameters
            .iter()
            .position(|p| p.name == name)
            .map(|at| (at, self.parameters[at].clone()));
        match self.parameters.iter_mut().find(|p| p.name == name) {
            Some(p) => p.expr = expression.to_string(),
            None => self.parameters.push(Parameter {
                name: name.to_string(),
                expr: expression.to_string(),
            }),
        }
        match self.parameter_value_with(name, outer) {
            Ok(value) => {
                self.apply_parameters_with(outer);
                Ok(value)
            }
            Err(e) => {
                // Put the table back, in the place it was: a rejected edit must not leave
                // the sketch driven by an expression that does not evaluate, and must not
                // reorder a table the user reads top to bottom either.
                self.parameters.retain(|p| p.name != name);
                if let Some((at, p)) = previous {
                    self.parameters.insert(at.min(self.parameters.len()), p);
                }
                Err(e)
            }
        }
    }

    /// Removes a parameter. Dimensions bound to expressions that mention it keep the
    /// value they had; [`Sketch::failed_bindings`] reports them so the user can see
    /// which dimensions stopped being driven.
    pub fn remove_parameter(&mut self, name: &str) -> bool {
        let before = self.parameters.len();
        self.parameters.retain(|p| p.name != name);
        before != self.parameters.len()
    }

    /// Renames a parameter of this sketch and rewrites every expression that mentions
    /// it, in the table and in the bound dimensions alike.
    ///
    /// Renaming is not remove-then-add: references are by name, so anything short of a
    /// rewrite turns every dependent expression into an unknown name. Refuses a name
    /// that is not valid or that the sketch already uses, rather than merging two
    /// parameters into one silently.
    pub fn rename_parameter(&mut self, from: &str, to: &str) -> Result<(), SketchError> {
        self.rename_parameter_with(from, to, &no_outer)
    }

    /// [`Sketch::rename_parameter`], refusing a new name that would *capture* references
    /// meant for the document.
    ///
    /// Renaming a sketch's `a` to `b` while the document also defines `b` does not merely
    /// shadow it: every expression in this sketch that already said `b`, meaning the
    /// document's, would silently start meaning the local one and the drawing would change
    /// shape with nothing reported. Shadowing a name nothing here reads is harmless and
    /// still allowed — it is the reinterpretation of existing text that is refused.
    pub fn rename_parameter_with(
        &mut self,
        from: &str,
        to: &str,
        outer: Outer,
    ) -> Result<(), SketchError> {
        if from == to {
            return Ok(());
        }
        if outer(to).is_ok() && self.mentions_parameter(to) {
            return Err(SketchError::InvalidArgument(format!(
                "this sketch already reads a document parameter named {to:?}: renaming \
                 {from:?} over it would quietly change what those expressions mean"
            )));
        }
        if !expr::is_valid_name(to) {
            return Err(SketchError::InvalidArgument(format!(
                "{to:?} is not a valid parameter name: use a letter or underscore \
                 followed by letters, digits or underscores"
            )));
        }
        if self.parameter(from).is_none() {
            return Err(SketchError::UnknownParameter(from.to_string()));
        }
        if self.parameter(to).is_some() {
            return Err(SketchError::InvalidArgument(format!(
                "this sketch already has a parameter named {to:?}"
            )));
        }
        for p in &mut self.parameters {
            if p.name == from {
                p.name = to.to_string();
            }
        }
        self.rewrite_references(from, to);
        Ok(())
    }

    /// Follows a rename of a *document* parameter through this sketch's expressions.
    ///
    /// A sketch that defines `from` itself is left alone: there the name means the local
    /// parameter, which the document's rename has nothing to do with. Returns whether
    /// anything was rewritten, so a caller can report how far a rename reached.
    pub fn rewrite_outer_parameter(&mut self, from: &str, to: &str) -> bool {
        if from == to || !expr::is_valid_name(to) || self.parameter(from).is_some() {
            return false;
        }
        self.rewrite_references(from, to)
    }

    /// Whether any expression in this sketch reads `name`, either in the table or in a
    /// bound dimension. Deleting a parameter something still reads is worth a warning.
    pub fn mentions_parameter(&self, name: &str) -> bool {
        let in_table = self
            .parameters
            .iter()
            .any(|p| expr::referenced_names(&p.expr).iter().any(|n| n == name));
        in_table
            || self
                .dimension_exprs
                .values()
                .any(|e| expr::referenced_names(e).iter().any(|n| n == name))
    }

    /// Rewrites `from` to `to` everywhere this sketch reads a name. Returns whether any
    /// text changed.
    fn rewrite_references(&mut self, from: &str, to: &str) -> bool {
        let mut touched = false;
        for p in &mut self.parameters {
            let rewritten = expr::rename(&p.expr, from, to);
            touched |= rewritten != p.expr;
            p.expr = rewritten;
        }
        let bindings: Vec<(ConstraintId, String)> = self
            .dimension_exprs
            .iter()
            .map(|(id, e)| (id, expr::rename(e, from, to)))
            .collect();
        for (id, rewritten) in bindings {
            if self
                .dimension_exprs
                .get(id)
                .is_some_and(|e| *e != rewritten)
            {
                touched = true;
                self.dimension_exprs.insert(id, rewritten);
            }
        }
        touched
    }

    /// Drives a dimension by an expression and applies it immediately.
    pub fn bind_dimension(
        &mut self,
        id: ConstraintId,
        expression: &str,
    ) -> Result<f64, SketchError> {
        self.bind_dimension_with(id, expression, &no_outer)
    }

    /// [`Sketch::bind_dimension`], resolving names through the document's table too.
    pub fn bind_dimension_with(
        &mut self,
        id: ConstraintId,
        expression: &str,
        outer: Outer,
    ) -> Result<f64, SketchError> {
        let constraint = self
            .constraints
            .get(id)
            .ok_or(SketchError::UnknownConstraint(id))?;
        if !constraint.is_dimension() {
            return Err(SketchError::NotADimension(id));
        }
        let value = self.evaluate_with(expression, outer)?;
        self.dimension_exprs.insert(id, expression.to_string());
        self.apply_binding(id, value);
        Ok(value)
    }

    /// Releases a dimension back to being a plain number, keeping its current value.
    pub fn unbind_dimension(&mut self, id: ConstraintId) {
        self.dimension_exprs.remove(id);
    }

    pub fn dimension_expr(&self, id: ConstraintId) -> Option<&str> {
        self.dimension_exprs.get(id).map(String::as_str)
    }

    /// Evaluates an expression against the parameter table.
    pub fn evaluate(&self, expression: &str) -> Result<f64, SketchError> {
        self.evaluate_with(expression, &no_outer)
    }

    /// Evaluates an expression against the sketch's table, then the document's.
    pub fn evaluate_with(&self, expression: &str, outer: Outer) -> Result<f64, SketchError> {
        expr::eval(expression, &|name| {
            self.resolve(name, &mut Vec::new(), outer)
        })
    }

    /// Re-drives every bound dimension. Called before each solve, and after any change
    /// to the table, so geometry and parameters never disagree.
    pub fn apply_parameters(&mut self) {
        self.apply_parameters_with(&no_outer);
    }

    /// [`Sketch::apply_parameters`] with the document's table behind the sketch's own.
    pub fn apply_parameters_with(&mut self, outer: Outer) {
        let bindings: Vec<(ConstraintId, String)> = self
            .dimension_exprs
            .iter()
            .map(|(id, e)| (id, e.clone()))
            .collect();
        for (id, expression) in bindings {
            if !self.constraints.contains_key(id) {
                self.dimension_exprs.remove(id);
                continue;
            }
            match self.evaluate_with(&expression, outer) {
                Ok(value) => self.apply_binding(id, value),
                Err(e) => log::debug!("dimension {id:?} keeps its value: {e}"),
            }
        }
    }

    /// Bound dimensions whose expression no longer evaluates, for the UI to flag.
    pub fn failed_bindings(&self) -> Vec<ConstraintId> {
        self.failed_bindings_with(&no_outer)
    }

    /// [`Sketch::failed_bindings`], judged with the document's table available.
    pub fn failed_bindings_with(&self, outer: Outer) -> Vec<ConstraintId> {
        self.dimension_exprs
            .iter()
            .filter(|(_, e)| self.evaluate_with(e, outer).is_err())
            .map(|(id, _)| id)
            .collect()
    }

    /// Writes an evaluated value into a dimension, converting from the unit the user
    /// types in. An angle keeps its sign, exactly as retyping it in the dimension box
    /// does, so re-driving one never flips the geometry to the mirror solution.
    fn apply_binding(&mut self, id: ConstraintId, value: f64) {
        let Some(c) = self.constraints.get_mut(id) else {
            return;
        };
        let value = match c {
            Constraint::Angle { value: current, .. } => value.abs().to_radians().copysign(*current),
            _ => value,
        };
        c.set_dimension_value(value);
    }

    /// Value of one name, with the chain of names already being resolved so a parameter
    /// that refers to itself is an error rather than a hang. The sketch's own table is
    /// consulted first; a name it does not hold is asked of the document.
    fn resolve(
        &self,
        name: &str,
        stack: &mut Vec<String>,
        outer: Outer,
    ) -> Result<f64, SketchError> {
        if stack.iter().any(|n| n == name) {
            stack.push(name.to_string());
            return Err(SketchError::CircularParameter(stack.join(" → ")));
        }
        let Some(parameter) = self.parameter(name) else {
            return outer(name);
        };
        let expression = parameter.expr.clone();
        stack.push(name.to_string());
        let value = expr::eval(&expression, &|inner| {
            self.resolve(inner, &mut stack.clone(), outer)
        });
        stack.pop();
        value
    }
}
