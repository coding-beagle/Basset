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

use serde::{Deserialize, Serialize};

use crate::{Constraint, ConstraintId, Sketch, SketchError, expr};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Parameter {
    pub name: String,
    /// What the user wrote. Kept verbatim so a panel shows `width / 2`, not `25`.
    pub expr: String,
}

impl Sketch {
    pub fn parameters(&self) -> &[Parameter] {
        &self.parameters
    }

    pub fn parameter(&self, name: &str) -> Option<&Parameter> {
        self.parameters.iter().find(|p| p.name == name)
    }

    /// Current value of a named parameter.
    pub fn parameter_value(&self, name: &str) -> Result<f64, SketchError> {
        self.resolve(name, &mut Vec::new())
    }

    /// Adds a parameter or replaces the expression of an existing one. The expression is
    /// evaluated before it is kept, so a name that does not exist or a cycle is reported
    /// to the user instead of quietly breaking every dimension that depends on it.
    pub fn set_parameter(&mut self, name: &str, expression: &str) -> Result<f64, SketchError> {
        if !expr::is_valid_name(name) {
            return Err(SketchError::InvalidArgument(format!(
                "{name:?} is not a valid parameter name: use a letter or underscore \
                 followed by letters, digits or underscores"
            )));
        }
        let previous = self.parameter(name).cloned();
        match self.parameters.iter_mut().find(|p| p.name == name) {
            Some(p) => p.expr = expression.to_string(),
            None => self.parameters.push(Parameter {
                name: name.to_string(),
                expr: expression.to_string(),
            }),
        }
        match self.parameter_value(name) {
            Ok(value) => {
                self.apply_parameters();
                Ok(value)
            }
            Err(e) => {
                // Put the table back: a rejected edit must not leave the sketch driven
                // by an expression that does not evaluate.
                self.parameters.retain(|p| p.name != name);
                if let Some(p) = previous {
                    self.parameters.push(p);
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

    /// Drives a dimension by an expression and applies it immediately.
    pub fn bind_dimension(
        &mut self,
        id: ConstraintId,
        expression: &str,
    ) -> Result<f64, SketchError> {
        let constraint = self
            .constraints
            .get(id)
            .ok_or(SketchError::UnknownConstraint(id))?;
        if !constraint.is_dimension() {
            return Err(SketchError::NotADimension(id));
        }
        let value = self.evaluate(expression)?;
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
        expr::eval(expression, &|name| self.resolve(name, &mut Vec::new()))
    }

    /// Re-drives every bound dimension. Called before each solve, and after any change
    /// to the table, so geometry and parameters never disagree.
    pub fn apply_parameters(&mut self) {
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
            match self.evaluate(&expression) {
                Ok(value) => self.apply_binding(id, value),
                Err(e) => log::debug!("dimension {id:?} keeps its value: {e}"),
            }
        }
    }

    /// Bound dimensions whose expression no longer evaluates, for the UI to flag.
    pub fn failed_bindings(&self) -> Vec<ConstraintId> {
        self.dimension_exprs
            .iter()
            .filter(|(_, e)| self.evaluate(e).is_err())
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
    /// that refers to itself is an error rather than a hang.
    fn resolve(&self, name: &str, stack: &mut Vec<String>) -> Result<f64, SketchError> {
        if stack.iter().any(|n| n == name) {
            stack.push(name.to_string());
            return Err(SketchError::CircularParameter(stack.join(" → ")));
        }
        let parameter = self
            .parameter(name)
            .ok_or_else(|| SketchError::UnknownParameter(name.to_string()))?;
        let expression = parameter.expr.clone();
        stack.push(name.to_string());
        let value = expr::eval(&expression, &|inner| {
            self.resolve(inner, &mut stack.clone())
        });
        stack.pop();
        value
    }
}
