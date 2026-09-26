//! The little expression language behind named sketch parameters.
//!
//! A parameter is a name bound to an expression, and a dimension can be driven by an
//! expression instead of a bare number, so a drawing can say `hole_spacing * 2` and mean
//! it. Every symbol in the grammar has to be obvious to someone reading a dimension in a
//! toolbar three months later, so the language stays arithmetic plus a named function
//! table and nothing else.
//!
//! # Grammar
//!
//! ```text
//! expression := term (("+" | "-") term)*
//! term       := unary (("*" | "/") unary)*
//! unary      := ("-" | "+") unary | power
//! power      := atom ("^" unary)?
//! atom       := number | call | name | "(" expression ")"
//! call       := name "(" (expression ("," expression)*)? ")"
//! number     := digits ["." digits] [("e" | "E") ["+" | "-"] digits]
//! name       := (letter | "_") (letter | digit | "_")*
//! ```
//!
//! `+ -` are the loosest and left-associative; `* /` bind tighter and are also
//! left-associative; `^` binds tighter still and is *right*-associative, so `2^3^2` is
//! 512 rather than 64. Unary minus sits outside a power on its left and inside one on its
//! right — `-2^2` is −4, `2^-1` is 0.5 — because that is what every calculator and
//! spreadsheet the user has already met does, and a dimension panel is a poor place to
//! discover a dialect.
//!
//! # Why there is no syntax tree
//! The parser is a recursive descent that returns `f64` from each rule instead of a node.
//! An expression here is a line of text a user typed into a box, so the tree would be a
//! dozen nodes at most, allocated and dropped to save arithmetic that costs less than the
//! allocation did. Returning values also keeps the code the same shape as the grammar
//! above — there is only one place a rule can be written down, so the two cannot drift.
//! The price is that nothing is reusable: see the limits below.
//!
//! # Limits, honestly
//! * No units. A value is a bare `f64` in whatever unit the dimension is typed in (see
//!   [`crate::parameters`]); nothing here knows millimetres from degrees, and nothing
//!   here converts.
//! * Trigonometry is in **radians**, in and out, like `f64`'s own methods — while angle
//!   dimensions around it are in degrees. That mismatch is deliberate rather than
//!   papered over with unit magic: `deg` and `rad` are in the function table so the user
//!   converts explicitly and can see where it happened.
//! * No comparisons, no booleans, no conditionals. An expression is always a number,
//!   which is what a dimension can be driven by.
//! * No memoisation. Every name is resolved through the caller's closure at the moment
//!   it is reached, and a parameter that names another re-evaluates it from its text, so
//!   a name mentioned twice is evaluated twice. That is linear in the size of the
//!   expression and exponential in the depth of a shared parameter chain; at the handful
//!   of parameters a sketch carries it is free, and it is the reason a parameter edit
//!   never needs a cache invalidated.

use crate::SketchError;

/// Evaluates `text`, resolving names through `lookup`.
///
/// A name is offered to `lookup` first and only falls back to a built-in constant when
/// `lookup` reports [`SketchError::UnknownParameter`], so a user parameter called `pi`
/// wins over the constant and any other lookup failure still reaches the caller.
pub fn eval(
    text: &str,
    lookup: &dyn Fn(&str) -> Result<f64, SketchError>,
) -> Result<f64, SketchError> {
    let tokens = tokenize(text)?;
    let mut parser = Parser {
        text,
        tokens: &tokens,
        at: 0,
        lookup,
    };
    let value = parser.expression()?;
    if parser.at < parser.tokens.len() {
        return Err(bad(text, "unexpected trailing input"));
    }
    if !value.is_finite() {
        return Err(bad(text, "value is not a finite number"));
    }
    Ok(value)
}

/// Whether `name` may be used as a parameter name: an identifier, so an expression can
/// never be ambiguous about where a name ends and an operator begins.
///
/// A built-in function name passes. `max` is a legal parameter name, and written without
/// brackets that is what it means; only `max(a, b)` calls the function.
pub fn is_valid_name(name: &str) -> bool {
    let mut chars = name.chars();
    chars
        .next()
        .is_some_and(|c| c.is_ascii_alphabetic() || c == '_')
        && chars.all(|c| c.is_ascii_alphanumeric() || c == '_')
}

/// Names an expression mentions, for cycle detection and for telling the user which
/// dimensions a parameter drives.
///
/// Only names used as *variables* count: a name directly followed by `(` is a function
/// call, not a dependency. Duplicates are dropped, first mention first, and text that
/// does not even tokenise yields nothing.
///
/// `pi` and `tau` are left out too, which is a guess rather than a fact: with no lookup
/// closure here there is no way to tell a user parameter named `pi` from the constant, so
/// this reports the common case. Evaluation still prefers the parameter, and the cycle
/// check in [`crate::parameters`] does not rely on this list to terminate.
pub fn referenced_names(text: &str) -> Vec<String> {
    let tokens = tokenize(text).unwrap_or_default();
    let mut out = Vec::new();
    for (i, spanned) in tokens.iter().enumerate() {
        let Token::Name(n) = &spanned.token else {
            continue;
        };
        if is_call_at(&tokens, i) || constant(n).is_some() {
            continue;
        }
        if !out.contains(n) {
            out.push(n.clone());
        }
    }
    out
}

/// Rewrites every use of the variable `from` in `text` as `to`.
///
/// Token-aware, so it renames the name and nothing that merely looks like it: a function
/// call keeps its name, `from` inside a longer identifier is left alone, and no number is
/// touched. The rest of the text — spacing, brackets, the lot — is copied through
/// verbatim, because the lexer keeps each token's byte span and only those spans are
/// rewritten.
///
/// The text comes back unchanged if `to` is not a valid name, or if `text` does not
/// tokenise: a rename is not the place to report a syntax error the user has not
/// finished typing.
pub fn rename(text: &str, from: &str, to: &str) -> String {
    if !is_valid_name(to) || !is_valid_name(from) {
        return text.to_string();
    }
    let Ok(tokens) = tokenize(text) else {
        return text.to_string();
    };
    let mut out = String::with_capacity(text.len());
    let mut cursor = 0;
    for (i, spanned) in tokens.iter().enumerate() {
        let Token::Name(n) = &spanned.token else {
            continue;
        };
        if n != from || is_call_at(&tokens, i) {
            continue;
        }
        out.push_str(&text[cursor..spanned.start]);
        out.push_str(to);
        cursor = spanned.end;
    }
    out.push_str(&text[cursor..]);
    out
}

fn bad(text: &str, why: &str) -> SketchError {
    SketchError::BadExpression(format!("{text:?}: {why}"))
}

/// Value of a built-in constant, or `None` for any other name.
fn constant(name: &str) -> Option<f64> {
    match name {
        "pi" => Some(std::f64::consts::PI),
        "tau" => Some(std::f64::consts::TAU),
        _ => None,
    }
}

/// Whether the name token at `i` is the name of a call rather than a variable.
fn is_call_at(tokens: &[Spanned], i: usize) -> bool {
    matches!(tokens.get(i + 1).map(|s| &s.token), Some(Token::Open))
}

#[derive(Clone, Debug, PartialEq)]
enum Token {
    Number(f64),
    Name(String),
    Plus,
    Minus,
    Star,
    Slash,
    Caret,
    Comma,
    Open,
    Close,
}

/// A token and the byte range of `text` it came from, so a rename can rebuild the
/// original text around the pieces it replaces.
#[derive(Clone, Debug, PartialEq)]
struct Spanned {
    token: Token,
    start: usize,
    end: usize,
}

fn tokenize(text: &str) -> Result<Vec<Spanned>, SketchError> {
    let bytes = text.as_bytes();
    let mut tokens = Vec::new();
    let mut i = 0;
    while i < bytes.len() {
        let start = i;
        let token = match bytes[i] {
            b' ' | b'\t' | b'\n' | b'\r' => {
                i += 1;
                continue;
            }
            b'+' => Some(Token::Plus),
            b'-' => Some(Token::Minus),
            b'*' => Some(Token::Star),
            b'/' => Some(Token::Slash),
            b'^' => Some(Token::Caret),
            b',' => Some(Token::Comma),
            b'(' => Some(Token::Open),
            b')' => Some(Token::Close),
            _ => None,
        };
        if let Some(token) = token {
            i += 1;
            tokens.push(Spanned {
                token,
                start,
                end: i,
            });
            continue;
        }
        let c = bytes[i];
        if c.is_ascii_digit() || c == b'.' {
            while i < bytes.len() && (bytes[i].is_ascii_digit() || bytes[i] == b'.') {
                i += 1;
            }
            // An exponent only joins the literal when it is actually followed by digits,
            // so `2e` is a number and a name rather than a malformed number.
            if i < bytes.len() && (bytes[i] | 0x20) == b'e' {
                let mut j = i + 1;
                if j < bytes.len() && (bytes[j] == b'+' || bytes[j] == b'-') {
                    j += 1;
                }
                if j < bytes.len() && bytes[j].is_ascii_digit() {
                    while j < bytes.len() && bytes[j].is_ascii_digit() {
                        j += 1;
                    }
                    i = j;
                }
            }
            let value = text[start..i]
                .parse::<f64>()
                .map_err(|_| bad(text, "not a number"))?;
            tokens.push(Spanned {
                token: Token::Number(value),
                start,
                end: i,
            });
        } else if c.is_ascii_alphabetic() || c == b'_' {
            while i < bytes.len() && (bytes[i].is_ascii_alphanumeric() || bytes[i] == b'_') {
                i += 1;
            }
            tokens.push(Spanned {
                token: Token::Name(text[start..i].to_string()),
                start,
                end: i,
            });
        } else {
            // Report the character, not the byte: `text` may hold anything a user typed.
            let c = text[i..]
                .chars()
                .next()
                .unwrap_or(char::REPLACEMENT_CHARACTER);
            return Err(bad(text, &format!("{c:?} means nothing here")));
        }
    }
    if tokens.is_empty() {
        return Err(bad(text, "nothing to evaluate"));
    }
    Ok(tokens)
}

/// English for a small count, so an arity complaint reads like a sentence.
fn count(n: usize) -> String {
    match n {
        0 => "none".to_string(),
        1 => "one".to_string(),
        2 => "two".to_string(),
        3 => "three".to_string(),
        4 => "four".to_string(),
        n => n.to_string(),
    }
}

fn wrong_arity(text: &str, name: &str, want: usize, got: usize) -> SketchError {
    let plural = if want == 1 { "" } else { "s" };
    bad(
        text,
        &format!(
            "{name} takes {} argument{plural}, not {}",
            count(want),
            count(got)
        ),
    )
}

fn one(text: &str, name: &str, args: &[f64]) -> Result<f64, SketchError> {
    match args {
        [x] => Ok(*x),
        _ => Err(wrong_arity(text, name, 1, args.len())),
    }
}

fn two(text: &str, name: &str, args: &[f64]) -> Result<(f64, f64), SketchError> {
    match args {
        [a, b] => Ok((*a, *b)),
        _ => Err(wrong_arity(text, name, 2, args.len())),
    }
}

/// The built-in function table. Arity is checked here, by which helper an arm reaches
/// for, so the list of names and the list of arities cannot disagree.
///
/// The trigonometric functions take and return **radians**, as `f64`'s own methods do.
/// Angle dimensions elsewhere in the crate are in degrees, so `sin(rad(30))` is how a
/// user writes what they mean; nothing here converts on their behalf.
fn call(text: &str, name: &str, args: &[f64]) -> Result<f64, SketchError> {
    let value = match name {
        "sqrt" => one(text, name, args)?.sqrt(),
        "abs" => one(text, name, args)?.abs(),
        "floor" => one(text, name, args)?.floor(),
        "ceil" => one(text, name, args)?.ceil(),
        "round" => one(text, name, args)?.round(),
        "sin" => one(text, name, args)?.sin(),
        "cos" => one(text, name, args)?.cos(),
        "tan" => one(text, name, args)?.tan(),
        "asin" => one(text, name, args)?.asin(),
        "acos" => one(text, name, args)?.acos(),
        "atan" => one(text, name, args)?.atan(),
        "deg" => one(text, name, args)?.to_degrees(),
        "rad" => one(text, name, args)?.to_radians(),
        "min" => {
            let (a, b) = two(text, name, args)?;
            a.min(b)
        }
        "max" => {
            let (a, b) = two(text, name, args)?;
            a.max(b)
        }
        "atan2" => {
            let (y, x) = two(text, name, args)?;
            y.atan2(x)
        }
        "hypot" => {
            let (a, b) = two(text, name, args)?;
            a.hypot(b)
        }
        _ => return Err(bad(text, &format!("no function named {name:?}"))),
    };
    Ok(value)
}

struct Parser<'a> {
    text: &'a str,
    tokens: &'a [Spanned],
    at: usize,
    lookup: &'a dyn Fn(&str) -> Result<f64, SketchError>,
}

impl Parser<'_> {
    fn peek(&self) -> Option<&Token> {
        self.tokens.get(self.at).map(|s| &s.token)
    }

    fn expression(&mut self) -> Result<f64, SketchError> {
        let mut value = self.term()?;
        while let Some(op) = self.peek().cloned() {
            match op {
                Token::Plus => {
                    self.at += 1;
                    value += self.term()?;
                }
                Token::Minus => {
                    self.at += 1;
                    value -= self.term()?;
                }
                _ => break,
            }
        }
        Ok(value)
    }

    fn term(&mut self) -> Result<f64, SketchError> {
        let mut value = self.unary()?;
        while let Some(op) = self.peek().cloned() {
            match op {
                Token::Star => {
                    self.at += 1;
                    value *= self.unary()?;
                }
                Token::Slash => {
                    self.at += 1;
                    let divisor = self.unary()?;
                    if divisor == 0.0 {
                        return Err(bad(self.text, "division by zero"));
                    }
                    value /= divisor;
                }
                _ => break,
            }
        }
        Ok(value)
    }

    fn unary(&mut self) -> Result<f64, SketchError> {
        match self.peek() {
            Some(Token::Minus) => {
                self.at += 1;
                Ok(-self.unary()?)
            }
            Some(Token::Plus) => {
                self.at += 1;
                self.unary()
            }
            _ => self.power(),
        }
    }

    /// `^`, right-associative. The exponent is a full `unary`, which is what puts a sign
    /// *inside* the power on the right (`2^-1`) while a sign on the left stays outside it
    /// (`-2^2`), and what makes `2^3^2` group to the right.
    fn power(&mut self) -> Result<f64, SketchError> {
        let base = self.atom()?;
        if self.peek() != Some(&Token::Caret) {
            return Ok(base);
        }
        self.at += 1;
        let exponent = self.unary()?;
        let value = base.powf(exponent);
        if !value.is_finite() {
            return Err(bad(self.text, "power is not a finite number"));
        }
        Ok(value)
    }

    fn atom(&mut self) -> Result<f64, SketchError> {
        match self.peek().cloned() {
            Some(Token::Number(v)) => {
                self.at += 1;
                Ok(v)
            }
            Some(Token::Name(n)) => {
                self.at += 1;
                if self.peek() == Some(&Token::Open) {
                    let args = self.arguments()?;
                    call(self.text, &n, &args)
                } else {
                    self.name_value(&n)
                }
            }
            Some(Token::Open) => {
                self.at += 1;
                let value = self.expression()?;
                if self.peek() != Some(&Token::Close) {
                    return Err(bad(self.text, "unclosed bracket"));
                }
                self.at += 1;
                Ok(value)
            }
            _ => Err(bad(self.text, "expected a number, a name or a bracket")),
        }
    }

    /// The bracketed argument list of a call, positioned on the opening bracket.
    fn arguments(&mut self) -> Result<Vec<f64>, SketchError> {
        self.at += 1;
        let mut args = Vec::new();
        if self.peek() != Some(&Token::Close) {
            loop {
                args.push(self.expression()?);
                if self.peek() == Some(&Token::Comma) {
                    self.at += 1;
                    continue;
                }
                break;
            }
        }
        if self.peek() != Some(&Token::Close) {
            return Err(bad(self.text, "unclosed bracket"));
        }
        self.at += 1;
        Ok(args)
    }

    /// One bare name: the caller's table first, the built-in constants only as a
    /// fallback, so a parameter is never shadowed by a constant and a lookup that failed
    /// for any other reason is reported as it happened.
    ///
    /// The fallback is taken only when the name *this* atom asked for is the one the
    /// table did not have. A parameter called `pi` whose own expression reads something
    /// that has since been deleted also fails with [`SketchError::UnknownParameter`] —
    /// naming the deleted parameter, not `pi` — and quietly answering 3.14159 there
    /// would re-drive the model to a different number with nothing reported.
    fn name_value(&self, name: &str) -> Result<f64, SketchError> {
        let error = match (self.lookup)(name) {
            Ok(value) => return Ok(value),
            Err(e) => e,
        };
        if let SketchError::UnknownParameter(missing) = &error
            && missing == name
            && let Some(value) = constant(name)
        {
            return Ok(value);
        }
        Err(error)
    }
}

#[cfg(test)]
mod tests {
    use std::f64::consts::{PI, TAU};

    use approx::assert_relative_eq;

    use super::*;

    /// A table with nothing in it: every name is unknown, which is what lets the
    /// constant fallback show through.
    fn empty(name: &str) -> Result<f64, SketchError> {
        Err(SketchError::UnknownParameter(name.to_string()))
    }

    fn ev(text: &str) -> f64 {
        eval(text, &empty).expect("expression should evaluate")
    }

    fn why(text: &str) -> String {
        eval(text, &empty)
            .expect_err("expression should be rejected")
            .to_string()
    }

    #[test]
    fn arithmetic_follows_the_precedence_taught_at_school() {
        assert_relative_eq!(ev("1 + 2 * 3"), 7.0);
        assert_relative_eq!(ev("2 * 3 + 1"), 7.0);
        assert_relative_eq!(ev("10 - 6 / 3"), 8.0);
    }

    #[test]
    fn brackets_override_precedence() {
        assert_relative_eq!(ev("(1 + 2) * 3"), 9.0);
        assert_relative_eq!(ev("((((2))))"), 2.0);
        assert_relative_eq!(ev("2 * (3 + (4 - 1))"), 12.0);
    }

    #[test]
    fn subtraction_and_division_are_left_associative() {
        assert_relative_eq!(ev("10 - 4 - 3"), 3.0);
        assert_relative_eq!(ev("16 / 4 / 2"), 2.0);
    }

    #[test]
    fn power_is_right_associative_and_binds_tighter_than_unary_minus() {
        assert_relative_eq!(ev("2^3^2"), 512.0);
        assert_relative_eq!(ev("-2^2"), -4.0);
        assert_relative_eq!(ev("2^-1"), 0.5);
        assert_relative_eq!(ev("(-2)^2"), 4.0);
    }

    #[test]
    fn power_binds_tighter_than_multiplication_and_addition() {
        assert_relative_eq!(ev("2 * 3^2"), 18.0);
        assert_relative_eq!(ev("3^2 * 2"), 18.0);
        assert_relative_eq!(ev("1 + 2^3"), 9.0);
        assert_relative_eq!(ev("2^0.5"), std::f64::consts::SQRT_2);
    }

    #[test]
    fn unary_signs_chain_and_a_leading_plus_is_accepted() {
        assert_relative_eq!(ev("--3"), 3.0);
        assert_relative_eq!(ev("---3"), -3.0);
        assert_relative_eq!(ev("+4"), 4.0);
        assert_relative_eq!(ev("5 - -2"), 7.0);
        assert_relative_eq!(ev("-+-4"), 4.0);
    }

    #[test]
    fn exponent_literals_read_with_either_sign_and_either_case() {
        assert_relative_eq!(ev("1e3"), 1000.0);
        assert_relative_eq!(ev("1E3"), 1000.0);
        assert_relative_eq!(ev("2.5e-4"), 0.00025);
        assert_relative_eq!(ev("2.5e+2"), 250.0);
        assert_relative_eq!(ev(".5e1"), 5.0);
        assert_relative_eq!(ev("1e3 * 2"), 2000.0);
    }

    #[test]
    fn an_e_without_digits_after_it_is_not_part_of_the_number() {
        // `2e` is a number beside a name, so it fails as a lookup, not as a literal.
        assert!(why("2e").contains("trailing input"));
    }

    #[test]
    fn every_one_argument_function_computes_its_value() {
        assert_relative_eq!(ev("sqrt(9)"), 3.0);
        assert_relative_eq!(ev("abs(-2.5)"), 2.5);
        assert_relative_eq!(ev("floor(2.7)"), 2.0);
        assert_relative_eq!(ev("ceil(2.1)"), 3.0);
        assert_relative_eq!(ev("round(2.5)"), 3.0);
        assert_relative_eq!(ev("sin(0)"), 0.0);
        assert_relative_eq!(ev("cos(0)"), 1.0);
        assert_relative_eq!(ev("tan(0)"), 0.0);
        assert_relative_eq!(ev("asin(1)"), PI / 2.0);
        assert_relative_eq!(ev("acos(1)"), 0.0);
        assert_relative_eq!(ev("atan(1)"), PI / 4.0);
    }

    #[test]
    fn every_two_argument_function_computes_its_value() {
        assert_relative_eq!(ev("min(3, -1)"), -1.0);
        assert_relative_eq!(ev("max(3, -1)"), 3.0);
        assert_relative_eq!(ev("atan2(1, 1)"), PI / 4.0);
        assert_relative_eq!(ev("hypot(3, 4)"), 5.0);
    }

    #[test]
    fn trigonometry_is_in_radians_and_deg_and_rad_convert_between_them() {
        assert_relative_eq!(ev("deg(pi)"), 180.0);
        assert_relative_eq!(ev("rad(180)"), PI);
        assert_relative_eq!(ev("sin(rad(30))"), 0.5, epsilon = 1e-12);
        // Degrees handed straight to `sin` are not 0.5, which is the whole point.
        assert!((ev("sin(30)") - 0.5).abs() > 0.1);
        assert_relative_eq!(ev("deg(rad(37.5))"), 37.5);
    }

    #[test]
    fn function_arguments_are_full_expressions_and_may_nest() {
        assert_relative_eq!(ev("sqrt(2^2 + 3 * 4)"), 4.0);
        assert_relative_eq!(ev("max(min(1, 2), sqrt(9))"), 3.0);
        assert_relative_eq!(ev("-sqrt(4)^2"), -4.0);
    }

    #[test]
    fn calling_a_function_with_the_wrong_number_of_arguments_names_the_arity() {
        assert!(why("sqrt(1, 2)").contains("sqrt takes one argument, not two"));
        assert!(why("sqrt()").contains("sqrt takes one argument, not none"));
        assert!(why("min(1)").contains("min takes two arguments, not one"));
        assert!(why("hypot(1, 2, 3)").contains("hypot takes two arguments, not three"));
        assert!(why("atan2(1)").contains("atan2 takes two arguments, not one"));
        assert!(why("deg()").contains("deg takes one argument, not none"));
    }

    #[test]
    fn calling_a_name_that_is_not_a_function_is_an_error() {
        assert!(why("wobble(1)").contains("no function named \"wobble\""));
        assert!(why("pi(1)").contains("no function named \"pi\""));
    }

    #[test]
    fn constants_are_there_when_no_parameter_claims_the_name() {
        assert_relative_eq!(ev("pi"), PI);
        assert_relative_eq!(ev("tau"), TAU);
        assert_relative_eq!(ev("tau / pi"), 2.0);
    }

    #[test]
    fn a_parameter_shadows_the_constant_of_the_same_name() {
        let table = |name: &str| match name {
            "pi" => Ok(3.0),
            other => Err(SketchError::UnknownParameter(other.to_string())),
        };
        assert_relative_eq!(eval("pi * 2", &table).unwrap(), 6.0);
        // `tau` is not in the table, so it still falls back to the constant.
        assert_relative_eq!(eval("tau", &table).unwrap(), TAU);
    }

    #[test]
    fn a_lookup_failure_other_than_unknown_parameter_propagates_untouched() {
        let table = |name: &str| Err(SketchError::CircularParameter(name.to_string()));
        let error = eval("pi", &table).expect_err("the cycle should reach the caller");
        assert!(matches!(error, SketchError::CircularParameter(n) if n == "pi"));
    }

    #[test]
    fn a_constant_does_not_stand_in_for_a_parameter_of_that_name_that_failed_to_resolve() {
        // `pi` is in the table, but reading it fails because what *it* refers to was
        // deleted. Answering 3.14159 here would re-drive the model to a different number
        // with nothing reported, so the failure has to reach the caller.
        let table = |name: &str| match name {
            "pi" => Err(SketchError::UnknownParameter("radius".to_string())),
            other => Err(SketchError::UnknownParameter(other.to_string())),
        };
        let error = eval("pi * 2", &table).expect_err("the deleted parameter should be named");
        assert!(matches!(error, SketchError::UnknownParameter(n) if n == "radius"));
        // A name that is genuinely absent still falls back to the constant.
        assert_relative_eq!(eval("pi * 2", &empty).unwrap(), TAU);
    }

    #[test]
    fn an_unknown_name_is_reported_as_an_unknown_parameter() {
        let error = eval("wobble", &empty).expect_err("no such parameter");
        assert!(matches!(error, SketchError::UnknownParameter(n) if n == "wobble"));
    }

    #[test]
    fn a_function_name_is_still_usable_as_a_bare_parameter() {
        let table = |name: &str| match name {
            "max" => Ok(7.0),
            "sqrt" => Ok(2.0),
            other => Err(SketchError::UnknownParameter(other.to_string())),
        };
        assert_relative_eq!(eval("max + 1", &table).unwrap(), 8.0);
        assert_relative_eq!(eval("sqrt * 3", &table).unwrap(), 6.0);
        // With brackets it is the function again, even though the table has the name.
        assert_relative_eq!(eval("max(1, 2)", &table).unwrap(), 2.0);
    }

    #[test]
    fn division_by_zero_is_an_error() {
        assert!(why("1 / 0").contains("division by zero"));
        assert!(why("1 / (2 - 2)").contains("division by zero"));
        assert!(why("1 / 0.0").contains("division by zero"));
    }

    #[test]
    fn a_result_that_is_not_finite_is_rejected() {
        assert!(why("10^400").contains("power is not a finite number"));
        assert!(why("0^-1").contains("power is not a finite number"));
        assert!(why("sqrt(-1)").contains("not a finite number"));
        assert!(why("1e308 * 10").contains("value is not a finite number"));
    }

    #[test]
    fn malformed_input_is_reported_rather_than_panicking() {
        assert!(why("").contains("nothing to evaluate"));
        assert!(why("   ").contains("nothing to evaluate"));
        assert!(why("1 +").contains("expected a number, a name or a bracket"));
        assert!(why("* 2").contains("expected a number, a name or a bracket"));
        assert!(why("(1 + 2").contains("unclosed bracket"));
        assert!(why("min(1, 2").contains("unclosed bracket"));
        assert!(why("1 + 2)").contains("unexpected trailing input"));
        assert!(why("1 2").contains("unexpected trailing input"));
        assert!(why("1.2.3").contains("not a number"));
        assert!(why("1 @ 2").contains("means nothing here"));
        assert!(why("1 + ¤").contains("means nothing here"));
        assert!(why("2 ^").contains("expected a number, a name or a bracket"));
        assert!(why("min(1,)").contains("expected a number, a name or a bracket"));
    }

    #[test]
    fn whitespace_between_tokens_does_not_matter() {
        assert_relative_eq!(ev("  1+\t2 *\n3  "), 7.0);
        assert_relative_eq!(ev("min ( 1 , 2 )"), 1.0);
    }

    #[test]
    fn referenced_names_skips_functions_and_constants_and_deduplicates() {
        assert_eq!(
            referenced_names("width + height * 2 - width"),
            vec!["width".to_string(), "height".to_string()]
        );
        assert_eq!(
            referenced_names("sqrt(width) + max(a, b) * pi + tau"),
            vec!["width".to_string(), "a".to_string(), "b".to_string()]
        );
        assert!(referenced_names("1 + 2").is_empty());
        assert!(referenced_names("@@@").is_empty());
        // A function name written bare is a variable, so it counts.
        assert_eq!(referenced_names("max * 2"), vec!["max".to_string()]);
    }

    #[test]
    fn rename_rewrites_bare_names_and_keeps_the_rest_of_the_text() {
        assert_eq!(rename("width * 2", "width", "w"), "w * 2");
        assert_eq!(
            rename("width + height - width", "width", "outer_width"),
            "outer_width + height - outer_width"
        );
        assert_eq!(rename("( a+b )*a", "a", "zzz"), "( zzz+b )*zzz");
    }

    #[test]
    fn rename_preserves_the_original_spacing_exactly() {
        let text = "  a   *\t( b +a )  ";
        assert_eq!(rename(text, "a", "q"), "  q   *\t( b +q )  ");
        assert_eq!(rename(text, "nothing", "q"), text);
    }

    #[test]
    fn rename_leaves_calls_longer_identifiers_and_numbers_alone() {
        assert_eq!(rename("max(a, b)", "max", "m"), "max(a, b)");
        assert_eq!(rename("max (a, b)", "max", "m"), "max (a, b)");
        assert_eq!(rename("max * max(1, 2)", "max", "m"), "m * max(1, 2)");
        assert_eq!(rename("a_width + width", "width", "w"), "a_width + w");
        assert_eq!(rename("widths + width", "width", "w"), "widths + w");
        assert_eq!(rename("1e3 + e3", "e3", "x"), "1e3 + x");
    }

    #[test]
    fn rename_returns_the_text_unchanged_when_it_cannot_be_done_honestly() {
        assert_eq!(rename("a + b", "a", "2b"), "a + b");
        assert_eq!(rename("a + b", "a", ""), "a + b");
        assert_eq!(rename("a + b", "a", "has space"), "a + b");
        assert_eq!(rename("a + @", "a", "b"), "a + @");
        assert_eq!(rename("a + b", "1", "b"), "a + b");
    }

    #[test]
    fn renamed_text_still_evaluates_to_the_same_value() {
        let table = |name: &str| match name {
            "w" => Ok(4.0),
            other => Err(SketchError::UnknownParameter(other.to_string())),
        };
        let text = "width^2 + sqrt(width)";
        let renamed = rename(text, "width", "w");
        assert_eq!(renamed, "w^2 + sqrt(w)");
        assert_relative_eq!(eval(&renamed, &table).unwrap(), 18.0);
    }

    #[test]
    fn is_valid_name_accepts_identifiers_and_nothing_else() {
        assert!(is_valid_name("a"));
        assert!(is_valid_name("_hidden"));
        assert!(is_valid_name("hole_spacing2"));
        assert!(is_valid_name("max"));
        assert!(is_valid_name("pi"));
        assert!(!is_valid_name(""));
        assert!(!is_valid_name("2fast"));
        assert!(!is_valid_name("has space"));
        assert!(!is_valid_name("a-b"));
        assert!(!is_valid_name("ø"));
    }
}
