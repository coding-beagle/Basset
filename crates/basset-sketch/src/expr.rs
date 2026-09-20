//! The little expression language behind named sketch parameters.
//!
//! A parameter is a name bound to an expression, and a dimension can be driven by an
//! expression instead of a bare number, so a drawing can say `hole_spacing * 2` and mean
//! it. The grammar is deliberately tiny — numbers, names, `+ - * /`, brackets and unary
//! minus — because every symbol in it has to be obvious to someone reading a dimension
//! in a toolbar three months later.

use crate::SketchError;

/// Evaluates `text`, resolving names through `lookup`.
pub fn eval(
    text: &str,
    lookup: &dyn Fn(&str) -> Result<f64, SketchError>,
) -> Result<f64, SketchError> {
    let tokens = tokenize(text)?;
    let mut parser = Parser {
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
pub fn is_valid_name(name: &str) -> bool {
    let mut chars = name.chars();
    chars
        .next()
        .is_some_and(|c| c.is_ascii_alphabetic() || c == '_')
        && chars.all(|c| c.is_ascii_alphanumeric() || c == '_')
}

/// Names an expression mentions, for cycle detection and for telling the user which
/// dimensions a parameter drives.
pub fn referenced_names(text: &str) -> Vec<String> {
    let mut out = Vec::new();
    for token in tokenize(text).unwrap_or_default() {
        if let Token::Name(n) = token
            && !out.contains(&n)
        {
            out.push(n);
        }
    }
    out
}

fn bad(text: &str, why: &str) -> SketchError {
    SketchError::BadExpression(format!("{text:?}: {why}"))
}

#[derive(Clone, Debug, PartialEq)]
enum Token {
    Number(f64),
    Name(String),
    Plus,
    Minus,
    Star,
    Slash,
    Open,
    Close,
}

fn tokenize(text: &str) -> Result<Vec<Token>, SketchError> {
    let chars: Vec<char> = text.chars().collect();
    let mut tokens = Vec::new();
    let mut i = 0;
    while i < chars.len() {
        let c = chars[i];
        match c {
            ' ' | '\t' => i += 1,
            '+' => {
                tokens.push(Token::Plus);
                i += 1;
            }
            '-' => {
                tokens.push(Token::Minus);
                i += 1;
            }
            '*' => {
                tokens.push(Token::Star);
                i += 1;
            }
            '/' => {
                tokens.push(Token::Slash);
                i += 1;
            }
            '(' => {
                tokens.push(Token::Open);
                i += 1;
            }
            ')' => {
                tokens.push(Token::Close);
                i += 1;
            }
            _ if c.is_ascii_digit() || c == '.' => {
                let start = i;
                while i < chars.len() && (chars[i].is_ascii_digit() || chars[i] == '.') {
                    i += 1;
                }
                let literal: String = chars[start..i].iter().collect();
                let value = literal
                    .parse::<f64>()
                    .map_err(|_| bad(text, "not a number"))?;
                tokens.push(Token::Number(value));
            }
            _ if c.is_ascii_alphabetic() || c == '_' => {
                let start = i;
                while i < chars.len() && (chars[i].is_ascii_alphanumeric() || chars[i] == '_') {
                    i += 1;
                }
                tokens.push(Token::Name(chars[start..i].iter().collect()));
            }
            _ => return Err(bad(text, &format!("{c:?} means nothing here"))),
        }
    }
    if tokens.is_empty() {
        return Err(bad(text, "nothing to evaluate"));
    }
    Ok(tokens)
}

struct Parser<'a> {
    tokens: &'a [Token],
    at: usize,
    lookup: &'a dyn Fn(&str) -> Result<f64, SketchError>,
}

impl Parser<'_> {
    fn peek(&self) -> Option<&Token> {
        self.tokens.get(self.at)
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
                        return Err(SketchError::BadExpression("division by zero".into()));
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
            _ => self.atom(),
        }
    }

    fn atom(&mut self) -> Result<f64, SketchError> {
        match self.peek().cloned() {
            Some(Token::Number(v)) => {
                self.at += 1;
                Ok(v)
            }
            Some(Token::Name(n)) => {
                self.at += 1;
                (self.lookup)(&n)
            }
            Some(Token::Open) => {
                self.at += 1;
                let value = self.expression()?;
                if self.peek() != Some(&Token::Close) {
                    return Err(SketchError::BadExpression("unclosed bracket".into()));
                }
                self.at += 1;
                Ok(value)
            }
            _ => Err(SketchError::BadExpression(
                "expected a number, a name or a bracket".into(),
            )),
        }
    }
}
