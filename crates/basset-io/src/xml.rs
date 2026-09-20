//! Just enough XML for 3MF: attribute escaping for the writer and a tag scanner for the
//! reader.
//!
//! A full XML library would be the safer default, but the model part we read is one we
//! wrote ourselves (or one from a well-behaved slicer), and everything we need lives in
//! attributes of empty elements. A ~100-line scanner keeps the dependency tree small and
//! is easy to extend when a later import feature needs more.

use std::borrow::Cow;

use crate::IoError;

/// Escapes text for use inside an attribute value or element content.
///
/// Control characters (other than tab/newline/CR) are dropped rather than escaped
/// because XML 1.0 forbids them even as character references, and a body name that
/// somehow contains one should not make the whole package unreadable.
pub(crate) fn escape(s: &str) -> Cow<'_, str> {
    let needs_work = s
        .chars()
        .any(|c| matches!(c, '<' | '>' | '&' | '"' | '\'') || is_forbidden(c));
    if !needs_work {
        return Cow::Borrowed(s);
    }
    let mut out = String::with_capacity(s.len() + 8);
    for c in s.chars() {
        match c {
            '<' => out.push_str("&lt;"),
            '>' => out.push_str("&gt;"),
            '&' => out.push_str("&amp;"),
            '"' => out.push_str("&quot;"),
            '\'' => out.push_str("&apos;"),
            c if is_forbidden(c) => {}
            c => out.push(c),
        }
    }
    Cow::Owned(out)
}

fn is_forbidden(c: char) -> bool {
    c.is_control() && !matches!(c, '\t' | '\n' | '\r')
}

/// Reverses [`escape`], plus the numeric character references any XML writer may emit.
pub(crate) fn unescape(s: &str) -> Result<Cow<'_, str>, IoError> {
    if !s.contains('&') {
        return Ok(Cow::Borrowed(s));
    }
    let mut out = String::with_capacity(s.len());
    let mut rest = s;
    while let Some(start) = rest.find('&') {
        out.push_str(&rest[..start]);
        let after = &rest[start + 1..];
        let end = after
            .find(';')
            .ok_or_else(|| malformed(format!("unterminated entity in '{s}'")))?;
        let entity = &after[..end];
        let ch = match entity {
            "lt" => '<',
            "gt" => '>',
            "amp" => '&',
            "quot" => '"',
            "apos" => '\'',
            _ => {
                let code = entity
                    .strip_prefix("#x")
                    .and_then(|h| u32::from_str_radix(h, 16).ok())
                    .or_else(|| entity.strip_prefix('#').and_then(|d| d.parse().ok()));
                code.and_then(char::from_u32)
                    .ok_or_else(|| malformed(format!("unknown entity '&{entity};'")))?
            }
        };
        out.push(ch);
        rest = &after[end + 1..];
    }
    out.push_str(rest);
    Ok(Cow::Owned(out))
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum TagKind {
    Open,
    Close,
    /// `<name .../>`
    Empty,
}

/// One tag with its attributes decoded. Namespace prefixes are stripped from the element
/// name so `<m:vertex>` and `<vertex>` compare equal; 3MF producers differ on prefixing.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct Tag<'a> {
    pub name: &'a str,
    pub kind: TagKind,
    pub attrs: Vec<(&'a str, Cow<'a, str>)>,
}

impl<'a> Tag<'a> {
    pub fn attr(&self, name: &str) -> Option<&str> {
        self.attrs
            .iter()
            .find(|(k, _)| *k == name)
            .map(|(_, v)| v.as_ref())
    }

    pub fn required(&self, name: &str) -> Result<&str, IoError> {
        self.attr(name)
            .ok_or_else(|| malformed(format!("<{}> is missing attribute '{name}'", self.name)))
    }

    pub fn parse_attr<T: std::str::FromStr>(&self, name: &str) -> Result<T, IoError> {
        let raw = self.required(name)?;
        raw.parse().map_err(|_| {
            malformed(format!(
                "<{}> attribute {name}='{raw}' is not a valid number",
                self.name
            ))
        })
    }
}

/// Yields element tags in document order, skipping text, comments, processing
/// instructions and CDATA. Text content is never needed for the parts of 3MF we read.
pub(crate) struct TagScanner<'a> {
    src: &'a str,
    pos: usize,
}

impl<'a> TagScanner<'a> {
    pub fn new(src: &'a str) -> Self {
        Self { src, pos: 0 }
    }

    pub fn next_tag(&mut self) -> Result<Option<Tag<'a>>, IoError> {
        loop {
            let rest = &self.src[self.pos..];
            let Some(lt) = rest.find('<') else {
                self.pos = self.src.len();
                return Ok(None);
            };
            let start = self.pos + lt;
            let body = &self.src[start..];

            if let Some(skip) = skip_non_element(body)? {
                self.pos = start + skip;
                continue;
            }

            let gt = find_tag_end(body).ok_or_else(|| malformed("unterminated tag"))?;
            let inner = &body[1..gt];
            self.pos = start + gt + 1;
            return parse_tag(inner).map(Some);
        }
    }
}

/// Length to skip if `body` (starting at `<`) is a comment, PI or CDATA section.
fn skip_non_element(body: &str) -> Result<Option<usize>, IoError> {
    for (open, close) in [
        ("<!--", "-->"),
        ("<?", "?>"),
        ("<![CDATA[", "]]>"),
        ("<!", ">"),
    ] {
        if let Some(after_open) = body.strip_prefix(open) {
            let end = after_open
                .find(close)
                .ok_or_else(|| malformed(format!("unterminated '{open}'")))?;
            return Ok(Some(open.len() + end + close.len()));
        }
    }
    Ok(None)
}

/// Finds the closing `>` of a tag, ignoring any `>` inside quoted attribute values.
fn find_tag_end(body: &str) -> Option<usize> {
    let mut quote: Option<char> = None;
    for (i, c) in body.char_indices() {
        match (quote, c) {
            (Some(q), c) if c == q => quote = None,
            (Some(_), _) => {}
            (None, '"' | '\'') => quote = Some(c),
            (None, '>') => return Some(i),
            _ => {}
        }
    }
    None
}

fn parse_tag(inner: &str) -> Result<Tag<'_>, IoError> {
    let (kind, inner) = if let Some(rest) = inner.strip_prefix('/') {
        (TagKind::Close, rest)
    } else if let Some(rest) = inner.strip_suffix('/') {
        (TagKind::Empty, rest)
    } else {
        (TagKind::Open, inner)
    };
    let inner = inner.trim();
    let name_end = inner
        .find(|c: char| c.is_whitespace())
        .unwrap_or(inner.len());
    let qualified = &inner[..name_end];
    if qualified.is_empty() {
        return Err(malformed("tag without a name"));
    }
    let name = qualified.rsplit(':').next().unwrap_or(qualified);

    let mut attrs = Vec::new();
    let mut rest = inner[name_end..].trim_start();
    while !rest.is_empty() {
        let eq = rest
            .find('=')
            .ok_or_else(|| malformed(format!("attribute without value in <{qualified}>")))?;
        let key = rest[..eq].trim_end();
        let after_eq = rest[eq + 1..].trim_start();
        let quote = after_eq.chars().next().filter(|c| matches!(c, '"' | '\''));
        let quote = quote
            .ok_or_else(|| malformed(format!("unquoted attribute '{key}' in <{qualified}>")))?;
        let value_body = &after_eq[1..];
        let close = value_body
            .find(quote)
            .ok_or_else(|| malformed(format!("unterminated attribute '{key}'")))?;
        attrs.push((key, unescape(&value_body[..close])?));
        rest = value_body[close + 1..].trim_start();
    }
    Ok(Tag { name, kind, attrs })
}

fn malformed(msg: impl Into<String>) -> IoError {
    IoError::Malformed3mf(msg.into())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn escape_and_unescape_are_inverse() {
        let raw = "a<b>&\"c'd";
        let escaped = escape(raw);
        assert_eq!(escaped, "a&lt;b&gt;&amp;&quot;c&apos;d");
        assert_eq!(unescape(&escaped).unwrap(), raw);
    }

    #[test]
    fn escape_borrows_when_nothing_to_do() {
        assert!(matches!(escape("plain name"), Cow::Borrowed(_)));
    }

    #[test]
    fn escape_drops_control_characters() {
        assert_eq!(escape("a\u{1}b\tc"), "ab\tc");
    }

    #[test]
    fn unescape_handles_numeric_references() {
        assert_eq!(unescape("&#65;&#x42;").unwrap(), "AB");
        assert!(unescape("&bogus;").is_err());
        assert!(unescape("&unterminated").is_err());
    }

    #[test]
    fn scanner_yields_tags_and_skips_noise() {
        let src = r#"<?xml version="1.0"?><!-- c --><a x="1" y='2>'><b:c/>text</a>"#;
        let mut s = TagScanner::new(src);
        let a = s.next_tag().unwrap().unwrap();
        assert_eq!(a.name, "a");
        assert_eq!(a.kind, TagKind::Open);
        assert_eq!(a.attr("x"), Some("1"));
        assert_eq!(a.attr("y"), Some("2>"));
        let c = s.next_tag().unwrap().unwrap();
        assert_eq!((c.name, c.kind), ("c", TagKind::Empty));
        let end = s.next_tag().unwrap().unwrap();
        assert_eq!((end.name, end.kind), ("a", TagKind::Close));
        assert!(s.next_tag().unwrap().is_none());
    }

    #[test]
    fn scanner_rejects_broken_tags() {
        assert!(TagScanner::new("<a x=1/>").next_tag().is_err());
        assert!(TagScanner::new("<a x=\"1/>").next_tag().is_err());
        assert!(TagScanner::new("<a").next_tag().is_err());
    }

    #[test]
    fn parse_attr_reports_bad_numbers() {
        let mut s = TagScanner::new(r#"<vertex x="abc"/>"#);
        let t = s.next_tag().unwrap().unwrap();
        assert!(t.parse_attr::<f64>("x").is_err());
        assert!(t.parse_attr::<f64>("y").is_err());
    }
}
