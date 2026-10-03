//! JSON-RPC 2.0 over newline-delimited stdio, as the MCP stdio transport specifies.
//!
//! Only what a tool-serving MCP server needs is implemented: `initialize`, the
//! `initialized` notification, `ping`, `tools/list` and `tools/call`. Everything else is
//! answered with the standard method-not-found error. Notifications (messages without an
//! id) never get a reply, as the spec requires.

use std::io::{BufRead, Write};

use serde_json::{Value, json};

use crate::Session;
use crate::tools::tool_definitions;

pub const PROTOCOL_VERSION: &str = "2025-06-18";

/// Runs the server until stdin closes.
pub fn serve<R: BufRead, W: Write>(reader: R, mut writer: W) -> std::io::Result<()> {
    let mut session = Session::new();
    for line in reader.lines() {
        let line = line?;
        if line.trim().is_empty() {
            continue;
        }
        let message: Value = match serde_json::from_str(&line) {
            Ok(v) => v,
            Err(e) => {
                let reply = error_reply(Value::Null, -32700, format!("parse error: {e}"));
                write_message(&mut writer, &reply)?;
                continue;
            }
        };
        if let Some(reply) = handle_message(&mut session, &message) {
            write_message(&mut writer, &reply)?;
        }
    }
    Ok(())
}

fn write_message<W: Write>(writer: &mut W, message: &Value) -> std::io::Result<()> {
    serde_json::to_writer(&mut *writer, message)?;
    writer.write_all(b"\n")?;
    writer.flush()
}

/// Answers one message; `None` for a notification or a response we have no use for.
pub fn handle_message(session: &mut Session, message: &Value) -> Option<Value> {
    let id = message.get("id").cloned();
    let Some(method) = message.get("method").and_then(Value::as_str) else {
        // A response to something we never sent, or garbage with an id.
        return id
            .filter(|id| !id.is_null())
            .map(|id| error_reply(id, -32600, "invalid request: no method".into()));
    };
    let params = message.get("params").cloned().unwrap_or(Value::Null);
    let result = dispatch(session, method, &params);
    let id = id?;
    Some(match result {
        Ok(result) => json!({ "jsonrpc": "2.0", "id": id, "result": result }),
        Err((code, message)) => error_reply(id, code, message),
    })
}

fn error_reply(id: Value, code: i64, message: String) -> Value {
    json!({ "jsonrpc": "2.0", "id": id, "error": { "code": code, "message": message } })
}

fn dispatch(session: &mut Session, method: &str, params: &Value) -> Result<Value, (i64, String)> {
    match method {
        "initialize" => {
            // Echo a version the client asked for when we know it; otherwise offer ours.
            let requested = params.get("protocolVersion").and_then(Value::as_str);
            let version = match requested {
                Some(v) if v == PROTOCOL_VERSION || v == "2025-03-26" || v == "2024-11-05" => v,
                _ => PROTOCOL_VERSION,
            };
            Ok(json!({
                "protocolVersion": version,
                "capabilities": { "tools": { "listChanged": false } },
                "serverInfo": {
                    "name": "basset-mcp",
                    "version": env!("CARGO_PKG_VERSION"),
                },
                "instructions": crate::tools::SERVER_INSTRUCTIONS,
            }))
        }
        "notifications/initialized" | "notifications/cancelled" | "notifications/progress" => {
            Ok(Value::Null)
        }
        "ping" => Ok(json!({})),
        "tools/list" => Ok(json!({ "tools": tool_definitions() })),
        "tools/call" => {
            let name = params
                .get("name")
                .and_then(Value::as_str)
                .ok_or((-32602, "tools/call needs a name".to_string()))?;
            let args = params.get("arguments").cloned().unwrap_or(json!({}));
            Ok(match session.call(name, &args) {
                Ok(value) => json!({
                    "content": [{ "type": "text", "text": pretty(&value) }],
                    "isError": false,
                }),
                Err(e) => json!({
                    "content": [{ "type": "text", "text": e.to_string() }],
                    "isError": true,
                }),
            })
        }
        _ => Err((-32601, format!("method not found: {method}"))),
    }
}

/// Pretty-printed JSON with scalar arrays kept on one line, so a point is `[1.0, 2.0]`
/// and not three lines, and a reply listing a hundred edges stays readable.
pub fn pretty(value: &Value) -> String {
    let mut out = String::new();
    write_pretty(value, 0, &mut out);
    out
}

fn write_pretty(value: &Value, depth: usize, out: &mut String) {
    let indent = |n: usize| "  ".repeat(n);
    match value {
        Value::Array(items) if items.iter().all(|v| !v.is_array() && !v.is_object()) => {
            out.push('[');
            for (i, v) in items.iter().enumerate() {
                if i > 0 {
                    out.push_str(", ");
                }
                out.push_str(&v.to_string());
            }
            out.push(']');
        }
        Value::Array(items) => {
            out.push_str("[\n");
            for (i, v) in items.iter().enumerate() {
                out.push_str(&indent(depth + 1));
                write_pretty(v, depth + 1, out);
                if i + 1 < items.len() {
                    out.push(',');
                }
                out.push('\n');
            }
            out.push_str(&indent(depth));
            out.push(']');
        }
        Value::Object(map) if map.is_empty() => out.push_str("{}"),
        Value::Object(map) => {
            out.push_str("{\n");
            for (i, (k, v)) in map.iter().enumerate() {
                out.push_str(&indent(depth + 1));
                out.push_str(&Value::String(k.clone()).to_string());
                out.push_str(": ");
                write_pretty(v, depth + 1, out);
                if i + 1 < map.len() {
                    out.push(',');
                }
                out.push('\n');
            }
            out.push_str(&indent(depth));
            out.push('}');
        }
        other => out.push_str(&other.to_string()),
    }
}
