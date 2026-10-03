//! The JSON-RPC framing: what a client sees on the wire.

use basset_mcp::Session;
use basset_mcp::protocol::handle_message;
use serde_json::{Value, json};

fn request(session: &mut Session, id: u64, method: &str, params: Value) -> Value {
    handle_message(
        session,
        &json!({ "jsonrpc": "2.0", "id": id, "method": method, "params": params }),
    )
    .expect("a request gets a reply")
}

#[test]
fn initialize_lists_tools_and_answers_ping() {
    let mut session = Session::new();
    let reply = request(
        &mut session,
        1,
        "initialize",
        json!({ "protocolVersion": "2025-06-18", "capabilities": {}, "clientInfo": { "name": "t", "version": "0" } }),
    );
    assert_eq!(reply["id"], 1);
    assert_eq!(reply["result"]["protocolVersion"], "2025-06-18");
    assert!(reply["result"]["capabilities"]["tools"].is_object());
    assert_eq!(reply["result"]["serverInfo"]["name"], "basset-mcp");

    // A notification gets no reply at all.
    assert!(
        handle_message(
            &mut session,
            &json!({ "jsonrpc": "2.0", "method": "notifications/initialized" })
        )
        .is_none()
    );

    let reply = request(&mut session, 2, "ping", json!({}));
    assert_eq!(reply["result"], json!({}));

    let reply = request(&mut session, 3, "tools/list", json!({}));
    let tools = reply["result"]["tools"].as_array().expect("tools array");
    let names: Vec<&str> = tools.iter().filter_map(|t| t["name"].as_str()).collect();
    for expected in [
        "create_sketch",
        "sketch_ops",
        "sketch_info",
        "extrude",
        "body_info",
        "check_document",
    ] {
        assert!(
            names.contains(&expected),
            "{expected} missing from {names:?}"
        );
    }
    for t in tools {
        assert_eq!(t["inputSchema"]["type"], "object", "{}", t["name"]);
        assert!(t["description"].as_str().is_some_and(|d| !d.is_empty()));
    }
}

#[test]
fn a_tool_call_returns_text_content_and_errors_are_flagged_not_thrown() {
    let mut session = Session::new();
    let reply = request(
        &mut session,
        1,
        "tools/call",
        json!({ "name": "create_sketch", "arguments": { "plane": "XY" } }),
    );
    assert_eq!(reply["result"]["isError"], false);
    let text = reply["result"]["content"][0]["text"].as_str().unwrap();
    let parsed: Value = serde_json::from_str(text).expect("the text is JSON");
    assert_eq!(parsed["sketch"], 1);

    let reply = request(
        &mut session,
        2,
        "tools/call",
        json!({ "name": "extrude", "arguments": { "sketch": 1, "regions": "all", "distance": 5 } }),
    );
    assert_eq!(reply["result"]["isError"], true, "{reply}");
    let text = reply["result"]["content"][0]["text"].as_str().unwrap();
    assert!(text.contains("encloses no region"), "{text}");

    let reply = request(
        &mut session,
        3,
        "tools/call",
        json!({ "name": "no_such_tool" }),
    );
    assert_eq!(reply["result"]["isError"], true);

    let reply = request(&mut session, 4, "nonsense/method", json!({}));
    assert_eq!(reply["error"]["code"], -32601);
}

#[test]
fn the_stdio_loop_frames_one_message_per_line() {
    let input = concat!(
        r#"{"jsonrpc":"2.0","id":1,"method":"initialize","params":{"protocolVersion":"2024-11-05"}}"#,
        "\n",
        r#"{"jsonrpc":"2.0","method":"notifications/initialized"}"#,
        "\n",
        "not json\n",
        r#"{"jsonrpc":"2.0","id":2,"method":"ping"}"#,
        "\n",
    );
    let mut out = Vec::new();
    basset_mcp::protocol::serve(input.as_bytes(), &mut out).unwrap();
    let lines: Vec<Value> = String::from_utf8(out)
        .unwrap()
        .lines()
        .map(|l| serde_json::from_str(l).unwrap())
        .collect();
    assert_eq!(lines.len(), 3, "{lines:?}");
    assert_eq!(lines[0]["id"], 1);
    assert_eq!(lines[0]["result"]["protocolVersion"], "2024-11-05");
    assert_eq!(lines[1]["error"]["code"], -32700);
    assert_eq!(lines[2]["id"], 2);
}

#[test]
fn a_script_can_name_what_earlier_calls_made() {
    let mut session = Session::new();
    let result = session
        .call(
            "run_script",
            &json!({ "calls": [
                { "tool": "create_sketch", "arguments": { "plane": "XY" } },
                { "tool": "sketch_ops", "arguments": { "sketch": "$0", "ops": [ { "op": "rectangle", "a": [0, 0], "b": [10, 10] } ] } },
                { "tool": "extrude", "arguments": { "sketch": "$0", "regions": "all", "distance": 5 } },
                { "tool": "extrude", "arguments": { "regions": [ { "body": "$2.body", "face": "${2}.0:EndCap" } ], "distance": 5, "operation": "join" } },
                { "tool": "body_info", "arguments": { "body": "$2.body" } }
            ] }),
        )
        .unwrap();
    assert_eq!(result["completed"], true, "{result}");
    let info = &result["results"][4]["result"];
    assert_eq!(info["volume"], 1000.0);
    assert_eq!(info["closed"], true);
}
