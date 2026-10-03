//! `basset-mcp`: a Model Context Protocol server over stdio.
//!
//! Every line on stdin is one JSON-RPC message and every line on stdout is one reply,
//! which is the framing MCP's stdio transport specifies. Logging goes to stderr so it
//! can never corrupt the stream.

fn main() {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("warn"))
        .target(env_logger::Target::Stderr)
        .init();
    let stdin = std::io::stdin();
    let stdout = std::io::stdout();
    if let Err(e) = basset_mcp::protocol::serve(stdin.lock(), stdout.lock()) {
        eprintln!("basset-mcp: {e}");
        std::process::exit(1);
    }
}
