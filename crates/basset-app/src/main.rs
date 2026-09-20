//! Basset desktop application.
//!
//! The binary is a thin shell: `app` owns the window, GPU and UI toolkit plumbing, and
//! `editor` owns everything a user thinks of as the program (document, camera, tools,
//! panels). Keeping the editor free of windowing types keeps it testable and lets the
//! shell be replaced without touching modelling behaviour.

mod app;
mod editor;

fn main() -> anyhow::Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();
    let path = std::env::args().nth(1).map(std::path::PathBuf::from);
    app::run(path)
}
