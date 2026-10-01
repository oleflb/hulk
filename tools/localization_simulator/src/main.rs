use clap::{Parser, Subcommand};
use color_eyre::Result;

mod cli;

#[derive(Debug, Parser)]
#[command(about = "Deterministic 3D-localization simulator and analysis tool")]
struct CommandLine {
    #[command(subcommand)]
    command: Command,
}

#[derive(Debug, Subcommand)]
enum Command {
    /// Run to completion without a display and emit a complete JSON analysis report.
    Headless(cli::HeadlessArgs),
}

fn main() -> Result<()> {
    color_eyre::install()?;
    tracing_subscriber::fmt()
        .with_max_level(tracing_subscriber::filter::LevelFilter::WARN)
        .init();
    let Command::Headless(arguments) = CommandLine::parse().command;
    cli::run(arguments)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_headless_without_requiring_gui_arguments() {
        let command_line = CommandLine::try_parse_from([
            "localization_simulator",
            "headless",
            "--scenario",
            "stationary",
            "--compact",
        ])
        .expect("headless arguments parse");
        assert!(matches!(command_line.command, Command::Headless(_)));
    }
}
