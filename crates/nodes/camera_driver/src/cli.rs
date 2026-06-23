use clap::Parser;

#[derive(Debug, Parser)]
pub struct Args {
    /// Router to connect to
    #[arg(long)]
    router: String,
    /// ros-z namespace
    #[arg(long)]
    namespace: String,
}
