use clap::Parser;

#[derive(Debug, Parser)]
pub struct Args {
    /// Router to connect to
    #[arg(long)]
    pub router: String,
    /// ros-z namespace
    #[arg(long)]
    pub namespace: String,
}
