use clap::Parser;

#[derive(Debug, Parser)]
pub(crate) struct Args {
    #[arg(
        long,
        help = "Robot graph namespace. Bare values like '42' become '/42'."
    )]
    pub(crate) robot: String,

    #[arg(long, help = "Zenoh router endpoint, e.g. tcp/10.0.24.42:7447.")]
    pub(crate) router: Option<String>,

    #[arg(long, default_value = "inputs/stereo_image_pair")]
    pub(crate) stereo_topic: String,
}

pub(crate) fn derive_namespace(robot: &str) -> String {
    if robot.starts_with('/') {
        robot.to_string()
    } else {
        format!("/{robot}")
    }
}
