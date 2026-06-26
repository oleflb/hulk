use std::env;

fn main() {
    println!("cargo:rustc-check-cfg=cfg(x5cam_x5_target)");
    println!("cargo:rerun-if-env-changed=X5CAM_FORCE_X5_TARGET");

    let arch = env::var("CARGO_CFG_TARGET_ARCH").unwrap_or_default();
    let os = env::var("CARGO_CFG_TARGET_OS").unwrap_or_default();
    let x5_target =
        env::var_os("X5CAM_FORCE_X5_TARGET").is_some() || (arch == "aarch64" && os == "linux");

    if !x5_target {
        return;
    }

    println!("cargo:rustc-cfg=x5cam_x5_target");
}
