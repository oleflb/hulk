use std::{env, path::PathBuf};

fn main() {
    println!("cargo:rustc-check-cfg=cfg(x5cam_x5_target)");
    println!("cargo:rerun-if-env-changed=X5_SOURCE_DIR");
    println!("cargo:rerun-if-env-changed=X5_SDK_INCLUDE");
    println!("cargo:rerun-if-env-changed=X5_SDK_LIB_DIR");
    println!("cargo:rerun-if-env-changed=X5CAM_FORCE_X5_TARGET");
    println!("cargo:rerun-if-changed=src/driver/ffi/wrapper.h");

    let arch = env::var("CARGO_CFG_TARGET_ARCH").unwrap_or_default();
    let os = env::var("CARGO_CFG_TARGET_OS").unwrap_or_default();
    let x5_target =
        env::var_os("X5CAM_FORCE_X5_TARGET").is_some() || (arch == "aarch64" && os == "linux");

    if !x5_target {
        return;
    }

    println!("cargo:rustc-cfg=x5cam_x5_target");

    let x5_source =
        env::var("X5_SOURCE_DIR").unwrap_or_else(|_| "/home/ole/hulk-stuff/x5/source".to_string());
    let sdk_include = env::var("X5_SDK_INCLUDE")
        .map(PathBuf::from)
        .unwrap_or_else(|_| PathBuf::from(&x5_source).join("hobot-multimedia-dev/usr/include"));

    let bindings = bindgen::Builder::default()
        .header("src/driver/ffi/wrapper.h")
        .clang_arg(format!("-I{}", sdk_include.display()))
        .default_enum_style(bindgen::EnumVariation::Consts)
        .derive_default(true)
        .parse_callbacks(Box::new(bindgen::CargoCallbacks::new()))
        .generate()
        .expect("failed to generate X5 SDK bindings");

    let out_path = PathBuf::from(env::var("OUT_DIR").expect("OUT_DIR missing"));
    bindings
        .write_to_file(out_path.join("x5_sdk_bindings.rs"))
        .expect("failed to write X5 SDK bindings");

    println!("cargo:rustc-link-search=native=/usr/hobot/lib");
    println!("cargo:rustc-link-search=native=/usr/lib/hbmedia");
    println!("cargo:rustc-link-search=native=/usr/lib");
    println!("cargo:rustc-link-search=native=/usr/lib/aarch64-linux-gnu");
    if let Ok(dir) = env::var("X5_SDK_LIB_DIR") {
        println!("cargo:rustc-link-search=native={dir}");
    } else {
        println!(
            "cargo:rustc-link-search=native={}",
            PathBuf::from(&x5_source)
                .join("hobot-multimedia/debian/usr/hobot/lib")
                .display()
        );
    }

    println!("cargo:rustc-link-lib=cam");
    println!("cargo:rustc-link-lib=vpf");
    println!("cargo:rustc-link-lib=hbmem");
    println!("cargo:rustc-link-lib=gdcbin");
    println!("cargo:rustc-link-lib=multimedia");
    println!("cargo:rustc-link-lib=pthread");
    println!("cargo:rustc-link-lib=dl");
}
