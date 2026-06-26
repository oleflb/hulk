use std::{env, path::PathBuf};

fn main() {
    if env::var_os("CARGO_FEATURE_ORIN_GST_CUDA").is_none() {
        return;
    }

    println!("cargo:rerun-if-changed=src/orin_jetson_wrapper.h");

    let out_dir = PathBuf::from(env::var("OUT_DIR").expect("OUT_DIR must be set"));
    let bindings = bindgen::Builder::default()
        .header("src/orin_jetson_wrapper.h")
        .allowlist_type("NvBufSurface")
        .allowlist_type("NvBufSurfaceParams")
        .allowlist_type("NvBufSurfaceMappedAddr")
        .allowlist_type("CUeglFrame")
        .allowlist_type("CUeglFrame_st")
        .allowlist_type("CUeglColorFormat")
        .allowlist_type("CUeglFrameType")
        .allowlist_type("CUarray_format")
        .allowlist_var("CU_EGL_.*")
        .allowlist_var("CUDA_EGL_.*")
        .allowlist_var("CU_AD_FORMAT_.*")
        .parse_callbacks(Box::new(bindgen::CargoCallbacks::new()))
        .generate()
        .expect("generate Jetson Orin bindings");

    bindings
        .write_to_file(out_dir.join("orin_jetson_bindings.rs"))
        .expect("write Jetson Orin bindings");
}
