#[allow(
    dead_code,
    improper_ctypes,
    non_camel_case_types,
    non_snake_case,
    non_upper_case_globals,
    unused_imports
)]
mod bindings {
    include!(concat!(env!("OUT_DIR"), "/x5_sdk_bindings.rs"));
}

pub use bindings::*;
