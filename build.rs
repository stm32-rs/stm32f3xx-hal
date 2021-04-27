use std::{env, fs::copy, path::PathBuf};

fn main() {
    copy_asm_lib();
    println!("cargo:rerun-if-changed=build.rs");
}

fn copy_asm_lib() {
    let out_dir = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    let name = env::var("CARGO_PKG_NAME").unwrap();
    copy("asm/asm.a", out_dir.join(format!("lib{}.a", name))).unwrap();
    println!("cargo:rustc-link-lib=static={}", name);
    println!("cargo:rustc-link-search={}", out_dir.display());
    println!("cargo:rerun-if-changed=asm/asm.a");
}
