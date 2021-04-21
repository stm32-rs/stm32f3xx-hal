use std::{env, fs::File, io::prelude::*, path::PathBuf};

fn main() {
    if cfg!(feature = "ld") {
        gen_memory_x();
    }
    println!("cargo:rerun-if-changed=build.rs");
}

fn gen_memory_x() {
    #![allow(clippy::unneeded_wildcard_pattern)]

    let mem_cfg_set = (
        cfg!(feature = "mem-4"),
        cfg!(feature = "mem-6"),
        cfg!(feature = "mem-8"),
        cfg!(feature = "mem-b"),
        cfg!(feature = "mem-c"),
        cfg!(feature = "mem-d"),
        cfg!(feature = "mem-e"),
    );
    let flash = match mem_cfg_set {
        (true, ..) => 16,
        (_, true, ..) => 32,
        (_, _, true, ..) => 64,
        (.., true, _, _, _) => 128,
        (.., _, true, _, _) => 256,
        (.., _, _, true, _) => 384,
        (.., _, _, _, true) => 512,
        _ => unreachable!(),
    };
    let ccmram = if cfg!(feature = "svd-f303") || cfg!(feature = "svd-f3x4") {
        match mem_cfg_set {
            (true, ..) | (_, true, ..) | (_, _, true, ..) => 4,
            (.., true, _, _, _) | (.., _, true, _, _) => 8,
            (.., _, _, true, _) | (.., _, _, _, true) => 16,
            _ => unreachable!(),
        }
    } else {
        0
    };
    let ram = match mem_cfg_set {
        (true, ..) | (_, true, ..) | (_, _, true, ..) => 16,
        (.., true, _, _, _) if cfg!(feature = "svd-f373") => 24,
        (.., true, _, _, _) if cfg!(feature = "svd-f302") => 32,
        (.., _, true, _, _) if cfg!(feature = "svd-f373") => 32,
        (.., true, _, _, _) if cfg!(feature = "svd-f303") => 40,
        (.., _, true, _, _) if cfg!(feature = "svd-f302") => 40,
        (.., _, true, _, _) if cfg!(feature = "svd-f303") => 48,
        (.., _, _, true, _) | (.., _, _, _, true) if cfg!(feature = "svd-f302") => 64,
        (.., _, _, true, _) | (.., _, _, _, true) if cfg!(feature = "svd-f303") => 80,
        _ => unreachable!(),
    } - ccmram;

    let out_dir = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    let mut file = File::create(out_dir.join("memory.x")).unwrap();
    writeln!(file, "MEMORY {{").unwrap();
    writeln!(file, "    FLASH (rx) : o = 0x8000000, l = {}K", flash).unwrap();
    if ccmram > 0 {
        writeln!(file, "    CCMRAM (rwx) : o = 0x10000000, l = {}K", ccmram).unwrap();
    }
    writeln!(file, "    RAM (rwx) : o = 0x20000000, l = {}K", ram).unwrap();
    writeln!(file, "}}").unwrap();
    println!("cargo:rustc-link-search={}", out_dir.display());
}
