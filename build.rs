use std::collections::hash_set::HashSet;
use std::env;
use std::fs::File;
use std::io::prelude::*;
use std::path::PathBuf;

// TODO(Sh3Rm4n - 2021-04-28):
// Remove when [feature="slice_group_by"] is stable.
// See with https://github.com/rust-lang/rust/issues/80552.
use slice_group_by::GroupBy;

const DEVICE_VARIANTS: [&str; 25] = [
    "stm32f301x6",
    "stm32f301x8",
    "stm32f318x8",
    "stm32f302x6",
    "stm32f302x8",
    "stm32f302xb",
    "stm32f302xc",
    "stm32f302xd",
    "stm32f302xe",
    "stm32f303x6",
    "stm32f303x8",
    "stm32f303xb",
    "stm32f303xc",
    "stm32f303xd",
    "stm32f303xe",
    "stm32f328x8",
    "stm32f358xc",
    "stm32f398xe",
    "stm32f373x8",
    "stm32f373xb",
    "stm32f373xc",
    "stm32f378xc",
    "stm32f334x4",
    "stm32f334x6",
    "stm32f334x8",
];

fn main() {
    check_device_feature();
    if cfg!(feature = "ld") {
        gen_memory_x();
    }
    println!("cargo:rerun-if-changed=build.rs");
}

/// Check device feature selection
fn check_device_feature() {
    // Check if the device is deprecated
    if !cfg!(feature = "device-selected") && cfg!(feature = "direct-call-deprecated") {
        eprintln!(
            "The (device-)feature you selected is deprecated, because it was split up into sub-devices.\n\
            \n\
            Example: The STM32F3Discovery board has a STM32F303VCT6 chip.\n\
            You probably used to use `stm32f303` but now functionalities for the sub-device were added.\n\
            In this case replace it with `stm32f303xc` to make your code build again.\n\
            \n\
            For more information, see \
            \x1b]8;;https://github.com/stm32-rs/stm32f3xx-hal#selecting-the-right-chip\x1b\\README \
            -> Selecting the right chip\x1b]8;;\x1b\\."
        );
        std::process::exit(1);
    }

    let device_variants: HashSet<String> =
        DEVICE_VARIANTS.iter().map(|s| (*s).to_string()).collect();

    // get all selected features via env variables
    let selected_features: HashSet<String> = env::vars()
        .filter_map(|(key, _)| {
            key.split("CARGO_FEATURE_")
                .nth(1)
                .map(|s| s.to_owned().to_ascii_lowercase())
        })
        .collect();

    // check if exactly one device was selected
    if device_variants.intersection(&selected_features).count() != 1 {
        eprintln!(
            "This crate requires you to specify your target chip as a feature.\n\
            \n\
            Please select **one** of the following (`x` denotes any character in [a-z]):\n"
        );

        // group device variants by type
        let mut device_variants: Vec<String> = device_variants.into_iter().collect();
        device_variants.sort_unstable();
        let device_variants = device_variants.linear_group_by(|a, b| a[..9] == b[..9]);

        // pretty print all avaliable devices
        for line in device_variants {
            for device in line {
                eprint!("{device} ");
            }
            eprintln!();
        }

        eprintln!(
            "\nExample: The STM32F3Discovery board has a STM32F303VCT6 chip.\n\
            So you need to specify stm32f303xc in your Cargo.toml (note that VC → xc).\n\
            \n\
            For more information, see \
            \x1b]8;;https://github.com/stm32-rs/stm32f3xx-hal#selecting-the-right-chip\x1b\\README \
            -> Selecting the right chip\x1b]8;;\x1b\\."
        );
        std::process::exit(1);
    }
}

/// Generate `memory.x` for selected device
///
/// Available RAM/CCMRAM/FLASH value is extracted from RM0313/RM0316/RM0364/RM0365/RM0366
fn gen_memory_x() {
    enum Mem {
        _4,
        _6,
        _8,
        B,
        C,
        D,
        E,
    }

    let mem = if cfg!(feature = "mem-4") {
        Mem::_4
    } else if cfg!(feature = "mem-6") {
        Mem::_6
    } else if cfg!(feature = "mem-8") {
        Mem::_8
    } else if cfg!(feature = "mem-b") {
        Mem::B
    } else if cfg!(feature = "mem-c") {
        Mem::C
    } else if cfg!(feature = "mem-d") {
        Mem::D
    } else if cfg!(feature = "mem-e") {
        Mem::E
    } else {
        eprintln!(
            "Memory size unknown.
This may be due to incorrect feature configuration in Cargo.toml or stm32f3xx-hal's internal issue."
        );
        std::process::exit(1);
    };

    let flash = match mem {
        Mem::_4 => 16,
        Mem::_6 => 32,
        Mem::_8 => 64,
        Mem::B => 128,
        Mem::C => 256,
        Mem::D => 384,
        Mem::E => 512,
    };
    let ccmram = if cfg!(feature = "svd-f303") || cfg!(feature = "svd-f3x4") {
        match mem {
            Mem::_4 | Mem::_6 | Mem::_8 => 4,
            Mem::B | Mem::C => 8,
            Mem::D | Mem::E => 16,
        }
    } else {
        0
    };
    let ram = match mem {
        Mem::_4 | Mem::_6 | Mem::_8 => 16,
        Mem::B if cfg!(feature = "svd-f373") => 24,
        Mem::B if cfg!(feature = "svd-f302") => 32,
        Mem::C if cfg!(feature = "svd-f373") => 32,
        Mem::B if cfg!(feature = "svd-f303") => 40,
        Mem::C if cfg!(feature = "svd-f302") => 40,
        Mem::C if cfg!(feature = "svd-f303") => 48,
        Mem::D | Mem::E if cfg!(feature = "svd-f302") => 64,
        Mem::D | Mem::E if cfg!(feature = "svd-f303") => 80,
        _ => {
            eprintln!(
                "Memory size unknown.
This may be due to incorrect feature configuration in Cargo.toml or stm32f3xx-hal's internal issue."
            );
            std::process::exit(1);
        }
    } - ccmram;

    let out_dir = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    let mut file = File::create(out_dir.join("memory.x")).unwrap();
    writeln!(file, "MEMORY {{").unwrap();
    writeln!(
        file,
        "    FLASH (rx) : ORIGIN = 0x8000000, LENGTH = {flash}K"
    )
    .unwrap();
    if ccmram > 0 {
        writeln!(
            file,
            "    CCMRAM (rwx) : ORIGIN = 0x10000000, LENGTH = {ccmram}K"
        )
        .unwrap();
    }
    writeln!(file, "    RAM (rwx) : ORIGIN = 0x20000000, LENGTH = {ram}K").unwrap();
    writeln!(file, "}}").unwrap();
    println!("cargo:rustc-link-search={}", out_dir.display());
}
