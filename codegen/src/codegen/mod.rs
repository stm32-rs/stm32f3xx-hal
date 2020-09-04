pub mod gpio;

use crate::cubemx::package::Package;

pub fn gen_autogen_comment(package: &Package) {
    println!("// auto-generated using codegen");
    println!(
        "// STM32CubeMX DB release: {}",
        package.pack_description.release
    );
}
