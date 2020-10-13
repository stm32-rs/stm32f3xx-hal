mod codegen;
mod cubemx;

use anyhow::Result;
use cubemx::Db;
use std::path::PathBuf;
use structopt::StructOpt;

#[derive(StructOpt)]
#[structopt(about = "Code generation for the stm32f3xx-hal crate")]
enum Command {
    #[structopt(about = "Generate GPIO mappings from an STM32CubeMX database")]
    Gpio {
        #[structopt(parse(from_os_str), help = "Path of the STM32CubeMX MCU database")]
        db_path: PathBuf,
    },
}

fn main() -> Result<()> {
    match Command::from_args() {
        Command::Gpio { db_path } => handle_gpio(db_path),
    }
}

fn handle_gpio(db_path: PathBuf) -> Result<()> {
    let db = cubemx::Db::new(db_path);

    emit_autogen_comment(&db)?;

    let gpio_ips = cubemx::load_f3_gpio_ips(&db)?;
    codegen::gpio::gen_mappings(&gpio_ips)?;

    Ok(())
}

fn emit_autogen_comment(db: &Db) -> Result<()> {
    let package = cubemx::package::load(&db)?;
    codegen::gen_autogen_comment(&package);

    Ok(())
}
