mod codegen;
mod cubemx;

use anyhow::Result;
use clap::{App, AppSettings, Arg, ArgMatches, SubCommand};
use cubemx::Db;

fn parse_args() -> ArgMatches<'static> {
    App::new("codegen")
        .about("Code generation for the stm32f3xx-hal crate")
        .setting(AppSettings::SubcommandRequiredElseHelp)
        .subcommand(
            SubCommand::with_name("gpio")
                .about("Generate GPIO mappings from an STM32CubeMX database")
                .arg(
                    Arg::with_name("db_path")
                        .help("Path of the STM32CubeMX MCU database (.../db/mcs/)")
                        .required(true),
                ),
        )
        .get_matches()
}

fn main() -> Result<()> {
    let args = parse_args();

    match args.subcommand() {
        ("gpio", Some(args)) => handle_gpio(args),
        _ => unreachable!(),
    }
}

fn handle_gpio(args: &ArgMatches) -> Result<()> {
    let db_path = args.value_of("db_path").unwrap();
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
