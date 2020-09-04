pub mod families;
pub mod ip;
pub mod mcu;
pub mod package;

mod db;

pub use db::Db;

use anyhow::Result;

pub fn load_f3_mcus(db: &Db) -> Result<Vec<mcu::Mcu>> {
    families::load_f3(db)?
        .sub_families
        .into_iter()
        .flat_map(|subfamily| subfamily.mcus.into_iter())
        .map(|mcu_| mcu::load(db, &mcu_.name))
        .collect()
}

pub fn load_f3_ips(db: &Db, ip_name: &str) -> Result<Vec<mcu::Ip>> {
    let f3_mcus = load_f3_mcus(db)?;
    let mut ips: Vec<_> = f3_mcus
        .into_iter()
        .flat_map(|mcu| mcu.ips.into_iter())
        .filter(|ip| ip.name == ip_name)
        .collect();

    ips.sort();
    ips.dedup();

    Ok(ips)
}

pub fn load_f3_gpio_ips(db: &Db) -> Result<Vec<ip::gpio::Ip>> {
    load_f3_ips(db, "GPIO")?
        .into_iter()
        .map(|ip_| ip::gpio::load(db, &ip_.version))
        .collect()
}
