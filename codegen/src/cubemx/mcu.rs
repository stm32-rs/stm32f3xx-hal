use crate::cubemx::Db;
use anyhow::Result;
use serde::Deserialize;

pub fn load(db: &Db, name: &str) -> Result<Mcu> {
    db.load_mcu(name)
}

#[derive(Debug, Deserialize)]
#[serde(rename_all = "PascalCase")]
pub struct Mcu {
    pub ref_name: String,
    #[serde(rename = "IP")]
    pub ips: Vec<Ip>,
}

#[derive(Debug, Deserialize, Eq, Ord, PartialEq, PartialOrd)]
#[serde(rename_all = "PascalCase")]
pub struct Ip {
    pub name: String,
    pub version: String,
}
