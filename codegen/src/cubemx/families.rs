use crate::cubemx::Db;
use anyhow::{Context, Result};
use serde::Deserialize;

pub fn load(db: &Db) -> Result<Families> {
    db.load("families")
}

pub fn load_f3(db: &Db) -> Result<Family> {
    load(db)?
        .families
        .into_iter()
        .find(|f| f.name == "STM32F3")
        .context("STM32F3 family not found")
}

#[derive(Debug, Deserialize)]
#[serde(rename_all = "PascalCase")]
pub struct Families {
    #[serde(rename = "Family")]
    pub families: Vec<Family>,
}

#[derive(Debug, Deserialize)]
#[serde(rename_all = "PascalCase")]
pub struct Family {
    pub name: String,
    #[serde(rename = "SubFamily")]
    pub sub_families: Vec<SubFamily>,
}

#[derive(Debug, Deserialize)]
#[serde(rename_all = "PascalCase")]
pub struct SubFamily {
    pub name: String,
    #[serde(rename = "Mcu")]
    pub mcus: Vec<Mcu>,
}

#[derive(Debug, Deserialize)]
#[serde(rename_all = "PascalCase")]
pub struct Mcu {
    pub name: String,
}
