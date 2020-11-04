use crate::cubemx::Db;
use anyhow::Result;
use serde::Deserialize;

pub fn load(db: &Db) -> Result<Package> {
    db.load("package")
}

#[derive(Debug, Deserialize)]
#[serde(rename_all = "PascalCase")]
pub struct Package {
    pub pack_description: PackDescription,
}

#[derive(Debug, Deserialize)]
#[serde(rename_all = "PascalCase")]
pub struct PackDescription {
    pub release: String,
}
