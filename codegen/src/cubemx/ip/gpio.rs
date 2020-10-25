use crate::cubemx::Db;
use anyhow::{bail, Context, Result};
use once_cell::sync::Lazy;
use regex::Regex;
use serde::Deserialize;
use std::path::PathBuf;

pub fn load(db: &Db, version: &str) -> Result<Ip> {
    let name = format!("GPIO-{}_Modes", version);
    let ip_path: PathBuf = ["IP", &name].iter().collect();
    db.load_mcu(&ip_path)
}

#[derive(Debug, Deserialize)]
#[serde(rename_all = "PascalCase")]
pub struct Ip {
    pub version: String,
    #[serde(rename = "GPIO_Pin")]
    pub pins: Vec<Pin>,
}

#[derive(Debug, Deserialize)]
#[serde(rename_all = "PascalCase")]
pub struct Pin {
    pub port_name: String,
    pub name: String,
    #[serde(rename = "PinSignal", default)]
    pub pin_signals: Vec<PinSignal>,
}

impl Pin {
    pub fn port(&self) -> Result<char> {
        static PORT_NAME: Lazy<Regex> = Lazy::new(|| Regex::new(r"^P(?P<id>[A-Z])$").unwrap());

        let captures = PORT_NAME
            .captures(&self.port_name)
            .with_context(|| format!("invalid GPIO port name: {}", self.port_name))?;

        let id = captures.name("id").unwrap().as_str();
        let id = id.parse()?;
        Ok(id)
    }

    pub fn number(&self) -> Result<u8> {
        static PIN_NAME: Lazy<Regex> =
            Lazy::new(|| Regex::new(r"^P[A-Z](?P<nr>\d{1,2})\b.*$").unwrap());

        let captures = PIN_NAME
            .captures(&self.name)
            .with_context(|| format!("invalid GPIO pin name: {}", self.name))?;

        let id = captures.name("nr").unwrap().as_str();
        let id = id.parse()?;
        Ok(id)
    }
}

#[derive(Debug, Deserialize)]
#[serde(rename_all = "PascalCase")]
pub struct PinSignal {
    pub name: String,
    specific_parameter: SpecificParameter,
}

impl PinSignal {
    pub fn af(&self) -> Result<u8> {
        let param = &self.specific_parameter;
        if param.name == "GPIO_AF" {
            parse_af(&param.possible_value)
        } else {
            bail!("PinSignal is missing a GPIO_AF parameter")
        }
    }
}

fn parse_af(s: &str) -> Result<u8> {
    static AF: Lazy<Regex> = Lazy::new(|| Regex::new(r"^GPIO_AF(?P<nr>\d{1,2})_\w+$").unwrap());

    let captures = AF
        .captures(s)
        .with_context(|| format!("invalid PinSignal AF: {}", s))?;

    let nr = captures.name("nr").unwrap().as_str().parse()?;
    Ok(nr)
}

#[derive(Debug, Deserialize)]
#[serde(rename_all = "PascalCase")]
struct SpecificParameter {
    name: String,
    possible_value: String,
}
